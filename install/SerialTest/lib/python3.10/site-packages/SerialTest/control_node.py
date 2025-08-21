import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
import subprocess
import signal
import time
import os


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # 保存進程對應表
        self.procs = {}

        self.create_subscription(
            Bool, '/Stage2_start',
            lambda msg: self.toggle_node(msg, 'Stage2', ['ros2', 'run', 'mainspace', 'Stage2']),
            10
        )
        self.create_subscription(
            Bool, '/cameraCoffee2_start',
            lambda msg: self.toggle_node(msg, 'cameraCoffee2', ['ros2', 'run', 'camera', 'cameraCoffee2']),
            10
        )
        self.create_subscription(
            Bool, '/cameraDesk2_start',
            lambda msg: self.toggle_node(msg, 'cameraDesk2', ['ros2', 'run', 'camera', 'cameraDesk2']),
            10
        )
        self.create_subscription(
            Bool, '/Stage3_start',
            lambda msg: self.toggle_node(msg, 'Stage3', ['ros2', 'run', 'mainspace', 'Stage3']),
            10
        )
        self.create_subscription(
            Bool, '/Stage4_start',
            lambda msg: self.toggle_node(msg, 'Stage4', ['ros2', 'run', 'mainspace', 'Stage4']),
            10
        )

    def toggle_node(self, msg, name, base_cmd):
        if msg.data:
            if name not in self.procs:
                unique_name = f"{name}_{int(time.time())}"
                cmd = base_cmd + ['--ros-args', '-r', f'__node:={unique_name}']
                self.get_logger().info(f"啟動 {unique_name}")
                # 放到新 process group，方便後面整組殺掉
                self.procs[name] = subprocess.Popen(cmd, preexec_fn=os.setsid)
        else:
            if name in self.procs:
                self.get_logger().info(f"停止 {name}")
                try:
                    os.killpg(os.getpgid(self.procs[name].pid), signal.SIGINT)
                    self.procs[name].wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f"{name} 沒有響應 SIGINT，強制 kill")
                    os.killpg(os.getpgid(self.procs[name].pid), signal.SIGKILL)
                    self.procs[name].wait()
                finally:
                    del self.procs[name]

    def cleanup(self):
        self.get_logger().info("清理所有子節點...")
        for name, proc in list(self.procs.items()):
            self.get_logger().info(f"關閉 {name}")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"{name} 沒響應，強制 kill")
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait()
            finally:
                del self.procs[name]
        self.get_logger().info("清理完成")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    executor = MultiThreadedExecutor(num_threads=4)  # 可以調整線程數量
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，開始清理")
    finally:
        node.cleanup()
        executor.shutdown()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
