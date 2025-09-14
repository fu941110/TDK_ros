#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mainspace.msg import Command
import os, psutil
import serial
import time
import threading
import queue

class MissionSerialNode(Node):
    def __init__(self):
        super().__init__('mission_serial_node')
        self.lock = threading.Lock()
        self.ack_queue = queue.Queue()

        self.command_publisher = self.create_publisher(Command, 'commandToROS', 10)

        self.subscription = self.create_subscription(
            Command,
            'commandToSTM',
            self.command_callback,
            10
        )
        self.last_sent_time = 0.0
        self.send_interval = 3.0
        self.running = True

        # 用來保存上一筆 ROS2 指令，與其超時檢查用
        self.last_ros_cmd = None
        self.waiting_final_ack = False
        self.final_ack_timer_start = 0.0

        self.create_timer(0.1, self.check_final_ack_timeout)

        # 開啟串列埠
        try:
            # port既得改
            self.ser = serial.Serial(
                '/dev/ttyACM1',
                115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False
            )
            self.ser.reset_input_buffer()   # 清掉接收端緩衝區 (RX)
            self.ser.reset_output_buffer()  # 清掉發送端緩衝區 (TX)

            self.get_logger().info('Serial port opened: /dev/ttyACM1')
            self.running = True
            self.reader_thread = threading.Thread(target=self.uart_reader_thread, daemon=True)
            self.reader_thread.start()
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            return

        # self.timer = self.create_timer(0.01, self.timer_callback)
        
        
    def uart_reader_thread(self):
        while self.running:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    continue
                self.get_logger().debug(f"[UART READ] {line}")

                if line.startswith("ACK:"):
                    try:
                        cmd_str = line.replace("ACK:", "").strip()
                        msg = Command()
                        msg.info = cmd_str
                        self.get_logger().info(f"[Receive info] {msg.info}")
                        self.command_publisher.publish(msg)

                        self.waiting_final_ack = False
                        self.last_ros_cmd = None

                    except ValueError:
                        self.get_logger().warn(f"[PARSE ERROR] {line}")
                elif line == "ACK":
                    self.get_logger().info(f"Got ACK")
                    self.ack_queue.put(True)
                    self.waiting_final_ack = True
                    self.final_ack_timer_start = time.time()
                else:
                    self.get_logger().warn(f"[UNKNOWN LINE] {line}")
            except Exception as e:
                self.get_logger().error(f"UART read exception: {e}")      

    def wait_for_ack(self, timeout=0.2):
        try:
            return self.ack_queue.get(timeout=timeout)
        except queue.Empty:
            return False

    def command_callback(self, msg):
        self.last_ros_cmd = msg.info
        self.waiting_final_ack = False  # 重置狀態

        with self.lock:
            max_retry = 3
            retry = 0
            while not self.wait_for_ack(timeout=0.2):
                now = time.time()
                if retry > max_retry:
                    self.get_logger().info(f"max retry times")
                    return
                if now - self.last_sent_time > self.send_interval:
                    try:
                        data = f"CMD:{msg.info}\n"
                        self.ser.write(data.encode('utf-8'))
                        self.last_sent_time = now
                        # self.get_logger().info(f"[ROS2 ⇒ STM32] Sent: {data.strip()}")
                        retry += 1
                        self.ser.flush()
                    except Exception as e:
                        self.get_logger().error(f"Serial write error: {e}")
                        return

    def check_final_ack_timeout(self):
        if self.waiting_final_ack and self.last_ros_cmd:
            elapsed = time.time() - self.final_ack_timer_start
            if elapsed > 10000000.0:
                # 超時補發
                msg = Command()
                msg.info = f"{self.last_ros_cmd}_OK"
                self.command_publisher.publish(msg)
                self.get_logger().warn(f"[TIMEOUT] No final ACK, auto-publish: {msg.info}")

                # 重置狀態
                self.waiting_final_ack = False
                self.last_ros_cmd = None
            
    def destroy_node(self):
        self.running = False
        self.reader_thread.join(timeout=1)
        self.ser.close()
        super().destroy_node()
            
def main(args=None):
    p = psutil.Process(os.getpid())
    p.nice(5)
    print("[other_serial_node] Starting node...")
    rclpy.init(args=args)
    node = MissionSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
