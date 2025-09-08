#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mainspace.msg import Position, ToStmSpeed, Coffee, Pause
import serial
import time
import threading
import queue

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')
        self.lock = threading.Lock()
        self.ack_queue = queue.Queue()
        self.tx_queue = queue.PriorityQueue()

        self.pending_cmd = None       
        self.pending_time = 0.0       
        self.ack_timeout = 0.3

        self.timer = self.create_timer(0.05, self.timer_callback)

        # Position publisher（STM32 ⇒ ROS2）
        self.position_publisher = self.create_publisher(Position, 'stm_position', 10)

        self.speed_subscription = self.create_subscription(
            ToStmSpeed,
            'motor_speed',
            self.speed_callback,
            10
        )
        self.coffee_subscription = self.create_subscription(
            Coffee,
            'coffee',
            self.coffee_callback,
            10
        )
        self.pause_subscription = self.create_subscription(
            Pause,
            'pause',
            self.pause_callback,
            10
        )
        

        # 開啟串列埠
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info('Serial port opened: /dev/ttyACM0')
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
                # self.get_logger().debug(f"[UART READ] {line}")

                if line == "ACK":
                    self.ack_queue.put(True)
                elif line.startswith("POS:"):
                    try:
                        parts = line.replace("POS:", "").split(',')
                        if len(parts) == 3:
                            x, y, theta = map(float, parts)
                            msg = Position(x=x, y=y, theta=theta)
                            self.position_publisher.publish(msg)
                            # self.get_logger().info(f"[ROS2 ⇐ STM32] {msg}")
                    except ValueError:
                        self.get_logger().warn(f"[PARSE ERROR] {line}")
                else:
                    self.get_logger().warn(f"[UNKNOWN LINE] {line}")
            except Exception as e:
                self.get_logger().error(f"UART read exception: {e}")

    def speed_callback(self, msg):
        data = f"Speed:{msg.vx:.4f},{msg.vy:.4f},{msg.w:.4f}\n"
        self.tx_queue.put((3, data))

    def coffee_callback(self, msg):
        data = f"Coffee:{msg.type},{msg.number}\n"
        self.tx_queue.put((1, data))

    def pause_callback(self, msg):
        data = f"Pause:{msg.pause}\n"
        self.tx_queue.put((0, data)) 

    # ----------- Timer 統一處理 UART ----------
    def timer_callback(self):
        now = time.time()
        try:
            with self.lock:
                # --- 如果有 pending CMD，先檢查 ACK ---
                if self.pending_cmd:
                    if not self.ack_queue.empty():
                        self.ack_queue.get_nowait()   
                        self.get_logger().info("ACK received, CMD done")
                        self.pending_cmd = None       
                    elif now - self.pending_time > self.ack_timeout:
                        self.ser.write(self.pending_cmd.encode('utf-8'))
                        self.pending_time = now
                        self.get_logger().warn(f"Resent CMD: {self.pending_cmd.strip()}")
                    return

                if not self.tx_queue.empty():
                    priority, data = self.tx_queue.get_nowait()
                    if data.startswith("Coffee:") or data.startswith("Pause:"):
                        self.ser.write(data.encode('utf-8'))
                        self.pending_cmd = data
                        self.pending_time = now
                        self.get_logger().info(f"Sent CMD: {data.strip()} (waiting for ACK)")
                    else:
                        # POS → 直接送，不等 ACK
                        self.ser.write(data.encode('utf-8'))

        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

            
    def destroy_node(self):
        self.running = False
        self.reader_thread.join(timeout=1)
        self.ser.close()
        super().destroy_node()
            
def main(args=None):
    print("[motor_serial_node] Starting node...")
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()