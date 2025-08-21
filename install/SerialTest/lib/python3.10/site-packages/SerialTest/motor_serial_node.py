#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mainspace.msg import Position, ToStmSpeed
import serial
import time
import threading
import queue

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')
        self.lock = threading.Lock()
        self.ack_queue = queue.Queue()

        # Position publisher（STM32 ⇒ ROS2）
        self.position_publisher = self.create_publisher(Position, 'stm_position', 10)

        # EncoderSpeed subscriber（ROS2 ⇒ STM32）
        self.subscription = self.create_subscription(
            ToStmSpeed,
            'motor_speed',
            self.speed_callback,
            10
        )
        self.last_sent_time = 0.0
        self.send_interval = 0.05
        

        # 開啟串列埠
        try:
            #　port: 下,ACM0 for 底盤
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
                
    def wait_for_ack(self, timeout=0.3):
        try:
            return self.ack_queue.get(timeout=timeout)
        except queue.Empty:
            return False

    def speed_callback(self, msg):
        now = time.time()
        if now - self.last_sent_time < self.send_interval:
            return

        with self.lock:  # 保護 serial.write
            try:
                data = f"Speed:{msg.vx},{msg.vy},{msg.w}\n"
                self.ser.write(data.encode('utf-8'))
                # self.get_logger().info(f"[ROS2 ⇒ STM32] Sent: {data.strip()}")
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")
                return

        if self.wait_for_ack():
            self.last_sent_time = now
        else:
            self.get_logger().warn("Timeout waiting for ACK")
            
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