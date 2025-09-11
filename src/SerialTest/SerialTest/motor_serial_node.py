#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from mainspace.msg import Position, ToStmSpeed, Coffee, Pause
import os, psutil
import serial
import time
import threading
import queue

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')
        self.lock = threading.Lock()
        self.ack_queue = queue.Queue()
        # self.tx_queue = queue.PriorityQueue()
        self.speed_queue = None
        self.coffee_queue = None
        self.pause_queue = None

        self.pending_cmd = None       
        self.pending_time = 0.0       
        self.ack_timeout = 1.0

        self.timer = self.create_timer(0.1, self.timer_callback)

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
            self.ser = serial.Serial(
                '/dev/ttyACM0',
                115200,
                timeout=0.01,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False
            )
            self.ser.reset_input_buffer()   # 清掉接收端緩衝區 (RX)
            self.ser.reset_output_buffer()  # 清掉發送端緩衝區 (TX)

            self.ser.dtr = False
            self.ser.rts = False
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
        data = f"V:{msg.vx} {msg.vy} {msg.w}\n"
        with self.lock:
            if self.speed_queue is None:
                self.speed_queue = data
                self.pending_time = 0

    def coffee_callback(self, msg):
        data = f"C:{4 * (msg.type) + msg.number}\n"
        with self.lock:
            if self.coffee_queue is None:
                self.coffee_queue = data
                self.pending_time = 0

    def pause_callback(self, msg):
        data = f"P:{int(msg.pause)}\n"
        with self.lock:
            if self.pause_queue is None:
                self.pause_queue = data
                self.pending_time = 0

    # ----------- Timer 統一處理 UART ----------
    def timer_callback(self):
        now = time.time()
        try:
            with self.lock:
                if self.pause_queue:
                    if not self.ack_queue.empty():
                        self.ack_queue.get_nowait()   
                        self.get_logger().info("ACK received, CMD done")
                        self.pause_queue = None
                    elif now - self.pending_time > self.ack_timeout:
                        self.ser.write(self.pause_queue.encode('utf-8'))
                        self.ser.flush()
                        self.pending_time = now
                        self.get_logger().warn(f"Resent CMD: {self.pause_queue.strip()}")
                elif self.coffee_queue:
                    if not self.ack_queue.empty():
                        self.ack_queue.get_nowait()   
                        self.get_logger().info("ACK received, CMD done")
                        self.coffee_queue = None
                    elif now - self.pending_time > self.ack_timeout:
                        self.ser.write(self.coffee_queue.encode('utf-8'))
                        self.ser.flush()
                        self.pending_time = now
                        self.get_logger().warn(f"Resent CMD: {self.coffee_queue.strip()}")
                elif self.speed_queue:
                    self.ser.write(self.speed_queue.encode('utf-8'))
                    self.ser.flush()

        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")
            
    def destroy_node(self):
        self.running = False
        self.reader_thread.join(timeout=1)
        self.ser.close()
        super().destroy_node()
            
def main(args=None):
    p = psutil.Process(os.getpid())
    p.nice(-10)
    print("[mission_serial_node] Starting node...")
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
