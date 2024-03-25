from ball_beam_msgs.srv import PotentiometerCalibration
import rclpy
from rclpy.node import Node

class PotentiometerClient(Node):

    def __init__(self):
        super().__init__('client_pot_calibration')

        self.cli = self.create_client(PotentiometerCalibration, '/potentiometer/req_move_and_read')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PotentiometerCalibration.Request()

    def send_request(self, increment_servo_by):
        self.req.servo_increment = increment_servo_by
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()