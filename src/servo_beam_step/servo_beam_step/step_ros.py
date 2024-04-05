from ball_beam_msgs.srv import TriggerStep
from ball_beam_msgs.msg import StepData
import rclpy
from rclpy.node import Node

class StepROS(Node):

    def __init__(self):
        super().__init__('step_response_node')

        self.cli = self.create_client(TriggerStep, '/step/req_new_exp')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TriggerStep.Request()

        self.subscription = self.create_subscription(
            StepData,
            '/step/data',
            self.receive_data,
            10000)
        
        self.time = []
        self.servo = []
        self.pot = []

    def send_request(self, step):
        self.time = []
        self.servo = []
        self.pot = []

        self.req.servo_step = step
        self.future = self.cli.call_async(self.req)

        return self.future

    def check_status(self):
        if self.future.done():
            return True
        else:
            return False

    def return_data(self):
        return self.req.servo_step, self.time, self.servo, self.pot

    def receive_data(self, msg):
        # Convertendo para segundo
        self.time.append(msg.time/1000)

        # Centralizando em 0 (58 graus na montagem) e convertendo para rad
        self.servo.append(3.1415*(msg.servo-58)/180.0)

        # Convertendo para rad
        self.pot.append(0.0010037983680867989*(msg.pot_read)-2.5051757456517563)