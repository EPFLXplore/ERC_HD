#simple service to test the button press client works

import rclpy
from rclpy.node import Node
from hd_interfaces.srv import ButtonPressControlPanel

class ButtonPressServer(Node):

    def __init__(self):
        super().__init__('button_press_server')
        # Create a service named 'button_press_service'
        self.srv = self.create_service(ButtonPressControlPanel, '/gui_node/button_press_service', self.handle_button_press)

    def handle_button_press(self, request, response):
        self.get_logger().info(f"Button pressed: {request.button_name}")
        response.response = f"Button {request.button_name} was pressed"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ButtonPressServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
