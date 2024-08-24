import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from hd_interfaces.srv import ButtonPressControlPanel

BUTTON_WIDTH = 38
BUTTON_HEIGHT = 30

GRID_H_SPACING = 2 * BUTTON_WIDTH + 20  # Horizontal spacing between buttons
GRID_V_SPACING = 2 * BUTTON_HEIGHT + 30  # Vertical spacing between buttons


class GuiNode(Node):
    def __init__(self):
        super().__init__("gui_node")
        
        #publishers
        self.publisher_ = self.create_publisher(String, "command_topic", 10)
        #service client
        self.client = self.create_client(ButtonPressControlPanel, '/gui_node/button_press_service')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Service not available, waiting...")
        

        self.window = tk.Tk()
        self.window.title("ROS 2 Command Sender")
        self.window.geometry("600x400")

        # Create GUI elements
        self.canvas = tk.Canvas(self.window, width=600, height=400)
        self.canvas.place(x=BUTTON_WIDTH, y=BUTTON_HEIGHT)

        # Draw the dark grey rectangle
        self.canvas.create_rectangle(
            0,
            0,
            BUTTON_WIDTH * 4 + 30,
            BUTTON_HEIGHT * 6 + 70,
            fill="darkgrey",
        )

        # Dynamically create buttons based on a grid
        self.create_buttons()

        # Close the node when the window is closed
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_buttons(self):
        # Get the button grid configuration
        grid_positions = [
            {"x": 1 * GRID_H_SPACING, "y": 0 * GRID_V_SPACING, "id": 0},
            {"x": 0 * GRID_H_SPACING, "y": 1 * GRID_V_SPACING, "id": 2},
            {"x": 1 * GRID_H_SPACING, "y": 1 * GRID_V_SPACING, "id": 4},
            {"x": 0 * GRID_H_SPACING, "y": 2 * GRID_V_SPACING, "id": 6},
            {"x": 1 * GRID_H_SPACING, "y": 2 * GRID_V_SPACING, "id": 8},
        ]

        for g in grid_positions:
            x, y, id_ = g["x"], g["y"], g["id"]

            grid = get_grid()

            # Create buttons dynamically
            for i, btn_info in enumerate(grid):
                button = tk.Button(
                    self.window,
                    text=f"{id_ + btn_info['id']}{btn_info['text']}",
                    command=lambda b=f"{id_ + btn_info['id']}{btn_info['text']}": self.send_command(
                        b
                    ),
                    bg="white",
                )
                button.place(
                    x=(x + btn_info["x"] + BUTTON_WIDTH),
                    y=(y + btn_info["y"] + BUTTON_HEIGHT),
                )

    def send_command(self, button_name):
        self.get_logger().info(f"Sent request for : {button_name}")
        request = ButtonPressControlPanel.Request()
        request.button_name = button_name
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f"Button Press Service Response: {response.response}")

    def on_closing(self):
        rclpy.shutdown()
        self.window.destroy()

    def run(self):
        self.window.mainloop()


def get_grid():
    return [
        {"x": 0 * BUTTON_WIDTH, "y": 0 * BUTTON_HEIGHT, "id": 0, "text": "u"},
        {"x": 1 * BUTTON_WIDTH, "y": 0 * BUTTON_HEIGHT, "id": 1, "text": "u"},
        {"x": 0 * BUTTON_WIDTH, "y": 1 * BUTTON_HEIGHT, "id": 0, "text": "d"},
        {"x": 1 * BUTTON_WIDTH, "y": 1 * BUTTON_HEIGHT, "id": 1, "text": "d"},
    ]


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    node.run()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
