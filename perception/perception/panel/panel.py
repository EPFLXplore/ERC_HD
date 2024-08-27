import yaml
from .button import Button


class Panel:

    def __init__(self, config_file, camera_matrix):
        self._init_from_config(config_file, camera_matrix)
        self.target = 0

    def _init_from_config(self, config_file, camera_matrix):
        with open(config_file, "r") as file:
            self.config = yaml.safe_load(file)
        self._load_buttons(camera_matrix)

    def draw(self, frame, transform):
        for button in self.buttons:
            button.draw(frame, transform)

    def get_target(self):
        return self.buttons[self.target].get_target()

    def set_target(self, target):
        button_id = int(target[0])
        self.buttons[self.target].target = "no"
        self.target = button_id
        if target[1] == "u":
            self.buttons[button_id].target = "top"
        if target[1] == "d":
            self.buttons[button_id].target = "bottom"

    def _load_buttons(self, camera_matrix):
        buttons_width = self.config["buttons_width"]
        buttons_height = self.config["buttons_height"]
        buttons_horizontal_distance = self.config["buttons_horizontal_distance"]
        buttons_vertical_distance = self.config["buttons_vertical_distance"]
        center_offset_fraction = self.config["center_offset_fraction"]

        all_buttons_info = {
            "height": buttons_height,
            "width": buttons_width,
            "click_offset_fraction": center_offset_fraction,
            "camera_matrix": camera_matrix,
        }

        buttons = self.config

        rows = {
            0: buttons_height // 2,
            1: buttons_height // 2 - buttons_vertical_distance,
            2: buttons_height // 2 - 2 * buttons_vertical_distance,
        }
        cols = {
            0: -buttons_width,
            1: 0,
            2: buttons_horizontal_distance - buttons_width,
            3: buttons_horizontal_distance,
        }

        self.buttons: list[Button] = []
        for button_name, button_config in self.config["buttons"].items():
            button_params = all_buttons_info.copy()
            print(
                f"button params keys: {button_params.keys()}, height: {button_params['height']}"
            )
            button_params["vertical_offset"] = rows[button_config["y"]]
            button_params["horizontal_offset"] = cols[button_config["x"]]
            button_params["relative_position"] = button_config["relative_position"]
            button = Button(**button_params)
            self.buttons.append(button)
        self.buttons[0].target = "top"
