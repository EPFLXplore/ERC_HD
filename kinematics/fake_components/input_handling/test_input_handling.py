#!/usr/bin/env python3

from input_handling.keyboard import KeyboardConfig


if __name__ == "__main__":
    config = KeyboardConfig()
    config.background_loop(daemon=False)
