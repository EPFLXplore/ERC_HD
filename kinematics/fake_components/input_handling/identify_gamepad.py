import evdev
import time


def connect() -> evdev.InputDevice:
    print("connecting to gamepad...")
    while True:
        for device in map(evdev.InputDevice, evdev.list_devices()):
            print(device)
            return device
        time.sleep(1)


def read_inputs(device: evdev.InputDevice):
    try:
        for event in device.read_loop():
            print(event.code)

    except (TypeError, IOError) as e:
        raise
        

def loop():
    device = connect()
    while True:
        read_inputs(device)
        time.sleep(0.05)


if __name__ == "__main__":
    loop()
