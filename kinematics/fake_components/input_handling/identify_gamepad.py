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
            print(event)

    except (TypeError, IOError) as e:
        raise
        

def loop():
    device = connect()
    while True:
        read_inputs(device)
        time.sleep(0.05)


if __name__ == "__main__":
    loop()




"""
new config:

name: Xbox Wireless Controller

RH:
    code: 2
    range: 0 - 65535  
    type. 3

RV:
    code: 5
    range: 0- 65535(0 en haut)
    type: 3

LH:
    code: 0
    range: 0-65535
    type:3

LV:
    code:1
    range:0- 65535(0 en haut)
    type:3

X:
    code: 307
    val: 0/1
    type: 1


A:
    code: 304
    val: 0/1
    type: 1

B:
    code: 305
    val: 0/1
    type: 1

Y:
    code:308
    val: 0/1
    type: 1


CT(Cross Top):
    code:17
    val: 0/-1
    type:3

CB(Cross Bottom):
    code:17
    val: 0/1
    type:3

CR(Cross Right):
    code: 16
    val: 0/-1
    type:3

CL(Cross Left):
    code:16
    val: 0/1
    type:3

H (Home button? with light):
    code: 316
    val: 0/1
    type:1

RT:
    code:9
    range: 0-1023
    type:3

RB:
    code: 311
    val:0/1
    type:1

LT:
    code:310 /10
    val:0/1
    type:3

LB:
    code:10 / 310
    range:0-1023
    type:1

2 fenetres:
    code:158
    val:0/1
    type:1

3 lignes:
    code:315
    val:0/1
    type:1


"""










"""
SZMY-POWER CO.,LTD. PLAYSTATION(R)3 Controller




RH:
    code: 3
    offset: 128
    amp: 128

RV:
    code: 4
    offset: 128
    amp: -128

LH:
    code: 0
    offset: 128
    amp: 128

LV:
    code: 1
    offset: 128
    amp: -128

R2:
    code: 5
    offset: 0
    amp: 255

L2:
    code: 2
    offset: 0
    amp: 255

R1:
    code: 311

L1:
    310

circle:
    305

cross:
    304

square:
    308

triangle:
    307

options:
    315

share:
    314

ps:
    316














































"""