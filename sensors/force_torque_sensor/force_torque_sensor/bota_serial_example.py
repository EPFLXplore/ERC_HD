"""Prints the readings of a Bota Systems Serial sensor.

Usage: python bota_serial_example.py <port>

This example expects a device layout according to
_expected_device_layout, see below.
"""

import sys
import struct
import time
import threading

from collections import namedtuple

import serial
from crc import Calculator, Configuration


class BotaSerialSensor:

    BOTA_PRODUCT_CODE = 123456
    BAUDERATE = 460800
    SINC_LENGTH = 512
    CHOP_ENABLE = 0
    FAST_ENABLE = 0
    FIR_DISABLE = 1
    TEMP_COMPENSATION = 0 # 0: Disabled (recommended), 1: Enabled
    USE_CALIBRATION = 1 # 1: calibration matrix active, 0: raw measurements
    DATA_FORMAT = 0 # 0: binary, 1: CSV
    BAUDERATE_CONFIG = 4 # 0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800
    FRAME_HEADER = b'\xAA'
    # Note that the time step is set according to the sinc filter size!
    time_step = 0.01;

    def __init__(self):#,port):
        self._port = "/dev/ttyACM0" #port
        self._ser = serial.Serial()
        self._pd_thread_stop_event = threading.Event()
        DeviceSet = namedtuple('DeviceSet', 'name product_code config_func')
        self._expected_device_layout = {0: DeviceSet('BFT-SENS-SER-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}
        self._status = None;
        self._fx = 0.0;
        self._fy = 0.0;
        self._fz = 0.0;
        self._mx = 0.0;
        self._my = 0.0;
        self._mz = 0.0;
        self._timestamp = 0.0;
        self._temperature = 0.0;
        self._data = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.t = 0


    def bota_sensor_setup(self):
        print("Trying to setup the sensor.")
        # Wait for streaming of data
        out = self._ser.read_until(bytes('App Init', 'ascii'))
        #if not self.contains_bytes(bytes('App Init', 'ascii'), out):
            #print("Sensor not streaming, check if correct port selected!")
            #return False
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        # Go to CONFIG mode
        cmd = bytes('C', 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,C,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,C,0', 'ascii'), out):
            print("Failed to go to CONFIG mode.")
            return False

        # Communication setup
        comm_setup = f"c,{self.TEMP_COMPENSATION},{self.USE_CALIBRATION},{self.DATA_FORMAT},{self.BAUDERATE_CONFIG}"
        #print(comm_setup)
        cmd = bytes(comm_setup, 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,c,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,c,0', 'ascii'), out):
            print("Failed to set communication setup.")
            return False
        self.time_step = 0.00001953125*self.SINC_LENGTH
        print("Timestep: {}".format(self.time_step))

        # Filter setup
        filter_setup = f"f,{self.SINC_LENGTH},{self.CHOP_ENABLE},{self.FAST_ENABLE},{self.FIR_DISABLE}"
        #print(filter_setup)
        cmd = bytes(filter_setup, 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,f,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,f,0', 'ascii'), out):
            print("Failed to set filter setup.")
            return False

        # Go to RUN mode
        cmd = bytes('R', 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,R,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,R,0', 'ascii'), out):
            print("Failed to go to RUN mode.")
            return False

        return True

    def contains_bytes(self, subsequence, sequence):
        return subsequence in sequence

    def _processdata_thread(self):
        while not self._pd_thread_stop_event.is_set():
            frame_synced = False
            crc16X25Configuration = Configuration(16, 0x1021, 0xFFFF, 0xFFFF, True, True)
            crc_calculator = Calculator(crc16X25Configuration)

            while not frame_synced and not self._pd_thread_stop_event.is_set():
                possible_header = self._ser.read(1)
                if self.FRAME_HEADER == possible_header:
                    #print(possible_header)
                    data_frame = self._ser.read(34)
                    crc16_ccitt_frame = self._ser.read(2)

                    crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                    checksum = crc_calculator.checksum(data_frame)
                    if checksum == crc16_ccitt:
                        print("Frame synced")
                        frame_synced = True
                    else:
                        
                        self._ser.read(1)

            while frame_synced and not self._pd_thread_stop_event.is_set():            
                start_time = time.perf_counter()
                frame_header = self._ser.read(1)

                if frame_header != self.FRAME_HEADER:
                    print("Lost sync")
                    frame_synced = False
                    break

                data_frame = self._ser.read(34)
                crc16_ccitt_frame = self._ser.read(2)

                crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                checksum = crc_calculator.checksum(data_frame)
                if checksum != crc16_ccitt:
                    print("CRC mismatch received")
                    break

                self._status = struct.unpack_from('H', data_frame, 0)[0]

                self._fx = struct.unpack_from('f', data_frame, 2)[0]
                self._fy = struct.unpack_from('f', data_frame, 6)[0]
                self._fz = struct.unpack_from('f', data_frame, 10)[0]
                self._mx = struct.unpack_from('f', data_frame, 14)[0]
                self._my = struct.unpack_from('f', data_frame, 18)[0]
                self._mz = struct.unpack_from('f', data_frame, 22)[0]

                self._timestamp = struct.unpack_from('I', data_frame, 26)[0]

                self._temperature = struct.unpack_from('f', data_frame, 30)[0]
                
                time_diff = time.perf_counter() - start_time

                self._data = [self._fx, self._fy, self._fz, self._mx, self._my, self._mz]
                self.t = self.t + 1
                #print("t=",self.t)
                #print("time_diff ",time_diff )

                #print("self._data",self._data)
                

                
                
 

    def _my_loop(self):

        try:
            while 1:
                '''
                print('Run my loop')

                print("Status {}".format(self._status))
                #print(self._fx)
                print("Fx {}".format(self._fx))
                print("Fy {}".format(self._fy))
                print("Fz {}".format(self._fz))
                print("Mx {}".format(self._mx))
                print("My {}".format(self._my))
                print("Mz {}".format(self._mz))

                print("Timestamp {}".format(self._timestamp))

                print("Temperature {}\n".format(self._temperature))
                '''

                #self._data = bota_sensor_1.get_sensor_data()
                #print("self._data=",self._data)
                #print("Sensor data:", self._data)
                # CA TOURNE ICI

                
                time.sleep(1.0)

                

        except KeyboardInterrupt:
            # ctrl-C abort handling
            print('stopped')
            return  # ADDED


    def run(self):

        self._ser.baudrate = self.BAUDERATE
        self._ser.port = self._port
        self._ser.timeout = 10

        try:
            self._ser.open()
            print("Opened serial port {}".format(self._port))
        except:
            raise BotaSerialSensorError('Could not open port')

        if not self._ser.is_open:
            raise BotaSerialSensorError('Could not open port')

        if not self.bota_sensor_setup():
            print('Could not setup sensor!')
            return

        #check_thread = threading.Thread(target=self._check_thread)
        #check_thread.start()
        proc_thread = threading.Thread(target=self._processdata_thread)
        proc_thread.start()
        
        '''
        device_running = True



        if device_running:
            print("device_running")
            #print("self._data=",self._data)
            
            return self._my_loop()
        '''
        '''
        self._pd_thread_stop_event.set()
        proc_thread.join()
        #check_thread.join()

        self._ser.close()
        '''

        '''
        if not device_running:
            raise BotaSerialSensorError('Device is not running')
        '''
            

    @staticmethod
    def _sleep(duration, get_now=time.perf_counter):
        now = get_now()
        end = now + duration
        while now < end:
            now = get_now()

    def get_sensor_data(self):
        return [self._fx, self._fy, self._fz, self._mx, self._my, self._mz]

class BotaSerialSensorError(Exception):
    def __init__(self, message):
        super(BotaSerialSensorError, self).__init__(message)
        self.message = message


if __name__ == '__main__':

    '''
    print('bota_serial_example started')

    if len(sys.argv) > 1:
        try:
            bota_sensor_1 = BotaSerialSensor(sys.argv[1]);
            bota_sensor_1.run()
        except BotaSerialSensorError as expt:
            print('bota_serial_example failed: ' + expt.message)
            sys.exit(1)
    else:
        print('usage: bota_serial_example portname')
        sys.exit(1)
    '''
    print('bota_serial_example started')
    

    #if len(sys.argv) > 1:
    try:
        bota_sensor_1 = BotaSerialSensor()
        bota_sensor_1.run()
        #print("self._data=",bota_sensor_1._data)

        '''
        while True:
            data = bota_sensor_1.get_sensor_data()
            print("Sensor data:", data)
            time.sleep(1.0)
        '''
    except BotaSerialSensorError as expt:
        print('bota_serial_example failed: ' + expt.message)
        sys.exit(1)
    #else:
        #print('usage: bota_serial_example portname')
        #sys.exit(1)
