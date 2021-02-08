import serial
import numpy as np
import threading
import time
from datetime import datetime
import pandas as pd
from os import path
import json


# CSV File Parameters
CSV_FILE_PATH = 'Data_Log.csv'
MAX_SATELLITE_DEVICES = 10
HEADERS = ['Date']
for i in range(MAX_SATELLITE_DEVICES):
    HEADERS = HEADERS + ['Temperature {}'.format(i),
                         'Lighting {}'.format(i)]


# JSON Parameters
JSON_PATH = 'dorn.json'


# Serial Parameters
PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200


# Temperature PID Gains
P = 1.0
I = 1.9e-5
D = -0.276


# Lighting Control Parameters
LIGHTING_TIME_OUT = 10
MAX_LUX = 1000
HEIGHT_STEP_SIZE = 0.05
TILT_STEP_SIZE = 0.5
ERROR_THRESHOLD = 0.1


# Timing Parameters
TEMP_CONTROL_PERIOD = 60  # seconds
LIGHT_CONTROL_PERIOD = 60
LOG_DATA_PERIOD = 60


def interpret_serial_data(serial_input):
    # This function converts the string received through serial to numerical data
    buffer = 2 * MAX_SATELLITE_DEVICES * [np.nan]

    dict = json.loads(serial_input)
    buffer[0] = dict['data']['temp']
    buffer[1] = dict['data']['light']

    n = 2
    for item in dict['children'].items():
        buffer[n] = item[1]['data']['temp']
        buffer[n + 1] = item[1]['data']['light']
        n += 2
    return buffer


def reset_json():
    data = {}
    data['current temperature'] = 0.0
    data['current lighting'] = 0.0
    data['reference temperature'] = 0.0
    data['reference lighting'] = 0.0
    data['manual heater control'] = False
    data['manual heater value'] = 0
    data['manual blind control'] = False
    data['manual height value'] = 0
    data['manual tilt value'] = 0
    with open('dorn.json', 'w') as outfile:
        json.dump(data, outfile)


def read_json(header):
    with open(JSON_PATH, 'r') as json_file:
        data = json.load(json_file)
    return data[header]


def write_to_json(header, value):
    dict = {header: value}
    with open(JSON_PATH, 'r+') as json_file:
        data = json.load(json_file)
        data.update(dict)
        json_file.seek(0)
        json.dump(data, json_file)


class Main_Controller:
    def __init__(self):
        self.data_buffer = 2 * MAX_SATELLITE_DEVICES * [np.nan]
        self.buffer_lock = threading.Lock()
        self.data_buffer[0] = 0

        self.serial_com = serial.Serial(PORT, BAUD_RATE)
        self.serial_com.flush()
        self.serial_is_running = False

        # Temperature parameters
        self.int_error = 0
        self.past_error = 0

        # Lighting parameters
        self.lighting_is_running = False
        self.theta = np.pi / 180
        self.height = 0

        # Checks to see if a log file already exists
        # If it does it adds data to an existing one
        # If it does not it creates a new one
        if not path.exists(CSV_FILE_PATH):
            df = pd.DataFrame(columns=HEADERS)
            df.to_csv(CSV_FILE_PATH, mode='w', header=True, index=False)

        if not path.exists(JSON_PATH):
            reset_json()


    def main_loop(self):
        temperature_time = time.time()
        lighting_time = time.time()
        log_time = time.time()

        while True:
            # if the serial communication is interrupted the controller will attempt to regain
            # communication at the start of each loop
            if not self.serial_is_running:
                listening_thread = threading.Thread(target=self.listen_to_serial)
                listening_thread.start()

            if time.time() - log_time >= LOG_DATA_PERIOD:
                log_time = time.time()
                self.log_data()

            if time.time() - temperature_time >= TEMP_CONTROL_PERIOD:
                temperature_time = time.time()
                self.control_temperature()

            if time.time() - lighting_time >= LIGHT_CONTROL_PERIOD:
                lighting_time = time.time()
                if not self.lighting_is_running:
                    lighting_thread = threading.Thread(target=self.control_lighting)
                    lighting_thread.start()


    def listen_to_serial(self):
        try:
            self.serial_is_running = True
            while True:
                # when inWaiting is greater than 0, it means a new line was received via serial
                if self.serial_com.inWaiting() > 0:
                    serial_input = self.serial_com.readline().decode('utf-8').rstrip()
                    serial_data = interpret_serial_data(serial_input)

                    self.buffer_lock.acquire()
                    self.data_buffer = serial_data
                    self.buffer_lock.release()
        except:
            # What happens if the serial communication is broken?
            self.serial_com.close()
            self.serial_is_running = False


    def log_data(self):
        current_time = [datetime.now().strftime("%Y-%m-%d %H:%M:%S")]
        self.buffer_lock.acquire()
        data = self.data_buffer.copy()
        self.buffer_lock.release()
        df = pd.DataFrame([current_time + data])
        df.to_csv(CSV_FILE_PATH, mode='a', header=False, index=False)


    def control_temperature(self):
        self.buffer_lock.acquire()
        data = self.data_buffer.copy()
        self.buffer_lock.release()
        temperature = np.nanmean(data[0::2])
        if np.isnan(temperature):
            print('No Temperature Data Found')
            return

        write_to_json('current temperature', temperature)
        reference_temperature = read_json('reference temperature')

        error = reference_temperature - temperature
        self.int_error += error
        diff_error = error - self.past_error
        pid_output = P * error + I * self.int_error + D * diff_error
        if pid_output > 0:
            temperature_action = 1
        else:
            temperature_action = 0
        self.past_error = error

        """
        Send action to ESP32 device connected to relay over serial
        """


    def control_lighting(self):
        try:
            self.lighting_is_running = True
            counter = 0
            while counter < LIGHTING_TIME_OUT:
                if counter == 0:
                    self.buffer_lock.acquire()
                    data = self.data_buffer.copy()
                    self.buffer_lock.release()
                    lighting = np.nanmean(data[1::2])
                else:
                    lighting = 0
                    """
                    Get lighting data from wall powered devices
                    """

                if np.isnan(lighting):
                    print("No Lighting Data Found")
                    break
                else:
                    lighting /= MAX_LUX

                write_to_json('current lighting', lighting)
                reference_lighting = read_json('reference lighting') / MAX_LUX

                error = reference_lighting - lighting
                if abs(error) <= ERROR_THRESHOLD:
                    break

                dh = -HEIGHT_STEP_SIZE * error * np.cos(self.theta)
                dh = np.clip(self.height + dh, 0, 1) - self.height
                dtheta = TILT_STEP_SIZE * error * self.height * np.sin(self.theta)
                dtheta = np.clip(self.theta + dtheta, np.pi / 180, np.pi / 2) - self.theta

                action = [dh, dtheta]

                """
                Send action to ESP32 device controlling blinds over serial
                Wait for response that motor movement has been completed
                """
                self.height += dh
                self.theta += dtheta
                counter += 1

            self.lighting_is_running = False
        except:
            self.lighting_is_running = False


if __name__ == '__main__':
    # main_controller = Main_Controller()
    # main_controller.main_loop()
    message = '{   "header": {     "DEVICE_TYPE":0,     "POWER_SOURCE": 0,     "DEVICE_ID": "5A:9E:AD:F5:06:69"   },   "data": {     "temp": -273.15,     "light": -273.15   },   "children": {     "9C:9E:BD:F4:06:69": {       "header": {         "DEVICE_TYPE": 1,         "POWER_SOURCE": 0,         "DEVICE_ID": "9C:9E:BD:F4:06:69"       },       "data": {         "temp": -273.15,         "light": -273.15       }     },     "7C:9E:BD:F4:06:68": {       "header": {         "DEVICE_TYPE": 1,         "POWER_SOURCE": 0,         "DEVICE_ID": "7C:9E:BD:F4:06:68"       },       "data": {         "temp": -273.15,         "light": -273.15       }     }   } }'
    temp, light = interpret_serial_data(message)
    print(temp)
    print(light)