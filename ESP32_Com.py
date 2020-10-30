import serial
from numpy import sin, cos, clip, pi
import time


# Temperature Control parameters
TEMP_P = 1.027512
TEMP_I = 9.1e-5
TEMP_D = 0.0011


# Lighting Control Parameters
STEP_SIZE = 10


class Controller:
    def __init__(self):
        # Initializing serial connection
        self.serial_comm = serial.Serial()
        self.serial_comm.baudrate = 9600
        self.serial_comm.port = '/dev/tty####'
        self.serial_comm.timeout = 10

        # Toggle on and off lighting and temperature control
        self.light_toggle = True
        self.temp_toggle = True

        # How frequently the lighting and temperature is measured [s]
        self.light_period = 60
        self.temp_period = 60

        # Lighting Control Parameters
        self.reference_lighting = 0.7  # normalized units
        self.measured_lighting = 0.7
        self.window_lighting = 0.8
        self.theta = pi / 2
        self.h = 0
        self.light_err_thresh = 0.01
        self.light_timeout = 5

        # Temperature control parameters
        self.reference_temperature = 20  # degrees Celsius
        self.measured_temperature = 20
        self.temp_integral_error = 0
        self.temp_past_error = 0

    def main_loop(self):
        try:
            self.serial_comm.open()
        except serial.SerialException:
            print("Could not connect")

        light_time = time.time()
        temp_time = time.time()
        while self.serial_comm.isOpen():
            self.receive_data()
            if self.light_toggle and time.time() - light_time >= self.light_period:
                light_time = time.time()
                self.control_lighting()
            if self.temp_toggle and time.time() - temp_time >= self.temp_period:
                temp_time = time.time()
                self.control_temp()
        print('Communication ended')

    def receive_data(self):
        # Not sure yet how input data will be formatted
        data = self.serial_comm.read()
        self.measured_temperature = data
        self.measured_lighting = data
        self.window_lighting = data

    def transmit_data(self, data):
        # Not sure yet how output data will be formatted
        self.serial_comm.write(data)

    def control_temp(self):
        error = self.reference_temperature - self.measured_temperature
        self.temp_integral_error += self.temp_period * error
        diff_error = (error - self.temp_past_error) / self.temp_period

        pid_output = TEMP_P * error + TEMP_I * self.temp_integral_error + TEMP_D * diff_error
        if pid_output > 0.5:
            action = 1
        else:
            action = 0
        self.transmit_data(action)

        self.temp_past_error = error

    def control_lighting(self):
        counter = 0
        while counter < self.light_timeout:
            self.receive_data()
            error = self.reference_lighting - self.measured_lighting
            if abs(error) < self.light_err_thresh:
                break

            constant = STEP_SIZE * error * self.window_lighting
            dh = -constant * cos(self.theta)
            dtheta = constant * self.h * sin(self.theta)
            new_h = clip(self.h + dh, 0, 1)
            new_theta = clip(self.theta + dtheta, pi / 180, pi / 2)
            action = [new_h - self.h, new_theta - self.theta]
            self.transmit_data(action)

            self.h = new_h
            self.theta = new_theta
            counter += 1
            time.sleep(0.1)


if __name__ == "__main__":
    controller = Controller()
    controller.main_loop()
