import os
import time
import random
import threading
import configparser
import serial
import serial.tools.list_ports
import numpy as np
import cv2
import pyautogui
import win32api
from mss import mss

# Settings Class
class Settings:
    def __init__(self):
        self.config = configparser.ConfigParser()
        script_dir = os.path.dirname(os.path.abspath(__file__))  # <-- burası önemli
        self.settings_path = os.path.join(script_dir, 'settings.ini')
        self.config.read(self.settings_path)

    def get(self, section, key):
        return self.config.get(section, key)

    def get_boolean(self, section, key):
        return self.config.getboolean(section, key)

    def get_float(self, section, key):
        return self.config.getfloat(section, key)

    def get_float_list(self, section, key):
        string_value = self.config.get(section, key)
        values_as_strings = string_value.strip('[]').split(',')
        return [float(value) for value in values_as_strings]

    def get_int(self, section, key):
        return self.config.getint(section, key)

    def save(self):
        with open(self.settings_path, 'w') as f:
            self.config.write(f)

    def set(self, section, key, value):
        if not self.config.has_section(section):
            self.config.add_section(section)
        self.config.set(section, key, str(value))
        self.save()

# Mouse Class
class Mouse:
    def __init__(self):
        self.settings = Settings()
        self.lock = threading.Lock()
        self.serial_port = serial.Serial()
        self.serial_port.baudrate = 115200
        self.serial_port.timeout = 0
        self.serial_port.port = self.find_serial_port()
        self.remainder_x = 0.0
        self.remainder_y = 0.0
        try:
            self.serial_port.open()
        except serial.SerialException:
            print("Colorbot is already open or Arduino is being used by another app.\nExiting in 10 seconds...")
            time.sleep(10)
            exit()

    def find_serial_port(self):
        com_port = self.settings.get('Settings', 'COM-Port')
        port = next((port for port in serial.tools.list_ports.comports() if com_port in port.description), None)
        if port is not None:
            return port.device
        else:
            print(f"Unable to detect your specified Arduino ({com_port}).\nPlease check its connection and the settings.ini file, then try again.\nExiting in 10 seconds...")
            time.sleep(10)
            exit()

    def move(self, x, y):
        x += self.remainder_x
        y += self.remainder_y
        move_x = int(x)
        move_y = int(y)
        self.remainder_x = x - move_x
        self.remainder_y = y - move_y

        if move_x != 0 or move_y != 0:
            with self.lock:
                self.serial_port.write(f'M{move_x},{move_y}\n'.encode())

    def click(self):
        with self.lock:
            self.serial_port.write('C\n'.encode())

# Capture Class
class Capture:
    def __init__(self, x, y, x_fov, y_fov):
        self.monitor = {
            "top": y,
            "left": x,
            "width": x_fov,
            "height": y_fov
        }

    def get_screen(self):
        with mss() as sct:
            screenshot = sct.grab(self.monitor)
            return np.array(screenshot)

# Colorbot Class
class Colorbot:
    def __init__(self, x, y, x_fov, y_fov):
        self.capturer = Capture(x, y, x_fov, y_fov)
        self.mouse = Mouse()
        self.settings = Settings()

        self.lower_color = np.array([150, 76,  123])
        self.upper_color = np.array([160, 197, 255])

        self.aim_enabled = self.settings.get_boolean('Aimbot', 'Enabled')
        self.aim_key = int(self.settings.get('Aimbot', 'toggleKey'), 16)
        self.x_speed = self.settings.get_float('Aimbot', 'xSpeed')
        self.y_speed = self.settings.get_float('Aimbot', 'ySpeed')
        self.x_fov = self.settings.get_int('Aimbot', 'xFov')
        self.y_fov = self.settings.get_int('Aimbot', 'yFov')
        self.target_offset = self.settings.get_float('Aimbot', 'targetOffset')

        self.trigger_enabled = self.settings.get_boolean('Triggerbot', 'Enabled')
        self.trigger_key = int(self.settings.get('Triggerbot', 'toggleKey'), 16)
        self.min_delay = self.settings.get_int('Triggerbot', 'minDelay')
        self.max_delay = self.settings.get_int('Triggerbot', 'maxDelay')
        self.x_range = self.settings.get_int('Triggerbot', 'xRange')
        self.y_range = self.settings.get_int('Triggerbot', 'yRange')

        self.kernel = np.ones((3, 3), np.uint8)
        self.screen_center = (self.x_fov // 2, self.y_fov // 2)

    def listen_aimbot(self):
        while True:
            if win32api.GetAsyncKeyState(self.aim_key) < 0:
                self.process("move")
            time.sleep(0.01)

    def listen_triggerbot(self):
        while True:
            if win32api.GetAsyncKeyState(self.trigger_key) < 0:
                self.process("click")
            time.sleep(0.01)

    def listen(self):
        if self.aim_enabled:
            threading.Thread(target=self.listen_aimbot).start()
        if self.trigger_enabled:
            threading.Thread(target=self.listen_triggerbot).start()

    def process(self, action):
        hsv = cv2.cvtColor(self.capturer.get_screen(), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        dilated = cv2.dilate(mask, self.kernel, iterations=5)
        thresh = cv2.threshold(dilated, 60, 255, cv2.THRESH_BINARY)[1]
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            min_distance = float('inf')
            closest_center = None

            for contour in contours:
                moments = cv2.moments(contour)
                if moments['m00'] != 0:
                    center = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
                    distance = np.sqrt((center[0] - self.screen_center[0]) ** 2 + (center[1] - self.screen_center[1]) ** 2)
                    if distance < min_distance:
                        min_distance = distance
                        closest_center = center

            if closest_center is not None:
                cX, cY = closest_center
                cY -= int(self.target_offset)

                if action == "move":
                    x_diff = cX - self.screen_center[0]
                    y_diff = cY - self.screen_center[1]
                    self.mouse.move(self.x_speed * x_diff, self.y_speed * y_diff)

                elif action == "click":
                    if (abs(cX - self.screen_center[0]) <= self.x_range and
                        abs(cY - self.screen_center[1]) <= self.y_range):
                        time.sleep(random.uniform(self.min_delay / 1000.0, self.max_delay / 1000.0))
                        self.mouse.click()

# Main Class
class Main:
    def __init__(self):
        self.settings = Settings()
        self.monitor = pyautogui.size()
        self.center_x, self.center_y = self.monitor.width // 2, self.monitor.height // 2
        self.x_fov = self.settings.get_int('Aimbot', 'xFov')
        self.y_fov = self.settings.get_int('Aimbot', 'yFov')
        self.colorbot = Colorbot(
            self.center_x - self.x_fov // 2, 
            self.center_y - self.y_fov // 2, 
            self.x_fov, 
            self.y_fov
        )

    def run(self):
        os.system('cls' if os.name == 'nt' else 'clear')
        os.system('title zozo client!')
        print('Enemy Outline Color: Purple')
        self.colorbot.listen()

if __name__ == '__main__':
    Main().run()
