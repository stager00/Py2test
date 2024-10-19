from picrawler import Picrawler
from time import sleep
import readchar
import subprocess
import math
import cv2
import RPi.GPIO as GPIO
import time

class PiCrawlerHoming(Picrawler):
    def __init__(self):
        super().__init__()
        self.position = [0, 0]  # Starting at origin (x, y)
        self.angle = 0  # Facing forward (0 degrees)
        self.reference_image = None
        self.setup_ultrasonic_sensor()

    def setup_ultrasonic_sensor(self):
        self.TRIG = 23
        self.ECHO = 24
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

    def get_distance(self):
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)
        start_time = time.time()
        stop_time = time.time()

        while GPIO.input(self.ECHO) == 0:
            start_time = time.time()

        while GPIO.input(self.ECHO) == 1:
            stop_time = time.time()

        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300) / 2
        return distance

    def avoid_obstacle(self):
        distance = self.get_distance()
        if distance < 20:  # Threshold distance in cm
            self.do_action('turn left', 1)
            self.do_action('forward', 1)

    def capture_reference_image(self):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if ret:
            self.reference_image = frame
        cap.release()

    def compare_images(self, current_image):
        diff = cv2.absdiff(self.reference_image, current_image)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
        return cv2.countNonZero(thresh)

    def update_position(self, command, distance):
        if command == 'forward':
            self.position[0] += distance * math.cos(math.radians(self.angle))
            self.position[1] += distance * math.sin(math.radians(self.angle))
        elif command == 'backward':
            self.position[0] -= distance * math.cos(math.radians(self.angle))
            self.position[1] -= distance * math.sin(math.radians(self.angle))
        elif command == 'turn left':
            self.angle = (self.angle + 90) % 360
        elif command == 'turn right':
            self.angle = (self.angle - 90) % 360

    def return_to_start(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if ret:
                difference = self.compare_images(frame)
                if difference < threshold:  # Define a suitable threshold
                    break
                # Adjust the robot's path based on the difference
                self.avoid_obstacle()
                self.do_action('forward', 1)
        cap.release()
        # Calculate the angle and distance to return to the start
        return_angle = math.degrees(math.atan2(-self.position[1], -self.position[0]))
        return_distance = math.sqrt(self.position[0]**2 + self.position[1]**2)
        # Adjust the robot's angle
        self.do_action('turn right', int((self.angle - return_angle) / 90))
        # Move the robot back to the start
        self.do_action('forward', int(return_distance / step_length))

crawler = PiCrawlerHoming()
speed = 90

manual = '''
Press keys on keyboard to control PiCrawler!
    W: Forward
    A: Turn left
    S: Backward
    D: Turn right
    O: Toggle between picrawler.py and picrawler2.py
    R: Return to home

    Ctrl^C: Quit
'''

def show_info():
    print("\033[H\033[J", end='')  # clear terminal window
    print(manual)

def switch_mode():
    global current_mode
    if current_mode == 'default':
        print("Switching to picrawler2.py")
        subprocess.run(['python3', 'picrawler2.py'])
        current_mode = 'high'
    else:
        print("Switching to picrawler.py")
        subprocess.run(['python3', 'picrawler.py'])
        current_mode = 'default'

def main(): 
    show_info()   
    while True:
        key = readchar.readkey()
        key = key.lower()
        if key in ('wsad'):
            if 'w' == key:
                crawler.do_action('forward', 1, speed)
                crawler.update_position('forward', 1)
            elif 's' == key:
                crawler.do_action('backward', 1, speed)
                crawler.update_position('backward', 1)
            elif 'a' == key:
                crawler.do_action('turn left', 1, speed)
                crawler.update_position('turn left', 0)
            elif 'd' == key:
                crawler.do_action('turn right', 1, speed)
                crawler.update_position('turn right', 0)
            sleep(0.05)
            show_info()
        elif key == 'o':
            switch_mode()
            break
        elif key == 'r':
            print("Returning to home")
            crawler.return_to_start()
            break
        elif key == readchar.key.CTRL_C:
            print("\n Quit")
            break
        
        sleep(0.02)

if __name__ == "__main__":
    crawler.capture_reference_image()
    main()
