from gpiozero import AngularServo
from gpiozero.pins.mock import MockFactory, MockPWMPin
from gpiozero import Device
import time
import os

# Check if running in simulation/mock mode
if os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
    Device.pin_factory = MockFactory(pin_class=MockPWMPin)

class RoboticArm:
    def __init__(self, base_pin=17, shoulder_pin=27, elbow_pin=22, gripper_pin=23):
        # Adjust min_angle and max_angle based on your specific servos
        self.base = AngularServo(base_pin, min_angle=-90, max_angle=90)
        self.shoulder = AngularServo(shoulder_pin, min_angle=-90, max_angle=90)
        self.elbow = AngularServo(elbow_pin, min_angle=-90, max_angle=90)
        self.gripper = AngularServo(gripper_pin, min_angle=-90, max_angle=90)
        
        self.reset()

    def reset(self):
        """Move arm to home position"""
        print("Resetting arm to home position...")
        self.base.angle = 0
        self.shoulder.angle = 0
        self.elbow.angle = 0
        self.open_gripper()
        time.sleep(1)

    def move_base(self, angle):
        print(f"Moving base to {angle}")
        self.base.angle = max(min(angle, 90), -90)
        time.sleep(0.5)

    def move_shoulder(self, angle):
        print(f"Moving shoulder to {angle}")
        self.shoulder.angle = max(min(angle, 90), -90)
        time.sleep(0.5)

    def move_elbow(self, angle):
        print(f"Moving elbow to {angle}")
        self.elbow.angle = max(min(angle, 90), -90)
        time.sleep(0.5)

    def open_gripper(self):
        print("Opening gripper")
        self.gripper.angle = -45 # Adjust as needed
        time.sleep(0.5)

    def close_gripper(self):
        print("Closing gripper")
        self.gripper.angle = 45 # Adjust as needed
        time.sleep(0.5)

    def cleanup(self):
        self.base.close()
        self.shoulder.close()
        self.elbow.close()
        self.gripper.close()
