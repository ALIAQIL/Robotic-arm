import sys
import os
import numpy as np
import cv2
import traceback

# Set mock mode BEFORE importing motor_control
os.environ['GPIOZERO_PIN_FACTORY'] = 'mock'

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from vision import ObjectDetector
from motor_control import RoboticArm

def test_vision():
    print("Testing Vision Module...")
    detector = ObjectDetector()
    
    # Create a black image
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Draw a red circle
    cv2.circle(img, (320, 240), 50, (0, 0, 255), -1)
    
    result = detector.detect_object(img)
    
    if result:
        x, y, area, label = result
        print(f"SUCCESS: Detected object {label} at ({x}, {y}) with area {area}")
        if abs(x - 320) < 5 and abs(y - 240) < 5 and label == "tomato":
            return True
    
    print("FAILURE: Could not detect object correctly")
    return False

def test_motor():
    print("\nTesting Motor Control Module (Mock Mode)...")
    # Mock mode already set globally
    
    try:
        arm = RoboticArm()
        arm.move_base(45)
        arm.move_shoulder(-30)
        arm.close_gripper()
        arm.reset()
        print("SUCCESS: Motor control sequences executed without error")
        return True
    except Exception as e:
        print(f"FAILURE: Motor control error: {repr(e)}")
        traceback.print_exc()
        return False

if __name__ == "__main__":
    v_success = test_vision()
    m_success = test_motor()
    
    if v_success and m_success:
        print("\nAll tests passed!")
        sys.exit(0)
    else:
        print("\nSome tests failed.")
        sys.exit(1)
