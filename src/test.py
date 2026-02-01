import cv2
import time
# from motor_control import RoboticArm  # Not used in test mode
from vision import ObjectDetector
from config import get_box_for_label, get_color_for_label, get_base_angle_for_box

# Dummy Arm class for testing
class DummyArm:
    class Joint:
        def __init__(self):
            self.angle = 0
    def __init__(self):
        self.base = self.Joint()
    def move_base(self, angle):
        print(f"[SIM] Base would move to angle: {angle}")
    def move_shoulder(self, angle):
        print(f"[SIM] Shoulder would move to angle: {angle}")
    def move_elbow(self, angle):
        print(f"[SIM] Elbow would move to angle: {angle}")
    def close_gripper(self):
        print("[SIM] Gripper would close")
    def open_gripper(self):
        print("[SIM] Gripper would open")
    def reset(self):
        print("[SIM] Arm would reset")
    def cleanup(self):
        print("[SIM] Cleanup called")

def main():
    print("Initializing Robotic Arm System (Test Mode)...")
    
    # Use Dummy Arm
    arm = DummyArm()
    detector = ObjectDetector()
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("System Ready. Press 'q' to quit.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Detect object
            result = detector.detect_object(frame)

            if result:
                cx, cy, area, label = result
                print(f"Object detected: {label} at x={cx}, y={cy}, area={area}")

                # Draw bounding circle
                color = get_color_for_label(label)
                cv2.circle(frame, (cx, cy), 10, color, -1)
                cv2.putText(frame, label, (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Simulate alignment
                frame_center_x = frame.shape[1] // 2
                error_x = cx - frame_center_x
                
                if abs(error_x) > 20:
                    if error_x > 0:
                        arm.move_base(arm.base.angle - 5)
                    else:
                        arm.move_base(arm.base.angle + 5)
                else:
                    print(f"{label.capitalize()} centered! Simulating pick sequence...")
                    arm.move_shoulder(45)
                    arm.move_elbow(-45)
                    arm.close_gripper()
                    arm.move_shoulder(0)
                    arm.move_elbow(0)
                    box_id = get_box_for_label(label)
                    angle = get_base_angle_for_box(box_id)
                    print(f"Simulating sorting {label} to Box {box_id}")
                    arm.move_base(angle)
                    arm.open_gripper()
                    arm.reset()
                    print("Sequence complete. Waiting for next object...\n")
                    time.sleep(1)

            cv2.imshow('Robotic Arm Vision (Test Mode)', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        arm.cleanup()

if __name__ == "__main__":
    main()
import cv2
import time
# from motor_control import RoboticArm  # Not used in test mode
from vision import ObjectDetector
from config import get_box_for_label, get_color_for_label, get_base_angle_for_box

# Dummy Arm class for testing
class DummyArm:
    class Joint:
        def __init__(self):
            self.angle = 0
    def __init__(self):
        self.base = self.Joint()
    def move_base(self, angle):
        print(f"[SIM] Base would move to angle: {angle}")
    def move_shoulder(self, angle):
        print(f"[SIM] Shoulder would move to angle: {angle}")
    def move_elbow(self, angle):
        print(f"[SIM] Elbow would move to angle: {angle}")
    def close_gripper(self):
        print("[SIM] Gripper would close")
    def open_gripper(self):
        print("[SIM] Gripper would open")
    def reset(self):
        print("[SIM] Arm would reset")
    def cleanup(self):
        print("[SIM] Cleanup called")

def main():
    print("Initializing Robotic Arm System (Test Mode)...")
    
    # Use Dummy Arm
    arm = DummyArm()
    detector = ObjectDetector()
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("System Ready. Press 'q' to quit.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Detect object
            result = detector.detect_object(frame)

            if result:
                cx, cy, area, label = result
                print(f"Object detected: {label} at x={cx}, y={cy}, area={area}")

                # Draw bounding circle
                color = get_color_for_label(label)
                cv2.circle(frame, (cx, cy), 10, color, -1)
                cv2.putText(frame, label, (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Simulate alignment
                frame_center_x = frame.shape[1] // 2
                error_x = cx - frame_center_x
                
                if abs(error_x) > 20:
                    if error_x > 0:
                        arm.move_base(arm.base.angle - 5)
                    else:
                        arm.move_base(arm.base.angle + 5)
                else:
                    print(f"{label.capitalize()} centered! Simulating pick sequence...")
                    arm.move_shoulder(45)
                    arm.move_elbow(-45)
                    arm.close_gripper()
                    arm.move_shoulder(0)
                    arm.move_elbow(0)
                    box_id = get_box_for_label(label)
                    angle = get_base_angle_for_box(box_id)
                    print(f"Simulating sorting {label} to Box {box_id}")
                    arm.move_base(angle)
                    arm.open_gripper()
                    arm.reset()
                    print("Sequence complete. Waiting for next object...\n")
                    time.sleep(1)

            cv2.imshow('Robotic Arm Vision (Test Mode)', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        arm.cleanup()

if __name__ == "__main__":
    main()
