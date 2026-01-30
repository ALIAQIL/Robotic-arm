import cv2
import time
from motor_control import RoboticArm
from vision import ObjectDetector
from config import get_box_for_label, get_color_for_label, get_base_angle_for_box

def main():
    print("Initializing Robotic Arm System...")
    
    # Initialize modules
    arm = RoboticArm()
    detector = ObjectDetector()
    
    # Initialize Camera
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

                # Draw bounding circle for visualization (generalized for any label)
                color = get_color_for_label(label)
                cv2.circle(frame, (cx, cy), 10, color, -1)
                cv2.putText(frame, label, (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Simple logic to align arm with object
                # Center of frame is roughly width/2
                frame_center_x = frame.shape[1] // 2
                
                # Calculate error
                error_x = cx - frame_center_x
                
                # Threshold to consider "centered"
                if abs(error_x) > 20:
                    # Move base to align
                    # This is a simplified proportional control
                    # In reality, you'd map pixel coordinates to real-world coordinates
                    current_angle = arm.base.angle
                    if error_x > 0:
                        arm.move_base(current_angle - 5) # Move left
                    else:
                        arm.move_base(current_angle + 5) # Move right
                else:
                    print(f"{label.capitalize()} centered! Initiating pick sequence...")
                    # Pick sequence for any object type
                    # 1. Extend arm
                    arm.move_shoulder(45)
                    arm.move_elbow(-45)
                    time.sleep(1)
                    
                    # 2. Grab
                    arm.close_gripper()
                    time.sleep(1)
                    
                    # 3. Lift
                    arm.move_shoulder(0)
                    arm.move_elbow(0)
                    time.sleep(1)
                    
                    # 4. Move to Box (generalized: any label → box via config)
                    box_id = get_box_for_label(label)
                    angle = get_base_angle_for_box(box_id)
                    print(f"Sorting {label} to Box {box_id}")
                    arm.move_base(angle)
                    
                    time.sleep(1)
                    
                    # 5. Drop
                    arm.open_gripper()
                    time.sleep(1)
                    
                    # 6. Return Home
                    arm.reset()
                    print("Sequence complete. Waiting for next object...")
                    time.sleep(2)

            # Display the resulting frame
            cv2.imshow('Robotic Arm Vision', frame)

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
