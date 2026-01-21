import cv2
import numpy as np
import time
import random
import math

class KinematicArm:
    def __init__(self):
        # Link lengths (arbitrary units, e.g., cm or pixels)
        self.L1 = 50  # Base height
        self.L2 = 100 # Shoulder to Elbow
        self.L3 = 100 # Elbow to Gripper
        
        # Angles in degrees
        self.theta_base = 0     # -90 to 90
        self.theta_shoulder = 0 # -90 to 90
        self.theta_elbow = 0    # -90 to 90
        self.gripper_open = True
        
        # Target angles
        self.target_base = 0
        self.target_shoulder = 0
        self.target_elbow = 0
        
        self.speed = 2.0 # Degrees per frame
        
        self.holding_object = None
        self.state = "IDLE"

    def update(self):
        # Smoothly move towards target angles
        diff_base = self.target_base - self.theta_base
        diff_shoulder = self.target_shoulder - self.theta_shoulder
        diff_elbow = self.target_elbow - self.theta_elbow
        
        # Move Base
        if abs(diff_base) > self.speed:
            self.theta_base += math.copysign(self.speed, diff_base)
        else:
            self.theta_base = self.target_base
            
        # Move Shoulder
        if abs(diff_shoulder) > self.speed:
            self.theta_shoulder += math.copysign(self.speed, diff_shoulder)
        else:
            self.theta_shoulder = self.target_shoulder
            
        # Move Elbow
        if abs(diff_elbow) > self.speed:
            self.theta_elbow += math.copysign(self.speed, diff_elbow)
        else:
            self.theta_elbow = self.target_elbow
            
        # Check if reached
        return (abs(diff_base) <= self.speed and 
                abs(diff_shoulder) <= self.speed and 
                abs(diff_elbow) <= self.speed)

    def forward_kinematics(self):
        # Convert to radians
        rad_base = math.radians(self.theta_base)
        rad_shoulder = math.radians(self.theta_shoulder)
        rad_elbow = math.radians(self.theta_elbow)
        
        # Calculate positions in 3D space
        # Coordinate system: Base is at (0, 0, 0)
        # Z is up, X is forward, Y is left
        
        # Shoulder Joint Position (Fixed relative to base rotation)
        # It rotates with the base, but is offset by L1 in Z
        shoulder_pos = np.array([0, 0, self.L1])
        
        # Elbow Position
        # Projected length on XY plane
        r_elbow = self.L2 * math.cos(rad_shoulder)
        z_elbow = self.L1 + self.L2 * math.sin(rad_shoulder)
        x_elbow = r_elbow * math.cos(rad_base)
        y_elbow = r_elbow * math.sin(rad_base)
        elbow_pos = np.array([x_elbow, y_elbow, z_elbow])
        
        # Gripper Position
        # Angle of forearm relative to horizon is (shoulder + elbow)
        total_angle = rad_shoulder + rad_elbow
        r_gripper = r_elbow + self.L3 * math.cos(total_angle)
        z_gripper = z_elbow + self.L3 * math.sin(total_angle)
        x_gripper = r_gripper * math.cos(rad_base)
        y_gripper = r_gripper * math.sin(rad_base)
        gripper_pos = np.array([x_gripper, y_gripper, z_gripper])
        
        return shoulder_pos, elbow_pos, gripper_pos

def main():
    width, height = 800, 600
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    
    arm = KinematicArm()
    objects = []
    
    # Box locations (in 3D space approx)
    # Box A: Left (Positive Y), Box B: Right (Negative Y)
    box_a_pos = (0, 150, 0) 
    box_b_pos = (0, -150, 0)
    
    last_spawn_time = time.time()
    
    print("Starting Realistic Kinematic Simulation...")
    print("Red = Tomato (Box A), Yellow = Potato (Box B)")
    print("Press 'q' to quit.")

    while True:
        canvas[:] = (30, 30, 30) # Dark gray background
        
        # --- Logic Update ---
        
        # Spawn objects
        if not objects and time.time() - last_spawn_time > 2:
            obj_type = random.choice(["tomato", "potato"])
            color = (0, 0, 255) if obj_type == "tomato" else (0, 255, 255)
            # Spawn in front (Positive X), random Y
            obj_x = random.randint(100, 200)
            obj_y = random.randint(-50, 50)
            objects.append({"type": obj_type, "x": obj_x, "y": obj_y, "z": 0, "color": color})
            last_spawn_time = time.time()
            print(f"Spawned {obj_type} at ({obj_x}, {obj_y})")

        # Arm State Machine
        reached = arm.update()
        shoulder_pos, elbow_pos, gripper_pos = arm.forward_kinematics()
        gx, gy, gz = gripper_pos
        
        if arm.state == "IDLE":
            arm.target_base = 0
            arm.target_shoulder = 45
            arm.target_elbow = -90 # Tucked in
            if objects:
                arm.state = "ALIGNING"
                
        elif arm.state == "ALIGNING":
            if objects:
                target = objects[0]
                # Calculate Base Angle
                angle = math.degrees(math.atan2(target["y"], target["x"]))
                arm.target_base = angle
                # Prepare to extend
                arm.target_shoulder = 0
                arm.target_elbow = 0
                if reached:
                    arm.state = "REACHING"
                    
        elif arm.state == "REACHING":
             if objects:
                target = objects[0]
                # Simple IK approximation for reaching (just extending shoulder/elbow)
                # For this demo, we just lower the arm to "ground" level
                dist = math.sqrt(target["x"]**2 + target["y"]**2)
                # Heuristic: Lower shoulder to reach out
                arm.target_shoulder = -20
                arm.target_elbow = -20
                if reached:
                    arm.state = "PICKING"
                    arm.holding_object = objects.pop(0)
                    time.sleep(0.5)
                    
        elif arm.state == "PICKING":
            # Lift up
            arm.target_shoulder = 30
            arm.target_elbow = -30
            if reached:
                arm.state = "MOVING_TO_BOX"
                if arm.holding_object["type"] == "tomato":
                    # Move to Box A (Left -> +90 deg base)
                    arm.target_base = 60 
                else:
                    # Move to Box B (Right -> -90 deg base)
                    arm.target_base = -60
                    
        elif arm.state == "MOVING_TO_BOX":
            if reached:
                arm.state = "DROPPING"
                arm.holding_object = None
                time.sleep(0.5)
                
        elif arm.state == "DROPPING":
            arm.state = "IDLE"

        # --- Visualization ---
        
        # 1. Top-Down View (Left side of screen)
        # Center is (200, 300)
        cx_top, cy_top = 200, 300
        scale = 1.0
        
        # Draw Ground Grid
        cv2.line(canvas, (cx_top, 0), (cx_top, 600), (50, 50, 50), 1)
        cv2.line(canvas, (0, cy_top), (400, cy_top), (50, 50, 50), 1)
        
        # Draw Boxes (Top View)
        # Box A (Left/Top in image coords)
        cv2.rectangle(canvas, (cx_top - 50, cy_top - 200), (cx_top + 50, cy_top - 100), (0, 0, 100), 2)
        cv2.putText(canvas, "Box A", (cx_top - 20, cy_top - 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Box B (Right/Bottom in image coords)
        cv2.rectangle(canvas, (cx_top - 50, cy_top + 100), (cx_top + 50, cy_top + 200), (0, 100, 100), 2)
        cv2.putText(canvas, "Box B", (cx_top - 20, cy_top + 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Draw Objects (Top View)
        # Map World (X, Y) to Image (X, Y). World X is Up (-Y image), World Y is Left (-X image)
        # Actually let's align: World X -> Image Y (Up), World Y -> Image X (Right)
        # But standard is X right, Y down.
        # Let's map: World X (Forward) -> Image Y (Up, so -Y)
        # World Y (Left) -> Image X (Left, so -X)
        # Center (0,0) -> (cx_top, cy_top)
        
        def world_to_top_view(x, y):
            # World X is forward (Up on screen)
            # World Y is Left (Left on screen)
            screen_x = int(cx_top - y * scale)
            screen_y = int(cy_top - x * scale)
            return screen_x, screen_y

        for obj in objects:
            ox, oy = world_to_top_view(obj["x"], obj["y"])
            cv2.circle(canvas, (ox, oy), 10, obj["color"], -1)
            
        # Draw Arm (Top View)
        # Base
        bx, by = world_to_top_view(0, 0)
        cv2.circle(canvas, (bx, by), 15, (200, 200, 200), -1)
        
        # Elbow Projection
        ex, ey = world_to_top_view(elbow_pos[0], elbow_pos[1])
        cv2.line(canvas, (bx, by), (ex, ey), (150, 150, 150), 3)
        
        # Gripper Projection
        g_sx, g_sy = world_to_top_view(gripper_pos[0], gripper_pos[1])
        cv2.line(canvas, (ex, ey), (g_sx, g_sy), (200, 200, 200), 3)
        cv2.circle(canvas, (g_sx, g_sy), 8, (0, 255, 0), -1)
        
        if arm.holding_object:
             color = arm.holding_object["color"]
             cv2.circle(canvas, (g_sx, g_sy), 6, color, -1)

        cv2.putText(canvas, "Top-Down View", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        # 2. Side View (Right side of screen)
        # Center is (600, 400)
        cx_side, cy_side = 600, 400
        
        # Draw Ground
        cv2.line(canvas, (400, cy_side), (800, cy_side), (100, 100, 100), 2)
        
        # Draw Arm (Side View - Projected on vertical plane defined by base angle)
        # We visualize R (distance from Z axis) vs Z
        def world_to_side_view(r, z):
            screen_x = int(cx_side + r * scale)
            screen_y = int(cy_side - z * scale)
            return screen_x, screen_y
            
        # Base
        sbx, sby = world_to_side_view(0, 0)
        ssx, ssy = world_to_side_view(0, arm.L1) # Shoulder joint
        cv2.line(canvas, (sbx, sby), (ssx, ssy), (100, 100, 100), 4)
        
        # Elbow
        r_elbow = math.sqrt(elbow_pos[0]**2 + elbow_pos[1]**2)
        sex, sey = world_to_side_view(r_elbow, elbow_pos[2])
        cv2.line(canvas, (ssx, ssy), (sex, sey), (150, 150, 150), 3)
        cv2.circle(canvas, (ssx, ssy), 5, (0, 0, 255), -1) # Joint
        
        # Gripper
        r_gripper = math.sqrt(gripper_pos[0]**2 + gripper_pos[1]**2)
        sgx, sgy = world_to_side_view(r_gripper, gripper_pos[2])
        cv2.line(canvas, (sex, sey), (sgx, sgy), (200, 200, 200), 3)
        cv2.circle(canvas, (sex, sey), 5, (0, 0, 255), -1) # Joint
        cv2.circle(canvas, (sgx, sgy), 8, (0, 255, 0), -1) # Gripper
        
        cv2.putText(canvas, "Side View (Extension)", (410, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        
        # Info
        cv2.putText(canvas, f"State: {arm.state}", (10, 550), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(canvas, f"Angles: B={arm.theta_base:.1f} S={arm.theta_shoulder:.1f} E={arm.theta_elbow:.1f}", (10, 580), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("Robotic Arm Kinematic Simulation", canvas)
        
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
