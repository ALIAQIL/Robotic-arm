"""
3D Realistic Robotic Arm Simulation
====================================
A 4-DOF robotic arm simulation with:
- Source box containing mixed objects
- Destination boxes for sorted objects
- 3D rendered arm with cylinders
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import math
import random

# ============================================================================
# HELPER FUNCTIONS FOR 3D SHAPES
# ============================================================================

def create_cylinder(start, end, radius, resolution=12):
    """Create vertices for a 3D cylinder between two points."""
    # Direction vector
    v = np.array(end) - np.array(start)
    length = np.linalg.norm(v)
    if length == 0:
        return [], []
    v = v / length
    
    # Find perpendicular vectors
    if abs(v[0]) < 0.9:
        not_v = np.array([1, 0, 0])
    else:
        not_v = np.array([0, 1, 0])
    
    n1 = np.cross(v, not_v)
    n1 = n1 / np.linalg.norm(n1)
    n2 = np.cross(v, n1)
    
    # Generate circle points
    theta = np.linspace(0, 2 * np.pi, resolution, endpoint=False)
    
    # Bottom and top circles
    bottom = []
    top = []
    for t in theta:
        point = np.array(start) + radius * (np.cos(t) * n1 + np.sin(t) * n2)
        bottom.append(point)
        point = np.array(end) + radius * (np.cos(t) * n1 + np.sin(t) * n2)
        top.append(point)
    
    # Create faces
    faces = []
    for i in range(resolution):
        next_i = (i + 1) % resolution
        # Side face
        face = [bottom[i], bottom[next_i], top[next_i], top[i]]
        faces.append(face)
    
    return faces

def create_box_wireframe(center, size):
    """Create vertices for a 3D box wireframe."""
    cx, cy, cz = center
    sx, sy, sz = size
    
    # 8 corners
    corners = [
        [cx - sx/2, cy - sy/2, cz],
        [cx + sx/2, cy - sy/2, cz],
        [cx + sx/2, cy + sy/2, cz],
        [cx - sx/2, cy + sy/2, cz],
        [cx - sx/2, cy - sy/2, cz + sz],
        [cx + sx/2, cy - sy/2, cz + sz],
        [cx + sx/2, cy + sy/2, cz + sz],
        [cx - sx/2, cy + sy/2, cz + sz],
    ]
    
    # Edges
    edges = [
        [corners[0], corners[1]], [corners[1], corners[2]], 
        [corners[2], corners[3]], [corners[3], corners[0]],
        [corners[4], corners[5]], [corners[5], corners[6]], 
        [corners[6], corners[7]], [corners[7], corners[4]],
        [corners[0], corners[4]], [corners[1], corners[5]], 
        [corners[2], corners[6]], [corners[3], corners[7]],
    ]
    
    return edges

# ============================================================================
# KINEMATIC ARM CLASS
# ============================================================================

class RoboticArm3D:
    def __init__(self):
        # Link lengths (cm)
        self.L0 = 5   # Base height
        self.L1 = 15  # Base to Shoulder
        self.L2 = 20  # Shoulder to Elbow
        self.L3 = 18  # Elbow to Wrist
        self.L4 = 8   # Wrist to Gripper
        
        # Joint angles (degrees)
        self.theta_base = 0       # Rotation around Z axis
        self.theta_shoulder = 45  # Shoulder angle
        self.theta_elbow = -30    # Elbow angle
        self.theta_wrist = 0      # Wrist angle (gripper orientation)
        
        # Target angles
        self.target_base = 0
        self.target_shoulder = 45
        self.target_elbow = -30
        self.target_wrist = 0
        
        self.speed = 3.0  # Degrees per frame
        
        self.gripper_open = True
        self.holding_object = None
        self.state = "IDLE"
        
    def move_towards_target(self, current, target):
        """Smoothly move an angle towards target."""
        diff = target - current
        if abs(diff) > self.speed:
            return current + math.copysign(self.speed, diff)
        return target
    
    def update(self):
        """Update all joint angles towards targets."""
        self.theta_base = self.move_towards_target(self.theta_base, self.target_base)
        self.theta_shoulder = self.move_towards_target(self.theta_shoulder, self.target_shoulder)
        self.theta_elbow = self.move_towards_target(self.theta_elbow, self.target_elbow)
        self.theta_wrist = self.move_towards_target(self.theta_wrist, self.target_wrist)
        
        # Check if all reached
        return (abs(self.theta_base - self.target_base) <= self.speed and
                abs(self.theta_shoulder - self.target_shoulder) <= self.speed and
                abs(self.theta_elbow - self.target_elbow) <= self.speed)
    
    def forward_kinematics(self):
        """Calculate all joint positions using forward kinematics."""
        rad_base = math.radians(self.theta_base)
        rad_shoulder = math.radians(self.theta_shoulder)
        rad_elbow = math.radians(self.theta_elbow)
        rad_wrist = math.radians(self.theta_wrist)
        
        # Base position (origin)
        p0 = np.array([0, 0, 0])
        
        # Top of base
        p1 = np.array([0, 0, self.L0 + self.L1])
        
        # Shoulder joint (rotates with base)
        shoulder_height = self.L0 + self.L1
        
        # Elbow position
        r_elbow = self.L2 * math.cos(rad_shoulder)
        z_elbow = shoulder_height + self.L2 * math.sin(rad_shoulder)
        x_elbow = r_elbow * math.cos(rad_base)
        y_elbow = r_elbow * math.sin(rad_base)
        p2 = np.array([x_elbow, y_elbow, z_elbow])
        
        # Wrist position
        total_angle_1 = rad_shoulder + rad_elbow
        r_wrist = r_elbow + self.L3 * math.cos(total_angle_1)
        z_wrist = z_elbow + self.L3 * math.sin(total_angle_1)
        x_wrist = r_wrist * math.cos(rad_base)
        y_wrist = r_wrist * math.sin(rad_base)
        p3 = np.array([x_wrist, y_wrist, z_wrist])
        
        # Gripper position
        total_angle_2 = total_angle_1 + rad_wrist
        r_gripper = r_wrist + self.L4 * math.cos(total_angle_2)
        z_gripper = z_wrist + self.L4 * math.sin(total_angle_2)
        x_gripper = r_gripper * math.cos(rad_base)
        y_gripper = r_gripper * math.sin(rad_base)
        p4 = np.array([x_gripper, y_gripper, z_gripper])
        
        return p0, p1, p2, p3, p4

# ============================================================================
# SIMULATION CLASS
# ============================================================================

class Simulation3D:
    def __init__(self):
        self.arm = RoboticArm3D()
        
        # Box positions (center x, y, z) and sizes
        self.source_box = {"center": (25, 0, 0), "size": (15, 15, 8), "color": "gray"}
        self.box_a = {"center": (-20, 20, 0), "size": (12, 12, 8), "color": "red", "label": "Box A\n(Tomatoes)"}
        self.box_b = {"center": (-20, -20, 0), "size": (12, 12, 8), "color": "goldenrod", "label": "Box B\n(Potatoes)"}
        
        # Objects in source box
        self.objects = []
        self.sorted_a = []
        self.sorted_b = []
        
        # Spawn initial objects
        self._spawn_objects(6)
        
        # State machine
        self.current_target = None
        self.wait_frames = 0
        
    def _spawn_objects(self, count):
        """Spawn objects in the source box."""
        cx, cy, cz = self.source_box["center"]
        sx, sy, sz = self.source_box["size"]
        
        for _ in range(count):
            obj_type = random.choice(["tomato", "potato"])
            color = "red" if obj_type == "tomato" else "goldenrod"
            
            x = cx + random.uniform(-sx/3, sx/3)
            y = cy + random.uniform(-sy/3, sy/3)
            z = cz + sz/2 + random.uniform(1, 3)
            
            self.objects.append({
                "type": obj_type,
                "pos": np.array([x, y, z]),
                "color": color,
                "radius": 1.5
            })
    
    def update(self):
        """Update simulation state."""
        if self.wait_frames > 0:
            self.wait_frames -= 1
            return
        
        reached = self.arm.update()
        _, _, _, _, gripper_pos = self.arm.forward_kinematics()
        
        if self.arm.state == "IDLE":
            self.arm.target_base = 0
            self.arm.target_shoulder = 60
            self.arm.target_elbow = -60
            if self.objects:
                self.current_target = self.objects[0]
                self.arm.state = "MOVING_TO_SOURCE"
                
        elif self.arm.state == "MOVING_TO_SOURCE":
            if self.current_target:
                obj_pos = self.current_target["pos"]
                angle = math.degrees(math.atan2(obj_pos[1], obj_pos[0]))
                self.arm.target_base = angle
                self.arm.target_shoulder = 20
                self.arm.target_elbow = -40
                if reached:
                    self.arm.state = "REACHING"
                    
        elif self.arm.state == "REACHING":
            self.arm.target_shoulder = 0
            self.arm.target_elbow = -20
            if reached:
                self.arm.state = "GRABBING"
                self.wait_frames = 10
                
        elif self.arm.state == "GRABBING":
            if self.current_target and self.current_target in self.objects:
                self.arm.holding_object = self.current_target
                self.objects.remove(self.current_target)
            self.arm.gripper_open = False
            self.arm.state = "LIFTING"
            
        elif self.arm.state == "LIFTING":
            self.arm.target_shoulder = 50
            self.arm.target_elbow = -50
            if reached:
                self.arm.state = "MOVING_TO_BOX"
                if self.arm.holding_object:
                    if self.arm.holding_object["type"] == "tomato":
                        self.arm.target_base = 135
                    else:
                        self.arm.target_base = -135
                        
        elif self.arm.state == "MOVING_TO_BOX":
            if reached:
                self.arm.state = "LOWERING"
                self.arm.target_shoulder = 30
                self.arm.target_elbow = -30
                
        elif self.arm.state == "LOWERING":
            if reached:
                self.arm.state = "DROPPING"
                self.wait_frames = 10
                
        elif self.arm.state == "DROPPING":
            if self.arm.holding_object:
                obj = self.arm.holding_object
                if obj["type"] == "tomato":
                    cx, cy, cz = self.box_a["center"]
                    obj["pos"] = np.array([cx + random.uniform(-3, 3), 
                                           cy + random.uniform(-3, 3), 
                                           cz + 4])
                    self.sorted_a.append(obj)
                else:
                    cx, cy, cz = self.box_b["center"]
                    obj["pos"] = np.array([cx + random.uniform(-3, 3), 
                                           cy + random.uniform(-3, 3), 
                                           cz + 4])
                    self.sorted_b.append(obj)
                self.arm.holding_object = None
            self.arm.gripper_open = True
            self.arm.state = "RETURNING"
            
        elif self.arm.state == "RETURNING":
            self.arm.target_base = 0
            self.arm.target_shoulder = 60
            self.arm.target_elbow = -60
            if reached:
                self.arm.state = "IDLE"
                self.current_target = None
        
        # Update held object position
        if self.arm.holding_object:
            _, _, _, _, gripper_pos = self.arm.forward_kinematics()
            self.arm.holding_object["pos"] = gripper_pos.copy()

# ============================================================================
# VISUALIZATION
# ============================================================================

def main():
    print("Starting 3D Robotic Arm Simulation...")
    print("Close the window to exit.")
    
    sim = Simulation3D()
    
    # Setup figure
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    def update_frame(frame):
        ax.clear()
        
        # Update simulation
        sim.update()
        
        # Get joint positions
        p0, p1, p2, p3, p4 = sim.arm.forward_kinematics()
        
        # Set axis limits
        ax.set_xlim(-40, 50)
        ax.set_ylim(-40, 40)
        ax.set_zlim(0, 50)
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_zlabel('Z (cm)')
        ax.set_title(f'4-DOF Robotic Arm Simulation | State: {sim.arm.state}', fontsize=12)
        
        # Draw ground
        xx, yy = np.meshgrid(range(-45, 55, 10), range(-45, 45, 10))
        ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.2, color='green')
        
        # Draw boxes
        for box_info in [sim.source_box, sim.box_a, sim.box_b]:
            edges = create_box_wireframe(box_info["center"], box_info["size"])
            for edge in edges:
                xs = [edge[0][0], edge[1][0]]
                ys = [edge[0][1], edge[1][1]]
                zs = [edge[0][2], edge[1][2]]
                ax.plot(xs, ys, zs, color=box_info["color"], linewidth=2)
        
        # Draw box labels
        ax.text(sim.source_box["center"][0], sim.source_box["center"][1], 
                sim.source_box["center"][2] + 12, "Source Box", fontsize=9, ha='center')
        ax.text(sim.box_a["center"][0], sim.box_a["center"][1], 
                sim.box_a["center"][2] + 12, sim.box_a["label"], fontsize=9, ha='center', color='red')
        ax.text(sim.box_b["center"][0], sim.box_b["center"][1], 
                sim.box_b["center"][2] + 12, sim.box_b["label"], fontsize=9, ha='center', color='goldenrod')
        
        # Draw objects
        for obj in sim.objects + sim.sorted_a + sim.sorted_b:
            u = np.linspace(0, 2 * np.pi, 10)
            v = np.linspace(0, np.pi, 10)
            r = obj["radius"]
            x = obj["pos"][0] + r * np.outer(np.cos(u), np.sin(v))
            y = obj["pos"][1] + r * np.outer(np.sin(u), np.sin(v))
            z = obj["pos"][2] + r * np.outer(np.ones(np.size(u)), np.cos(v))
            ax.plot_surface(x, y, z, color=obj["color"], alpha=0.9)
        
        # Draw held object
        if sim.arm.holding_object:
            obj = sim.arm.holding_object
            u = np.linspace(0, 2 * np.pi, 10)
            v = np.linspace(0, np.pi, 10)
            r = obj["radius"]
            x = obj["pos"][0] + r * np.outer(np.cos(u), np.sin(v))
            y = obj["pos"][1] + r * np.outer(np.sin(u), np.sin(v))
            z = obj["pos"][2] + r * np.outer(np.ones(np.size(u)), np.cos(v))
            ax.plot_surface(x, y, z, color=obj["color"], alpha=0.9)
        
        # Draw arm segments as cylinders
        segments = [
            (p0, p1, 'dimgray', 3),     # Base
            (p1, p2, 'steelblue', 2.5), # Upper arm
            (p2, p3, 'royalblue', 2),   # Forearm
            (p3, p4, 'navy', 1.5),      # Gripper
        ]
        
        for start, end, color, radius in segments:
            faces = create_cylinder(start, end, radius, resolution=8)
            if faces:
                poly = Poly3DCollection(faces, alpha=0.9)
                poly.set_facecolor(color)
                poly.set_edgecolor('black')
                ax.add_collection3d(poly)
        
        # Draw joints as spheres
        for pos, color in [(p1, 'red'), (p2, 'orange'), (p3, 'yellow')]:
            u = np.linspace(0, 2 * np.pi, 8)
            v = np.linspace(0, np.pi, 8)
            r = 2
            x = pos[0] + r * np.outer(np.cos(u), np.sin(v))
            y = pos[1] + r * np.outer(np.sin(u), np.sin(v))
            z = pos[2] + r * np.outer(np.ones(np.size(u)), np.cos(v))
            ax.plot_surface(x, y, z, color=color, alpha=0.9)
        
        # Draw two-finger gripper
        # Calculate finger positions based on gripper direction
        rad_base = math.radians(sim.arm.theta_base)
        rad_total = math.radians(sim.arm.theta_shoulder + sim.arm.theta_elbow + sim.arm.theta_wrist)
        
        # Direction the gripper is pointing (forward direction)
        forward = np.array([
            math.cos(rad_total) * math.cos(rad_base),
            math.cos(rad_total) * math.sin(rad_base),
            math.sin(rad_total)
        ])
        
        # Perpendicular direction for finger spread (side direction)
        side = np.array([-math.sin(rad_base), math.cos(rad_base), 0])
        
        # Finger parameters
        finger_length = 5
        finger_radius = 0.8
        spread = 3 if sim.arm.gripper_open else 0.8  # Open vs closed
        
        # Finger 1 (left)
        finger1_base = p4 + side * spread
        finger1_tip = finger1_base + forward * finger_length
        
        # Finger 2 (right)
        finger2_base = p4 - side * spread
        finger2_tip = finger2_base + forward * finger_length
        
        # Draw fingers as cylinders
        finger_color = 'lime' if sim.arm.gripper_open else 'orangered'
        for f_base, f_tip in [(finger1_base, finger1_tip), (finger2_base, finger2_tip)]:
            faces = create_cylinder(f_base, f_tip, finger_radius, resolution=6)
            if faces:
                poly = Poly3DCollection(faces, alpha=0.9)
                poly.set_facecolor(finger_color)
                poly.set_edgecolor('black')
                ax.add_collection3d(poly)
        
        # Draw finger tips
        ax.scatter(*finger1_tip, color=finger_color, s=50, marker='o')
        ax.scatter(*finger2_tip, color=finger_color, s=50, marker='o')
        
        # Status text
        ax.text2D(0.02, 0.98, f"Base: {sim.arm.theta_base:.1f}°", transform=ax.transAxes, fontsize=9)
        ax.text2D(0.02, 0.94, f"Shoulder: {sim.arm.theta_shoulder:.1f}°", transform=ax.transAxes, fontsize=9)
        ax.text2D(0.02, 0.90, f"Elbow: {sim.arm.theta_elbow:.1f}°", transform=ax.transAxes, fontsize=9)
        ax.text2D(0.02, 0.86, f"Objects left: {len(sim.objects)}", transform=ax.transAxes, fontsize=9)
        
        return []
    
    ani = animation.FuncAnimation(fig, update_frame, frames=None, 
                                   interval=50, blit=False, cache_frame_data=False)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
