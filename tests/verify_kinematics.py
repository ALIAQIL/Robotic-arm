import sys
import os
import math
import numpy as np

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from simulation import KinematicArm

def test_forward_kinematics():
    print("Testing Forward Kinematics...")
    arm = KinematicArm()
    
    # Test 1: Home Position (All 0)
    # Base=0, Shoulder=0, Elbow=0
    # Arm should be pointing straight up/forward depending on definition
    # In our code:
    # Shoulder pos = (0, 0, L1)
    # Elbow pos: Shoulder=0 -> r=L2, z=L1. Base=0 -> x=L2, y=0. -> (L2, 0, L1)
    # Gripper pos: Elbow=0 -> r=L2+L3, z=L1. -> (L2+L3, 0, L1)
    
    arm.theta_base = 0
    arm.theta_shoulder = 0
    arm.theta_elbow = 0
    
    s, e, g = arm.forward_kinematics()
    
    expected_g = np.array([arm.L2 + arm.L3, 0, arm.L1])
    
    if not np.allclose(g, expected_g, atol=1.0):
        print(f"FAILURE: Home Position mismatch. Expected {expected_g}, got {g}")
        return False
    print("SUCCESS: Home Position verified")
    
    # Test 2: Reach Up (Shoulder 90)
    # Shoulder=90 -> Arm points straight up
    # r_elbow = L2 * cos(90) = 0
    # z_elbow = L1 + L2 * sin(90) = L1 + L2
    # r_gripper = 0 + L3 * cos(90+0) = 0
    # z_gripper = L1 + L2 + L3
    # x, y should be 0
    
    arm.theta_shoulder = 90
    s, e, g = arm.forward_kinematics()
    
    expected_g_up = np.array([0, 0, arm.L1 + arm.L2 + arm.L3])
    
    if not np.allclose(g, expected_g_up, atol=1.0):
        print(f"FAILURE: Reach Up mismatch. Expected {expected_g_up}, got {g}")
        return False
    print("SUCCESS: Reach Up verified")
    
    return True

if __name__ == "__main__":
    if test_forward_kinematics():
        sys.exit(0)
    else:
        sys.exit(1)
