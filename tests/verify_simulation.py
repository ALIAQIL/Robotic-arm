import sys
import os
import time

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from simulation import VirtualArm

def test_virtual_arm():
    print("Testing Virtual Arm Logic...")
    arm = VirtualArm()
    
    # Test initial state
    if arm.state != "IDLE":
        print(f"FAILURE: Initial state should be IDLE, got {arm.state}")
        return False
        
    # Test movement logic
    arm.target_x = 400
    arm.target_y = 400
    arm.state = "MOVING_TO_OBJ"
    
    # Update until reached
    steps = 0
    reached = False
    while steps < 100:
        if arm.update():
            reached = True
            break
        steps += 1
        
    if not reached:
        print("FAILURE: Arm did not reach target")
        return False
        
    if arm.x != 400 or arm.y != 400:
        print(f"FAILURE: Arm position mismatch. Got ({arm.x}, {arm.y})")
        return False
        
    print("SUCCESS: Virtual Arm logic verified")
    return True

if __name__ == "__main__":
    if test_virtual_arm():
        sys.exit(0)
    else:
        sys.exit(1)
