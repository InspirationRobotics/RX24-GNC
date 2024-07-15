from simple_motion import Simple_Motion_Sim
import time


def strafe_wamv_left():
    sim.set_thrust(0, normalized_thrust=0.2) # front right
    sim.set_thrust(1, normalized_thrust=-0.2) # front left
    sim.set_thrust(2, normalized_thrust=-0.138) # back right
    sim.set_thrust(3, normalized_thrust=0.138) # back left

def strafe_wamv_right():
    sim.set_thrust(0, normalized_thrust=-0.2) # front right
    sim.set_thrust(1, normalized_thrust=0.2) # front left
    sim.set_thrust(2, normalized_thrust=0.138) # back right
    sim.set_thrust(3, normalized_thrust=-0.138) # back left

def strafe_left():
    sim.set_thrust(0, normalized_thrust=1.0) # front right
    sim.set_thrust(1, normalized_thrust=-1.0) # front left
    sim.set_thrust(2, normalized_thrust=-1.0) # back right
    sim.set_thrust(3, normalized_thrust=1.0) # back left

def strafe_right():
    sim.set_thrust(0, normalized_thrust=-1) # front right
    sim.set_thrust(1, normalized_thrust=1) # front left
    sim.set_thrust(2, normalized_thrust=1) # back right
    sim.set_thrust(3, normalized_thrust=-1) # back left

def rotate_wamv_ccw():
    sim.set_thrust(0, normalized_thrust=0.2)
    sim.set_thrust(1, normalized_thrust=-0.2)
    sim.set_thrust(2, normalized_thrust=0.138)
    sim.set_thrust(3, normalized_thrust=-0.138)

def rotate_wamv_cw():
    sim.set_thrust(0, normalized_thrust=-0.2)
    sim.set_thrust(1, normalized_thrust=0.2)
    sim.set_thrust(2, normalized_thrust=-0.138)
    sim.set_thrust(3, normalized_thrust=0.138)

def rotate_ccw():
    sim.set_thrust(0, normalized_thrust=0.2)
    sim.set_thrust(1, normalized_thrust=-0.2)
    sim.set_thrust(2, normalized_thrust=0.2)
    sim.set_thrust(3, normalized_thrust=-0.2)

def full_thrust():
    for thruster in sim.get_thrusters():
        thruster.set_normalized_thrust(1.0)

def full_four_thrust():
    for i in range(4):
        sim.set_thrust(i, normalized_thrust=1.0)

def stop_move():
    for i in range(4):
        sim.set_thrust(i, normalized_thrust=0)    

def test_func():
    while sim.active:
        strafe_left()
        time.sleep(2)
        strafe_right()
        time.sleep(2)
    stop_move()

if __name__ == "__main__":
    sim = Simple_Motion_Sim("configs/simple_config.json", test_func, verbose=True)
    sim.run_simulation()