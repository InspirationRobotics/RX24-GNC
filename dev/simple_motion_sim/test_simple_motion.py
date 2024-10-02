from simple_motion import Simple_Motion_Sim
import time
import numpy as np


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

def thruster_distances():
    thrusters = sim.get_thrusters()
    distances = [np.linalg.norm(thruster.position - (0,0)) for thruster in thrusters]
    return distances

def rotate_and_strafe(dir_vector, rotation):
    dist = thruster_distances()
    # Front right = Vy + Vx + W * d
    fr = dir_vector[1] + dir_vector[0] + rotation * dist[0]
    # Front left = Vy - Vx - W * d
    fl = dir_vector[1] - dir_vector[0] - rotation * dist[1]
    # Back right = Vy - Vx + W * d
    br = dir_vector[1] - dir_vector[0] + rotation * dist[2]
    # Back left = Vy + Vx - W * d
    bl = dir_vector[1] + dir_vector[0] - rotation * dist[3]
    # Normalize
    max_val = max([fr, fl, br, bl])
    if max_val > 1:
        fr /= max_val
        fl /= max_val
        br /= max_val
        bl /= max_val
    br = br * 0.69
    bl = bl * 0.69
    sim.set_thrust(0, normalized_thrust=fr)
    sim.set_thrust(1, normalized_thrust=fl)
    sim.set_thrust(2, normalized_thrust=br)
    sim.set_thrust(3, normalized_thrust=bl)
    pass

def test_func():
    while sim.active:
        # strafe_wamv_left()
        # time.sleep(2)
        # strafe_wamv_right()
        # time.sleep(2)
        rotate_and_strafe((0.0, 0.0), 0.01)
    stop_move()

if __name__ == "__main__":
    sim = Simple_Motion_Sim("configs/wamv_config.json", test_func, verbose=False)
    sim.run_simulation()