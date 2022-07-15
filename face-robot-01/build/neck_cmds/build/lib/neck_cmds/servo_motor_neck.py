from adafruit_servokit import ServoKit
import numpy as np
import time
import random
########################################
################# NECK #################
########################################
from ctypes import *
file_path = "./src/neck_cmds/neck_cmds/"
lib = CDLL(file_path + "ctypes2/lx16a.so")


# motor ID from the bottom to the top:

# 12
# 11
# 10

# 1. Define action space


def connect_usb():
    for i in range(10):
        try:
            print("Try USB%d..."%i)
            filename = "/dev/ttyUSB%d"%i
            IO1 = lib.IO_init(c_char_p(filename.encode('ascii')))
            if IO1 < 0:
                sys.exit()
            break
        except:
            pass

class NECK_Controller():
    def __init__(self,motor_index, min, max, m_speed):
        self.min = min
        self.max = max
        self.range = max - min 
        self.id = motor_index
        self.position = lib.posRead(self.id)
        self.pre_value = 0.5
        self.speed = m_speed

    def act(self, cmd, m_speed = 100):
        if cmd <0 or cmd >1:
            print(cmd)
            for _ in range(10):
                print("Wrong value!")
            
        if cmd>1:
            cmd = 1
        elif cmd<0:
            cmd = 0


        cmd_value = int(cmd * self.range + self.min)

        # read joint position
        # self.position = lib.posRead(self.id)
        # print(self.position)
        
        # actuate the commands: (id, value, speed) 
        # move_time = self.speed * abs(output - self.position)
        lib.move(self.id, cmd_value, m_speed)   
        self.pre_value = cmd
        # time.sleep(0.001 * move_time)

    def pos_feedback(self):
        self.position = lib.posRead(self.id)
        norm_pos = (self.position - self.min)/self.range
        return norm_pos

def start():
    for i in range(10,13):
        # value 500 should be the middle position of the neck joint.
        lib.move(i, 500,1000)
    



def smooth(new_value, num_step=30, k = 0.2):
    for i in range(num_step):
        for j in range(3):
            new_v = k * (new_value[j] - c_group[j].pre_value) + c_group[j].pre_value
            c_group[j].act(new_v)
            # print()

        time.sleep(0.02)

    print("done")

# init controller
connect_usb()
c10 = NECK_Controller(10, 200, 800, 4)
c11 = NECK_Controller(11, 430, 570, 4)
c12 = NECK_Controller(12, 400, 600, 4)

c_group = [c10, c11, c12]
start()


def move_necks(new_value):
    pre_value = np.array([c10.pre_value, c11.pre_value, c12.pre_value])
    length_list = np.array(new_value) - pre_value

    for i in range(3):
        c_group[i].act(new_value[i])
    time.sleep(max(abs(length_list)) * 2)

def move_steps(new_value):
    pre_value = np.array([c10.pre_value, c11.pre_value, c12.pre_value])
    length_list = np.array(new_value) - pre_value

    steps = int(max(abs(length_list)) / 0.05)
    if steps > 1:
        for i in range(steps):
            step_value = ((i + 1) / steps) * length_list + pre_value
            move_necks(step_value)
    else:
        move_necks(new_value)


def track_neck(data, kx=0.05, ky = 0.05):
    xs = data[0]/abs(data[2])
    ys = data[1]/abs(data[2])

    new_v = [c10.pre_value - kx * xs, 0.5, c12.pre_value + ky * ys]
    # new_v = [c10.pre_value - kx * xs, 0.5, 0.5]

    move_steps(new_v)

 
# Generate random
def act_rdm_cmd():
    cmd = 0.25 * np.random.randint(0,5,11)
    for i in range(11):
        all_motors[i].norm_act(cmd[i])
    return cmd

if __name__ == "__main__":
    slowly_open()

    # for i in range(100):
    #     print(act_rdm_cmd())
    #     time.sleep(1)
    # # jaw.norm_act(0)

    reset_face()