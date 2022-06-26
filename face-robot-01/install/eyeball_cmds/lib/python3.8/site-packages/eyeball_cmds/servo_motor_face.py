from adafruit_servokit import ServoKit
import numpy as np
import time
import random

########################################
################# Face #################
########################################
pi = 3.14159
# 
kit = [ServoKit(channels = 16, address = 0x41), 
       ServoKit(channels = 16, address = 0x42)]
# kit = [ServoKit(channels = 16, address = 0x41)] # only for neck

class Actuator(object):
    def __init__(self, idx_tuple, min_angle, max_angle, init_angle, inverse_flag = False):
        self.idx = idx_tuple      # data format: (kit number, channel)
        self.v_min = min_angle
        self.v_max = max_angle
        self.range = self.v_max - self.v_min
        self.v_cur = init_angle
        self.norm_v_cur = (self.v_cur - self.v_min)/self.range
        if inverse_flag == True:
            self.norm_v_cur = 1 - self.norm_v_cur
        self.norm_v_init = self.norm_v_cur
        self.inverse_value = inverse_flag
        self.norm_act(self.norm_v_init)

    # Input normalized cmds: [0,1]
    def norm_act(self, norm_cmd):
        if norm_cmd > 1.0 or norm_cmd < 0:
            if norm_cmd > 1.0:
                norm_cmd = 1.0
            if norm_cmd < 0:
                norm_cmd = 0.01

            print(norm_cmd,"Norm cmd is not right, it should between 0 ~ 1.")
    

        self.norm_v_cur = norm_cmd
        
        if self.inverse_value == True:
            norm_cmd = 1 - norm_cmd


        kit[self.idx[0]].servo[self.idx[1]].angle = norm_cmd * (self.range) + self.v_min

        self.v_cur = norm_cmd * (self.range) + self.v_min

    
    # Input angle: [0,180]
    def act(self, cmd):
        if cmd > self.v_max or cmd < self.v_min:
            print(cmd,"Cmd is out of action space")
            quit()
        else:
            kit[self.idx[0]].servo[self.idx[1]].angle = cmd
        self.v_cur = cmd
        self.norm_v_cur = (cmd - self.v_min)/self.range


    def ret(self):
        kit[self.idx[0]].servo[self.idx[1]].angle = self.v_min
        self.v_cur = self.v_min 
        self.norm_v_cur = 0

# Neck motors Initialization
neck_motor_list = []
for i in range(6):
    neck_motor_list.append(Actuator((0,i),0,180,90))


# Motors Initialization:
#    
#      1   0   9 
#   2             8
#   3             7  
#      4   5   6   
#
# increase value -> counter-clockwise rotation


m0_init = 0
m1_init = 30
m9_init = 55
m2_init = 113
m8_init = 116
m3_init = 63
m7_init = 106
m4_init = 70
m6_init = 100
m5_init = 100
jaw_init = 72

ry_init = 85
rp_init = 97
ru_init = 88
rl_init = 125
ro_init = 115
ri_init = 115

ly_init = 85
lp_init = 70
lu_init = 100
ll_init = 165
lo_init = 130
li_init = 145

m0 = Actuator(idx_tuple = (0,6), min_angle = 0, max_angle = 50, init_angle = m0_init)

m1 = Actuator(idx_tuple = (0,7), min_angle = 0,  max_angle = 60,  init_angle = m1_init)
m9 = Actuator(idx_tuple = (0,15), min_angle = 15, max_angle = 85,  init_angle = m9_init, inverse_flag=True)

m2 = Actuator(idx_tuple = (0,8), min_angle = 55,  max_angle = 170, init_angle = m2_init)
m8 = Actuator(idx_tuple = (0,14), min_angle = 75,  max_angle = 180, init_angle = m8_init, inverse_flag=True)

m3 = Actuator(idx_tuple = (0,9), min_angle = 20,  max_angle = 140,  init_angle = m3_init)
m7 = Actuator(idx_tuple = (0,13), min_angle = 22, max_angle = 140, init_angle = m7_init, inverse_flag=True)

m4 = Actuator(idx_tuple = (0,10), min_angle = 30,  max_angle = 70,  init_angle = m4_init)
m6 = Actuator(idx_tuple = (0,12), min_angle = 100,  max_angle = 125,  init_angle = m6_init, inverse_flag=True)

m5 = Actuator(idx_tuple = (0,11), min_angle = 75,  max_angle = 170,  init_angle = m5_init)
jaw= Actuator(idx_tuple = (1,0),  min_angle = 30, max_angle = 72,  init_angle = jaw_init)

r_eye_yaw =    Actuator(idx_tuple = (1,1), min_angle = 45, max_angle = 125, init_angle = ry_init)
r_eye_pitch =  Actuator(idx_tuple = (1,2), min_angle = 82, max_angle = 113, init_angle = rp_init) # + up
r_eyelid_up =  Actuator(idx_tuple = (1,3), min_angle = 10, max_angle = 90, init_angle = ru_init, inverse_flag = True)
r_eyelid_low = Actuator(idx_tuple = (1,4), min_angle = 125, max_angle = 180, init_angle = rl_init,inverse_flag = True)

r_eyebrow_outter= Actuator(idx_tuple = (1,5), min_angle = 90, max_angle = 145, init_angle = ro_init)
r_eyebrow_inner = Actuator(idx_tuple = (1,6), min_angle = 95, max_angle = 130, init_angle = ri_init,inverse_flag=True)

l_eye_yaw =    Actuator(idx_tuple = (1,7), min_angle = 50, max_angle = 130, init_angle = ly_init) #  max -> turn left
l_eye_pitch =  Actuator(idx_tuple = (1,8), min_angle = 55, max_angle = 85, init_angle = lp_init, inverse_flag=True) #  - up 

l_eyelid_up =  Actuator(idx_tuple = (1,9), min_angle = 100, max_angle = 180, init_angle = lu_init) ##  close 100
l_eyelid_low = Actuator(idx_tuple = (1,10), min_angle = 105, max_angle = 165, init_angle = ll_init) ## close 160

l_eyebrow_outter= Actuator(idx_tuple = (1,11), min_angle = 105, max_angle = 150, init_angle = lo_init,inverse_flag=True)
l_eyebrow_inner = Actuator(idx_tuple = (1,12), min_angle = 135, max_angle = 165, init_angle = li_init)


# Because during initialization, the eyes closed.
l_eyebrow_outter.norm_act(0.4)
r_eyebrow_outter.norm_act(0.4)
l_eyebrow_inner.norm_act(0.4)
r_eyebrow_inner.norm_act(0.4)

# 35 degrees of freedom and 26 motors
static_face_data = [
m0_init, m1_init, m9_init, m2_init, m8_init, m3_init, m7_init, m4_init, m6_init, m5_init, jaw_init, 
ry_init, rp_init, ru_init, rl_init, ro_init, ri_init, 
ly_init, lp_init, lu_init, ll_init, lo_init, li_init
]

all_motors = [m0, m1, m9, m2, m8, m3, m7, m4, m6, m5, jaw, 
    r_eye_yaw, r_eye_pitch, r_eyelid_up, r_eyelid_low, r_eyebrow_outter, r_eyebrow_inner,
    l_eye_yaw, l_eye_pitch, l_eyelid_up, l_eyelid_low, l_eyebrow_outter, l_eyebrow_inner
]


def reset_face():
    all_motors = [m0, m1, m9, m2, m8, m3, m7, m4, m6, m5, jaw, 
    r_eye_yaw, r_eye_pitch, r_eyelid_up, r_eyelid_low, r_eyebrow_outter, r_eyebrow_inner,
    l_eye_yaw, l_eye_pitch, l_eyelid_up, l_eyelid_low, l_eyebrow_outter, l_eyebrow_inner
    ]
    for i in range(len(all_motors)):
        all_motors[i].act(static_face_data[i])


def slowly_open(random_time = False):

    if random_time == True:
        steps = random.randint(10,60)
    else:
        steps = 60
    for j in range(steps):
        r_eyelid_up.norm_act(0.9*(j+1)/steps)
        l_eyelid_up.norm_act(0.9*(j+1)/steps)
        r_eyelid_low.norm_act(1-0.85*(j+1)/steps)
        l_eyelid_low.norm_act(1-0.85*(j+1)/steps) # 30, bigger than right
        time.sleep(0.02)

def track_eye(data, kx = 0.05, ky = 0.05):
    xs = data[0] / abs(data[2])
    ys = data[1] / abs(data[2])
    r_eye_yaw.norm_act(r_eye_yaw.norm_v_cur - kx * xs)
    l_eye_yaw.norm_act(l_eye_yaw.norm_v_cur - kx * xs)

    r_eye_pitch.norm_act(r_eye_pitch.norm_v_cur + ky * ys)
    l_eye_pitch.norm_act(l_eye_pitch.norm_v_cur + ky * ys)

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


