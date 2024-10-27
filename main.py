import numpy as np
import Adafruit_PCA9685

import time

# leg positions and indices:
#......./\__/\
#.....0/  |   \1 *(0,0,0), (0,63,0)
#......./\__/\
#.....2/  |   \3 *(53.75, 9.675, 0), (53.75, 72.675,0)
#......./\__/\
#.....4/      \5 *(107.5,0,0), (107.5, 63, 0)
#3 servos/leg
#needs around 50Hz for smooth movement


#         J2
#        /\
#       /  \
#J0 ___/J1  \
#       :....\ (x,y,z)



def calc_angel(n1, n2):
    a = np.dot(n1, n2)/(np.linalg.norm(n1)*np.linalg.norm(n2))
    return(np.arccos(a))

def calc_rotation(x0, y0, theta):
    x1 = x0*np.cos(theta) - y0*np.sin(theta)
    y1 = x0*np.sin(theta) + y0*np.cos(theta)
    return(x1, y1)

def xy_vec_from_angle(angle):
    return(np.asarray([np.cos(angle, np.sin(angle))]))

def calc_dist2D(v1, v2):
    dist = np.sqrt((v1[0]-v2[0])**2 + (v1[1]-v2[1])**2)
    return(dist)

def cart2pol(xy):
    rho = np.linalg.norm(xy)
    phi = np.arctan2(xy[1], xy[0])
    return(np.array((rho, phi)))



class leg:
    def move_servo(self, channel, angel, pwm_dev_): #angel in rad
        us_per_pulse = 1e6/self.freq
        bit_per_pulse = us_per_pulse/4096

        pulse_per_rad = 2000/3.14159
        pulse = (angel * pulse_per_rad)+ 500
        signal = pulse/bit_per_pulse
        pwm_dev_.set_pwm(channel, 0, int(signal))


    def move(self): #sleep after calling that function on all legs
        self.J0, self.J1, self.J2 = self.calculate_leg_angles(self.position, self.l0, self.l1, self.l2, self.mirrored)
        self.move_servo(self.channel_J0, self.J0 + self.correct_J0, self.pwm_dev) 
        self.move_servo(self.channel_J1, self.J1 + self.correct_J1, self.pwm_dev) 
        self.move_servo(self.channel_J2, self.J2 + self.correct_J2, self.pwm_dev)


    def calculate_leg_angles(self, endeffector_pos_, l0_, l1_, l2_, mirrored): #xyz
        r_phi = cart2pol(endeffector_pos_[:-1])
        r = r_phi[0]
        r_eff = r-l0_
        phi = r_phi[1]

        plane_projected = np.array((r_eff,endeffector_pos_[2]))
        c = np.sqrt(plane_projected[0]**2 + plane_projected[1]**2) #d12
        alpha = np.arccos((l1_**2 + c **2 - l2_ **2)/(2*l1_*c))
        beta = np.arccos((l2_**2 + c **2 - l1_ **2)/(2*l2_*c))
        gamma = np.pi - alpha - beta

        theta = cart2pol(plane_projected)[1]
        alpha_eff = alpha + theta
        J1 = np.pi/2 - alpha_eff

        if mirrored:
            J1 = np.pi-J1
            phi = np.pi-phi
            gamma = np.pi-gamma
            return(phi, J1, gamma) #J0, J1 incl. correction for angle, J1

        else:    
            return(phi, J1, gamma) #J0, J1 incl. correction for angle, J1


    def current_endeffector_pos_global(self):
        if self.mirrored:
            return (self.hexapod_position + np.asarray([1,-1,1])* self.position)
        else:
            return (self.hexapod_position + self.position)


    def endeffector_pos_from_global(self, endeffector_pos_):
        if self.mirrored:
            return(np.asarray([1,-1,1])* endeffector_pos_  - np.asarray([1,-1,1])* self.hexapod_position ) #np.asarray([1,-1,1])
        else:
            return(endeffector_pos_ - self.hexapod_position )


    def __init__(self, channel_J0,channel_J1,channel_J2, origin_, pwm_dev_, mirrored, freq, hexapod_position_):
        self.position = np.copy(origin_) #np.asarray([0.0,4.0, -1.0])
        self.origin = np.copy(origin_)
        self.hexapod_position = hexapod_position_

        self.l0 = 12
        self.l1 = 45
        self.l2 = 65

        self.pwm_dev = pwm_dev_
        self.freq = freq

        self.channel_J0 = channel_J0
        self.channel_J1 = channel_J1
        self.channel_J2 = channel_J2
        self.mirrored = mirrored 

        self.correct_J0 = 0
        self.correct_J1 = 0
        self.correct_J2 = 0

        self.J0, self.J1, self.J2 = self.calculate_leg_angles(self.position, self.l0, self.l1, self.l2, self.mirrored)


class hexapod:

    def __init__(self):

        origin = np.asarray([0.0,50, -30])
        self.robot_pos = np.asarray([0,0,0])
        self.freq = 50 #def global variable
        self.pwm1 = Adafruit_PCA9685.PCA9685()
        self.pwm1.set_pwm_freq(self.freq) #remember to set update freq to servos.
        self.pos2d = np.asarray([0.0,0.0])

        self.pwm2 = Adafruit_PCA9685.PCA9685(0x41)
        self.pwm2.set_pwm_freq(self.freq) #remember to set update freq to servos.

        #Hardcode leg servos

        self.leg0 = leg(9, 10, 11, np.asarray([20.0,40, -30]), self.pwm2, True, self.freq, np.asarray([53.75,-31.495,0]) )
        self.leg1 = leg(0, 1, 2, np.asarray([20,40, -30]), self.pwm1, False, self.freq, np.asarray([53.75,31.495,0]) )
        self.leg2 = leg(13, 14, 15, np.asarray([0.0,50, -30]), self.pwm2, True, self.freq, np.asarray([0,-41.174, 0]))

        self.leg3 = leg(4, 5, 6, np.asarray([0.0,50, -30]), self.pwm1, False, self.freq, np.asarray([0,41.174, 0]))
        self.leg4 = leg(0, 1, 2, np.asarray([-20.0,40, -30]), self.pwm2, True, self.freq, np.asarray([-53.75,-31.495,0]))
        self.leg5 = leg(12, 13, 14, np.asarray([-20.0,40, -30]), self.pwm1, False, self.freq, np.asarray([-53.75,31.495,0]))

        #all legs can be accessed via this list to iterate
        self.leg_list = [self.leg0, self.leg1, self.leg2,self.leg3,self.leg4,self.leg5]


        #Hardcode corrections in degree for each leg due to misalignment during assembly

        self.leg0.correct_J0 = -0.15
        self.leg0.correct_J1 = -0.1
        self.leg1.correct_J1 = 0.05
        self.leg2.correct_J0 = -0.15
        self.leg4.correct_J0 = -0.15
        self.leg5.correct_J1 = 0.05


    def all_legs_position(self, position):
        self.leg0.position  = np.copy(position)
        self.leg1.position  = np.copy(position)
        self.leg2.position  = np.copy(position)
        self.leg3.position  = np.copy(position)
        self.leg4.position  = np.copy(position)
        self.leg5.position  = np.copy(position)

    def all_legs_move(self):

        self.leg0.move()
        self.leg1.move()
        self.leg2.move()
        self.leg3.move()
        self.leg4.move()
        self.leg5.move()




robot = hexapod()
robot.all_legs_move()
time.sleep(0.5)


#set all legs on the ground:

for l in robot.leg_list:
    l.position[2] += 15
    robot.all_legs_move()
    time.sleep(0.1)
    l.position[2] -= 15
    robot.all_legs_move()
    time.sleep(0.02)

leg_delta = np.zeros((6))
max_delta = 10
theta = 0.015 #rad




for j in range(1000):
    i = j+15
    if (i % 30 == 0):
        theta *=-1
    for l in range(len(robot.leg_list)):
        x0 = robot.leg_list[l].current_endeffector_pos_global()[0]
        y0 = robot.leg_list[l].current_endeffector_pos_global()[1]
        if robot.leg_list[l].mirrored:
            x1, y1 = calc_rotation(x0, y0, theta)
        else:
            x1, y1 = calc_rotation(x0, y0, theta)


        robot.leg_list[l].position = robot.leg_list[l].endeffector_pos_from_global(np.array([x1, y1,robot.leg_list[l].current_endeffector_pos_global()[2] ]))


    robot.all_legs_move()
    time.sleep(0.001)
exit()
for i in range(1000):

    while(np.max(leg_delta<max_delta)):
        for l in range(len(robot.leg_list)):
            robot.leg_list[l].position[0] += 2
            leg_delta[l] = calc_dist2D(robot.leg_list[l].position,robot.leg_list[l].origin) 
            print(leg_delta[l])

        robot.all_legs_move()
        time.sleep(0.001)

    for l in range(len(robot.leg_list)):
        l = 5-l
        robot.leg_list[l].position[2] +=10
        robot.all_legs_move()
        time.sleep(0.05) #move z

        robot.leg_list[l].position[0] = np.copy(robot.leg_list[l].origin[0])  - max_delta +1
        robot.leg_list[l].position[1] = np.copy(robot.leg_list[l].origin[1])
        robot.all_legs_move()
        time.sleep(0.1)
        leg_delta[l] = calc_dist2D(robot.leg_list[l].position,robot.leg_list[l].origin) 

        robot.leg_list[l].position[2] -=10
        robot.all_legs_move()
        time.sleep(0.05) #move z


#for i in range(1000):

exit()


origin = np.asarray([0.0,50, -10])
dx = 0.5
max_dist_2d = 20

leg1 = leg(0, 1, 2, origin, pwm1)
leg1.move()
time.sleep(0.1)
q = 1


for i in range(1000):q


#build object leg
#has position, bool touching ground, origin, acceptabel range

#build object robot





