from math import pi
import numpy as np
from Kinematics.IK import Rz
from move import move
import scipy.io
from DXLmotor import set_traj_params, set_all_cmd_position, set_pid_gain

# Initialization
print('======== Set up ==============')
q_initial = [[0, -0.43117439,  1.64356286,
                  0.43117439, -1.64356286,
                  0.43117439, -1.64356286,
                 -0.43117439,  1.64356286]]
# set_traj_params(10, 500)
set_traj_params(50, 10)
set_pid_gain(600, 0, 10)
set_all_cmd_position(q_initial)
print('Finished')

# Waiting input command
print('========= Waiting CMD ============')
print('========= 1. FORWARD ================')
print('========= 2. INITIAL ================')
print('========= 3. ROT/SIDE ===============')
print('========= 0. Exit    ================')
while 1:
    cmd = input('User Command:')

    if cmd == '1':
        print('==================== Move Forward =====================')
        # set up start and end points
        # pos_list_initial = np.array([[481, -481, 481, -481],
        #                              [300, 300, -300, -300],
        #                              [0, 0, 0, 0]])
        pos_list_initial = np.array([[100, -100, 100, -100],
                                     [100, 100, -100, -100],
                                     [0, 0, 0, 0]]) * 4.4

        a = 0
        pb = np.array([10*a**1, 130, 0])   # Forward = 130
        # pb = np.array([60, 0, 0])
        angle = 0
        # angle = -5*a**1/180 * pi * 1
        T = np.block([[Rz(angle), np.array([pb]).transpose()], [0, 0, 0, 1]])
        pos_list_end = T.dot(np.block([[pos_list_initial], [1, 1, 1, 1]]))

        # get joint trajectory
        q_list = move(pos_list_initial, pos_list_end, angle)
        q_list = np.array(q_list)
        q1 = q_list[:, 0]*1.0
        q_list[:, 0] = q1

        # update motors
        set_all_cmd_position(q_list)
        print('One Step Finished!')
        print(q_list)

        # data logout
        toMatlab = 0
        if toMatlab:
            scipy.io.savemat('Joint_list.mat', mdict={'joint_list': np.array(q_list)})

    if cmd == '2':
        print('==================== Initializing =====================')
        q_initial = [[-0.4834, -0.33288898,  0.85959274,  1.02986114, -1.61795491,
                        0.30217778, -0.90478687, -0.74922026,  2.00329612]]
        set_traj_params(100, 20)
        set_pid_gain(600, 0, 10)
        set_all_cmd_position(q_initial)
        print('Initialization finished!')

    if cmd == '3':
        print('==================== Rotating / Side Walking ===================')
        pos_list_initial = np.array([[100, -100, 100, -100],
                                     [100, 100, -100, -100],
                                     [0, 0, 0, 0]]) * 4.4
        pb = np.array([80, 0, 0])
        # angle = -5 * a ** 1 / 180 * pi * 1
        angle = 0
        T = np.block([[Rz(angle), np.array([pb]).transpose()], [0, 0, 0, 1]])
        pos_list_end = T.dot(np.block([[pos_list_initial], [1, 1, 1, 1]]))

        # get joint trajectory
        q_list = move(pos_list_initial, pos_list_end, angle)

        # update motors
        set_all_cmd_position(q_list)
        print('One Step Finished!')
        print(q_list)

    if cmd == '0':
        print('Exit !!!')
        break



