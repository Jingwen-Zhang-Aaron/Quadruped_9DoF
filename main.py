from math import pi
import numpy as np
from Kinematics.IK import Rz
from move import move
from move_updated import move_updated
import scipy.io
from DXLmotor import set_traj_params, set_all_cmd_position, set_pid_gain, set_ff_gain

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

# set_traj_params(300, 500)
# set_ff_gain(100, 200)
# set_pid_gain(850, 0, 10)
set_traj_params(300, 400)
set_ff_gain(100, 200)
set_pid_gain(800, 0, 5)
# Waiting input command
print('========= Waiting CMD ============')
print('========= 1. FORWARD ================')
print('========= 2. INITIAL ================')
print('========= 3. SIDE    ===============')
print('========= 4. ROT     ===============')
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

        pb = np.array([0, 100, 0])   # Forward = 130
        angle = 0

        T = np.block([[Rz(angle), np.array([pb]).transpose()], [0, 0, 0, 1]])
        pos_list_end = T.dot(np.block([[pos_list_initial], [1, 1, 1, 1]]))

        # get joint trajectory
        q_list = move_updated(pos_list_initial, pos_list_end, angle)
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
        q_initial = [[ 0.        , -0.38324988,  1.57628332,  0.38324988, -1.57628332,
                        0.38324988, -1.57628332, -0.38324988,  1.57628332]]
        set_traj_params(50, 10)
        set_pid_gain(600, 0, 10)
        set_all_cmd_position(q_initial)
        print('Initialization finished!')

    if cmd == '3':
        print('==================== Side Walking ===================')
        pos_list_initial = np.array([[100, -100, 100, -100],
                                     [100, 100, -100, -100],
                                     [0, 0, 0, 0]]) * 4.5
        pb = np.array([70, 0, 0])    # 80
        angle = 0
        T = np.block([[Rz(angle), np.array([pb]).transpose()], [0, 0, 0, 1]])
        pos_list_end = T.dot(np.block([[pos_list_initial], [1, 1, 1, 1]]))

        # get joint trajectory
        q_list = move_updated(pos_list_initial, pos_list_end, angle, LEG2_COM_SCALE = 1.2)

        # update motors
        set_all_cmd_position(q_list)
        print('One Step Finished!')
        print(q_list)

    if cmd == '4':
        print('==================== Rotating  ===================')
        pos_list_initial = np.array([[100, -100, 100, -100],
                                     [100, 100, -100, -100],
                                     [0, 0, 0, 0]]) * 4.5
        pb = np.array([0, 0, 0])    # 80
        angle = -6 / 180 * pi * 1
        # angle = 0
        T = np.block([[Rz(angle), np.array([pb]).transpose()], [0, 0, 0, 1]])
        pos_list_end = T.dot(np.block([[pos_list_initial], [1, 1, 1, 1]]))

        # get joint trajectory
        q_list = move_updated(pos_list_initial, pos_list_end, angle, LEG2_COM_SCALE = 1.5)

        # update motors
        set_all_cmd_position(q_list)
        print('One Step Finished!')
        print(q_list)

    if cmd == '0':
        print('Exit !!!')
        break



