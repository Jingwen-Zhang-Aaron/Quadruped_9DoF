from robot_player import MotionManager, DxlOptions
from robot_player.motion_manager import AngleOffset
from robot_player import to_player_angle_offset, from_player_angle_offset
from robot_player.dxl.dxl_control_table import XSERIES
from time import sleep
import platform
from math import pi
import numpy as np
import scipy.io

ids = [2, 3]
motor_ids = [[2, 3]]

if platform.system() == 'Windows':
    ports = ['COM15', 'COM16']
else:
    ports = ['/dev/ttyUSB0']

dopts = DxlOptions(motor_ids,
                   motor_types=['XSERIES'],
                   ports=ports,
                   baudrate=3000000,
                   protocol_version=2)

offset = AngleOffset(trans=[1]*2, offset=[pi, pi])

def set_vel_limit(maxvel = 128):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # Set trapezoidal profile parameters
        # 0 means infinite velocity and acceleration
        mm.device.set_torque_enable(ids, [0] * 2)
        mm.device._write_data(ids, XSERIES.VELOCITY_LIMIT, [maxvel] * 2, 4)
        mm.device.set_torque_enable(ids, [1] * 2)

def set_traj_params(maxvel=0, maxaccel=0):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # Set trapezoidal profile parameters
        # 0 means infinite velocity and acceleration
        mm.device.set_torque_enable(ids, [0] * 2)
        mm.device._write_data(ids, XSERIES.PROFILE_VELOCITY, [maxvel] * 2, 4)
        mm.device._write_data(ids, XSERIES.PROFILE_ACCELERATION, [maxaccel] * 2, 4)
        mm.device.set_torque_enable(ids, [1] * 2)



def set_pid_gain(Pgain=800, Igain=0, Dgain=0):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # Set position PID gain
        mm.device.set_torque_enable(ids, [0] * 2)
        mm.device._write_data(ids, XSERIES.POSITION_P_GAIN, [Pgain] * 2, 2)
        mm.device._write_data(ids, XSERIES.POSITION_I_GAIN, [Igain] * 2, 2)
        mm.device._write_data(ids, XSERIES.POSITION_D_GAIN, [Dgain] * 2, 2)
        mm.device.set_torque_enable(ids, [1] * 2)


def set_ff_gain(Gain1=0, Gain2=0):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # Set position Feedforward gain
        mm.device.set_torque_enable(ids, [0] * 2)
        mm.device._write_data(ids, XSERIES.FEEDFORWARD_1ST_GAIN, [Gain1] * 2, 2)
        mm.device._write_data(ids, XSERIES.FEEDFORWARD_2ND_GAIN, [Gain2] * 2, 2)
        mm.device.set_torque_enable(ids, [1] * 2)


def set_single_cmd_position(pos_list, motor_id):
    # eg. motor_id = [1], pos_list = [[0],[pi/2]]
    with MotionManager(motor_id, dt=.005, options=dopts) as mm:

        # move one single motor with id
        for pl in pos_list:
            mm.set_goal_position(motor_id, to_player_angle_offset(pl, offset))
            #mm.wait(2.0)
            qcurr = from_player_angle_offset(mm.get_all_present_position(), offset)
            # Keep reading encoder until motor reach goal position
            while np.max(abs(np.array(qcurr) - np.array(pl))) >= 0.005:
                qcurr = from_player_angle_offset(mm.get_all_present_position(), offset)
            print(qcurr)


def set_all_cmd_position(pos_list):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # move all motors together
        # pos_list = [[pi,pi], [0,0], [pi/2,pi/2]]
        for pl in pos_list:
            mm.set_goal_position(ids, to_player_angle_offset(pl, offset))
            # mm.wait(0.3)
            qcurr = from_player_angle_offset(mm.get_all_present_position(), offset)
            # Keep reading encoder until motor reach goal position
            k = 0
            while np.max(abs(np.array(qcurr[1:]) - np.array(pl[1:]))) >= 0.05 and k < 100:
                qcurr = from_player_angle_offset(mm.get_all_present_position(), offset)
                k = k + 1
            print(np.max(abs(np.array(qcurr[1:]) - np.array(pl[1:]))))
            print(k)
            print(qcurr)

if __name__ == '__main__':
    # test traj
    test_traj = scipy.io.loadmat('test_traj.mat')
    q_list = test_traj['x']
    print(q_list)

    # initilization
    q_initial = [[-0.43117439, 1.64356286]]
    set_traj_params(50, 10)
    set_pid_gain(600, 0, 10)
    set_all_cmd_position(q_initial)

    # track the traj
    cmd = input('User Command:')
    if cmd == '1':
        set_traj_params(256, 220)
        set_ff_gain(100, 300)
        set_pid_gain(600, 0, 5)
        set_all_cmd_position(q_list)

        exit()