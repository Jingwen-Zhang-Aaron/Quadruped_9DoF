
# while 1:
#     a = input('CMD: ')
#     if a == 'haha':
#         print('haha')
#     if a == 'no':
#         print('no')
#     if a == 'break':
#         break
#
# print('Its over')

from robot_player import MotionManager, DxlOptions
from robot_player.motion_manager import AngleOffset
from robot_player import to_player_angle_offset, from_player_angle_offset
from robot_player.dxl.dxl_control_table import XSERIES
import time
import platform
from math import pi
import numpy as np
"""
test for chain of X series dynamixels, 2 on a single chain plugged into each port.
Baudrate is 3000000
ID is [2,3]
"""

def allclose(l1, l2, tol=2.):
    for i, j in zip(l1, l2):
        if abs(i - j) > tol:
            raise AssertionError("elements of lists are {}, which is not within tolerance {}".format(abs(i - j), tol))
    return True

ids = [1]
motor_ids = [[1]]

if platform.system() == 'Windows':
    ports = ['COM15','COM16']
else:
    ports = ['/dev/ttyUSB0']

dopts = DxlOptions(motor_ids,
                   motor_types=['XSERIES'],
                   ports=ports,
                   baudrate=57600,
                   protocol_version=2)

offset = AngleOffset(trans=[1], offset=[pi])

def test_set_command_position():
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # checking that the set_all/get_all position commands work
        position_list = [[0], [-0.6234],  [1.8903],
                                [0.6234], [-1.8903],
                                [0.6234], [-1.8903],
                                [-0.6234],  [1.8903], [2.1056]]
        for pl in position_list:
            mm.set_goal_position(ids, to_player_angle_offset(pl, offset))
            # mm.wait(0.8)
            qcurr = from_player_angle_offset(mm.get_all_present_position(), offset)
            counter_time = 0
            k = 0
            # while np.max(abs(np.array(qcurr) - np.array(pl))) >= 0.005:
            while k < 500:
                start = time.time()
                qcurr = from_player_angle_offset(mm.get_all_present_position(), offset)
                end_time = time.time() - start
                counter_time += end_time
                k = k + 1
            # print(qcurr)
            print(qcurr)
            print(counter_time)
            print(counter_time/500)


def set_traj_params(maxvel=0, maxaccel=0):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # Set trapezoidal profile parameters
        # 0 means infinite velocity and acceleration
        mm.device.set_torque_enable(ids, [0] * 1)
        mm.device._write_data(ids, XSERIES.PROFILE_VELOCITY, [maxvel] * 1, 4)
        mm.device._write_data(ids, XSERIES.PROFILE_ACCELERATION, [maxaccel] * 1, 4)
        mm.device.set_torque_enable(ids, [1] * 1)

def set_pid_gain(Pgain=800, Igain=0, Dgain=0):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # Set position PID gain
        mm.device.set_torque_enable(ids, [0] * 1)
        mm.device._write_data(ids, XSERIES.POSITION_P_GAIN, [Pgain] * 1, 2)
        mm.device._write_data(ids, XSERIES.POSITION_I_GAIN, [Igain] * 1, 2)
        mm.device._write_data(ids, XSERIES.POSITION_D_GAIN, [Dgain] * 1, 2)
        mm.device.set_torque_enable(ids, [1] * 1)


def set_ff_gain(Gain1=0, Gain2=0):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # Set position Feedforward gain
        mm.device.set_torque_enable(ids, [0] * 1)
        mm.device._write_data(ids, XSERIES.FEEDFORWARD_1ST_GAIN, [Gain1] * 1, 2)
        mm.device._write_data(ids, XSERIES.FEEDFORWARD_2ND_GAIN, [Gain2] * 1, 2)
        mm.device.set_torque_enable(ids, [1] * 1)

def set_velocity(vel):
    with MotionManager(ids, dt=.005, options=dopts) as mm:
        # vel -128 ~ 128
        # Set velocity as vel*0.229 rev/min
        mm.device.set_torque_enable(ids, [0] * 1)
        mm.device._write_data(ids, XSERIES.GOAL_VELOCITY, [vel] * 1, 4)
        mm.device.set_torque_enable(ids, [1] * 1)

if __name__ == '__main__':

    #set_traj_params(100, 10000)
    #set_traj_params(100, 500)
    set_traj_params(100, 20)
    #set_velocity(1)
    set_pid_gain(600, 0, 10)
    #set_ff_gain(Gain1=2, Gain2=1)
    test_set_command_position()


