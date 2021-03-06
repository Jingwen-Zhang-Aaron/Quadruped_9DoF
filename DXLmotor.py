from robot_player import MotionManager, DxlOptions
from robot_player.motion_manager import AngleOffset
from robot_player import to_player_angle_offset, from_player_angle_offset
from robot_player.dxl.dxl_control_table import XSERIES
from time import sleep
import platform
from math import pi
import numpy as np

ids = [1, 2, 3, 4, 5, 6, 7, 8, 9]
motor_ids = [[1, 2, 3, 4, 5, 6, 7, 8, 9]]

if platform.system() == 'Windows':
    ports = ['COM15', 'COM16']
else:
    ports = ['/dev/ttyUSB0']

dopts = DxlOptions(motor_ids,
                   motor_types=['XSERIES'],
                   ports=ports,
                   baudrate=3000000,
                   protocol_version=2)

offset = AngleOffset(trans=[1]*9, offset=[pi-0.055, pi, pi, pi, pi, pi, pi, pi, pi])

def set_traj_params(maxvel=0, maxaccel=0):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # Set trapezoidal profile parameters
        # 0 means infinite velocity and acceleration
        mm.device.set_torque_enable(ids, [0] * 9)
        mm.device._write_data(ids, XSERIES.PROFILE_VELOCITY, [maxvel] * 9, 4)
        mm.device._write_data(ids, XSERIES.PROFILE_ACCELERATION, [maxaccel] * 9, 4)
        mm.device.set_torque_enable(ids, [1] * 9)



def set_pid_gain(Pgain=800, Igain=0, Dgain=0):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # Set position PID gain
        mm.device.set_torque_enable(ids, [0] * 9)
        mm.device._write_data(ids, XSERIES.POSITION_P_GAIN, [Pgain] * 9, 2)
        mm.device._write_data(ids, XSERIES.POSITION_I_GAIN, [Igain] * 9, 2)
        mm.device._write_data(ids, XSERIES.POSITION_D_GAIN, [Dgain] * 9, 2)
        mm.device.set_torque_enable(ids, [1] * 9)


def set_ff_gain(Gain1=0, Gain2=0):
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # Set position Feedforward gain
        mm.device.set_torque_enable(ids, [0] * 9)
        mm.device._write_data(ids, XSERIES.FEEDFORWARD_1ST_GAIN, [Gain1] * 9, 2)
        mm.device._write_data(ids, XSERIES.FEEDFORWARD_2ND_GAIN, [Gain2] * 9, 2)
        mm.device.set_torque_enable(ids, [1] * 9)


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
            while np.max(abs(np.array(qcurr[1:]) - np.array(pl[1:]))) >= 0.05 and k < 50:
                qcurr = from_player_angle_offset(mm.get_all_present_position(), offset)
                k = k + 1
            print(qcurr)