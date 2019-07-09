from math import pi
import numpy as np
from Kinematics.IK import quad_IK_xyz, quad_IK_xyza, Rz
from Kinematics.IK import getBodyOrientation


def move_updated(pos_list_initial, pos_list_end, angle):

    # Define parameters
    d1 = 300
    a1 = 100
    a2 = 270
    a3 = 270
    d4 = 283
    param = [d1, a1, a2, a3, d4]

    # Boundary Conditions
    pos_list1 = pos_list_initial.copy()
    pb0 = np.array([np.mean(pos_list1[0, :]), np.mean(pos_list1[1, :]), np.mean(pos_list1[2, :])])
    pb6 = np.array([np.mean(pos_list_end[0, :]), np.mean(pos_list_end[1, :]), np.mean(pos_list_end[2, :])])
    pb = pb6 - pb0

    pos_list2 = np.hstack((pos_list_end[0:3, 0:1], pos_list1[0:3, 1:4]))
    pos_list3 = np.hstack((pos_list_end[0:3, 0:2], pos_list1[0:3, 2:4]))
    pos_list4 = np.hstack((pos_list_end[0:3, 0:3], pos_list1[0:3, 3:4]))
    pos_list5 = pos_list_end[0:3, 0:4]

    pb1 = pb0 + pb/5 * 0 + np.array([0, 0, d4])

    move_len = 105

    # move body to lift RF leg (1)
    move_dir1 = np.array([pos_list1[1, 2]-pos_list1[1, 1], -(pos_list1[0, 2]-pos_list1[0, 1]), 0])
    move_dir1 = move_dir1 / np.linalg.norm(move_dir1)
    pb2 = pb0 + pb/5 * 1 + move_dir1 * move_len + np.array([0, 0, d4])
    # move body to lift LF leg (2)
    move_dir2 = np.array([-(pos_list2[1, 3] - pos_list2[1, 0]), pos_list2[0, 3] - pos_list2[0, 0], 0])
    move_dir2 = 1.53 * move_dir2 / np.linalg.norm(move_dir2)
    pb3 = pb0 + pb / 5 * 2 + move_dir2 * move_len + np.array([0, 0, d4])
    # move body to lift RB leg (3)
    move_dir3 = np.array([pos_list3[1, 3] - pos_list3[1, 0], -(pos_list3[0, 3] - pos_list3[0, 0]), 0])
    move_dir3 = move_dir3 / np.linalg.norm(move_dir3)
    pb4 = pb0 + pb / 5 * 3 + move_dir3 * move_len + np.array([0, 0, d4])
    # move body to lift LB leg (4)
    move_dir4 = np.array([-(pos_list4[1, 2] - pos_list4[1, 1]), pos_list4[0, 2] - pos_list4[0, 1], 0])
    move_dir4 = move_dir4 / np.linalg.norm(move_dir4)
    pb5 = pb0 + pb / 5 * 4 + move_dir4 * move_len + np.array([0, 0, d4])

    pb6 = pb0 + pb + np.array([0, 0, d4])

    output_params = [1, -1, -1, 1]
    output_list1, T_BW1, errors1 = quad_IK_xyz(pb2, 0, pos_list1, param, output_params)
    output_list2, T_BW2, errors2 = quad_IK_xyz(pb2, angle/4, pos_list2, param, output_params)

    output_list3, T_BW3, errors3 = quad_IK_xyz(pb3, angle/4, pos_list2, param, output_params)
    output_list4, T_BW4, errors4 = quad_IK_xyz(pb3, angle/4*2, pos_list3, param, output_params)

    output_list5, T_BW5, errors5 = quad_IK_xyz(pb4, angle/4*2, pos_list3, param, output_params)
    output_list6, T_BW6, errors6 = quad_IK_xyz(pb4, angle/4*3, pos_list4, param, output_params)

    output_list7, T_BW7, errors7 = quad_IK_xyz(pb5, angle/4*3, pos_list4, param, output_params)
    output_list8, T_BW8, errors8 = quad_IK_xyz(pb5, angle/4*4, pos_list5, param, output_params)

    # Leg lifting height
    height = 85   # 100
    # number of way points
    num = 10
    # Initialization
    q_list = []
    T_BW = []
    pb_list = []

    # move CoM
    frame = np.arange(num+1)
    for i in frame:
        pb = pb1 + (pb2 - pb1) / frame[-1] * i
        output_list, T, errors = quad_IK_xyz(pb, 0, pos_list1, param, output_params)
        q = np.hstack((output_list[0, :], output_list[1, 1:3]))
        q = np.hstack((q, output_list[2, 1:3]))
        q = np.hstack((q, output_list[3, 1:3]))
        q_list.append(q)
        T_BW.append(T)
        pb_list.append(pb)

    # Swing RF leg
    swingleg = 1
    frame = np.arange(num+1)
    for i in frame:
        t = pi/frame[-1] * i
        z = d4 + height * np.sin(pi - t)
        pb = np.hstack((pb2[0:2], z))
        yaw = 0 + (angle/4 - 0) / frame[-1] * i
        output_list, T, errors = quad_IK_xyza(pb, yaw, pos_list1, swingleg, param, output_params)

        if i < num / 2:
            output_list[swingleg - 1, 1:3] = output_list1[swingleg - 1, 1:3]
        else:
            output_list[swingleg - 1, 1:3] = output_list2[swingleg - 1, 1:3]

        # output_list[swingleg-1, 1:3] = output_list1[swingleg-1, 1:3] + (output_list2[swingleg-1, 1:3] -
        #                                                                 output_list1[swingleg-1, 1:3])/frame[-1]*i
        q = np.hstack((output_list[0, :], output_list[1, 1:3]))
        q = np.hstack((q, output_list[2, 1:3]))
        q = np.hstack((q, output_list[3, 1:3]))
        q_list.append(q)
        T_BW.append(T)
        pb_list.append(pb)

    # Move CoM
    frame = np.arange(num+1)
    for i in frame:
        pb = pb2 + (pb3 - pb2) / frame[-1] * i
        output_list, T, errors = quad_IK_xyz(pb, yaw, pos_list2, param, output_params)
        q = np.hstack((output_list[0, :], output_list[1, 1:3]))
        q = np.hstack((q, output_list[2, 1:3]))
        q = np.hstack((q, output_list[3, 1:3]))
        q_list.append(q)
        T_BW.append(T)
        pb_list.append(pb)

    # Swing RL leg
    swingleg = 2
    frame = np.arange(num+1)
    for i in frame:
        t = pi/frame[-1] * i
        z = d4 + height * np.sin(pi - t)
        pb = np.hstack((pb3[0:2], z))
        yaw = angle/4 + (angle/2 - angle/4) / frame[-1] * i
        output_list, T, errors = quad_IK_xyza(pb, yaw, pos_list2, swingleg, param, output_params)

        if i < num / 2:
            output_list[swingleg - 1, 1:3] = output_list3[swingleg - 1, 1:3]
        else:
            output_list[swingleg - 1, 1:3] = output_list4[swingleg - 1, 1:3]

        # output_list[swingleg-1, 1:3] = output_list3[swingleg-1, 1:3] + (output_list4[swingleg-1, 1:3] -
        #                                                                 output_list3[swingleg-1, 1:3])/frame[-1]*i
        q = np.hstack((output_list[0, :], output_list[1, 1:3]))
        q = np.hstack((q, output_list[2, 1:3]))
        q = np.hstack((q, output_list[3, 1:3]))
        q_list.append(q)
        T_BW.append(T)
        pb_list.append(pb)

    # Move CoM
    frame = np.arange(num+1)
    for i in frame:
        pb = pb3 + (pb4 - pb3) / frame[-1] * i
        output_list, T, errors = quad_IK_xyz(pb, yaw, pos_list3, param, output_params)
        q = np.hstack((output_list[0, :], output_list[1, 1:3]))
        q = np.hstack((q, output_list[2, 1:3]))
        q = np.hstack((q, output_list[3, 1:3]))
        q_list.append(q)
        T_BW.append(T)
        pb_list.append(pb)

    # Swing BR leg
    swingleg = 3
    pitch0, yaw0 = getBodyOrientation(T_BW5, 0, pos_list3, -1)
    pitch1, yaw1 = getBodyOrientation(T_BW6, 0, pos_list4, -1)
    pitch2, yaw2 = getBodyOrientation(T_BW8, 0, pos_list5, -1)
    frame = np.arange(num+1)
    for i in frame:
        t = pi/frame[-1] * i
        z = d4 + height * np.sin(pi - t)
        pb = np.hstack((pb4[0:2], z))
        yaw = yaw0 + (yaw1 - yaw0) / frame[-1] * i
        output_list, T, errors = quad_IK_xyza(pb, yaw, pos_list3, swingleg, param, output_params)

        if i < num / 2:
            output_list[swingleg - 1, 1:3] = output_list5[swingleg - 1, 1:3]
        else:
            output_list[swingleg - 1, 1:3] = output_list6[swingleg - 1, 1:3]

        # output_list[swingleg-1, 1:3] = output_list5[swingleg-1, 1:3] + (output_list6[swingleg-1, 1:3] -
        #                                                                 output_list5[swingleg-1, 1:3])/frame[-1]*i
        q = np.hstack((output_list[0, :], output_list[1, 1:3]))
        q = np.hstack((q, output_list[2, 1:3]))
        q = np.hstack((q, output_list[3, 1:3]))
        q_list.append(q)
        T_BW.append(T)
        pb_list.append(pb)

    # Move CoM
    frame = np.arange(num+1)
    for i in frame:
        pb = pb4 + (pb5 - pb4) / frame[-1] * i
        output_list, T, errors = quad_IK_xyz(pb, angle/4*3, pos_list4, param, output_params)
        q = np.hstack((output_list[0, :], output_list[1, 1:3]))
        q = np.hstack((q, output_list[2, 1:3]))
        q = np.hstack((q, output_list[3, 1:3]))
        q_list.append(q)
        T_BW.append(T)
        pb_list.append(pb)

    # Swing BL leg
    swingleg = 4
    frame = np.arange(num+1)
    for i in frame:
        t = pi/frame[-1] * i
        z = d4 + height * np.sin(pi - t)
        pb = np.hstack((pb5[0:2], z))
        yaw = yaw1 + (yaw2 - yaw1) / frame[-1] * i
        output_list, T, errors = quad_IK_xyza(pb, yaw, pos_list4, swingleg, param, output_params)

        if i < num / 2:
            output_list[swingleg - 1, 1:3] = output_list7[swingleg - 1, 1:3]
        else:
            output_list[swingleg - 1, 1:3] = output_list8[swingleg - 1, 1:3]

        # output_list[swingleg-1, 1:3] = output_list7[swingleg-1, 1:3] + (output_list8[swingleg-1, 1:3] -
        #                                                                 output_list7[swingleg-1, 1:3])/frame[-1]*i
        q = np.hstack((output_list[0, :], output_list[1, 1:3]))
        q = np.hstack((q, output_list[2, 1:3]))
        q = np.hstack((q, output_list[3, 1:3]))
        q_list.append(q)
        T_BW.append(T)
        pb_list.append(pb)

    # Move CoM
    frame = np.arange(num+1)
    for i in frame:
        pb = pb5 + (pb6 - pb5) / frame[-1] * i
        output_list, T, errors = quad_IK_xyz(pb, angle, pos_list5, param, output_params)
        q = np.hstack((output_list[0, :], output_list[1, 1:3]))
        q = np.hstack((q, output_list[2, 1:3]))
        q = np.hstack((q, output_list[3, 1:3]))
        q_list.append(q)
        T_BW.append(T)
        pb_list.append(pb)


    return q_list