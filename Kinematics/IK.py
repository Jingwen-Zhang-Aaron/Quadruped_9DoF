import numpy as np


def allLegs_IK(pos_list, IKlegs, param, output_params):
    # Parameters Definition
    d1 = param[0]
    a1 = param[1]
    a2 = param[2]
    a3 = param[3]
    d4 = param[4]

    # define toe positions
    toe_RF = pos_list[:, 0]
    toe_LF = pos_list[:, 1]
    toe_RB = pos_list[:, 2]
    toe_LB = pos_list[:, 3]

    errors = [0, 0, 0, 0, 0]

    # RF leg
    if IKlegs[0] > 0:
        X = toe_RF[0]
        Y = toe_RF[1]
        Z = toe_RF[2]
        L = np.sqrt(X ** 2 + Z ** 2 - d4 ** 2)
        body = np.arctan2(- d4 * X - L * Z, L * X - d4 * Z)
        cos_value = ((L - a1) ** 2 + (Y - d1) ** 2 - a2 ** 2 - a3 ** 2) / 2 / a2 / a3
        if abs(cos_value) > 1:
            errors[0] = 1
            output_list1 = [np.nan, np.nan, np.nan]
        else:
            sin_value = np.sqrt(1 - cos_value ** 2) * np.sign(output_params[0])
            femur = np.arctan2(sin_value, cos_value)
            a = a2 + a3 * np.cos(femur)
            c = -a3 * np.sin(femur)
            d = L - a1
            e = a3 * np.sin(femur)
            f = a2 + a3 * np.cos(femur)
            g = Y - d1
            coxa = np.arctan2(a * g - d * e, d * f - c * g)
            if a * f - c * e < 0:
                coxa = coxa + np.pi
            output_list1 = [body, coxa, femur]
    else:
        output_list1 = [0, 0, 0]

    # LF leg
    if IKlegs[1] > 0:
        X = toe_LF[0]
        Y = toe_LF[1]
        Z = toe_LF[2]
        L = np.sqrt(X ** 2 + Z ** 2 - d4 ** 2)
        body = np.arctan2(L * Z - d4 * X, - d4 * Z - L * X)
        cos_value = ((L - a1) ** 2 + (Y - d1) ** 2 - a2 ** 2 - a3 ** 2) / 2 / a2 / a3
        if abs(cos_value) > 1:
            errors[1] = 1
            output_list2 = [np.nan, np.nan, np.nan]
        else:
            sin_value = np.sqrt(1 - cos_value ** 2) * np.sign(output_params[1])
            femur = np.arctan2(sin_value, cos_value)
            a = a2 + a3 * np.cos(femur)
            c = -a3 * np.sin(femur)
            d = L - a1
            e = a3 * np.sin(femur)
            f = a2 + a3 * np.cos(femur)
            g = d1 - Y
            coxa = np.arctan2(a * g - d * e, d * f - c * g)
            if a * f - c * e < 0:
                coxa = coxa + np.pi
            output_list2 = [body, coxa, femur]
    else:
        output_list2 = [0, 0, 0]

    # RB leg
    if IKlegs[2] > 0:
        X = toe_RB[0]
        Y = toe_RB[1]
        Z = toe_RB[2]
        body = 0
        cos_value = ((X - a1) ** 2 + (Y + d1) ** 2 - a2 ** 2 - a3 ** 2) / 2 / a2 / a3
        if abs(cos_value) > 1:
            errors[2] = 1
            output_list3 = [np.nan, np.nan, np.nan]
        else:
            sin_value = np.sqrt(1 - cos_value ** 2) * np.sign(output_params[2])
            femur = np.arctan2(sin_value, cos_value)
            a = a2 + a3 * np.cos(femur)
            c = -a3 * np.sin(femur)
            d = X - a1
            e = a3 * np.sin(femur)
            f = a2 + a3 * np.cos(femur)
            g = Y + d1
            coxa = np.arctan2(a * g - d * e, d * f - c * g)
            if a * f - c * e < 0:
                coxa = coxa + np.pi
            output_list3 = [body, coxa, femur]
    else:
        output_list3 = [0, 0, 0]

    # LB leg
    if IKlegs[3] > 0:
        X = toe_LB[0]
        Y = toe_LB[1]
        Z = toe_LB[2]

        body = 0
        cos_value = ((X + a1) ** 2 + (Y + d1) ** 2 - a2 ** 2 - a3 ** 2) / 2 / a2 / a3
        if abs(cos_value) > 1:
            errors[3] = 1
            output_list4 = [np.nan, np.nan, np.nan]
        else:
            sin_value = np.sqrt(1 - cos_value ** 2) * np.sign(output_params[3])
            femur = np.arctan2(sin_value, cos_value)
            a = a2 + a3 * np.cos(femur)
            c = -a3 * np.sin(femur)
            d = - X - a1
            e = a3 * np.sin(femur)
            f = a2 + a3 * np.cos(femur)
            g = - Y - d1
            coxa = np.arctan2(a * g - d * e, d * f - c * g)
            if a * f - c * e < 0:
                coxa = coxa + np.pi
            output_list4 = [body, coxa, femur]
    else:
        output_list4 = [0, 0, 0]

    ep = 1e-6
    if abs(output_list1[0] - output_list2[0]) > ep and np.sum(np.array(IKlegs) > 0) == 4:
        errors[4] = 1

    return output_list1, output_list2, output_list3, output_list4, errors


def quad_IK_xyz(pb, yaw, pos_list, param, output_params):
    d4 = param[4]

    p1 = pos_list[:, 0]
    p2 = pos_list[:, 1]
    p3 = pos_list[:, 2]
    p4 = pos_list[:, 3]

    p12 = p1 - p2
    p34 = p3 - p4
    p13 = p1 - p3
    pb1 = pb - p1
    pb3 = pb - p3

    r1 = np.arctan2(p34[1], p34[0])
    r2 = np.arcsin(p34[2] / np.linalg.norm(p34))
    R = np.dot(Rz(r1), Ry(r2))

    # f1*np.cos(pitch) - f2*np.sin(pitch) = d4
    f1 = R[:, 2].dot(pb3)
    f2 = R[:, 1].dot(pb3)
    if f1 ** 2 + f2 ** 2 >= d4 ** 2:
        pitch = np.arctan2(np.sqrt(f1 ** 2 + f2 ** 2 - d4 ** 2), d4) + np.arctan2(-f2, f1)
        if np.abs(pitch) > np.pi / 2:
            pitch = np.arctan2(-np.sqrt(f1 ** 2 + f2 ** 2 - d4 ** 2), d4) + np.arctan2(-f2, f1)

    g1 = R[:, 0].dot(p12)
    g2 = (R[:, 1] * np.cos(pitch) + R[:, 2] * np.sin(pitch)).dot(p12)
    g3 = (R[:, 2] * np.cos(pitch) - R[:, 1] * np.sin(pitch)).dot(p12)

    g4 = R[:, 0].dot(pb1)
    g5 = (R[:, 1] * np.cos(pitch) + R[:, 2] * np.sin(pitch)).dot(pb1)
    g6 = (R[:, 2] * np.cos(pitch) - R[:, 1] * np.sin(pitch)).dot(pb1)

    h1 = g3 ** 2 * (g4 ** 2 + g5 ** 2) - 2 * g1 * g3 * g4 * g6 - 2 * g2 * g5 * (g1 * g4 + g3 * g6) + g2 ** 2 * (
            g4 ** 2 + g6 ** 2) + g1 ** 2 * (g5 ** 2 + g6 ** 2)
    h2 = 2 * d4 * (g1 * g3 * g4 - g1 ** 2 * g6 + g2 * (g3 * g5 - g2 * g6))
    h3 = d4 ** 2 * (g1 ** 2 + g2 ** 2) - (g2 * g4 - g1 * g5) ** 2

    delta = h2 ** 2 - 4 * h1 * h3

    if delta >= 0:
        z = (- h2 + np.sqrt(delta)) / (2 * h1)
        x = (d4 * g2 + (g3 * g5 - g2 * g6) * z) / (g2 * g4 - g1 * g5)
        y = (d4 * g1 + (g3 * g4 - g1 * g6) * z) / (- g2 * g4 + g1 * g5)
        if x <= 1e-6:
            x = 0
        if y <= 1e-6:
            y = 0

    # q1 = np.arccos(z) * np.sign(x)  # q1 has the same sign with x since yaw < 90 deg
    if np.abs(z - 1) >= 1e-6:
        if z > 1:
            z = 1
        q1 = np.arccos(z) * np.sign(x)  # q1 has the same sign with x since yaw < 90 deg
        yaw = np.arctan2(y * np.sign(q1), x * np.sign(q1))
        R_BW = R.dot(Rx(pitch)).dot(Rz(yaw))
    else:
        R_BW = Rz(yaw)

    T_BW = np.block([[R_BW, np.array([pb]).transpose()], [0, 0, 0, 1]])
    pos_list = np.dot(np.linalg.inv(T_BW), np.block([[pos_list], [1, 1, 1, 1]]))

    IKlegs = [1, 1, 1, 1]
    output_list1, output_list2, output_list3, output_list4, errors = allLegs_IK(pos_list, IKlegs, param, output_params)

    output_list = np.array([output_list1, output_list2, output_list3, output_list4])

    return output_list, T_BW, errors


def quad_IK_xyza(pb, yaw, pos_list, swingleg, param, output_params):
    d4 = param[4]

    flag = 0
    if swingleg > 2:
        pos_list = np.fliplr(pos_list)
        swingleg = 5 - swingleg
        flag = 1

    p1 = pos_list[:, 0]
    p2 = pos_list[:, 1]
    p3 = pos_list[:, 2]
    p4 = pos_list[:, 3]

    p12 = p1 - p2
    p34 = p3 - p4
    p13 = p1 - p3
    pb1 = pb - p1
    pb2 = pb - p2
    pb3 = pb - p3

    r1 = np.arctan2(p34[1], p34[0])
    r2 = np.arcsin(p34[2] / np.linalg.norm(p34))
    R = np.dot(Rz(r1), Ry(r2))

    # f1*np.cos(pitch) - f2*np.sin(pitch) = d4
    f1 = R[:, 2].dot(pb3)
    f2 = R[:, 1].dot(pb3)
    if f1 ** 2 + f2 ** 2 >= d4 ** 2:
        pitch = np.arctan2(np.sqrt(f1 ** 2 + f2 ** 2 - d4 ** 2), d4) + np.arctan2(-f2, f1)

    # R_BW = np.dot(R, np.dot(Rx(pitch), Rz(yaw)))
    R_BW = R.dot(Rx(pitch)).dot(Rz(yaw))

    xb = R_BW[:, 0]
    zb = R_BW[:, 2]
    if swingleg == 1:
        a = xb.dot(pb2)
        b = zb.dot(pb2)
        q1 = np.arctan2(-np.sqrt(a ** 2 + b ** 2 - d4 ** 2), d4) + np.arctan2(a, b)
        if np.abs(q1) > np.pi / 2:
            q1 = np.arctan2(np.sqrt(a ** 2 + b ** 2 - d4 ** 2), d4) + np.arctan2(a, b)
    elif swingleg == 2:
        a = xb.dot(pb1)
        b = zb.dot(pb1)
        q1 = np.arctan2(-np.sqrt(a ** 2 + b ** 2 - d4 ** 2), d4) + np.arctan2(a, b)
        if np.abs(q1) > np.pi / 2:
            q1 = np.arctan2(np.sqrt(a ** 2 + b ** 2 - d4 ** 2), d4) + np.arctan2(a, b)

    T_BW = np.block([[R_BW, np.array([pb]).transpose()], [0, 0, 0, 1]])
    pos_list = np.dot(np.linalg.inv(T_BW), np.block([[pos_list], [1, 1, 1, 1]]))

    IKlegs = [1, 1, 1, 1]
    IKlegs[swingleg - 1] = 0
    output_list1, output_list2, output_list3, output_list4, errors = allLegs_IK(pos_list, IKlegs, param, output_params)

    output_list = np.array([output_list1, output_list2, output_list3, output_list4])

    if flag > 0:
        output_list = np.flipud(output_list)
        # R_BW = np.dot(R_BW, np.dot(Ry(q1), Rz(np.pi)))
        R_BW = R_BW.dot(Ry(q1)).dot(Rz(np.pi))
        T_BW = np.block([[R_BW, np.array([pb]).transpose()], [0, 0, 0, 1]])

    output_list[0, 0] = q1
    return output_list, T_BW, errors


def getBodyOrientation(T_BW, q1, pos_list, flag):
    # flag determines which two legs are back legs
    if flag < 0:
        pos_list = np.fliplr(pos_list)
        T_BW[0:3, 0:3] = np.dot(T_BW[0:3, 0:3], np.dot(Ry(q1), Rz(np.pi)))

    p3 = pos_list[0:3, 2]
    p4 = pos_list[0:3, 3]

    p34 = p3 - p4

    r1 = np.arctan2(p34[1], p34[0])
    r2 = np.arcsin(p34[2] / np.linalg.norm(p34))
    R = np.dot(np.linalg.inv(np.dot(Rz(r1), Ry(r2))), T_BW[0:3, 0:3])

    pitch = np.arctan2(-R[1, 2], R[2, 2])
    yaw = np.arctan2(-R[0, 1], R[0, 0])
    return pitch, yaw


def Rx(t):
    return np.array([[1, 0, 0], [0, np.cos(t), -np.sin(t)], [0, np.sin(t), np.cos(t)]])


def Ry(t):
    return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])


def Rz(t):
    return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])

# test IK
# param = [300,100,270,270,283]
# d = 283
# pos_list = np.array([[100+270*np.sqrt(2), -270*np.sqrt(2)-100, 100+270*np.sqrt(2), -100-270*np.sqrt(2)],
#                      [300, 300, -300, -300],
#                      [-283, -283, -283, -283]])
# pb = np.array([0, 0, 0])
# output_params = [1, -1, -1, 1]
# yaw = 0
# output_list, T_BW, errors = quad_IK_xyz(pb, yaw, pos_list, param, output_params)
# print(output_list)
# print(T_BW)

# test IK
# param = [300,100,270,270,283]
# d = 283
# pos_list = np.array([[100+270*np.sqrt(2), -270*np.sqrt(2)-100, 100+270*np.sqrt(2), -100-270*np.sqrt(2)],
#                      [300, 300, -300, -300],
#                      [-283, -283, -283, -283]])
# pb = np.array([0, 0, 0])
# output_params = [1, -1, -1, 1]
# yaw = 0
# output_list, T_BW, errors = quad_IK_xyza(pb, yaw, pos_list, 1, param, output_params)
# print(output_list)
# print(T_BW)
