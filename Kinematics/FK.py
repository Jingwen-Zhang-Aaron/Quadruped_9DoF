import numpy as np


def singleLeg_FK(q1, q2, q3, param, output_param):

    # Parameters Definition
    d1 = param[0]
    a1 = param[1]
    a2 = param[2]
    a3 = param[3]
    d4 = param[4]
    
    # Angle Definition
    # Don't have installation zero here, check the python code to retrieve values
    theta_body0 = 0  # installation zero, unit:rad
    theta_coxa0 = 0
    theta_femur0 = 0

    theta_body = q1      #net angle, unit:rad
    theta_coxa = q2
    theta_femur = q3

    theta_body_tot = theta_body + theta_body0
    theta_coxa_tot = theta_coxa + theta_coxa0  #total angle, unit:rad
    theta_femur_tot = theta_femur + theta_femur0

    # DH Parameters
    alpha = [-np.pi/2, np.pi/2, 0, 0]
    a = [0, a1, a2, a3]
    d = [d1, 0, 0, -d4]
    theta = [theta_body_tot, theta_coxa_tot, theta_femur_tot, 0]

    i=0
    T0_1 = np.array([ [np.cos(theta[i]),                    -np.sin(theta[i]),                    0,                   a[i]],
                      [np.sin(theta[i])*np.cos(alpha[i]),   np.cos(theta[i])*np.cos(alpha[i]),   -np.sin(alpha[i]),   -np.sin(alpha[i])*d[i]],
                      [np.sin(theta[i])*np.sin(alpha[i]),   np.cos(theta[i])*np.sin(alpha[i]),    np.cos(alpha[i]),    np.cos(alpha[i])*d[i]],
                      [0,                                   0,                                    0,                   1                    ]])

    i=1
    T1_2 = np.array([ [np.cos(theta[i]), -np.sin(theta[i]), 0, a[i]],
                      [np.sin(theta[i]) * np.cos(alpha[i]), np.cos(theta[i]) * np.cos(alpha[i]), -np.sin(alpha[i]), -np.sin(alpha[i]) * d[i]],
                      [np.sin(theta[i]) * np.sin(alpha[i]), np.cos(theta[i]) * np.sin(alpha[i]), np.cos(alpha[i]),   np.cos(alpha[i]) * d[i]],
                      [0,                                   0,                                   0,                  1]])

    i=2
    T2_3 = np.array(  [[np.cos(theta[i]),                   -np.sin(theta[i]),                     0,                 a[i]],
                      [np.sin(theta[i]) * np.cos(alpha[i]), np.cos(theta[i]) * np.cos(alpha[i]), -np.sin(alpha[i]), -np.sin(alpha[i]) * d[i]],
                      [np.sin(theta[i]) * np.sin(alpha[i]), np.cos(theta[i]) * np.sin(alpha[i]),  np.cos(alpha[i]),  np.cos(alpha[i]) * d[i]],
                      [0,                                   0,                                    0,                 1]])

    i=3
    T3_ef = np.array( [[np.cos(theta[i]),                    -np.sin(theta[i]),                   0,                  a[i]],
                      [np.sin(theta[i]) * np.cos(alpha[i]), np.cos(theta[i]) * np.cos(alpha[i]), -np.sin(alpha[i]), -np.sin(alpha[i]) * d[i]],
                      [np.sin(theta[i]) * np.sin(alpha[i]), np.cos(theta[i]) * np.sin(alpha[i]), np.cos(alpha[i]),   np.cos(alpha[i]) * d[i]],
                      [0,                                   0,                                   0,                  1]])

    # T 0 -> 2
    T0_2 = T0_1.dot(T1_2)
    # T 0 -> 3
    T0_3 = T0_1.dot(T1_2).dot(T2_3)
    # T 0-> ef
    T0_ef = T0_1.dot(T1_2).dot(T2_3).dot(T3_ef)

    ## Plot Leg
    p1_b = T0_1[0:3,3]
    p2_b = T0_2[0:3,3]
    p3_b = T0_3[0:3,3]
    pef_b = T0_ef[0:3,3]

    R0_1 = T0_1[0:3, 0:3]
    R0_ef = T0_ef[0:3, 0:3]

    if output_param == 'T_tot':
        return T0_ef
    elif output_param == 'R0_1':
        return R0_1
    elif output_param == 'R0_ef':
        return R0_ef
    elif output_param == 'pef_b':
        return pef_b



def quad_FK(q_list, param, output_param):
    
    # Define angles
    q1 = q_list[0]
    q2 = q_list[1]
    q3 = q_list[2]
    q4 = q_list[3]
    q5 = q_list[4]
    q6 = q_list[5]
    q7 = q_list[6]
    q8 = q_list[7]
    q9 = q_list[8]
    
    # Define parameters
    d1 = param[0]
    a1 = param[1]
    a2 = param[2]
    a3 = param[3]
    d4 = param[4]
    
    # Right Front (RF) leg
    param = [d1,a1,a2,a3,d4]
    output1 = singleLeg_FK(q1, q2, q3, param, output_param)
    
    # Left Front (LF) leg
    param = [d1,-a1,-a2,-a3,d4]
    output2 = singleLeg_FK(q1, q4, q5, param, output_param)
    
    # Right Back (RB) leg
    param = [-d1,a1,a2,a3,d4]
    output3 = singleLeg_FK(0, q6, q7, param, output_param)
    
    # Left Back (LB) leg
    param = [-d1,-a1,-a2,-a3,d4]
    output4 = singleLeg_FK(0, q8, q9, param, output_param)
    
    return output1, output2, output3, output4

# test
# q_list = [0,-np.pi/4,np.pi/2,np.pi/4,-np.pi/2,-np.pi/4,np.pi/2,np.pi/4,-np.pi/2]
# param = [300,100,270,270,283]
# output_param = 'pef_b'
# output1, output2, output3, output4 = quad_FK(q_list, param, output_param)
# print(output1)
# print(output2)
# print(output3)
# print(output4)
