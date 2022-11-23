#!/usr/bin/python3
import numpy as np
import math
from HW2_utils import FKHW2

'''
    Name:   <Jarunyawat> <Buasri> <05>
            <Thanakrit> <Meangsuwan> <20>
    Date:   <25-10-22>
    Description:
'''

# Question 1
def endEffectorJacobianHW2(q):
    '''
        q : format list 1x3 [[i_11, i_12, i_13]]
        q unit: rad
        type something here

        return format list 6x3
        [ [i_11, i_12, i_13],
          [i_21, i_22, i_23],
          [i_31, i_32, i_33],
          [i_41, i_42, i_43],
          [i_51, i_52, i_53],
          [i_61, i_62, i_63] ]
    '''
    Jw = list()
    Jv = list()
    R,P,R_e,p_e = FKHW2(q)
    for j in range(3):
        #Jw คือ unit vector ที่แสดงทิศทางของ joint ที่จะหมุนเทียบกับ global frame โดยที่ทุก joint เป็น revolute ่joint
        #ดังนั้นทิศทางหมุนบน local frame คือ [0,0,1] และต้องใช้ rotation matrix ในการแปลงให้ไปอยู่ใน global frame
        Jw.append(R[:,:,j] @ np.array([0,0,1]).T)
        #Jv คือทิศทางความเร็วเชิงเส้นของจุด origin บน global frame เมื่อเทียบจากการหมุนของจุดบน joint ต่างๆ
        Jv.append(np.cross(R[:,:,j] @ np.array([0,0,1]).T, (p_e-P[:,j])))
    Je = np.vstack((np.array(Jw).T,np.array(Jv).T))
    return Je

# Question 2
def checkSingularityHW2(q):
    '''
        q : format list 1x3 [[i_11, i_12, i_13]]
        q unit: rad
        type something here
        
        return format bool
    '''
    Je = endEffectorJacobianHW2(q)
    #เนื่องจากหุ่นยนต์มี 3 joint จึงเลือกใช้ jacobian 3 ล่าง ในการทำ j_star เพื่อนำไปหา determinant
    J_star = np.array(Je[3:,:])
    #เมื่อค่า absolute ของ determinant ของ J_star มีค่าเข้าใกล้ 0 จะทำให้หุ่นยนต์เกิสภาวะ singularity
    if np.abs(np.linalg.det(J_star)) <= 0.001:
        return True
    return False

# Question 3
def computeEffortHW2(q,w):
    '''
        q : format list 1x3 [[i_11, i_12, i_13]]
        q unit: rad
        type something here

        return format list 1x3
        [ [i_11, i_12, i_13] ]
    '''
    Je = endEffectorJacobianHW2(q)
    #หา rotation matrix ของ end effector เทียบ global frame
    R,P,R_e,p_e = FKHW2(q)
    #แปลง moment ใน end effector frame ให้อยู่ใน global frame
    n_0 = R_e @ w[:3]
    #แปลง force ใน end effector frame ให้อยู่ใน global frame
    f_0 = R_e @ w[3:]
    #สร้าง wrench
    w_0 = np.hstack((n_0,f_0))
    #หา torque ที่ต้องใช้ในแต่ละ joint
    tau = Je.T @ w_0
    return tau