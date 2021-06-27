import rclpy
from rclpy.node import Node
import numpy as np
from qpsolvers import solve_qp
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32
import math
import osqp
import time
from scipy.linalg import null_space as nsp
import matplotlib.pyplot as plt

#Doubts to be cleared:
#Initial placement of robots around the object: If two robots are placed initially facing a side, both would move towards the robot's centre
# Mechanism for avoiding robot collision among themselves or with the environment(other than the load) - should it be autonomous or taken care of by the human
#the distance condition for setting flag variable: it is usually dependent on the load's side length???

#fmes_list = []
#f_list = []
#inter = 0
#start_time = 0



p1 = np.zeros((3,1))
p2 = np.zeros((3,1))
p3 = np.zeros((3,1))
p4 = np.zeros((3,1))
p5 = np.zeros((3,1))
p6 = np.zeros((3,1))
f1 = np.zeros((3,1))
f2 = np.zeros((3,1))
f3 = np.zeros((3,1))
f4 = np.zeros((3,1))
f5 = np.zeros((3,1))
f6 = np.zeros((3,1))
q1 = np.zeros((4,1))
q2 = np.zeros((4,1))
q3 = np.zeros((4,1))
q4 = np.zeros((4,1))
q5 = np.zeros((4,1))
q6 = np.zeros((4,1))
loadp = np.zeros((3,1))
loadq = np.zeros((4,1))
flag_list = np.zeros((6,1))
num_robots = 6



res1 = 0
res2 = 0
res3 = 0
res4 = 0
res5 = 0
res6 = 0
flg1 = 0
flg2 = 0
flg3 = 0
flg4 = 0
flg5 = 0
flg6 = 0
posdes1 = np.zeros((3,1))
posdes2 = np.zeros((3,1))
posdes3 = np.zeros((3,1))
posdes4 = np.zeros((3,1))
posdes5 = np.zeros((3,1))
posdes6 = np.zeros((3,1))
rotload1 = np.zeros((3,3))
rotload2 = np.zeros((3,3))
rotload3 = np.zeros((3,3))
rotload4 = np.zeros((3,3))
rotload5 = np.zeros((3,3))
rotload6 = np.zeros((3,3))
thetades1 = 0
thetades2 = 0
thetades3 = 0
thetades4 = 0
thetades5 = 0
thetades6 = 0
xd1 = 0
yd1 = 0
xd2 = 0
yd2 = 0
xd3 = 0
yd3 = 0
xd4 = 0
yd4 = 0

'''H = np.zeros((9,6))
H[0][0] = 1
H[1][1] = 1
H[3][2] = 1
H[4][3] = 1
H[6][4] = 1
H[7][5] = 1
'''
H = np.zeros((12,4))   #pwoF model
H[0][0] = 1
H[3][1] = 1
H[6][2] = 1
H[9][3] = 1

M = np.zeros((6,3))   # HF model
M[0][0] = 1
M[2][1] = 1
M[4][2] = 1

tau_set = np.array([10.0,5.0,0.0]).reshape((3,))


class Control(Node):

    def __init__(self):
        super().__init__('control_node')
        self.velpublisher1_ = self.create_publisher(Float32, 'velocity1', 1)
        self.angvelpublisher1_ = self.create_publisher(Float32, 'angvelocity1',1)
        self.velpublisher2_ = self.create_publisher(Float32, 'velocity2', 1)
        self.angvelpublisher2_ = self.create_publisher(Float32, 'angvelocity2',1)
        self.velpublisher3_ = self.create_publisher(Float32, 'velocity3', 1)
        self.angvelpublisher3_ = self.create_publisher(Float32, 'angvelocity3',1)
        self.velpublisher4_ = self.create_publisher(Float32, 'velocity4', 1)
        self.angvelpublisher4_ = self.create_publisher(Float32, 'angvelocity4',1)
        self.velpublisher5_ = self.create_publisher(Float32, 'velocity5', 1)
        self.angvelpublisher5_ = self.create_publisher(Float32, 'angvelocity5',1)
        self.velpublisher6_ = self.create_publisher(Float32, 'velocity6', 1)
        self.angvelpublisher6_ = self.create_publisher(Float32, 'angvelocity6',1)

        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscriptionf1 = self.create_subscription(
            Vector3,
            'forcevalue1',
            self.listener_callbackf1,
            1)
        self.subscriptionf1  # prevent unused variable warning

        self.subscriptionf2 = self.create_subscription(
            Vector3,
            'forcevalue2',
            self.listener_callbackf2,
            1)
        self.subscriptionf2  # prevent unused variable warning

        self.subscriptionf3 = self.create_subscription(
            Vector3,
            'forcevalue3',
            self.listener_callbackf3,
            1)
        self.subscriptionf3  # prevent unused variable warning

        self.subscriptionf4 = self.create_subscription(
            Vector3,
            'forcevalue4',
            self.listener_callbackf4,
            1)
        self.subscriptionf4

        self.subscriptionf5 = self.create_subscription(
            Vector3,
            'forcevalue5',
            self.listener_callbackf5,
            1)
        self.subscriptionf5

        self.subscriptionf6 = self.create_subscription(
            Vector3,
            'forcevalue6',
            self.listener_callbackf6,
            1)
        self.subscriptionf6

        self.subscriptionr1 = self.create_subscription(
            Int32,
            'res1',
            self.listener_callbackr1,
            1)
        self.subscriptionr1

        self.subscriptionr2 = self.create_subscription(
            Int32,
            'res2',
            self.listener_callbackr2,
            1)
        self.subscriptionr2

        self.subscriptionr3 = self.create_subscription(
            Int32,
            'res3',
            self.listener_callbackr3,
            1)
        self.subscriptionr3

        self.subscriptionr4 = self.create_subscription(
            Int32,
            'res4',
            self.listener_callbackr4,
            1)
        self.subscriptionr4

        self.subscriptionr5 = self.create_subscription(
            Int32,
            'res5',
            self.listener_callbackr5,
            1)
        self.subscriptionr5

        self.subscriptionr6 = self.create_subscription(
            Int32,
            'res6',
            self.listener_callbackr6,
            1)
        self.subscriptionr6






        self.subscriptionp1 = self.create_subscription(
            Vector3,
            'pos1',
            self.listener_callbackp1,
            1)
        self.subscriptionp1 

        self.subscriptionp2 = self.create_subscription(
            Vector3,
            'pos2',
            self.listener_callbackp2,
            1)
        self.subscriptionp2

        self.subscriptionp3 = self.create_subscription(
            Vector3,
            'pos3',
            self.listener_callbackp3,
            1)
        self.subscriptionp3

        self.subscriptionp4 = self.create_subscription(
            Vector3,
            'pos4',
            self.listener_callbackp4,
            1)
        self.subscriptionp4

        self.subscriptionp5 = self.create_subscription(
            Vector3,
            'pos5',
            self.listener_callbackp5,
            1)
        self.subscriptionp5

        self.subscriptionp6 = self.create_subscription(
            Vector3,
            'pos6',
            self.listener_callbackp6,
            1)
        self.subscriptionp6

        self.subscriptionloadp = self.create_subscription(
            Vector3,
            'lpos',
            self.listener_callbackloadp,
            1)
        self.subscriptionloadp

        self.subscriptionquatl = self.create_subscription(
            Quaternion,
            'lquat',
            self.listener_callbackquatl,
            1)



        self.subscriptionq1 = self.create_subscription(
            Quaternion,
            'quat1',
            self.listener_callbackq1,
            1)
        self.subscriptionq1

        self.subscriptionq2 = self.create_subscription(
            Quaternion,
            'quat2',
            self.listener_callbackq2,
            1)
        self.subscriptionq2

        self.subscriptionq3 = self.create_subscription(
            Quaternion,
            'quat3',
            self.listener_callbackq3,
            1)
        self.subscriptionq3

        self.subscriptionq4 = self.create_subscription(
            Quaternion,
            'quat4',
            self.listener_callbackq4,
            1)
        self.subscriptionq4

        self.subscriptionq5 = self.create_subscription(
            Quaternion,
            'quat5',
            self.listener_callbackq5,
            1)
        self.subscriptionq5

        self.subscriptionq6 = self.create_subscription(
            Quaternion,
            'quat6',
            self.listener_callbackq6,
            1)
        self.subscriptionq6

    def quaterniontorpy(self,Q):
        quat0 = Q[3]
        quat1 = Q[0]
        quat2 = Q[1]
        quat3 = Q[2]
     
    # First row of the rotation matrix
        r00 = 2 * (quat0 * quat0 + quat1 * quat1) - 1
        r01 = 2 * (quat1 * quat2 - quat0 * quat3)
        r02 = 2 * (quat1 * quat3 + quat0 * quat2)
     
    # Second row of the rotation matrix
        r10 = 2 * (quat1 * quat2 + quat0 * quat3)
        r11 = 2 * (quat0 * quat0 + quat2 * quat2) - 1
        r12 = 2 * (quat2 * quat3 - quat0 * quat1)
     
    # Third row of the rotation matrix
        r20 = 2 * (quat1 * quat3 - quat0 * quat2)
        r21 = 2 * (quat2 * quat3 + quat0 * quat1)
        r22 = 2 * (quat0 * quat0 + quat3 * quat3) - 1
     
    # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
        return rot_matrix


    def dot_prod(self, G, i, j):
        G_i = G[:,i]
        G_j = G[:,j]
        return G_i.T @ G_j

    def gradi(self,cos_list, sin_list, x_list, y_list,i,j):
        '''temp_1 = y_list[i]*y_list[j]*sin_list[i]*cos_list[j]   #for pioneer robot
        temp_2 = x_list[i]*y_list[j]*cos_list[i]*cos_list[j]
        temp_3 = y_list[i]*x_list[j]*sin_list[i]*sin_list[j]
        temp_4 = x_list[i]*x_list[j]*cos_list[i]*sin_list[j]
        grad_i = -sin_list[i]*cos_list[j] + cos_list[i]*sin_list[j] - temp_1 - temp_2 + temp_3 + temp_4'''
        temp_1 = y_list[i]*y_list[j]*sin_list[j]*cos_list[i]      # for kuka robot
        temp_2 = y_list[i]*x_list[j]*cos_list[i]*cos_list[j]
        temp_3 = -x_list[i]*y_list[j]*sin_list[i]*sin_list[j]
        temp_4 = -x_list[i]*x_list[j]*sin_list[i]*cos_list[j]
        grad_i = cos_list[i]*sin_list[j] - sin_list[i]*cos_list[j] + temp_1 + temp_2 + temp_3 + temp_4

        return grad_i

    def grad_descent(self,theta_list,cos_list,sin_list,x_list,y_list,G,i):
        
        '''cur_theta = 0
        rate = 0.001
        iter = 0
        max_iters = 1000
        
        while(iter != max_iters):
            gradient = 0
            cos_list[i] = math.cos(cur_theta)
            sin_list[i] = math.sin(cur_theta)
            
            
            for j in range(4):
                if i!=j:
                    gradient = gradient + self.dot_prod(G,i,j)*self.gradi(cos_list, sin_list, x_list, y_list, i, j)
            
            cur_theta = cur_theta - rate*gradient
            iter = iter+1
        return cur_theta'''
        gradient = 0
        #cur_theta = theta_list[i]
        
        for j in range(4):
                if i!=j:
                    gradient = gradient + self.dot_prod(G,i,j)*self.gradi(cos_list, sin_list, x_list, y_list, i, j)

        #cur_theta = cur_theta - gradient
        return gradient


    def frame_potential(self, G):
        frame_pot = 0
        for i in range(4):
            for j in range(i,4):
                if i!=j:
                    frame_pot = frame_pot + (G[:,i].T @ G[:,j])*(G[:,i].T @ G[:,j])

        return frame_pot


    def gotoPose(self,obj,q):
        K = [0.1,0.1,0.1]
        ex = obj[0] - q[0]
        ey = obj[1] - q[1]
        rho = math.sqrt(ex*ex + ey*ey)
        gamma = math.atan2(ey,ex) - obj[2] + math.pi
        delta = gamma + obj[2] - q[2]
        gamma = -math.pi + math.fmod(gamma - math.pi, 2*math.pi)
        delta = -math.pi + math.fmod(delta - math.pi, 2*math.pi)
        vOmega = [K[0] * rho * math.cos(gamma),
                K[1] * gamma + K[0] * np.sinc(gamma) * math.cos(gamma) * (gamma + K[2]*K[0]*delta)]
        return vOmega



    def gotoPose2(self, obj, q):
        gain_v = 8
        gain_w = 2
        epsilon = 0.03
        v = gain_v*obj[1]
        w = gain_w*math.atan2(obj[0], obj[1])
        diff_ang = q[2] - obj[2]
        diff_ang = math.atan2(math.sin(diff_ang), math.cos(diff_ang))
        if (abs(obj[1])+abs(obj[0]))< epsilon :
            v = 0.0
            w = -6*diff_ang

        vOmega = [v,w]
        return vOmega

  

    



    
        



    def timer_callback(self):
        
        global f1
        global f2
        global f3
        global f4
        global f5
        global f6
        global p1
        global p2
        global p3
        global p4
        global p5
        global p6
        global res1
        global res2
        global res3
        global res4
        global res5
        global res6
        global flg1
        global flg2
        global flg3
        global flg4
        global flg5
        global flg6
        global xd1
        global yd1
        global xd2
        global yd2
        global xd3
        global yd3
        global xd4
        global yd4
        global posdes1
        global posdes2
        global posdes3
        global posdes4
        global posdes5
        global posdes6
        global rotload1
        global rotload2
        global rotload3
        global rotload4
        global rotload5
        global rotload6
        global thetades1
        global thetades2
        global thetades3
        global thetades4
        global thetades5
        global thetades6
        global q1
        global q2
        global q3
        global q4
        global q5
        global q6
        global loadp
        global loadq
        global H
        global tau_set
        global M
        global fmes_list
        global f_list
        global start_time
        global sum
        global flag_list
        global num_robots
        #global inter
        frame_pot = []

        

        msg_vel1 = Float32()
        msg_angvel1 = Float32()
        msg_vel2 = Float32()
        msg_angvel2 = Float32()
        msg_vel3 = Float32()
        msg_angvel3 = Float32()
        msg_vel4 = Float32()
        msg_angvel4 = Float32()
        msg_vel5 = Float32()
        msg_angvel5 = Float32()
        msg_vel6 = Float32()
        msg_angvel6 = Float32()



        

        
        rot1 = self.quaterniontorpy(q1)
        rot2 = self.quaterniontorpy(q2)
        rot3 = self.quaterniontorpy(q3)
        rot4 = self.quaterniontorpy(q4)
        rot5 = self.quaterniontorpy(q5)
        rot6 = self.quaterniontorpy(q6)
        rotl = self.quaterniontorpy(loadq)
        rot1 = rot1.reshape((3,3))
        rot2 = rot2.reshape((3,3))
        rot3 = rot3.reshape((3,3))
        rot4 = rot4.reshape((3,3))
        rot5 = rot5.reshape((3,3))
        rot6 = rot6.reshape((3,3))
        rotl = rotl.reshape((3,3))
        cos_list = [float(rot1[0][0]), float(rot2[0][0]), float(rot3[0][0]), float(rot4[0][0]), float(rot5[0][0]), float(rot6[0][0])]
        sin_list = [float(rot1[1][0]), float(rot2[1][0]), float(rot3[1][0]), float(rot4[1][0]), float(rot5[1][0]), float(rot6[1][0])]
        theta_list = [math.atan2(sin_list[0],cos_list[0]),math.atan2(sin_list[1],cos_list[1]),math.atan2(sin_list[2],cos_list[2]),math.atan2(sin_list[3],cos_list[3]), math.atan2(sin_list[4],cos_list[4]), math.atan2(sin_list[5],cos_list[5])]

        
        f = np.array([f1[2],f2[2],f3[2],f4[2], f5[2], f6[2]]).reshape(6,1)

        
        if rot1[2][2] <= 0:
            rot1 = -rot1
        
        if rot2[2][2] <= 0:
            rot2 = -rot2

        if rot3[2][2] <= 0:
            rot3 = -rot3
        
        if rot4[2][2] <= 0:
            rot4 = -rot4

        if rot5[2][2] <= 0:
            rot5 = -rot5

        if rot6[2][2] <= 0:
            rot6 = -rot6

        
        #f_up = 2
        #f_low = 1

        
        #for i in range(4):
            #if flag_list[i] == 0 and f[i] >= f_up:
                #flag_list[i] = 1
            #elif flag_list[i] == 1 and f[i] < f_low:
                #flag_list[i] = 1


        if res1 == 1 and flg1 == 0:
            #loadch1 = p1
            # Storing the robot's position and rotation matrix  with respect to the load's frame
            posdes1 = rotl.T @ (p1 - loadp)                       
            rotload1 = rotl.T @ rot1
            thetades1 = math.atan2(rotload1[1][0], rotload1[0][0])

            t_p1 = rot1.T @ (p1 - loadp)
            t_p1[0] = t_p1[0] + 0.50                                  # To account for distance between the robot and the dummy positions
            t_p1 = rot1 @ t_p1
            xd1 = t_p1[0]
            yd1 = t_p1[1]

        if res2 == 1 and flg2 == 0:
            #loadch2 = p2
            posdes2 = rotl.T @ (p2 - loadp)
            rotload2 = rotl.T @ rot2
            thetades2 = math.atan2(rotload2[1][0], rotload2[0][0])
            t_p2 = rot2.T @ (p2 - loadp)
            t_p2[0] = t_p2[0] + 0.50
            t_p2  = rot2 @ t_p2
            xd2 = t_p2[0]
            yd2 = t_p2[1]

        if res3 == 1 and flg3 == 0:
            #loadch3 = p3
            posdes3 = rotl.T @ (p3 - loadp)
            rotload3 = rotl.T @ rot3
            thetades3 = math.atan2(rotload3[1][0], rotload3[0][0])
            t_p3 = rot3.T @ (p3 - loadp)
            t_p3[0] = t_p3[0] + 0.50
            t_p3 = rot3 @ t_p3
            xd3 = t_p3[0]
            yd3 = t_p3[1]

        if res4 == 1 and flg4 == 0:
            #loadch4 = p4
            posdes4 = rotl.T @ (p4 - loadp)
            rotload4 = rotl.T @ rot4
            thetades4 = math.atan2(rotload4[1][0], rotload4[0][0])
            t_p4 = rot4.T @ (p4 - loadp)
            t_p4[0] = t_p4[0] + 0.50
            t_p4 = rot4 @ t_p4
            xd4 = t_p4[0]
            yd4 = t_p4[1]

        if res5 == 1 and flg5 == 0:
            #loadch4 = p4
            posdes5 = rotl.T @ (p5 - loadp)
            rotload5 = rotl.T @ rot5
            thetades5 = math.atan2(rotload5[1][0], rotload5[0][0])

        if res6 == 1 and flg6 == 0:
            #loadch4 = p4
            posdes6 = rotl.T @ (p6 - loadp)
            rotload6 = rotl.T @ rot6
            thetades6 = math.atan2(rotload6[1][0], rotload6[0][0])


        


        
        

        
        

        

        






        if res1 == 1:
            flg1 = 1
            
        if res2 == 1:
            flg2 = 1

        if res3 == 1:
            flg3 = 1

        if res4 == 1:
            flg4 = 1

        if res5 == 1:
            flg5 = 1

        if res6 == 1:
            flg6 = 1

        posdes_list = [posdes1, posdes2, posdes3, posdes4, posdes5, posdes6]
        rotload_list = [rotload1,rotload2,rotload3,rotload4, rotload5, rotload6]
        p_list = [p1,p2,p3,p4,p5,p6]
        rot_list = [rot1, rot2, rot3, rot4, rot5, rot6]


        posdesabs1 = rotl @ posdes1 + loadp      # The desired position of the robot in absolute frame
        posdesabs2 = rotl @ posdes2 + loadp
        posdesabs3 = rotl @ posdes3 + loadp
        posdesabs4 = rotl @ posdes4 + loadp
        posdesabs5 = rotl @ posdes5 + loadp
        posdesabs6 = rotl @ posdes6 + loadp

        posdesabs_list = [posdesabs1, posdesabs2, posdesabs3, posdesabs4, posdesabs5, posdesabs6]

        rotload_abs1 = rotl @ rotload1
        rotload_abs2 = rotl @ rotload2
        rotload_abs3 = rotl @ rotload3
        rotload_abs4 = rotl @ rotload4
        rotload_abs5 = rotl @ rotload5
        rotload_abs6 = rotl @ rotload6

        rotloadabs_list = [rotload_abs1, rotload_abs2, rotload_abs3, rotload_abs4, rotload_abs5, rotload_abs6]        # Rotation matrix 0Rr corresponding to fixed LRr

        thetadesabs1 = math.atan2(rotload_abs1[1][0], rotload_abs1[0][0])
        thetadesabs2 = math.atan2(rotload_abs2[1][0], rotload_abs2[0][0])
        thetadesabs3 = math.atan2(rotload_abs3[1][0], rotload_abs3[0][0])
        thetadesabs4 = math.atan2(rotload_abs4[1][0], rotload_abs4[0][0])
        thetadesabs5 = math.atan2(rotload_abs5[1][0], rotload_abs5[0][0])
        thetadesabs6 = math.atan2(rotload_abs6[1][0], rotload_abs6[0][0])

        thetadesabs_list = [thetadesabs1, thetadesabs2, thetadesabs3, thetadesabs4, thetadesabs5, thetadesabs6]


        loadp1 = rot1.T @ (loadp - p1)
        loadp2 = rot2.T @ (loadp - p2)
        loadp3 = rot3.T @ (loadp - p3)
        loadp4 = rot4.T @ (loadp - p4)
        loadp5 = rot5.T @ (loadp - p5)
        loadp6 = rot6.T @ (loadp - p6)


        

        


       


        

        
        
        





        
        


    

        

       

        


               

        '''loadp2targ = loadp
        if flg2 == 1:
            loadp2targ[0] = loadp[0] + xd2
            loadp2targ[1] = loadp[1] + yd2
        
        loadp3targ = loadp
        if flg3 == 1:
            loadp3targ[0] = loadp[0] + xd3
            loadp3targ[1] = loadp[1] + yd3

        loadp4targ = loadp
        if flg4 == 1:
            loadp4targ[0] = loadp[0] + xd4
            loadp4targ[1] = loadp[1] + yd4'''


        #print(loadch1)


        '''temp_p1 = loadp
        temp_p1[0] = temp_p1[0] + xd1
        temp_p1[1] = temp_p1[1] + yd1
        
        temp_p2 = loadp
        temp_p2[0] = temp_p2[0] + xd2
        temp_p2[1] = temp_p2[1] + yd2
        
        temp_p3 = loadp
        temp_p3[0] = temp_p3[0] + xd3
        temp_p3[1] = temp_p3[1] + yd3
        
        temp_p4 = loadp
        temp_p4[0] = temp_p4[0] + xd4
        temp_p4[1] = temp_p4[1] + yd4'''

        ''' temp_p1 = np.zeros((3,1))
        temp_p2 = np.zeros((3,1))
        temp_p3 = np.zeros((3,1))
        temp_p4 = np.zeros((3,1))

        temp_p1[0] = loadp[0] + xd1
        temp_p2[0] = loadp[0] + xd2
        temp_p3[0] = loadp[0] + xd3
        temp_p4[0] = loadp[0] + xd4

        temp_p1[1] = loadp[1] + yd1
        temp_p2[1] = loadp[1] + yd2
        temp_p3[1] = loadp[1] + yd3
        temp_p4[1] = loadp[1] + yd4

        temp_p1[2] = loadp[2] 
        temp_p2[2] = loadp[2] 
        temp_p3[2] = loadp[2] 
        temp_p4[2] = loadp[2] 

        
        #print("------")
        #print(temp_p1)
        

        

        

        
        
        
        


        temp_loadp1 = rot1.T @ (temp_p1 - p1)
        temp_loadp2 = rot2.T @ (temp_p2 - p2)
        temp_loadp3 = rot3.T @ (temp_p3 - p3)
        temp_loadp4 = rot4.T @ (temp_p4 - p4)

        #print(temp_loadp1)'''

        

        

        

        


        
        

               
        
        '''if flg1 == 0:
            loadp1 = rot1.T @ (loadp - p1)
        else:
            loadp1 = rot1.T @ (temp_p1 - p1)

        if flg2 == 0:
            loadp2 = rot2.T @ (loadp - p2)
        else:
            loadp2 = rot2.T @ (temp_p2 - p2)

        if flg3 == 0:
            loadp3 = rot3.T @ (loadp - p3)
        else:
            loadp3 = rot3.T @ (temp_p3 - p3)

        if flg4 == 0:
            loadp4 = rot4.T @ (loadp - p4)
        else:
            loadp4 = rot4.T @ (temp_p4 - p4)'''

        



        
        
        

        xp = [float(loadp1[0]),float(loadp2[0]),float(loadp3[0]),float(loadp4[0]),float(loadp5[0]),float(loadp6[0])]
        yp = [float(loadp1[1]),float(loadp2[1]),float(loadp3[1]),float(loadp4[1]),float(loadp5[1]),float(loadp6[1])]

        

        '''temp_xp = [float(temp_loadp1[0]),float(temp_loadp2[0]),float(temp_loadp3[0]),float(temp_loadp4[0])]
        temp_yp = [float(temp_loadp1[1]),float(temp_loadp2[1]),float(temp_loadp3[1]),float(temp_loadp4[1])]
        #dist_p = [math.sqrt(temp_xp[0]*temp_xp[0]+temp_yp[0]*temp_yp[0]),math.sqrt(temp_xp[1]*temp_xp[1]+temp_yp[1]*temp_yp[1]),math.sqrt(temp_xp[2]*temp_xp[2]+temp_yp[2]*temp_yp[2]),math.sqrt(temp_xp[3]*temp_xp[3]+temp_yp[3]*temp_yp[3])]
        dist_p = [math.sqrt(temp_xp[i]*temp_xp[i] + temp_yp[i]*temp_yp[i]) for i in range(4)]'''
        flg_list = [flg1,flg2,flg3,flg4,flg5,flg6]
        #print(temp_xp)
        #print(xp)
        #print(dist_p)


        
        epsilon_pos = 2
        epsilon_theta = 0.1
        
        #print(error_y)
        #print("------")
        #print(error_x)
        #print("--------")
        #print(math.sqrt(error_x*error_x+error_y*error_y))

        
        #print(err_theta)
        
        #what condition to impose for contact?

        for i in range(num_robots):
            #if flg_list[i] == 0:
            err_x = float(posdesabs_list[i][0]) - float(p_list[i][0])
            err_y = float(posdesabs_list[i][1]) - float(p_list[i][1])
            '''if i==1:
                print(math.sqrt(err_x*err_x + err_y*err_y))
                print("-----------")
                print(abs(float(thetadesabs_list[i])-float(theta_list[i])))'''
                
            if (math.sqrt(err_x*err_x+err_y*err_y) < (epsilon_pos/3)) and (abs(float(thetadesabs_list[i])-float(theta_list[i]))<epsilon_theta) and flag_list[i] == 0:
                flag_list[i] = 1
            
            if flag_list[i] == 1:
                

                if (math.sqrt(err_x*err_x+err_y*err_y) >= epsilon_pos):
                    flag_list[i] = 0
                else:
                    flag_list[i] = 1

                
           # elif (math.sqrt(err_x*err_x+err_y*err_y) >= epsilon_pos) and flag_list[i] == 1:
               # flag_list[i] = 0
            #else:
                #flag_list[i] = flag_list[i]
            '''else:
                if temp_xp[i] >= 0.1:
                    flag_list[i] = 0
                else:
                    flag_list[i] = 1'''


        print(flag_list)
        print("-------")
        #print(loadp)
        #print("-------")
        
        


        


        
        
                
            
                

            

        

        
        
        

        #res_list = [res1,res2,res3,res4]
        #print(res1)        
        #print(res_list)
        

                
        posdif10 = p1[0]- loadp[0]
        posdif11 = p1[1] - loadp[1]
        posdif20 = p2[0]- loadp[0]
        posdif21 = p2[1] - loadp[1]
        posdif30 = p3[0]- loadp[0]
        posdif31 = p3[1] - loadp[1]
        posdif40 = p4[0] - loadp[0]
        posdif41 = p4[1] - loadp[1]
        posdif50 = p5[0] - loadp[0]
        posdif51 = p5[1] - loadp[1]
        posdif60 = p6[0] - loadp[0]
        posdif61 = p6[1] - loadp[1]

        
        

        P1 = np.array([[1,0,0],[0,1,0],[-posdif11[0],posdif10[0],1]]).reshape((3,3))
        P2 = np.array([[1,0,0],[0,1,0],[-posdif21[0],posdif20[0],1]]).reshape((3,3))

        P3 = np.array([[1,0,0],[0,1,0],[-posdif31[0],posdif30[0],1]]).reshape((3,3))
        P4 = np.array([[1,0,0],[0,1,0],[-posdif41[0],posdif40[0],1]]).reshape((3,3))
        P5 = np.array([[1,0,0],[0,1,0],[-posdif51[0],posdif50[0],1]]).reshape((3,3))
        P6 = np.array([[1,0,0],[0,1,0],[-posdif61[0],posdif60[0],1]]).reshape((3,3))

        y_list = [float(posdif11),float(posdif21),float(posdif31),float(posdif41),float(posdif51),float(posdif61)]
        
        x_list = [float(posdif10),float(posdif20),float(posdif30),float(posdif40),float(posdif50),float(posdif60)]
        

        #print(P2[2][1])

        G_final = np.zeros((3,num_robots))
        

        if flag_list[0] == 1:
            G_final[0][0] = -rot1[1][0] #rot1[0][0] Difference between pioneer and KUKA due to lead axis change from x to y
            G_final[1][0] = rot1[0][0]
            G_final[2][0] = P1[2][0]*-rot1[1][0] + P1[2][1]*rot1[0][0]

        if flag_list[1] == 1:
            G_final[0][1] = -rot2[1][0]
            G_final[1][1] = rot2[0][0]
            G_final[2][1] = P2[2][0]*-rot2[1][0] + P2[2][1]*rot2[0][0]

        if flag_list[2] == 1:
            G_final[0][2] = -rot3[1][0]
            G_final[1][2] = rot3[0][0]
            G_final[2][2] = P3[2][0]*-rot3[1][0] + P3[2][1]*rot3[0][0]

        if flag_list[3] == 1:
            G_final[0][3] = -rot4[1][0]
            G_final[1][3] = rot4[0][0]
            G_final[2][3] = P4[2][0]*-rot4[1][0] + P4[2][1]*rot4[0][0]

        if flag_list[4] == 1:
            G_final[0][4] = -rot5[1][0]
            G_final[1][4] = rot5[0][0]
            G_final[2][4] = P5[2][0]*-rot5[1][0] + P5[2][1]*rot5[0][0]

        if flag_list[5] == 1:
            G_final[0][5] = -rot6[1][0]
            G_final[1][5] = rot6[0][0]
            G_final[2][5] = P6[2][0]*-rot6[1][0] + P6[2][1]*rot6[0][0]

        #print(G_final)
        




        

        #print(P1).


        '''G1tr = rot1.T @ P1.T
        G2tr = rot2.T @ P2.T
        G3tr = rot3.T @ P3.T
        G4tr = rot4.T @ P4.T

        

        Gtr = np.vstack((G1tr,G2tr,G3tr,G4tr))'''
        #print(Gtr.T)
        

        
        #G_final = Gtr.T @ H
        #print(G_final)
        (row,col) = G_final.shape


        for i in range(row):
            for j in range(col):
                G_final[i][j] = round(G_final[i][j],2)

        #print(G_final)

        




        #F = G_final @ M
        

        
        f_set = np.zeros((num_robots,1))
        

                      
        #f_set = np.linalg.pinv(G_final) @ tau_set      # direct pseudo inverse from the desired fx,fy,mz around com
        # but may produce negative force components - mathematically no problem but robot cannot pull the load

        # Qp solver to account for constraints
    
        v = [0.0,0.0,0.0,0.0,0.0,0.0]
        w = [0.0,0.0,0.0,0.0,0.0,0.0]
        #vl = [0.0,0.0,0.0,0.0]
        #vr = [0.0,0.0,0.0,0.0]
        kv = 0.20
        kw = 0.3
        #gains to be tuned here.....
        gain_v = 2
        gain_w = 1
        #d = 0.331
        #r = 0.0975
        P_mat = np.zeros((9,9))
        lambda_ = np.zeros((9,1))
        wt = 5
        P_mat[0][0] = P_mat[1][1] = P_mat[2][2] = P_mat[3][3] = P_mat[4][4] = P_mat[5][5] = 1
        P_mat[6][6] = P_mat[7][7] = P_mat[8][8] = wt
        Iden  = -np.eye(3)
        A_mat = np.hstack((G_final, Iden))
        tau_eq = np.array([0.0,0.0,0.0]).reshape((3,))
        lambda_eq = solve_qp(P = P_mat, q = np.zeros(9), A = A_mat, b = tau_eq,lb = np.array([1.5,1.5,1.5,1.5,1.5,1.5,-100.0,-100.0,-100.0]), solver = 'quadprog')
        lambda_ = solve_qp(P = P_mat, q = np.zeros(9), A = A_mat, b = tau_set,lb = np.array([0.0,0.0,0.0,0.0,0.0,0.0,-100.0,-100.0,-100.0]), solver = 'quadprog')
        #lambda_eq = solve_qp(P = np.eye(4), q = np.zeros(4), A = G_final, b = tau_eq,lb = np.array([0.1,0.1,0.1,0.1]), solver = 'quadprog')

        
        #print(lambda_eq[0:4])
        #f_eq = nsp(G_final)
        #print(f_eq)

        f_set = lambda_[0:6] + lambda_eq[0:6]
        #k = 0
        
        #f_set = np.array([10.0,0.0,0.0,0.0])
        



        
        

        

        
        
        
        
        
        
        
        
        

         
         #
        
         
        for i in range(num_robots):
        #the conditions for setting velocities and angular velocities...
            if flag_list[i] == 0:
                if flg_list[i] == 0:
                    v[i] = gain_v*(yp[i])
                    w[i] = gain_w*math.atan2(xp[i],yp[i])
                
                
                
                else:
                    p_temp = rot_list[i].T @ (posdesabs_list[i] - p_list[i])
                
                    q = [posdesabs_list[i][0], posdesabs_list[i][1],thetadesabs_list[i] ]
                    obj = [float(p_temp[0]), float(p_temp[1]),theta_list[i]]
                    vOmega = self.gotoPose2(obj,q)
                    v[i] = vOmega[0]
                    w[i] = vOmega[1]
                    
            
                
                
                    #w[i] = gain_w*math.atan2(yp[i],xp[i])
            
                    #pdes = rotl @ posdes_list[i] + loadp
                    #Rdes = rotl @ rotload_list[i]
                    #th_des = math.atan2(Rdes[1][0], Rdes[0][0])
                    #th_des = -math.pi + math.fmod(th_des - math.pi, 2*math.pi)
                    #th_act = theta_list[i]
                    #th_act = -math.pi + math.fmod(th_act - math.pi, 2*math.pi)
                                     # Converting the desired position in robot's frame
                    #print(p_temp)
            else:
                #w[i] = -self.grad_descent(theta_list,cos_list,sin_list,x_list,y_list,G_final,i)
                
                
                v[i] = 0.0#kv*(float(f_set[i])-float(f[i])) 
                w[i] = 0.0    # gradient to be derived again...
                

                
                
                
                
                    
                    

                '''ptemp = rot_list[i] @ (pdes - p_list[i])
                    v[i] = 0.1*ptemp[0]
                    w[i] = 0.1*math.atan2(ptemp[1], ptemp[0])'''
                    #error = pdes - p_list[i]
                    #if (math.sqrt(error[0]*error[0] + error[1]*error[1]) < 0.01) :
                        #w[i] = 0.1*(math.atan2(Rdes[1][0],Rdes[0][0])-theta_list[i])


                    #if i==1:
                        #print(thetadesabs_list[i])
                        #print("-----")
                        #print(pdes)
                        #print("-----")
                        #print(p_list[i])
                        #print(theta_list[i])
                        #print(p_temp)
                       
                       
                        
                        


                    
                        




                '''gain_v = 3
                    v[i] = gain_v*(temp_xp[i])
                    gain_w = 0.5
                    w[i] = gain_w*math.atan2(temp_yp[i],temp_xp[i])'''
                    #gain_w = 0.2
                    #w[i] = gain_w*math.atan2(temp_yp[i],temp_xp[i])
                    
                
                
                    
            
                    
                


                
                
                #inter = v[i]
                #w[i] = gain_w*math.atan2(yp[i],xp[i])
                
                

            
            #else:

                #lambda_ = solve_qp(P = P_mat, q = np.zeros(7), A = A_mat, b = tau_set, lb = np.zeros(7),solver = 'quadprog')
                #f_set = lambda_[0:4]
                
                #v[i] = kv*(float(f_set[i])-float(f[i])) 
                
                #w[i] = gain_w*math.atan2(temp_yp[i],temp_xp[i])
                #w[i] =-self.grad_descent(theta_list,cos_list,sin_list,x_list,y_list,G_final,i)
                #gain_w = 0.2
                #w[i] = gain_w*math.atan2(temp_yp[i],temp_xp[i])

            
            
                    






            
            
            
                
            
             


            

                

            
                




            '''flag = False




            if res1 == 1 and res2 == 1 and res3 == 1 and res4 == 1:
                flag = True
            else:
                flag = False
               
                #for i in range(4):
                    #w[i] = self.frame_min(G_final,i,x_list,y_list,cos_list,sin_list)
                

                

            else:
                for i in range(4):
                    if flg_list[i] == 0:
                        w[i] = gain_w*math.atan2(yp[i],xp[i])
                    else:
                        gain_w = 0.2
                        w[i] = gain_w*math.atan2(temp_yp[i],temp_xp[i])'''

   
        
        if flag_list[0] == 1 and flag_list[1] == 1 and flag_list[2] == 1 and flag_list[3] == 1 and flag_list[4] == 1 and flag_list[5] == 1:
            w[0] = self.grad_descent(theta_list,cos_list,sin_list,x_list,y_list,G_final,0)
            v[0] = kv*(float(f_set[0])-float(f[0])) 
            w[1] = self.grad_descent(theta_list,cos_list,sin_list,x_list,y_list,G_final,1)
            v[1] = kv*(float(f_set[1])-float(f[1])) 
            w[2] = self.grad_descent(theta_list,cos_list,sin_list,x_list,y_list,G_final,2)
            v[2] = kv*(float(f_set[2])-float(f[2])) 
            w[3] = self.grad_descent(theta_list,cos_list,sin_list,x_list,y_list,G_final,3)
            v[3] = kv*(float(f_set[3])-float(f[3]))
            w[4] = self.grad_descent(theta_list,cos_list,sin_list,x_list,y_list,G_final,4)
            v[4] = kv*(float(f_set[4])-float(f[4]))
            w[5] = self.grad_descent(theta_list,cos_list,sin_list,x_list,y_list,G_final,5)
            v[5] = kv*(float(f_set[5])-float(f[5]))
                 
                
               # 
                
               # 
                
               # 
                


        #for i in range(4):
            #vl[i] = float(v[i] - w[i]*d)
            #vr[i] = float(v[i] + w[i]*d)

        


        frame_pot = self.frame_potential(G_final)

        #print(flag_list)

        #print(G_final)



        print(frame_pot)
        print("-----")
        print(f_set)
        print("----")
        #print(dist_p)
        #print(flag_list)
        #print("------")
        print(f)
        #print(G_final)
        #print(flag_list)
       # print(v)



        

            


         

        

        
         

        

                

        #except:
           #print("Error in the solver: Robots don't move")
           #pass

        

        '''vl = [0.0,0.0,0.0,0.0]
        vr = [0.0,0.0,0.0,0.0]

        position_desired = np.array([-1.5,1.5,0.138]).reshape((3,))
        cur_pos0 = np.array([float(p_list[0][0]), float(p_list[0][1]), float(p_list[0][2])]).reshape((3,))
        #print(cur_pos0)
        pos_des_r0 = rot_list[0].T @ (position_desired - cur_pos0)
        obj = [pos_des_r0[0], pos_des_r0[1], theta_list[0]]
        q = [position_desired[0], position_desired[1], 0]
        vOmega = self.gotoPose2(obj, q)
        vel = vOmega[0]
        ang_vel = vOmega[1]
        vl[0] = vel - ang_vel*d
        vr[0] = vel + ang_vel*d

        print(p_list[0])
        print("-------------")
        print(theta_list[0])'''

        

        #pos_diff = position_desired - p_list[0]
        #print(pos_diff)
       # pos_des_r0 = 
        

        #print(f)
        #print("------")
        #print(f_set)
        #print("-------")
        #print(v[0])
        #print(v[0])
        #print("-----")
        #print(w[0])
        
        
        #print(start_time)
        #plt.axis([0, 10, 0, 1])
        
        '''if (int(time.time()) - start_time) <= 50 :
            fmes_list.append(float(f_set[0]))
            f_list.append(float(f[0]))

        else:
            x = [i for i in range(len(f_list))]
            plt.plot(x,fmes_list)
            plt.plot(x, f_list)
            #plt.pause(0.05)

        plt.show()'''

        #print(v)

        #v = [0.0,0.0,0.0,0.0]
        #w = [0.0,0.0,0.0,0.0]

        
        
        
        
        
        
        

        

        #plt.plot(time_list, fmes_list)
        

        #print(fmes_list)
        
        

        msg_vel1.data = v[0]
        msg_angvel1.data = w[0]
        msg_vel2.data = v[1] #v[2]
        msg_angvel2.data = w[1] #v[2]
        msg_vel3.data = v[2] #v[3]
        msg_angvel3.data = w[2] #v[3]
        msg_vel4.data = v[3]
        msg_angvel4.data = w[3]
        msg_vel5.data = v[4]
        msg_angvel5.data = w[4]
        msg_vel6.data = v[5]
        msg_angvel6.data = w[5]
                
        

        self.velpublisher1_.publish(msg_vel1)
        self.angvelpublisher1_.publish(msg_angvel1)
        self.velpublisher2_.publish(msg_vel2)
        self.angvelpublisher2_.publish(msg_angvel2)
        self.velpublisher3_.publish(msg_vel3)
        self.angvelpublisher3_.publish(msg_angvel3)
        self.velpublisher4_.publish(msg_vel4)
        self.angvelpublisher4_.publish(msg_angvel4)
        self.velpublisher5_.publish(msg_vel5)
        self.angvelpublisher5_.publish(msg_angvel5)
        self.velpublisher6_.publish(msg_vel6)
        self.angvelpublisher6_.publish(msg_angvel6)

        

        #print(f_set)

        #self.get_logger().info('Publishing left wheel speed of robot 1: "%f"' % msg_left1.data)
        #self.get_logger().info('Publishing right wheel speed of robot 1: "%f"' % msg_right1.data)
        #self.get_logger().info('Publishing left wheel speed of robot 2: "%f"' % msg_left2.data)
        #self.get_logger().info('Publishing right wheel speed of robot 2: "%f"' % msg_right2.data)
        #self.get_logger().info('Publishing left wheel speed of robot 3: "%f"' % msg_left3.data)
        #self.get_logger().info('Publishing right wheel speed of robot 3: "%f"' % msg_right3.data)

    def listener_callbackf1(self, msg):
        global f1
       # self.get_logger().info('The x,y,z components of contact forces 1 are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        f1[0] = msg.x
        f1[1] = msg.y
        f1[2] = msg.z
        




    def listener_callbackf2(self, msg):
        global f2
        #self.get_logger().info('The x,y,z components of contact forces 2 are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        f2[0] = msg.x
        f2[1] = msg.y
        f2[2] = msg.z

    def listener_callbackf3(self, msg):
        global f3
       # self.get_logger().info('The x,y,z components of contact forces 3 are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        f3[0] = msg.x
        f3[1] = msg.y
        f3[2] = msg.z

    def listener_callbackf4(self, msg):
        global f4
       # self.get_logger().info('The x,y,z components of contact forces 3 are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        f4[0] = msg.x
        f4[1] = msg.y
        f4[2] = msg.z

    def listener_callbackf5(self, msg):
        global f5
       # self.get_logger().info('The x,y,z components of contact forces 3 are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        f5[0] = msg.x
        f5[1] = msg.y
        f5[2] = msg.z

    def listener_callbackf6(self, msg):
        global f6
       # self.get_logger().info('The x,y,z components of contact forces 3 are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        f6[0] = msg.x
        f6[1] = msg.y
        f6[2] = msg.z
    
    def listener_callbackr1(self, msg):
        global res1
        res1 = msg.data

    def listener_callbackr2(self, msg):
        global res2
        res2 = msg.data

    def listener_callbackr3(self, msg):
        global res3
        res3 = msg.data

    def listener_callbackr4(self, msg):
        global res4
        res4 = msg.data

    def listener_callbackr5(self, msg):
        global res5
        res5 = msg.data

    def listener_callbackr6(self, msg):
        global res6
        res6 = msg.data

    def listener_callbackp1(self, msg):
        global p1
       # self.get_logger().info('The x,y,z components of contact 1 position are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        p1[0] = msg.x
        p1[1] = msg.y
        p1[2] = msg.z

    def listener_callbackp2(self, msg):
        global p2
       # self.get_logger().info('The x,y,z components of contact 2 position are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        p2[0] = msg.x
        p2[1] = msg.y
        p2[2] = msg.z

    def listener_callbackp3(self, msg):
        global p3
        #self.get_logger().info('The x,y,z components of contact 3 position are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        p3[0] = msg.x
        p3[1] = msg.y
        p3[2] = msg.z

    def listener_callbackp4(self, msg):
        global p4
        #self.get_logger().info('The x,y,z components of contact 3 position are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        p4[0] = msg.x
        p4[1] = msg.y
        p4[2] = msg.z

    def listener_callbackp5(self, msg):
        global p5
        #self.get_logger().info('The x,y,z components of contact 3 position are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        p5[0] = msg.x
        p5[1] = msg.y
        p5[2] = msg.z

    def listener_callbackp6(self, msg):
        global p6
        #self.get_logger().info('The x,y,z components of contact 3 position are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        p6[0] = msg.x
        p6[1] = msg.y
        p6[2] = msg.z

    def listener_callbackq1(self, msg):
        global q1
       # self.get_logger().info('The x,y,z,w quaternions of contact frame 1 are "%f","%f","%f","%f"' % (msg.x,msg.y,msg.z,msg.w))
        q1[0] = msg.x
        q1[1] = msg.y
        q1[2] = msg.z
        q1[3] = msg.w

    def listener_callbackq2(self, msg):
        global q2
       # self.get_logger().info('The x,y,z,w quaternions of contact frame 2 are "%f","%f","%f","%f"' % (msg.x,msg.y,msg.z,msg.w))
        q2[0] = msg.x
        q2[1] = msg.y
        q2[2] = msg.z
        q2[3] = msg.w

    def listener_callbackq3(self, msg):
        global q3
        #self.get_logger().info('The x,y,z,w quaternions of contact frame 3 are "%f","%f","%f","%f"' % (msg.x,msg.y,msg.z,msg.w))
        q3[0] = msg.x
        q3[1] = msg.y
        q3[2] = msg.z
        q3[3] = msg.w

    def listener_callbackq4(self, msg):
        global q4
        #self.get_logger().info('The x,y,z,w quaternions of contact frame 3 are "%f","%f","%f","%f"' % (msg.x,msg.y,msg.z,msg.w))
        q4[0] = msg.x
        q4[1] = msg.y
        q4[2] = msg.z
        q4[3] = msg.w

    def listener_callbackq5(self, msg):
        global q5
        #self.get_logger().info('The x,y,z,w quaternions of contact frame 3 are "%f","%f","%f","%f"' % (msg.x,msg.y,msg.z,msg.w))
        q5[0] = msg.x
        q5[1] = msg.y
        q5[2] = msg.z
        q5[3] = msg.w

    def listener_callbackq6(self, msg):
        global q6
        #self.get_logger().info('The x,y,z,w quaternions of contact frame 3 are "%f","%f","%f","%f"' % (msg.x,msg.y,msg.z,msg.w))
        q6[0] = msg.x
        q6[1] = msg.y
        q6[2] = msg.z
        q6[3] = msg.w

    def listener_callbackloadp(self, msg):
        global loadp
        #self.get_logger().info('The x,y,z components of load position are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        loadp[0] = msg.x
        loadp[1] = msg.y
        loadp[2] = msg.z

    def listener_callbackquatl(self,msg):
        global loadq
        loadq[0] = msg.x
        loadq[1] = msg.y
        loadq[2] = msg.z
        loadq[3] = msg.w
        



    

    

def main(args=None):
    rclpy.init(args=args)
    control = Control()
    #time.sleep(5)
    #global start_time
    

    start_time = int(time.time())
    rclpy.spin(control)
   
   
    
    
    control.destroy_node()
    rclpy.shutdown()
    


    

if __name__ == '__main__':
    main()

    

    

     

     
