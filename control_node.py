import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

p1 = np.zeros((3,1))
p2 = np.zeros((3,1))
p3 = np.zeros((3,1))
f1 = np.zeros((3,1))
f2 = np.zeros((3,1))
f3 = np.zeros((3,1))
q1 = np.zeros((4,1))
q2 = np.zeros((4,1))
q3 = np.zeros((4,1))
loadp = np.zeros((3,1))
H = np.zeros((3,9))
H[0][0] = 1
H[1][3] = 1
H[2][6] = 1


class Control(Node):

    def __init__(self):
        super().__init__('control_node')
        self.lpublisher1_ = self.create_publisher(Float32, 'leftMotorSpeed1', 10)
        self.rpublisher1_ = self.create_publisher(Float32, 'rightMotorSpeed1',10)
        self.lpublisher2_ = self.create_publisher(Float32, 'leftMotorSpeed2', 10)
        self.rpublisher2_ = self.create_publisher(Float32, 'rightMotorSpeed2',10)
        self.lpublisher3_ = self.create_publisher(Float32, 'leftMotorSpeed3', 10)
        self.rpublisher3_ = self.create_publisher(Float32, 'rightMotorSpeed3',10)

        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscriptionf1 = self.create_subscription(
            Vector3,
            'forcevalue1',
            self.listener_callbackf1,
            10)
        self.subscriptionf1  # prevent unused variable warning

        self.subscriptionf2 = self.create_subscription(
            Vector3,
            'forcevalue2',
            self.listener_callbackf2,
            10)
        self.subscriptionf2  # prevent unused variable warning

        self.subscriptionf3 = self.create_subscription(
            Vector3,
            'forcevalue3',
            self.listener_callbackf3,
            10)
        self.subscriptionf3  # prevent unused variable warning

        self.subscriptionp1 = self.create_subscription(
            Vector3,
            'pos1',
            self.listener_callbackp1,
            10)
        self.subscriptionp1 

        self.subscriptionp2 = self.create_subscription(
            Vector3,
            'pos2',
            self.listener_callbackp2,
            10)
        self.subscriptionp2

        self.subscriptionp3 = self.create_subscription(
            Vector3,
            'pos3',
            self.listener_callbackp3,
            10)
        self.subscriptionp3

        self.subscriptionloadp = self.create_subscription(
            Vector3,
            'lpos',
            self.listener_callbackloadp,
            10)
        self.subscriptionloadp

        self.subscriptionq1 = self.create_subscription(
            Quaternion,
            'quat1',
            self.listener_callbackq1,
            10)
        self.subscriptionq1

        self.subscriptionq2 = self.create_subscription(
            Quaternion,
            'quat2',
            self.listener_callbackq2,
            10)
        self.subscriptionq2

        self.subscriptionq3 = self.create_subscription(
            Quaternion,
            'quat3',
            self.listener_callbackq3,
            10)
        self.subscriptionq3

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
        


    def timer_callback(self):
        
        global f1
        global f2
        global f3
        global p1
        global p2
        global p3
        global q1
        global q2
        global q3
        global loadp
        global H

        msg_left1 = Float32()
        msg_right1 = Float32()
        msg_left2 = Float32()
        msg_right2 = Float32()
        msg_left3 = Float32()
        msg_right3 = Float32()

        msg_left1.data = 0.50
        msg_right1.data = 0.50
        msg_left2.data = 0.50
        msg_right2.data = 0.50
        msg_left3.data = 0.50
        msg_right3.data = 0.50

        rot1 = self.quaterniontorpy(q1)
        rot2 = self.quaterniontorpy(q2)
        rot3 = self.quaterniontorpy(q3)

        posdif10 = p1[0]-loadp[0]
        posdif11 = p1[1] - loadp[1]
        posdif20 = p2[0]-loadp[0]
        posdif21 = p2[1] - loadp[1]
        posdif30 = p3[0]-loadp[0]
        posdif31 = p3[1] - loadp[1]

        P1 = np.array([[1,0,0],[0,1,0],[-posdif11[0],posdif10[0],1]])
        P2 = np.array([[1,0,0],[0,1,0],[-posdif21[0],posdif20[0],1]])
        P3 = np.array([[1,0,0],[0,1,0],[-posdif31[0],posdif30[0],1]])

        G1tr = rot1.T @ P1.T
        G2tr = rot2.T @ P2.T
        G3tr = rot3.T @ P3.T

        Gtr = np.vstack((G1tr[0],G2tr[0],G3tr[0]))

        Gt = H @ Gtr

        f = np.array([f1[2],f2[2],f3[2]]).reshape(3,1)

        tau = Gt.T @ f

        print(tau)












        self.lpublisher1_.publish(msg_left1)
        self.rpublisher1_.publish(msg_right1)
        self.lpublisher2_.publish(msg_left2)
        self.rpublisher2_.publish(msg_right2)
        self.lpublisher3_.publish(msg_left3)
        self.rpublisher3_.publish(msg_right3)

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
       # self.get_logger().info('The x,y,z components of contact forces 2 are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        f2[0] = msg.x
        f2[1] = msg.y
        f2[2] = msg.z

    def listener_callbackf3(self, msg):
        global f3
       # self.get_logger().info('The x,y,z components of contact forces 3 are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        f3[0] = msg.x
        f3[1] = msg.y
        f3[2] = msg.z

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

    def listener_callbackloadp(self, msg):
        global loadp
        #self.get_logger().info('The x,y,z components of load position are "%f","%f","%f"' % (msg.x,msg.y,msg.z))
        loadp[0] = msg.x
        loadp[1] = msg.y
        loadp[2] = msg.z



    

    

def main(args=None):
    rclpy.init(args=args)
    control = Control()
    rclpy.spin(control)
    control.destroy_node()
    rclpy.shutdown()


    

if __name__ == '__main__':
    main()

    

    

     

     
