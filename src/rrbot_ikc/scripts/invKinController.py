#!/usr/bin/env python3

from queue import Empty
import rospy
import tf
import math
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Point,Vector3
from sensor_msgs.msg import JointState
from pid import PID
from copy import copy

class InvKinController:
    def __init__(self,frequency) -> None:
        # Initial value decleration
        self.sampling_time = float(copy(1/frequency))
        self.frequency = frequency
        self.j1_pose = None
        self.j2_pose = None
        self.l1_length = 2.0
        self.l2_length = 1.0
        self.l3_length = 1.0
        self.target_pose_y = 0.2 # Since there is no mevement in y direction

        self.joint1_command = Float64()
        self.joint2_command = Float64()

        self.joint_references = Vector3() # x : joint 1 angle y : joint 2 angle
        self.joint_pose_normalized = Vector3() # x : joint 1 angle y : joint 2 angle

        self.ref_x,self.ref_y,self.ref_z = 0.0,0.0,0.0
        
        # Controller Definitions
        self.joint1_controller = PID(Kp=0.125,Ki=2.5,Kd=0,Kp1 =10,inner_loop_p=True,saturation_limit=2)
        self.joint2_controller = PID(Kp=0.125,Ki=2.5,Kd=0,Kp1 =10,inner_loop_p=True,saturation_limit=2)

        # Init the node
        rospy.init_node('manipulator_position_controller',anonymous=True)
        # Publisher and Subscriber Decleration
        self.joint1_controller_pub = rospy.Publisher('/rrbot/joint1_velocity_controller/command',Float64,queue_size=1)
        self.joint2_controller_pub = rospy.Publisher('/rrbot/joint2_velocity_controller/command',Float64,queue_size=1)
        self.measured_position_publisher = rospy.Publisher('end_effector_measured_position',Vector3,queue_size=1)
        self.joint_reference_publihser = rospy.Publisher('/joint_references',Vector3,queue_size=1)
        self.joint_pose_pub = rospy.Publisher('joint_poses_normalized',Vector3,queue_size=1)

        rospy.Subscriber('position_setpoint',Point,self.getPositionSetpoint)
        rospy.Subscriber('/rrbot/joint_states',JointState,self.getPositionOfJoints)

        self.end_effector_listener = tf.TransformListener()
        self.end_effector_position = Vector3()

  
    def getPositionOfJoints(self,msg):
        self.j1_pose = math.atan2(math.sin(msg.position[0]-math.pi/2),math.cos(msg.position[0]-math.pi/2))
        self.j2_pose = math.atan2(math.sin(msg.position[1]),math.cos(msg.position[1]))
        # print(f"Joint Positions : {self.j1_pose,self.j2_pose}\n\n\n")

        # print(f"Position Values for joints :: {msg.position[0]%math.pi*2,msg.position[1]%math.pi*2}")
        

    def getPositionSetpoint(self,msg):
        self.ref_x = msg.x
        self.ref_y = 0.2
        self.ref_z = msg.z
        # print(f"Ref_x , ref_y : {self.ref_x,self.ref_z}")

    def getPositionOfEndEffector(self):
        pass

    def calculateInverseKinematic(self,target_pose_x,target_pose_y,target_pose_z):
        joint_1_angle = []
        joint_2_angle = []
        target_pose_y = 0.2


        # Solution for angle 2
        A = target_pose_x
        B = (target_pose_z-self.l1_length)*-1
        C = math.sqrt(A**2+B**2)
        if C > (self.l2_length+self.l1_length):
            return
        cos_theta2 = (C**2-self.l2_length**2-self.l3_length**2)/(2*self.l2_length*self.l3_length)
        sin_theta2 = math.sqrt(1-cos_theta2**2)
        # print(f"Cos_theta2,sin_theta2 : {cos_theta2,sin_theta2}\n")
        joint2_sol_1 = math.acos(cos_theta2)
        # print(f"Joint2_sol_1 : {joint2_sol_1}\n")
        joint_2_angle.append(joint2_sol_1)

        # Solution for angle 1 
        cos_phi_minus_theta1 = (C**2+self.l2_length**2-self.l3_length**2)/(2*self.l2_length*C)
        # print(f"cos_phi_minus_theta1 : {cos_phi_minus_theta1}\n")
        sin_phi_minus_theta1 = math.sqrt(1-cos_phi_minus_theta1**2)       
        Phi = math.atan2(B,A)
        # print(f"Phi : {Phi}\n")
        joint_1_sol_1 = Phi-math.atan2(sin_phi_minus_theta1,cos_phi_minus_theta1)
        joint_1_sol_2 = Phi-math.atan2(-sin_phi_minus_theta1,cos_phi_minus_theta1)
        joint_1_sol_1 = math.atan2(math.sin(joint_1_sol_1),math.cos(joint_1_sol_1))
        joint_1_sol_2 = math.atan2(math.sin(joint_1_sol_2),math.cos(joint_1_sol_2))
        joint_1_angle.append(joint_1_sol_1)
        # print(f"Joint_1_angle : {joint2_sol_1}")

        return joint_1_angle,joint_2_angle

    def execute(self):
        # Wait for messages
        rospy.wait_for_message('position_setpoint',Float64)
        rospy.wait_for_message('/rrbot/joint_states',Float64)
        rate = rospy.Rate(self.frequency)
        joint_1_angle,joint_2_angle = [],[]

        while not rospy.is_shutdown():
            try:
                joint_1_angle,joint_2_angle = self.calculateInverseKinematic(self.ref_x,self.ref_y,self.ref_z)
            except:
                pass
                # print("Singularity Error")
            # print(f"Joint Ref Angles : {joint_1_angle,joint_2_angle}\n\n\n")
            self.joint1_controller.set_setpoint(joint_1_angle[0])#joint_1_angle[0]
            self.joint2_controller.set_setpoint(joint_2_angle[0])

            self.joint_references.x = joint_1_angle[0]
            self.joint_references.y = joint_2_angle[0]
            self.joint_reference_publihser.publish(self.joint_references)

            self.joint_pose_normalized.x = self.j1_pose
            self.joint_pose_normalized.y = self.j2_pose
            self.joint_pose_pub.publish(self.joint_pose_normalized)
            

            if (self.j1_pose and self.j2_pose) is not None: 
                joint1_controller_out=self.joint1_controller.execute(self.j1_pose,self.sampling_time)
                joint2_controller_out=self.joint2_controller.execute(self.j2_pose,self.sampling_time)
                # print(f"Joint Positions : {self.j1_pose,self.j2_pose}\n\n\n")
                # print(f"Controller output : {self.joint1_controller.err_cum}")
            self.joint1_controller_pub.publish(joint1_controller_out)
            self.joint2_controller_pub.publish(joint2_controller_out)

            # Read end effector position data
            try:
                (end_position,end_orientation) = self.end_effector_listener.lookupTransform('/world','/hokuyo_link',rospy.Time(0))
            except:
                continue


            # End effector position information
            self.end_effector_position.x = end_position[0]
            self.end_effector_position.z = end_position[2]
            self.measured_position_publisher.publish(self.end_effector_position)
            
            rate.sleep()


if __name__ == '__main__':
    IKC = InvKinController(100)
    try:
        IKC.execute()
    except:
        pass