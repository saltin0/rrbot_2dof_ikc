#!/usr/bin/env python
import rospy
# Import necessary packages
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
import math as m
import tf
import matplotlib.pyplot as plt

ref_angles=[3, 0]
old_err1,sum_err1,old_err2,sum_err2 = 0,0,0,0
j1_norm,j2_norm = 0,0

def inverse_kinematics(target_pos_x,target_pos_z):
    """
    You should implement a inverse kinematic model that uses x and z positions of the target point and
    returns with required angles for joint1 and joint2 to move end effector to the desired target position

    :param target_pos_x,target_pos_z:
    :return: joint1_ref, joint2_ref
    """
    return ref_1,ref_2

def read_states(msg):
    global j1_norm,j2_norm
    # Normalize the joint 1 position angle (in radians)
    j1_norm = m.atan2(m.sin(msg.position[0]),m.cos(msg.position[0]))
    # Normalize the joint 1 position angle (in radians)
    j2_norm = m.atan2(m.sin(msg.position[1]), m.cos(msg.position[1]))

def pid_control(ref1,ref2,state1,state2,kp,ki,kd,dt):
    global old_err1,sum_err1,old_err2,sum_err2 # Global variables

    # Find the nearest angle between reference1 and current angle of joint1
    err1 = m.atan2(m.sin(ref1-state1),m.cos(ref1-state1))
    sum_err1 = sum_err1 + dt*(old_err1+err1)/2.0 # Error sum for integral controller
    u1 = kp*err1 + ki*sum_err1 + kd* (err1-old_err1) # PID output

    # Find the nearest angle between reference1 and current angle of joint2
    err2 = ref2 - state2
    sum_err2 = sum_err2 + dt * (old_err2 + err2) / 2.0 # Error sum for integral controller
    u2 = kp * err2 + ki * sum_err2 + kd * (err2 - old_err2) # PID output

    # Update old errors
    old_err1 = err1
    old_err2 = err2
    return u1,u2 # Return with PID outputs of joint1 and joint2


if __name__ == '__main__':
    #Initialize a ros node
    rospy.init_node('pid_example', anonymous=True)

    # ROS publishers and subscribers
    # Create a publisher for control joint1 angular velocity
    pub1 = rospy.Publisher("/rrbot/joint1_velocity_controller/command",Float64,queue_size=1)
    # Create a publisher for control joint2 angular velocity
    pub2 = rospy.Publisher("/rrbot/joint2_velocity_controller/command", Float64, queue_size=1)
    # Create a subscriber to get joint angles
    rospy.Subscriber("/rrbot/joint_states", JointState, read_states)

    pose_pub = rospy.Publisher("/real_pos",Vector3,queue_size=1)
    ref_pub = rospy.Publisher("/ref_pos", Vector3, queue_size=1)

    # Create tf transform listener to get transformation and rotation information between the frames.
    listener = tf.TransformListener()

    # Rate parameter
    rate = rospy.Rate(50)
    dt = 0.02


    while not rospy.is_shutdown():
        # Give reference angles and get control signal (angular velocity) for joint1 (u1) and joint2 (u2)
        u1,u2=pid_control(ref_angles[0],ref_angles[1],j1_norm,j2_norm,1.0,0.0,0.0,dt)

        # Create messages to send message over "/rrbot/joint1_velocity_controller/command" topic
        msg1=Float64()
        msg1.data = 1
        # Send this message over "/rrbot/joint1_velocity_controller/command" topic
        pub1.publish(msg1)

        # Create messages to send message over "/rrbot/joint2_velocity_controller/command" topic
        msg2 = Float64()
        msg2.data = 1
        # Send this message over "/rrbot/joint2_velocity_controller/command" topic
        pub2.publish(msg2)

        # Get the transformation between end effector link (hokuyo_link) and world frame.
        try:
            (trans, rot) = listener.lookupTransform('/world','/hokuyo_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # Send the current x and z position of the end effector via "/real_pos" topic
        curr_pos=Vector3()
        curr_pos.x=round(trans[0],3)
        curr_pos.z=round(trans[2],3)
        pose_pub.publish(curr_pos)

        # Send the current x and z position of the end effector via "/ref_pos" topic
        ref_pos = Vector3()
        ref_pos.x = 1.0
        ref_pos.z = 2.0
        ref_pub.publish(ref_pos)

        rate.sleep()
