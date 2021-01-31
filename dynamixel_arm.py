import rospy
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64
import time
from Arm import Arm, ArmPositionController
from constants import *

def callback_arm_joystick(msg):
    global delta_x, delta_y

    delta_x = msg.x * 0.5 if abs(msg.x) > 0.01 else 0
    delta_y = msg.y * 0.5 if abs(msg.y) > 0.01 else 0

def callback_arm_reboot(msg):
    global reboot
    if msg.data:
        reboot = True

def callback_arm_torque(msg):
    global torque
    if msg.data:
        torque = not torque
        arm.set_torque(ids, torque)

def callback_arm_estop(msg):
    rospy.signal_shutdown("Estop was pressed")

arm = Arm.Arm(ids, offsets)
controller = Arm.ArmPositionController(arm, rotation_motor)
reboot = False
torque = True
delta_x = 0
delta_y = 0
position = (0, 0)

rospy.init_node('dynamixel_arm')

rospy.Subscriber("dynamixel_arm_estop", Bool, callback_arm_estop)
rospy.Subscriber("dynamixel_arm_joystick", Vector3, callback_arm_joystick)
rospy.Subscriber("dynamixel_arm_torque", Bool, callback_arm_torque)
rospy.Subscriber("dynamixel_arm_reboot", Bool, callback_arm_reboot)

rate = rospy.Rate(20)
while not rospy.is_shutdown():
    # if reboot:
    #     controller.start_reboot_sequence()
    #     reboot = False
    #     continue

    # if abs(delta_x) + abs(delta_y) > 0:
    #     x, y = position
    #     position = (x + delta_x, y + delta_y)
    #     controller.move(position[0], position[1])
    rate.sleep()
    pass