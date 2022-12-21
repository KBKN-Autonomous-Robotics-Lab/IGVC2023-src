#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios



msg = """
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit
"""

e = """
Communications Failed
"""


def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (round(target_linear_vel,5), round(target_angular_vel,5))


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input
  




if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_key')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    max_vel_linear = rospy.get_param("teleop_key/max_vel_linear")
    max_vel_angular = rospy.get_param("teleop_key/max_vel_angular")
    linear_vel_step = rospy.get_param("teleop_key/linear_vel_step")
    angular_vel_step = rospy.get_param("teleop_key/angular_vel_step")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        print("Linear velocity range: -{0} ~ {0},  step: {1}".format(max_vel_linear, linear_vel_step))
        print("Angular velocity range: -{0} ~ {0},  step: {1}".format(max_vel_angular, angular_vel_step))
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w' :
                target_linear_vel = constrain(target_linear_vel+linear_vel_step, -max_vel_linear, max_vel_linear)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = constrain(target_linear_vel-linear_vel_step, -max_vel_linear, max_vel_linear)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = constrain(target_angular_vel+angular_vel_step, -max_vel_angular, max_vel_angular)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = constrain(target_angular_vel-angular_vel_step, -max_vel_angular, max_vel_angular)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (linear_vel_step/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (angular_vel_step/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)