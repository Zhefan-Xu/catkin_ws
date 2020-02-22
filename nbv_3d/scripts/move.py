#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from math import radians, copysign, sqrt, pow, pi, atan2
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from tf.transformations import euler_from_quaternion
import message_filters
from nav_msgs.msg import Odometry

#global reach, gx, gy
reach = False
gx = -100
gy = -100
idx = 1



def callback(path_msg):
    #x = path_msg.poses[0].pose.position.x
    global reach, gx, gy
    x = path_msg.poses[len(path_msg.poses)-1].pose.position.x
    y = path_msg.poses[len(path_msg.poses)-1].pose.position.y
    reach = (gx == x and gy == y)


    path = convertPath(path_msg)
    
    if (reach == False):
        for i in range(len(path)-1):
            makeMove(path[i], path[i+1])
            if (i+1 == len(path)-1):
                reach = True
    reach = (gx == x and gy == y)
    #rospy.loginfo("finish")
    (gx, gy) = path[len(path)-1]

# def callback(path_msg, odom_msg):
#     #x = path_msg.poses[0].pose.position.x
#     print(1)
#     global idx
#     goal_pub = rospy.Publisher("/get_goal", Point, queue_size=10)
#     positionx = odom_msg.pose.pose.position.x
#     positiony = odom_msg.pose.pose.position.y
#     positionz = odom_msg.pose.pose.position.z

#     path = convertPath(path_msg)
#     (next_x, next_y, next_z) = path[idx]
#     next_point = Point()
#     next_point.x = next_x
#     next_point.y = next_y
#     next_point.z = next_z

#     epsx = abs(positionx - next_x)
#     epsy = abs(positiony - next_y)
#     epsz = abs(positionz - next_z)
#     reach = epsx < 0.1 and epsy < 0.1 and epsz < 0.1
#     if (reach == False):
#         goal_pub(next_point)
#     elif (idx >= len(path)-1):
#         idx = 1
#     else:
#         idx += 1 



def listener():
    rospy.init_node("move", anonymous = True)
    rospy.Subscriber("path", Path, callback)
    #rospy.Timer(rospy.Duration(2), callback)
    rospy.spin()

# def listener():
#     rospy.init_node("move", anonymous = True)
#     path_sub = message_filters.Subscriber("path", Path)
#     odom_sub = message_filters.Subscriber("odom", Odometry)
#     ts = message_filters.ApproximateTimeSynchronizer([path_sub, odom_sub], 10, 0.1, allow_headerless=True)
#     ts.registerCallback(callback)
#     rospy.spin()


def makeMove(start, goal):
    (gx, gy) = goal
    (sx, sy) = start
    dx = gx - sx
    dy = gy - sy
    angle = atan2(dy, dx)
    #goal_x = self.current_origin_x + 0.05 * gx
    #goal_y = self.current_origin_y + 0.05 * gy
    GotoPoint((gx, gy), angle)


def convertPath(path_msg):
    path = []
    for point in path_msg.poses:
        x = point.pose.position.x
        y = point.pose.position.y
        path.append((x, y))
    return path

# def convertPath(path_msg):
#     path = []
#     for point in path_msg.poses:
#         x = point.pose.position.x
#         y = point.pose.position.y
#         z = point.pose.position.z
#         path.append((x, y, z))
#     return path

class GotoPoint():
    def __init__(self, goal, angle):
        #rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        (goal_x, goal_y) = goal
        goal_z = angle
        #(goal_x, goal_y, goal_z) = self.getkey()
        #if goal_z > 180 or goal_z < -180:
        #    print("you input wrong z range.")
        #    self.shutdown()
        #goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            move_cmd.angular.z = angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        (position, rotation) = self.get_odom()

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            r.sleep()


        #rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
    def getkey(self):
        x, y, z = raw_input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
if __name__ == "__main__":
    listener()