#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import time
import math
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
from modify_map import modify
np.set_printoptions(threshold=np.inf)

class monitor:
    def __init__(self):
        self.camera_angle = np.pi*2
        self.camera_dis = 60 # 3/0.05


    def planning(self):
        self.update()
        x_idx = int((self.current_x_pos - self.current_origin_x)/0.05)
        y_idx = int((self.current_y_pos - self.current_origin_y)/0.05)
        self.update()
        current_node = (x_idx, y_idx)

        a = 0
        while (a == 0):
            max_gain = 0
            i = 0 
            while i <= 1000 or max_gain == 0: # Sample 100 points
                random_node = self.randomSample()
                frontier, angle = self.visible(random_node)
                dis = self.distance(current_node, random_node)
                gain = len(frontier) * np.exp(-0.05*dis)
                i += 1
                if (gain > max_gain and random_node != current_node):
                    max_gain = gain
                    max_node = random_node
                    max_angle = angle
                    max_frontier = frontier
            print(current_node)
            print(max_node)


            path = self.rrt(current_node, max_node, max_angle)
            if (path == None):
                print("bad sample!")
                continue
            else:
                print(path)
                for i in range(len(path)-1):
                    self.makeMove(path[i], path[i+1])
                    print('complete ' + str(i))

            
            current_node = max_node
            self.update()
            a += 1

    def makeMove(self, start, goal):
        (gx, gy) = goal
        (sx, sy) = start
        dx = gx - sx
        dy = gy - sy
        angle = atan2(dy, dx)

        goal_x = self.current_origin_x + 0.05 * gx
        goal_y = self.current_origin_y + 0.05 * gy

        GotoPoint((goal_x, goal_y), angle)




    def rrt(self, start, goal, max_angle):
        self.nodes = [start]
        self.came_from = dict()
        self.came_from[str(start)] = None
        t0 = time.clock()
        while (True):
            delta_t = time.clock() - t0
            if (delta_t > 40):
                break
            #q_rand = self.randomConfig()
            q_rand = self.randomSample()
            #print(q_rand)
            status = self.extend(q_rand, goal)

            if (status == True):
                print("found")
                #print(self.came_from)
                path = self.pathFinder(goal)
                #print(path)
                new_path = self.shortCut(path, goal)
                #print(new_path)
                print("complete path")
                return new_path
                break


    def extend(self, q_rand, goal):
        q_near = self.nearest(q_rand)
        #print(q_near)
        (xn, yn) = q_near
        q_new = q_near
        directions = self.direction(q_near)
        min_distance = 1000
        for direction in directions:
            
            (xd, yd) = direction
            if (abs(xd-xn)+abs(yd-yn)) == 2:
                x_ = xn + (xd-xn)/2**0.5
                y_ = yn + (yd-yn)/2**0.5
                q_adjust = (x_, y_)
                dis = self.distance(q_rand, q_adjust)

            else:
                dis = self.distance(q_rand, direction)
            if (dis < min_distance and direction not in self.nodes):
                min_distance = dis
                q_new = direction

        if (q_new == q_near or q_new in self.nodes):
            return False

        if (q_new == goal):

            self.nodes.append(goal)
            self.came_from[str(goal)] = q_near
            return True

        if (self.distance(q_new, goal) <= 2**0.5):
            (x1, y1) = q_new
            x1, y2 = int(x1), int(y1)
            (x2, y2) = q_near
            x2, y2 = int(x2), int(y2)
            (xg, yg) = goal
            xg, yg = int(xg), int(yg)
            if (self.current_map[x1][y1] == 0):
                self.nodes.append(q_new)
                self.came_from[str(q_new)] = q_near
                self.nodes.append(goal)
                self.came_from[str(goal)] = q_new
                return True

            return False
        else:
            (x1, y1) = q_new
            x1, y2 = int(x1), int(y1)
            (x2, y2) = q_near
            x2, y2 = int(x2), int(y2)
            if (self.current_map[x1][y1] == 0):
                self.nodes.append(q_new)
                self.came_from[str(q_new)] = q_near
            return False
            
    def nearest(self, q_rand):
        min_distance = 1000
        for node in self.nodes:
            dis = self.distance(node, q_rand)
            if min_distance > dis:
                min_distance = dis
                q_near = node

        return q_near

    def pathFinder(self, goal):
        current = goal
        path = [goal]
        while (self.came_from[str(current)] != None):
            path.insert(0, self.came_from[str(current)])
            current = self.came_from[str(current)]
        return path
    def checkCollision(self, node1, node2):
        (x1, y1) = node1 #x1 = 51 y1 = 50
        (x2, y2) = node2 #x2 = 52 y2 = 52

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        if (x1 != x2 and y1 != y2):
            k = float(y2 - y1)/float(x2 - x1)
            b = y2 - k * x2
            def line1(x):
                return k*x+b
            def line2(y):
                return y/k - b/k

            x_range = np.arange(min(x1, x2)+1, max(x1, x2))
            for x in x_range:
                y_line = line1(x)
                if math.ceil(y_line) <= self.current_y_max:
                    y_upper = int(math.ceil(y_line))
                    y_lower = y_upper - 1
                    check_surroundings1 = self.current_map[x][y_upper] == 0 and self.current_map[x][y_lower] == 0 
                else:
                    y_lower = int(math.ceil(y_line)) - 1
                    check_surroundings1 = self.current_map[x][y_lower] == 0



                if (check_surroundings1 == False): #and check_linepoint1 == False:
                    return False

            y_range = np.arange(min(y1, y2)+1, max(y1, y2))
            for y in y_range:
                x_line = line2(y)
                if math.ceil(x_line) <= self.current_x_max:
                    x_upper = int(math.ceil(x_line))
                    x_lower = x_upper - 1
                    check_surroundings2 = self.current_map[x_upper][y] == 0 and self.current_map[x_lower][y] == 0 
                else:
                    x_lower = int(math.ceil(x_line)) - 1
                    check_surroundings2 = self.current_map[x_lower][y] == 0


                if (check_surroundings2 == False): #and check_linepoint2 == False:
                    return False

        elif (dx == 0):
            y_range = np.arange(min(y1, y2), max(y1, y2)+1)
            for y in y_range:
                if (self.current_map[x1][y] != 0):
                    return False

        elif (dy == 0):
            x_range = np.arange(min(x1, x2), max(x1, x2)+1)
            for x in x_range:
                if (self.current_map[x][y1] != 0):
                    return False
        return True

    def shortCut(self, path, goal):
        new_path = [path[0]]
        p = len(path) - 1
        current_node = path[0]
        while (True):
            if (self.checkCollision(current_node, path[p]) == True): # No Collision
                new_path.append(path[p])
                if (path[p] == goal):
                    break
                current_node = path[p]
                p = len(path)
            p -= 1



        return new_path

    def update(self):
        self.current_map, self.current_free = modify(self.map, 2) ## radius = 2
        self.current_x_max, self.current_x_min, self.current_y_max, self.current_y_min = \
        self.x_max, self.x_min, self.y_max, self.y_min
        self.current_origin_x = self.origin_x
        self.current_origin_y = self.origin_y




        
    def callback_map(self,msg):
        ## In this case msg is OccupancyGrid

        ## Getting Information
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.x_max, self.x_min, self.y_max, self.y_min = self.map_width-1, 0, self.map_height-1, 0
        self.resolution = msg.info.resolution
        mymap = msg.data ## raw map
        self.map = np.array(mymap).reshape(self.map_height, self.map_width)
        self.map = self.map.T


    def randomSample(self):
        index = np.random.randint(0, len(self.current_free))
        return self.current_free[index]

    def visible(self, current_node):
        frontier = []
        # 180 degree camera

        (xc, yc) = current_node

        # Direction angle (random)

        a = np.random.uniform(0, np.pi*2)
        xv = np.cos(a)
        yv = np.sin(a)

        x_min, x_max, y_min, y_max = xc-self.camera_dis, xc+self.camera_dis, yx-self.camera_dis, yc+self.camera_dis
        x_range = np.arange(x_min, x_max+1)
        y_range = np.arange(y_min, y_max+1)
        for x in x_range:
            for y in y_range:
                dis = self.distance(current_node, (x, y))
                inrange = x <= self.current_x_max and x >= self.current_x_min and y <= self.current_y_max and y >= self.current_y_min
                dx = x - xc
                dy = y - yc
                cosa = (dx * xv + dy * yv)/((dx*dx+dy*dy)**0.5 * (xv*xv+yv*yv)**0.5)
                angle = np.arccos(cosa)
                if (inrange == True and self.current_map[x][y] == -1 and angle <= self.camera_angle/2 and dis <= self.camera_dis):
                    directions = self.direction((x, y))
                    for direction in directions:
                        xd, yd = direction
                        if (xd, yd) == 0:
                            frontier.append((x, y))
                            break


        return frontier, a

    def direction(self, node):
        (xc, yc) = node
        directions = []
        if (xc-1 >= self.x_min):
            left = (xc-1, yc)
            directions.append(left)
            if (yc+1 <= self.x_max):
                upleft = (xc-1, yc+1)
                directions.append(upleft)
            if (yc-1 >= self.y_min) :
                downleft = (xc-1, yc-1)
                directions.append(downleft)

        if (xc+1 <= self.x_max):
            right = (xc+1, yc)
            directions.append(right)
            if (yc+1 <= self.x_max):
                upright = (xc+1, yc+1)
                directions.append(upright)
            if (yc-1 >= self.y_min):
                downright = (xc+1, yc-1)
                directions.append(downright)

        if (yc+1 <= self.x_max):
            up = (xc, yc+1)
            directions.append(up)

        if (yc-1 >= self.x_min):
            down = (xc, yc-1)
            directions.append(down)
   
        return directions

    def distance(self, node1, node2):
        (x1, y1) = node1
        (x2, y2) = node2
        return ((x1 - x2)**2 + (y1 - y2)**2)**0.5




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


        rospy.loginfo("Stopping the robot...")
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

def main():
    monitor1 = monitor()
    rospy.init_node('monitor', anonymous = True)
    #rospy.Subscriber('/initialpose')
    rospy.Subscriber('/map', OccupancyGrid, monitor1.callback_map)
    time.sleep(0.1)
    monitor1.planning()
    rospy.spin()

if (__name__ == '__main__'):
    main()
