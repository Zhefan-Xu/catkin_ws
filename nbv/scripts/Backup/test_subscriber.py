#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import math
from math import atan2
np.set_printoptions(threshold=np.inf)

class monitor:
    def __init__(self):
        #self.x_pos = 0
        #self.y_pos = 0
        #self.orientation = 0

        self.free = []
        self.camera_angle = np.pi*2
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        #self.current_node = (self.x_pos, self.y_pos)

    def planning(self):
        self.update()
        x_idx = int((self.current_x_pos - self.current_origin_x)/0.05)
        y_idx = int((self.current_y_pos - self.current_origin_y)/0.05)
        current_node = (x_idx, y_idx)
        a = 1
        while (True):
            max_gain = 0
            i = 0 
            while i <= 200 or max_gain == 0: # Sample 100 points
                random_node = self.randomSample()
                frontier, angle = self.visible(random_node)
                dis = self.distance(current_node, random_node)
                gain = len(frontier) * np.exp(-5*dis)
                i += 1
                if (gain > max_gain and random_node != current_node):
                    max_gain = gain
                    max_node = random_node
                    max_angle = angle
                    max_frontier = frontier
            print(current_node)
            print(max_node)
            #print(max_frontier)
            #print(self.current_map[200][200])

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
        x = self.x_pos
        y = self.y_pos
        
        (gx, gy) = goal
        (sx, sy) = start
        dx = gx - sx
        dy = gy - sy
        angle = atan2(dy, dx)
        print(angle)
        speed = Twist()
        r = rospy.Rate(4)
        while not rospy.is_shutdown():
            #x_idx = int((self.x_pos-self.current_origin_x)/0.05)
            #y_idx = int((self.x_pos-self.current_origin_x)/0.05)
            (z1, z2,theta) = euler_from_quaternion([self.orientation.x,self.orientation.y, self.orientation.z,self.orientation.w])
            #inc_x = gx - x_idx
            #inc_y = gy - y_idx
            #angle = atan2(inc_y, inc_x)
            #angle = atan2(inc_y, inc_x)
            print(abs(angle - theta))
            if abs(angle - theta) > 0.15:
                speed.linear.x = 0
                speed.angular.z = 0.05
            elif (inc_x > 0.1):
                speed.linear.x = 0.05
                speed.angular.z = 0.0
            else:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                self.pub.publish(speed)
                break
            self.pub.publish(speed)
            r.sleep()

    def rrt(self, start, goal, max_angle):
        self.nodes = [start]
        self.came_from = dict()
        self.came_from[str(start)] = None
        t0 = time.clock()
        while (True):
            delta_t = time.clock() - t0
            if (delta_t > 40):
                break
            q_rand = self.randomConfig()
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

    def randomConfig(self):
        x = np.random.uniform(self.current_x_min, self.current_x_max)
        y = np.random.uniform(self.current_y_min, self.current_y_max)
        q_rand = (x, y)
        return q_rand

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

            #plt.plot(x1, y1, 'bo', markersize = 2)
            #plt.plot([x1,x2], [y1,y2], 'c-')
            #plt.plot([x1,xg], [y1,yg], 'c-')
            return False
        else:
            (x1, y1) = q_new
            x1, y2 = int(x1), int(y1)
            (x2, y2) = q_near
            x2, y2 = int(x2), int(y2)
            if (self.current_map[x1][y1] == 0):
                #plt.plot(x1, y1, 'bo', markersize = 2)
                #plt.plot([x1,x2], [y1,y2], 'c-')
                #plt.pause(0.0001)
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

                
                #if (y_line == int(y_line)):
                #    check_linepoint1 = self.map[x][int(y_line)] == 1
                #check_linepoint1 = False

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

                
                #if (x_line == int(x_line)):
                #    check_linepoint2 = self.map[int(x_line)][y] == 1
                #check_linepoint2 = False

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
        self.current_map = self.map
        self.current_free = self.free
        self.current_x_max, self.current_x_min, self.current_y_max, self.current_y_min = \
        self.x_max, self.x_min, self.y_max, self.y_min
        self.current_origin_x = self.origin_x
        self.current_origin_y = self.origin_y
        self.current_x_pos = self.x_pos
        self.current_y_pos = self.y_pos


    def callback_position(self,msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.orientation = msg.pose.pose.orientation
        #rospy.loginfo('Position: '+str(self.x_pos)+', '+str(self.y_pos))
        
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
        #self.area_known = (self.map_width*self.map_height) - np.count_nonzero(self.map)

        ## Step 1: random sampling
        # Get free space array
        for x in range(self.map_width):
            for y in range(self.map_height):
                if self.map[x][y] == 0 and (x, y) not in self.free:
                    self.free.append((x, y))

        # Sampling the max gain point


        #self.current_node = max_node
        #current
        #self.rrt(current_node, max_node, max_angle)
        #print(max_node, max_angle, max_gain, max_frontier)
        #print(self.map[200])


        #rospy.loginfo('x index: ' + str(round((self.x_pos - self.origin_x)/self.resolution))+\
        #', y index: ' + str(round((self.y_pos - self.origin_y)/self.resolution)))

        ## Origin of map
        #rospy.loginfo('x: '+str(self.origin_x)+' ,y: '+str(self.origin_y))
        
        ## Map
        #rospy.loginfo(self.mymap[202])

        ## Map width, height, resolution
        #rospy.loginfo('map width: '+str(self.map_width))
        #rospy.loginfo('map height: '+str(self.map_height))
        #rospy.loginfo('map resolution: '+str(self.resolution))
        #rospy.loginfo('known area: '+str(self.area_known))

    def randomSample(self):
        index = np.random.randint(0, len(self.current_free))
        return self.current_free[index]

    def visible(self, current_node):
        frontier = []
        # 180 degree camera

        (xc, yc) = current_node

        # Direction angle (random)
        angle_direction = [0, np.pi/4, np.pi/2, np.pi*3/4, np.pi, -np.pi/4, -np.pi/2, -np.pi*3/4]
        index = np.random.randint(0, 8)
        a = angle_direction[index]
        # x, y for direction vector
        xv = np.cos(a)
        yv = np.sin(a)
        # Find the possible range x and y:
        directions = self.direction(current_node)


        for direction in directions:
            ## check if the direction is already mapped
            (x, y) = direction
            inrange = x <= self.current_x_max and x >= self.current_x_min and y <= self.current_y_max and y >= self.current_y_min
            if inrange and (self.current_map[x][y] == -1):
                dx = x - xc
                dy = y - yc

                cosa = (dx * xv + dy * yv)/((dx*dx+dy*dy)**0.5 * (xv*xv+yv*yv)**0.5)
                angle = np.arccos(cosa)
                if (angle <= self.camera_angle/2):
                    frontier.append(direction)
                    #self.map[x][y] = 3

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

def main():
    monitor1 = monitor()
    rospy.init_node('monitor', anonymous = True)
    #rospy.Subscriber('/initialpose')
    rospy.Subscriber('/odom', Odometry, monitor1.callback_position)
    time.sleep(1)
    rospy.Subscriber('/map', OccupancyGrid, monitor1.callback_map)
    time.sleep(0.1)
    monitor1.planning()
    rospy.spin()

if (__name__ == '__main__'):
    main()