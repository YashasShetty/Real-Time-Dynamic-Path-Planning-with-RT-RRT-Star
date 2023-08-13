import math
import random
import argparse
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, pi
from utils import Node, Base_Class

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time


e = """
Communications Failed
"""

x = 0.0
y = 0.0 
theta = 0.0
pathpoints =[]

def callback(msg):
    # print(msg.pose.pose)
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def test(pathpoints):
	msg=Twist()

	pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	rospy.init_node('robot_talker',anonymous=True)

	# rospy.init_node('check_odometry')
	goal_reached = False

	try :
		while not rospy.is_shutdown():
			msg.angular.z=0
			msg.linear.x=-0.1
			#buff='my current time is %s" %rospy.get_time()
			
			odom_sub = rospy.Subscriber('/odom', Odometry, callback)
			# rospy.spin()

			# pathpoints = [[-4,-4],[-2,-3],[-2,-2],[0,-3]]
			
			skipper = 0
			
			while goal_reached is False:
				for points in pathpoints:
					point_reached = False
					skipper += 1
					if skipper % 4 == 0 or  skipper == len(pathpoints):
                        
						while point_reached is False:
							goal_x = points[0]
							goal_y = points[1]
                                                        

							inc_x = goal_x - x 
							inc_y = goal_y - y

							dist = math.sqrt(inc_x**2 + inc_y**2)

							angle_to_goal = math.atan2(inc_y, inc_x)

							angle_diff = angle_to_goal-theta
							if abs(angle_to_goal-theta) > 0.5 :
								msg.angular.z=np.sign(angle_to_goal-theta)*0.5
								msg.linear.x=-0.0
							elif abs(angle_to_goal-theta) > 0.1 :
								msg.angular.z=np.sign(angle_to_goal-theta)*0.1
								msg.linear.x=-0.0
							else :
								msg.angular.z=0
								if dist > 0.35 :
									msg.linear.x= 0.22
								elif dist > 0.1:
									msg.linear.x= 0.05
								else :
									msg.linear.x= 0.0
									point_reached = True

							# print("dist: ", dist )
							# print("angle diff :",angle_diff )
							# print("lin vel", msg.linear.x)
							# print("ang vel", msg.angular.z)
							pub.publish(msg)
                            # print(points[0],points[1])
				goal_reached = True
					
			

			time.sleep(0.1)


	except :
		print(e)
	finally :	
		msg.angular.z=0
		msg.linear.x=-0
		#buff='my current time is %s" %rospy.get_time()
		pub.publish(msg)

	

class RT_RRTStar(Base_Class):

    class Node(Node):
        def __init__(self, x, y):
            super().__init__(x, y)
        # cost to go
        def cost(self):
            if not self.parent:
                return 0.0
            cost = self.get_path_cost()
            node = self.parent
            while node.parent:
                cost = cost + node.get_path_cost()
                node = node.parent
            return cost
        #cost to come
        def get_path_cost(self):
            path_horizontal = self.path_horizontal()
            path_vertical = self.path_vertical()
            dx = path_horizontal[1] - path_horizontal[0]
            dy = path_vertical[1] - path_vertical[0]
            return math.hypot(dx,dy)

    def __init__(self, start, goal, circular_obstacles, line_obstacles, world_map,
                 expand_radius=0.2,
                 goal_sample_rate=5,
                 connect_circle_dist=0.3,
                 node_list = [],
                 kmax = 100
                 ):
        super().__init__(start, goal, circular_obstacles, line_obstacles, world_map, expand_radius, goal_sample_rate, node_list)
        self.node_list = node_list
        self.start_time = time.time()
        self.kmax = kmax
        self.connect_circle_dist = connect_circle_dist
    #backtracking for the final code
    def backtracking2(self, goal_ind):
        path = []
        node = self.node_list[goal_ind]
        while node.parent:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        
        return path
    
    #generate the output of the graph in matplotlib
    def generate_graph(self, rnd=None):
        
        # fig = plt.figure()
        # ax = fig.add_subplot(111)
        
        # rect1 = matplotlib.patches.Rectangle((-2, -1),
        #                                     4, 2,
        #                                     color ='green')
        # ax.add_patch(rect1)
  
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        # if rnd is not None:
            # plt.plot(rnd.x, rnd.y, "^k")
        
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_horizontal(), node.path_vertical(), "c")
                # plt.plot(node.path_horizontal(), node.path_vertical(), "-w")
        
        obstacle_count = 0
        for (ox, oy, size) in self.circular_obstacles:
            if obstacle_count < 2:
                self.generate_circle(ox, oy, size, color= 'r')
                obstacle_count += 1
            else :
                self.generate_circle(ox, oy, size,color= 'g')

        obstacle_count = 0
        for (sx,sy,ex,ey) in self.line_obstacles:
            if obstacle_count < 8:
                x1, y1 = [sx, ex], [sy, ey]
                plt.plot(x1, y1,color = 'g')
                obstacle_count += 1
            else:
                x1, y1 = [sx, ex], [sy, ey]
                plt.plot(x1, y1,color = 'r')

        pathpoints.append([self.start.x,self.start.y])
        plt.axis('square')
        plt.plot(self.start.x, self.start.y, "xr" , markersize=12)
        plt.plot(self.start.x, self.start.y, "Db" , markersize=10)
        plt.plot(self.end.x, self.end.y, "oy" , markersize = 10)
        
        # plt.plot(self.start.x, self.start.y, "xw" , markersize=10)
        # plt.plot(self.end.x, self.end.y, "Dw" , markersize = 10)
        plt.title(label="RT-RRT")

        plt.axis([self.min_randx, self.max_randx, self.min_randy, self.max_randy])
        plt.xticks(color='w')
        plt.yticks(color='w')
        plt.grid(False)
        plt.pause(0.001)
    
    #main RT-RRTstar planning algorithm
    def planning(self):
        if not self.node_list:
            self.node_list = [self.start]
        
        start_ind = self.get_node_index_xy(self.start.x, self.start.y)
        if self.node_list[start_ind].parent:
            self.change_root(start_ind)
            
        qs = []
        i = 0
        
        while time.time() - self.start_time < 0.5:
            
            rnd = self.get_random_node()
            nearest_ind = self.neighbour_node_index(rnd)
            nearest_node = self.node_list[nearest_ind]
            
            new_node = self.rewire_random_node(nearest_node, rnd, self.expand_radius)

            if self.is_in_obstacle(new_node, self.circular_obstacles, self.line_obstacles):
                near_inds = self.find_near_nodes(new_node)
                if len(near_inds) < self.kmax:
                    new_node = self.choose_parent(new_node, near_inds)
                    if new_node and (not self.check_node_in_list(new_node.x, new_node.y)):
                        self.node_list.append(new_node)
                        self.rewire_node(new_node, near_inds)

            if i % 5 == 0:
                self.generate_graph(rnd)
            i = i + 1
        
        last_index = self.search_best_goal_node()
        if last_index:
            path = self.backtracking(last_index)
            next_ind = self.get_next_root_ind(path)
            return True, path, self.node_list, next_ind

        path = self.backtracking2(self.get_node_closest_to_end())
        next_ind = self.get_next_root_ind(path)
        return False, path, self.node_list, next_ind

    #rewiring the nodes according toi algorithm 4 and 5
    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None
    
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.rewire_random_node(near_node, new_node)
            if t_node and self.is_in_obstacle(t_node, self.circular_obstacles, self.line_obstacles):
                costs.append(t_node.cost())
            else:
                costs.append(float("inf"))
        min_cost = min(costs)

        if min_cost == float("inf"):
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.rewire_random_node(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        
        return new_node            

    #search for best goal while rewiring the random node
    def search_best_goal_node(self):
        dist_to_goal_list = [self.cost_to_go(n.x, n.y) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_radius]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.rewire_random_node(self.node_list[goal_ind], self.end)
            if self.is_in_obstacle(t_node, self.circular_obstacles, self.line_obstacles):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost() for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost() == min_cost:
                end_ind = self.get_node_index_xy(self.end.x, self.end.y)
                if end_ind:
                    self.node_list[end_ind].parent = self.node_list[i]
                return i

        return None

    #finding nearby nodes
    def find_near_nodes(self, new_node):
        dist_list = [(node.x - new_node.x) ** 2 +
                     (node.y - new_node.y) ** 2 for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= self.connect_circle_dist  ** 2]
        return near_inds

    #check the node in the list
    def check_node_in_list(self, x, y):
        for node in self.node_list:
            if node.x == x and node.y == y:
                return True
        return False
    
    #generate the node index
    def get_node_index_xy(self, x, y):
        for i in range(0,len(self.node_list)):
            node = self.node_list[i]
            if node.x == x and node.y == y:
                return i
        return None

    #unpdate the cost
    def calc_new_cost(self, from_node, to_node):
        d, theta = self.cost_to_come(from_node, to_node)
        return from_node.cost() + d
    
    def get_node_closest_to_end(self):
        dist_to_goal_list = [self.cost_to_go(n.x, n.y) for n in self.node_list]
        min_ind = dist_to_goal_list.index(min(dist_to_goal_list))
        return min_ind
        
    def find_next_nodes_of(self, cnode, node_list):
        next_inds = [node_list.index(i) for i in node_list if i.parent and i.parent.x == cnode[0] and i.parent.y == cnode[1]]
        return next_inds

    #rewire the nodes
    def rewire_node(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.Node(near_node.x,near_node.y)
            edge_node.parent = new_node
            if not edge_node:
                continue

            no_collision = self.is_in_obstacle(edge_node, self.circular_obstacles, self.line_obstacles)
            cost_improved = edge_node.cost() < near_node.cost()

            if no_collision and cost_improved:
                self.node_list[i].parent = new_node

    #change the porent to root
    def change_root(self, next_root_ind):
        current_root_ind = self.get_node_index_xy(self.node_list[next_root_ind].parent.x, self.node_list[next_root_ind].parent.y)
        self.node_list[next_root_ind].parent =  None
        self.node_list[current_root_ind].parent = self.node_list[next_root_ind]

    def get_next_root_ind(self, path):
        next_ind = 0
        if len(path) == 2:
            next_root_x = path[1][0]
            next_root_y = path[1][1]
            next_ind = self.get_node_index_xy(next_root_x,next_root_y)
        
        elif len(path) > 2:
            next_root_x = path[-2][0]
            next_root_y = path[-2][1]
            next_ind = self.get_node_index_xy(next_root_x,next_root_y)
        else: 
            next_root_x = path[0][0]
            next_root_y = path[0][1]
            next_ind = self.get_node_index_xy(next_root_x,next_root_y)
        return next_ind
    
def main():
   
    starttime = time.time()

    fig = plt.figure()
    ax = fig.add_subplot()

    # parser = argparse.ArgumentParser(description='Arguments for RT-RRT')
    
    # parser.add_argument('--Start', default = [-2 , -0.5 ] , nargs='+', type=float,  help = 'Starting x and y position')
    # parser.add_argument('--Goal',  default = [-3 , -0.5 ] , nargs='+', type=float, help = 'Goal x and y position')
    # parser.add_argument('--World_Map',  default = [-4  , 4 , -4 ,4 ] , nargs='+', type=float ,help = 'World Map [ x_lower , x_upper , y_lower , y_upper]')
    # parser.add_argument('--time', default = 30.0 ,  type=float ,help = 'Time for robot to find  and reach the goal')
    # parser.add_argument('--expand', default = 0.2 ,  type=float ,help = 'Expand Radius')
    # parser.add_argument('--rewire_radius', default = 0.2 ,  type=float ,help = 'Rewire Radius')


    # # parser.add_argument('--Line_Obstacles',  default = [ (4, 0.3, 0, 0.9),(-2, 0.5, 1, 0.5) ] , nargs='+', type=float ,help = 'Line Obstacles')

    # Args = parser.parse_args()
    # start = Args.Start
    # goal = Args.Goal
    # World_Map = Args.World_Map
    # t = Args.time
    # expand = Args.expand
    # rewire_radius = Args.rewire_radius
        
    # rewire_radius = Args.rewire_radius

    starttime = time.time()

    obstacleList = [
        (1,-1 ,1),
        (2,2,1),
        (1,-1 ,0.5),
        (2,2,0.5)
    ]

    #lines (x1 , y1 , x2 , y2)
    linearObstacleList = [

        (3, 0 , 4,0),
        (4,0 , 4,-3),
        (3,-3 , 4 ,-3),
        (3,0 , 3,-3), 


        (-2, 0 , -1,0),
        (-2,0 , -2,3),
        (-2,3 , -1 ,3),
        (-1,0 , -1,3),

        (2.5,0.5 , 4.5,0.5),
        (4.5,0.5 , 4.5,-3.5),
        (2.5,-3.5 , 4.5,-3.5),
        (2.5,0.5 , 2.5,-3.5), 


        (-2.5,-0.5 , -0.5,-0.5),
        (-2.5,-0.5 , -2.5,3.5),
        (-2.5,3.5 , -0.5,3.5),
        (-0.5 ,-0.5, -0.5, 3.5)
    ]

    #hyperparameters which can be tuned
    sx=-4.0
    sy=-0.5
    gx=0.0
    gy=2
    gx2 = -gx
    t = 30.0
    expand = 0.3
    rewire_radius = 1.2
    rewire = expand * rewire_radius
    World_Map = [-5 , 5 , -5 , 5]
    nodelist = []
    # def start_in_obstacle( start ):
    #     x , y = start[0] , start[0]
    # if start_in_obstacle([sx,sy]):
    #     print("start or goal is in obstacle")
        
        
    # if start_in_obstacle([gx,gy]):
    #     print("goal is in obstacle")
        
    
    rt_rrt_star = RT_RRTStar(start=[sx, sy],
                      goal=[gx, gy],
                      world_map=World_Map,
                      circular_obstacles=obstacleList,
                      line_obstacles=linearObstacleList,
                      expand_radius = expand,
                      connect_circle_dist = rewire,
                      node_list = [] )
    path_found, path_nodes, nodelist, next_ind = rt_rrt_star.planning()
    
    while time.time() - starttime < t:
        if time.time() - starttime > t/3:
            gx = gx2
            #main rt_rrt_star
        rt_rrt_star = RT_RRTStar(start=[nodelist[next_ind].x, nodelist[next_ind].y],
                      goal=[gx, gy + (time.time() - starttime)/ 15.0],
                      world_map=World_Map,
                      circular_obstacles=obstacleList,
                      line_obstacles=linearObstacleList,
                      expand_radius = expand,
                      connect_circle_dist = rewire,
                      node_list = nodelist )
        path_found, path_nodes, nodelist, next_ind = rt_rrt_star.planning()
        # print(path_nodes)
        
    endtime = time.time()
    print("Time taken to run the code: " , endtime - starttime)
    
    if path_found:
        if path_nodes:        
            rt_rrt_star.generate_graph()
            
            plt.plot([x for (x, y) in path_nodes], [y for (x, y) in path_nodes], '-r', alpha = 0.8)
            # plt.grid(False)
            # ax = plt.gca()
            ax.axis('equal')
            # ax.set_aspect('equal', adjustable='box')
            plt.pause(0.1)
            # plt.title(label="RT-RRT")
            plt.draw()
            # plt.show()
            
    else:
        print("path not found in the given time")

    test(pathpoints)

    print(pathpoints)
    


if __name__ == '__main__':
    
    main()


