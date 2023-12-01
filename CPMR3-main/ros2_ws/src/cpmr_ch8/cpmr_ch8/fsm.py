from enum import Enum
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import json
import os
import cv2
import random
from scipy.spatial import KDTree
# =======================================================================================================================================
# ORIGINAL FSM PROCEDURE:
# 1. At start (0,0,0) we wait 2 seconds -> FSM_STATES = AT_START
# 2. Now begin to move to task starting position (2,2,0) -> FSM_STATES = HEADING_TO_TASK
#    a. rotate so we are facing goal: logger = turning towards goal
#    b. once orienged correctly, drive to goal: logger = driving to goal
#    c. once at goal: logger = at goal pose and terminate HEADING_TO_TASK
#===================================================================================================
# HERE, WE NEED TO ADD OUR NEW TASK STATE AND PERFORM THE LAWN MOWING
#===================================================================================================
# 3. Now, we can start our return to origin -> FSM_STATES = RETURNING_FROM_TASK
#    a. rotate so we are facing our goal (0,0,0): logger = turning towards goal
#    b. once correctly oriented, drive to goal: logger = driving to goal
#    c. once at goal (0,0,0): logger = at goal pose and terminate FSM state
# 4. FSM_STATES = TASK_DONE
#


# DECLARABLES
# row_length
# row_offset
# design of our 'lawn-mowing' area:
# width (row_length) = 6m (so 6 squares)
# height = 4
# our row offset is going to determine how many times we 'cut the grass' In other words, its how many times we will be turning and how 
# comprehensively we cut the grass
# we will keep track of our row offset and our row 'number' so we know whether we will be turning the robot +90 degrees or -90 degrees(270 degrees)

row_width = 6
num_rows = 4
row_offset = 0.5 

# HARD CODED PARAMETER SET:



def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class FSM_STATES(Enum):
    AT_START = 'AT STart',
    FIND_PATH = 'Finding Path',
    HEADING_TO_TASK = 'Heading To Task Enterance ',
    HEADING_TO_RADIO_SITE = 'Heading To Radioactive site ',
    SCAN_SITE = 'Scanning Area',
    RETURNING_FROM_RADIO_SITE= 'Returning From Radioactive site ',    
    RETURNING_FROM_TASK = 'Heading Home',
    TASK_DONE = 'Task Done'

class FSM(Node):

    def __init__(self):
        super().__init__('FSM')
        self.get_logger().info(f'{self.get_name()} created')

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

        # the blackboard
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_theta = 0.0
        self._cur_state = FSM_STATES.AT_START
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self. currentGoal = []
        self.pathList = [] 
        self.robotSpeed=0.3
        self.currentIndex =0
        self.numberOfNodes = 1500

        self.fileName = "map.json"
        self.world_size = (1000, 1000)
        self.world = np.full((self.world_size[0], self.world_size[1], 3), 255, dtype=np.uint8)

    
   

    class point:
        def __init__(self, x, y,next = None, prev = None,radius = 1):
            self.x = x
            self.y = y
            self.next = next
            self.prev = prev
            self.radius = radius
            self.color = (127,127,0)

    #=======================================================================================================================================


    # here we are going to read in the json objects
    def read_json_file(self,filename):
        with open(filename, 'r') as file:
            data = json.load(file)
            return data



    white_image = np.ones((1000, 1000, 3), dtype=np.uint8) * 255



    # USE THIS METHOD TO DETERMINE IF A GENERATED POINT IS ON AN OBSTACLE
    def point_obst_overlap(self,world, p):
        def is_not_free(x, y):
            overlap = False
            if all(world[y, x] == [0, 0, 0]):
                overlap = True
            return overlap

        mapx, mapy = p.x, p.y

        if is_not_free(mapx, mapy):
            return True    
        return False

    # USE THIS METHOD TO DETIRMINE IF A LINE BETWEEN TWO VALID POINTS CROSSES OVER AN OBSTACLE
    def line_color_intersection(self,world, v1, v2):
        # we are going to check if the pixel at the specified coordinates is white
        def is_not_white(x, y):
            lineOverlap = False
            if all(world[y, x] == [0, 0, 0]):
                lineOverlap = True
                print('LINE COLOR INTERSECTION HAS OCCURED')
            return lineOverlap 
        # first, we are going to assign the point coordinates to new variables
        x1, y1, x2, y2 = v1.x, v1.y, v2.x, v2.y
        dx, dy = abs(x2 - x1), abs(y2 - y1)
        # now, the x and y coordinates will be that of the first point
        x, y = x1, y1
        print('X COORD TO BE CHECKED = ',x)
        print('Y VALUE TO BE CHECKED = ', y)
        # here, we do some shifting by steps. If x1 > x2. For example, if x1 is greater than x2 then we 
        # will take a negative step in the x axis with sx. Similar for the y axis with sy
        sx = -1 if x1 > x2 else 1
        sy = -1 if y1 > y2 else 1
        # err represents the direction we will move in along the line to get from one point to the next
        err = dx - dy
        while True:
            if x == x2 and y == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
            if is_not_white(x, y):
                return True
        print('NO LINE COLOR INTERSECTION')
        return False

    #=======================================================================================================================================


    #=======================================================================================================================================

    # GET THE DISTANCE BETWEEN TWO POINTS
    def findDistace (self,p1, p2):
        distance = math.sqrt((p1.x - p2.x)**2 + (p2.y - p1.y)**2)
        return distance

    # DRAW LINE BETWEEN POINTS
    def drawLine(self,v1,v2,color =(128,0,128), thickness=2):
        cv2.line(self.world, (v1.x,v1.y), (v2.x,v2.y), color, thickness)


    def bigBrainAlgo(self,exploredVertexList,listOfVertix):
        newConnection = 0 # This will be two different vertexes we will be returning
        smallestDistance = float('inf')
        for exploredV in exploredVertexList:
            for unexploredVertix in listOfVertix:
                calculateDistance = self.findDistace(exploredV,unexploredVertix)
                if calculateDistance < smallestDistance and self.line_color_intersection(self.world, exploredV, unexploredVertix) == False:
                    smallestDistance = calculateDistance
                    newConnection = (exploredV,unexploredVertix)

        return newConnection

    # FIND THE CLOSEST NODE TO A GRAPH
    def findClosestNodeToGraph(self,exploredVertexList, listOfVertix):
        # Convert vertex points to a list of tuples
        unexploredPoints = [(v.x, v.y) for v in listOfVertix]

        # Create a KD-tree with unexplored vertices
        tree = KDTree(unexploredPoints)

        smallestDistance = float('inf')
        newConnection = None

        for exploredV in exploredVertexList:
            # Find the nearest neighbor to exploredV in the KD-tree
            distance, index = tree.query((exploredV.x, exploredV.y))

            #if line_color_intersection(map, exploredV, listOfVertix[index]):
            if distance < smallestDistance and self.line_color_intersection(self.world, exploredV, listOfVertix[index]) == False:

                smallestDistance = distance
                newConnection = (exploredV, listOfVertix[index])
                print('NEW CONNECTION MADE')
            
        
            # Early exit if the distance is zero
            if distance == 0:
                break
        if newConnection == None:
            newConnection = self.bigBrainAlgo(exploredVertexList,listOfVertix)
        return newConnection

    # HERE WE WILL GENERATE RANDOM POINTS AND ADD THEM TO A LIST OF POINTS
    def buildMap(self,data):

        for _, circle in data.items():
            center = (int(circle["x"]) * 100, int(circle["y"]) * 100)
            radius = int(circle["r"] * 100)  # Assuming radius is a fraction of world size
            color = (0, 0, 0)  # circle obstacles are filled black
        
            cv2.circle(self.world, center, radius, color, -1)

        # first need to define the list of generated points
        randomPoints = [(np.random.randint(10, self.world.shape[1]-10), np.random.randint(10, self.world.shape[0]-10)) for _ in range(self.numberOfNodes)]
        listOfVertix = []
        # now, populate the map
        for i in range(0,self.numberOfNodes):
            v = self.point(x = randomPoints[i][0], y = randomPoints[i][1])
            # if a point is generated on an obstacle, change its colour and do NOT add it to new list
            if self.point_obst_overlap(self.world,v):
                v.color = (0, 255, 255) 
            else: 
                listOfVertix.append(v)
            # print(v.x)
            cv2.circle(self.world, (v.x,v.y), v.radius, v.color, thickness=-1)

        return listOfVertix

    #=======================================================================================================================================
    def getPathTo(self,start_x = 30, start_y = 700, finish_x = 130, finish_y = 850):
        
        # get the list of points
        listOfVertix = self.buildMap(data=self.read_json_file(self.fileName))

        # this is essentially our RRT list of nodes 
        exploredVertexList = []

        # RRT list to return to drive_to_goal
        rrt = []
        # starting index will be the first index of the list (its always random since the list is always randomly generated)
        startVertex = self.point(start_x, start_y)
        
        finishPoint = self.point(finish_x, finish_y)
        finishPoint.color=(255, 255, 0)
        print(f"{startVertex.x,startVertex.y}")
        print(f"{finishPoint.x,finishPoint.y}")

        cv2.circle(self.world, (startVertex.x,startVertex.y), 6, (0,255,0), thickness=-1)
        cv2.circle(self.world, (finishPoint.x,finishPoint.y), 6, (255,255,0), thickness=-1)

        # INSERT A POINT AT A RANDOM SPOT IN THE LIST (this will be replaced by the robot odometry position in gazebo)
        random_index = random.randint(0, len(listOfVertix))
        listOfVertix.insert(random_index, finishPoint)
        exploredVertexList.append(startVertex)

        # iterate through the list of points (vertices) until we reach the goal vertex
        while(len(listOfVertix) > 0):
            # graphNode is the node we are searching FROM and newNode is the node we are searching FOR
            graphNode, newNode = self.findClosestNodeToGraph(exploredVertexList, listOfVertix)

            graphNode.next = newNode
            newNode.prev = graphNode
            # if line_color_intersection(map, graphNode, newNode) == False:
            self.drawLine(graphNode, newNode) 
            exploredVertexList.append(newNode)
            listOfVertix.remove(newNode)
            # random.shuffle(listOfVertix)


            print('ELEMENTS IN LIST OF VERTIX: ', len(listOfVertix))
            print('GRAPH NODE: ', graphNode.x, graphNode.y)
            print('NEW NODE: ', newNode.x, newNode.y)
            print('********************************NODE ADDED TO RRT*********************')

            # check if we have reached the goal            
            if newNode.x == finishPoint.x and newNode.y == finishPoint.y:
                print('FINISH POINT REACHED. BREAK OUT OF LOOP')
                break
            
            print('\n')  
        while(newNode.prev != None):
            pointX = round(newNode.x/100.0,2)
            pointY = round(10 - newNode.y/100.0,2)
            rrt.append([pointX,pointY])
            print(f"Location: {newNode.x} , {newNode.y}")
            self.drawLine(newNode, newNode.prev,(0, 0, 255), 4) 
            newNode = newNode.prev
        cv2.imwrite("createdMap.jpg",self.world)
        return rrt 

    def _drive_to_goal(self, goal_x, goal_y, goal_theta):
        self.get_logger().info(f'CURRENT GOAL ({goal_x}, {goal_y})')
        self.get_logger().info(f'{self.get_name()} drive to goal')
        twist = Twist()

        x_diff = goal_x - self._cur_x
        y_diff = goal_y - self._cur_y
        dist = x_diff * x_diff + y_diff * y_diff
        self.get_logger().info(f'{self.get_name()} {x_diff} {y_diff}')

        # turn to the goal
        heading = math.atan2(y_diff, x_diff)
        if abs(self._cur_theta - heading) > math.pi/20: 
            if heading > self._cur_theta:
                twist.angular.z = 0.10
            else:
               twist.angular.z = -0.10
            self.get_logger().info(f'{self.get_name()} turning towards goal')
            self._publisher.publish(twist)
            return False

        # since we are now pointing to the right direction, go there
        if dist > 0.1*0.1:
            twist.linear.x = self.robotSpeed 
            self._publisher.publish(twist)
            self.get_logger().info(f'{self.get_name()} driving to goal')
            return False

        # we are there, set the correct angle
        if abs(goal_theta - self._cur_theta) > math.pi/20: 
            if goal_theta > self._cur_theta:
                twist.angular.z = 0.005
            else:
               twist.angular.z = -0.005
            self.get_logger().info(f'{self.get_name()} turning to goal direction')
            self._publisher.publish(twist)
        self.get_logger().info(f'{self.get_name()} at goal pose')
        return True
        
    def _do_state_at_start(self):
        self.get_logger().info(f'{self.get_name()} in start state')
        # getting the current time
        # and checking to see if 2 seconds have elapsed since program launch so that we can drive to goal
        now = self.get_clock().now().nanoseconds * 1e-9
        if now > (self._start_time + 2):
            # once the 2 seconds have passed, lets head to our task
            self._cur_state = FSM_STATES.HEADING_TO_TASK
        # self.get_logger().info(f'FINISHED START STATE in start state')

    def _do_state_heading_to_task(self):
        self.get_logger().info(f'{self.get_name()} heading to task {self._cur_x} {self._cur_y} {self._cur_theta}')
        if self._drive_to_goal(3, 3, math.pi/2):
            self._cur_state = FSM_STATES.FIND_PATH

    def _do_state_find_path(self):
        self.get_logger().info(f'{self.get_name()} in path finding state')
        self.get_logger().info(f'{self.get_name()} Current location is {self._cur_x, self._cur_y} ')

        # readImageTemp.process_json_data(data)
        # You need to convert to pixels when sending path information
        self.pathList = self.getPathTo(
            start_x = self._cur_x * 100,
            start_y = ( 10 - self._cur_y ) * 100,
            finish_x = 5 * 100, 
            finish_y = (10-5) * 100 ,
        )
    
        # Open the file in write mode ('w')
        with open("MapPathList.txt", 'w') as file:
            # Iterate through the list and write each element to a new line
            for item in self.pathList:
                file.write(f"{item}\n")

        self.currentGoal = [self.pathList[0][0],self.pathList[0][1],(math.pi/2)]
        self._cur_state = FSM_STATES.HEADING_TO_RADIO_SITE

    #=======================================================================================================================
    # HERE WE ARE HEADING TO GOAL (I.E., HEADING TO THE STARTING POSITION WHERE WE WILL BEGIN TO CUT THE GRASS)
    def _do_state_heading_radio_Site(self):
        print(self.currentGoal)
        isAtGoal = self._drive_to_goal(*self.currentGoal)
        self.get_logger().info(f'{self.currentGoal} PERFORMING TASK ')
        x=0
        y=1 
        import time
        
        if isAtGoal:
            curPoint = [self.currentGoal[0],self.currentGoal[1]]
            index = -1 
            try:
                # Get the index of the target tuple
                index = self.pathList.index(curPoint)
                # Print the index
                print(f"Current index: {index}")
            except ValueError:
                print(f"{curPoint} not found in the list.")
            if index == (len(self.pathList)-1): # Is at Goal 
                self.get_logger().info(f'{self.get_name()} Have reached the Radioactive site')
                self._cur_state = FSM_STATES.RETURNING_FROM_TASK
            else: 
                self._cur_state = FSM_STATES.SCAN_SITE
                #  self.get_logger().info(f'{self.get_name()} completed Scan of radioactive area')
                print(self.pathList)
                time.sleep(5)
                newCurX = self.pathList[index+1][0]
                newCurY = self.pathList[index+1][1]
                newTheta = self._cur_theta
                newGoal = [newCurX,newCurY,newTheta]
                self.currentGoal =  [newCurX,newCurY,newTheta]
                print(f"____RESET NEW GOAL TO {newGoal}____")
        
    def _do_state_returning_from_radio_Site(self): # not complete
        print(self.currentGoal)
        isAtGoal = self._drive_to_goal(*self.currentGoal)
        self.get_logger().info(f'{self.currentGoal} PERFORMING TASK ')
        x=0
        y=1 
        import time
        
        if isAtGoal:
            curPoint = [self.currentGoal[0],self.currentGoal[1]]
            index = -1 
            try:
                # Get the index of the target tuple
                index = self.pathList.index(curPoint)
                # Print the index
                print(f"Current index: {index}")
            except ValueError:
                print(f"{curPoint} not found in the list.")
            if len(self.pathList)-1 - index == 0: # Is at Goal 
                self.get_logger().info(f'{self.get_name()} Have reached the beginging of site')
                self._cur_state = FSM_STATES.RETURNING_FROM_TASK
            else: 
                self._cur_state = FSM_STATES.RETURNING_FROM_RADIO_SITE
                #  self.get_logger().info(f'{self.get_name()} completed Scan of radioactive area')
                print(self.pathList)
                time.sleep(5)
                newCurX = self.pathList[index-1][0]
                newCurY = self.pathList[index-1][1]
                newTheta = self._cur_theta
                newGoal = [newCurX,newCurY,newTheta]
                self.currentGoal =  [newCurX,newCurY,newTheta]
                print(f"____RESET NEW GOAL TO {newGoal}____")
        
       	 	 
    #=======================================================================================================================
    
    # HERE WE ARE SCANNING THE RADIOACTIVE AREA FOLLOWING THE LAWNMOWER PATTERN
    def _do_state_scan_site(self):
        isAtGoal = self._drive_to_goal(*self.currentGoal)
            # self.get_logger().info(f'{self.currentGoal} \n')
        x=0
        y=1
        
        
        if isAtGoal:
            if self.currentGoal[x] == 3.5 and self.currentGoal[y] == 2:
                self.get_logger().info(f'{self.get_name()} completed mowing grass')
                self._cur_state = FSM_STATES.RETURNING_FROM_TASK
            
            elif self.currentGoal == self.goalList[3]:
                self.get_logger().info(f'{self.get_name()} Turning to next row')
                for goal in self.goalList:
                    goal[x] += 1
                self.currentGoal = self.goalList[0]
            else:
                self._cur_state = FSM_STATES.SCAN_SITE
                self.get_logger().info(f'{self.get_name()} mowing the row')
                index = self.goalList.index(self.currentGoal)
                self.currentGoal = self.goalList[index+1]
    

    #=======================================================================================================================

    # HERE, WE ARE RETURNING TO THE ORIGIN (NOT THE POINT WHERE WE STARTED THE TASK, BUT WHERE WE STARTED THE PROGRAM)
    def _do_state_returning_from_task(self):
        self.get_logger().info(f'{self.get_name()} returning from task ')
        if self._drive_to_goal(0.0, 0.0, 0):
            self._cur_state = FSM_STATES.TASK_DONE

    # THIS IS THE STATE WHERE WE RECOGNIZE THAT A SPECIFIC TASK IS DONE
    def _do_state_task_done(self):
        self.get_logger().info(f'{self.get_name()} task done')

    # AT_START = 'AT STart',
    # FIND_PATH = 'Finding Path',
    # HEADING_TO_TASK = 'Heading To Task Enterance ',
    # HEADING_TO_RADIO_SITE = 'Heading To Radio Active site ',
    
    # PERFORMING_TASK = 'Scanning Area',-------
    # RETURNING_FROM_RADIO_SITE= 'Returning From Radio Active site ',    
    # RETURNING_FROM_TASK = 'Heading Home',
    # TASK_DONE = 'Task Done'
    def _state_machine(self):
        if self._cur_state == FSM_STATES.AT_START:
            self._do_state_at_start()
        
        elif self._cur_state == FSM_STATES.HEADING_TO_TASK:
            self._do_state_heading_to_task()

        elif self._cur_state == FSM_STATES.FIND_PATH:
            self._do_state_find_path()

        elif self._cur_state == FSM_STATES.HEADING_TO_RADIO_SITE:
            self._do_state_heading_radio_Site()

        elif self._cur_state == FSM_STATES.SCAN_SITE:
            self._do_state_scan_site()
        
        elif self._cur_state == FSM_STATES.RETURNING_FROM_RADIO_SITE:
            self._do_state_heading_to_task()    
        
        elif self._cur_state == FSM_STATES.RETURNING_FROM_TASK:
            self._do_state_returning_from_task()
        
        elif self._cur_state == FSM_STATES.TASK_DONE:
            self._do_state_task_done()
        
        else:
            self.get_logger().info(f'{self.get_name()} bad state {self._cur_state}')

    def _listener_callback(self, msg):
        pose = msg.pose.pose

        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_theta = yaw
        self._state_machine()



def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
