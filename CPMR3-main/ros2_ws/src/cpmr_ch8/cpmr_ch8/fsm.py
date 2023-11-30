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
    HEADING_TO_TASK = 'Heading to Task',
    # need to add a task state(s)
    #==============================================
    PERFORMING_TASK = 'Cutting the grass'
    #===============================================
    RETURNING_FROM_TASK = 'Returning from Task',
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
        self.goalList = [[3,3,math.pi/2],[3,6,math.pi],[3.5,6,3*(math.pi/2)],[3.5,3,math.pi]] 
        self. currentGoal = self.goalList[0] 
        self.robotSpeed=0.3
        self.currentIndex =0 

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
            
    def _do_state_heading_to_task(self):
        self.get_logger().info(f'{self.get_name()} heading to task {self._cur_x} {self._cur_y} {self._cur_theta}')
        if self._drive_to_goal(3, 3, math.pi/2):
            self._cur_state = FSM_STATES.PERFORMING_TASK

    #=======================================================================================================================
    # HERE WE ARE HEADING TO GOAL (I.E., HEADING TO THE STARTING POSITION WHERE WE WILL BEGIN TO CUT THE GRASS)
    def _do_state_performing_task(self):
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
                self._cur_state = FSM_STATES.PERFORMING_TASK
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

    def _state_machine(self):
        if self._cur_state == FSM_STATES.AT_START:
            self._do_state_at_start()
        elif self._cur_state == FSM_STATES.HEADING_TO_TASK:
            self._do_state_heading_to_task()
        elif self._cur_state == FSM_STATES.PERFORMING_TASK:
            self._do_state_performing_task()
        elif self._cur_state == FSM_STATES.RETURNING_FROM_TASK:
            self._do_state_returning_from_task()
        elif self._cur_state == FSM_STATES.TASK_DONE:
            self._do_state_task_done()
        else:
            self.get_logger().info(f'{self.get_name()} bad state {state_cur_state}')

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
