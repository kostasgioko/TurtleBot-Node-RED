#!/usr/bin/env python
import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

#-----------------
#This node is responsible for receiving the goal messages from Node-RED
#and making them compatible with ROS, as Node-RED can't provide timestamps.

#If the robot isn't looking towards the goal, the path planning doesn't work
#properly. Therefore, it splits the actual goal into two subgoals, one being
#a rotation towards the desired position and the second one being the actual
#movement. When the final goal is reached, it sends a confirmation message. 
#-----------------


#Initialize ROS node.
rospy.init_node('send_goal', anonymous=True)

#Initialize ROS Publisher for goal_reached topic.
pub = rospy.Publisher('/goal_reached', Bool, queue_size=1)

#Initialize Action client for sending navigation goals.
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()

#Initialize robot pose as global.
robot_pose = Pose()




#Callback function for robot_pose.
def robot_pose_callback(data):
    
    global robot_pose
    robot_pose = data



 
#Callback function when goal is received from Node-RED.
def callback(data):

    #If the robot isn't looking towards the goal, the path planning doesn't work properly.
    #So rotate it towards the goal first, then send the actual target pose.
    global robot_pose

    
    #Rotate robot towards goal.
    #Find required angle to rotate.    
    dy = data.position.y - robot_pose.position.y
    dx = data.position.x - robot_pose.position.x
    
    #Roll, pitch, yaw.
    yaw = math.atan2(dy, dx)
    pitch = 0
    roll = 0

    cy = math.cos(yaw * 0.5);
    sy = math.sin(yaw * 0.5);
    cp = math.cos(pitch * 0.5);
    sp = math.sin(pitch * 0.5);
    cr = math.cos(roll * 0.5);
    sr = math.sin(roll * 0.5);   

    #Form message header.
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    #Position is the same as the current, so the robot will only rotate.
    goal.target_pose.pose.position.x = robot_pose.position.x
    goal.target_pose.pose.position.y = robot_pose.position.y
    goal.target_pose.pose.position.z = 0

    #Convert to quaternion.
    goal.target_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
    goal.target_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
    goal.target_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
    goal.target_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

    #Send goal to robot.
    client.send_goal(goal)
    wait = client.wait_for_result()



    #Send movement goal.
    #Form message.
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = data

    #Send goal to robot.
    client.send_goal(goal)
    wait = client.wait_for_result()



    #Send feedback that goal is reached.
    result = Bool()
    result.data = True
    pub.publish(result)




#This function subscribes to the necessary topics.
def listener():

    #Subscriber node for goal message.
    rospy.Subscriber("/send_goal_nodered", Pose, callback)

    #Subscriber node for robot pose message.
    rospy.Subscriber("/robot_pose", Pose, robot_pose_callback)

    #Wait for messages.
    rospy.spin()



#Main function, calls listener.
if __name__ == '__main__':
    listener()
