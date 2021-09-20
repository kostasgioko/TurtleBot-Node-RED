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

#It does the same as the node for a single robot, but for two robots.
#-----------------


#Initialize ROS node.
rospy.init_node('send_goal_multi', anonymous=True)

#Initialize ROS Publishers for goal_reached topics.
pub1 = rospy.Publisher('/robot1_goal_reached', Bool, queue_size=1)
pub2 = rospy.Publisher('/robot2_goal_reached', Bool, queue_size=1)

#Initialize Action client for sending navigation goals.
client1 = actionlib.SimpleActionClient('robot1/move_base',MoveBaseAction)
client1.wait_for_server()

client2 = actionlib.SimpleActionClient('robot2/move_base',MoveBaseAction)
client2.wait_for_server()

#Initialize robot poses as global.
robot1_pose = Pose()
robot2_pose = Pose()




#Callback function for robot1_pose.
def robot1_pose_callback(data):
    
    global robot1_pose
    robot1_pose = data

#Callback function for robot2_pose.
def robot2_pose_callback(data):
    
    global robot2_pose
    robot2_pose = data



 
#Callback function for when goal is received for robot1 from Node-RED.
def robot1_goal(data):

    #If the robot isn't looking towards the goal, the path planning doesn't work properly.
    #So rotate it towards the goal first, then send the actual target pose.
    global robot1_pose

    
    #Rotate robot towards goal.
    #Find required angle to rotate.    
    dy = data.position.y - robot1_pose.position.y
    dx = data.position.x - robot1_pose.position.x
    
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
    goal.target_pose.pose.position.x = robot1_pose.position.x
    goal.target_pose.pose.position.y = robot1_pose.position.y
    goal.target_pose.pose.position.z = 0

    #Convert to quaternion.
    goal.target_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
    goal.target_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
    goal.target_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
    goal.target_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

    #Send goal to robot.
    client1.send_goal(goal)
    wait = client1.wait_for_result()



    #Send movement goal.
    #Form message.
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = data

    #Send goal to robot.
    client1.send_goal(goal)
    wait = client1.wait_for_result()



    #Send feedback that goal is reached.
    result = Bool()
    result.data = True
    pub1.publish(result)



#Callback function for when goal is received for robot2 from Node-RED.
def robot2_goal(data):

    #If the robot isn't looking towards the goal, the path planning doesn't work properly.
    #So rotate it towards the goal first, then send the actual target pose.
    global robot2_pose

    
    #Rotate robot towards goal.
    #Find required angle to rotate.    
    dy = data.position.y - robot2_pose.position.y
    dx = data.position.x - robot2_pose.position.x
    
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
    goal.target_pose.pose.position.x = robot2_pose.position.x
    goal.target_pose.pose.position.y = robot2_pose.position.y
    goal.target_pose.pose.position.z = 0

    #Convert to quaternion.
    goal.target_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
    goal.target_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
    goal.target_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
    goal.target_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

    #Send goal to robot.
    client2.send_goal(goal)
    wait = client2.wait_for_result()



    #Send movement goal.
    #Form message.
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = data

    #Send goal to robot.
    client2.send_goal(goal)
    wait = client2.wait_for_result()



    #Send feedback that goal is reached.
    result = Bool()
    result.data = True
    pub2.publish(result)




#This function subscribes to the necessary topics.
def listener():

    #Subscriber nodes for goal messages.
    rospy.Subscriber("/robot1_send_goal_nodered", Pose, robot1_goal)
    rospy.Subscriber("/robot2_send_goal_nodered", Pose, robot2_goal)

    #Subscriber nodes for robot pose messages.
    rospy.Subscriber("/robot1/robot_pose", Pose, robot1_pose_callback)
    rospy.Subscriber("/robot2/robot_pose", Pose, robot2_pose_callback)

    #Wait for messages.
    rospy.spin()



#Main function, calls listener.
if __name__ == '__main__':
    listener()
