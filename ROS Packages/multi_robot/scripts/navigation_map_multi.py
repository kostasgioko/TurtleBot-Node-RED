#!/usr/bin/env python
import sys
import rospy
import math
import re
import yaml
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D, Pose
from std_msgs.msg import Bool
import numpy as np
from skimage.draw import line
from PIL import Image
import pika, base64
from io import BytesIO

#-----------------
#This node is responsible for reading the saved SLAM map, drawing the
#robots and the potential navigation goals and forming the image, before
#sending it to Node-RED through the RabbitMQ broker.

#If it receives a robot_pose, it checks whether or not there are also
#navigation goals to be reached and draws them on the map. Afterwards,
#it sends the image to Node-RED.
#-----------------


#Establish connection with broker.
credentials = pika.PlainCredentials('bot', 'bot')
parameters = pika.ConnectionParameters('155.207.33.189', 8076, 'kostasgioko', credentials)

connection = pika.BlockingConnection(parameters)
channel = connection.channel()



#This functions reads a PGM file and returns a numpy array with the image data.
def read_pgm(filename, byteorder='>'):
    #Return image data from a raw PGM file as numpy array.
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))


#Some initializations.
def initialize():
    #Global variable initialization.
    global map_name
    global map_yaml
    global map_data
    global width
    global height
    global resolution
    global origin
    global robot1_goal_flag
    global robot1_goal
    global robot1_pose
    global robot1_first_time
    global robot2_goal_flag
    global robot2_goal
    global robot2_pose
    global robot2_first_time
    global counter

    robot1_goal_flag = False
    robot1_first_time = True
    robot1_goal = Pose()
    robot1_pose = Pose()
    robot2_goal_flag = False
    robot2_first_time = True
    robot2_goal = Pose()
    robot2_pose = Pose()
    counter = 0

    #File paths.
    folder_path = "/home/giokotos/turtlebot_maps/"

    map_name = folder_path + sys.argv[1] + ".pgm"
    map_yaml = folder_path + sys.argv[1] + ".yaml"

    #Load the image and form the map_data array.
    map_data = read_pgm(map_name, byteorder='<')

    #Get the necessary variables.
    height = map_data.shape[0]
    width = map_data.shape[1]

    #Read the YAML file for the other variables.
    with open(map_yaml, 'r') as stream:
        map_info = yaml.safe_load(stream)

    resolution = map_info["resolution"]
    origin = map_info["origin"]




#These are the callback functions for the send_goal_nodered topics.
#They raise the goal flag so we know to draw the goal, and saves the navigation goal.
def robot1_goal_set_callback(data):

    global robot1_goal_flag
    global robot1_goal

    robot1_goal_flag = True        
    robot1_goal = data

def robot2_goal_set_callback(data):

    global robot2_goal_flag
    global robot2_goal

    robot2_goal_flag = True        
    robot2_goal = data





#These are the callback functions for the goal_reached topics.
#If the goal is reached, lower the flag.
def robot1_goal_reached_callback(data):

    global robot1_goal_flag

    robot1_goal_flag = False

def robot2_goal_reached_callback(data):

    global robot2_goal_flag

    robot2_goal_flag = False



#These are the callback functions for the robot_pose topics.
#They update the robot's pose and call the draw_map function.
def robot1_pose_callback(data):

    global robot1_pose
    global robot1_first_time
    global robot2_first_time

    robot1_first_time = False
    robot1_pose = data

    if not robot2_first_time:
        draw_map()

def robot2_pose_callback(data):

    global robot2_pose
    global robot1_first_time
    global robot2_first_time

    robot2_first_time = False
    robot2_pose = data

    if not robot1_first_time:
        draw_map()





#This function draws the robots and the navigation goals (if given) on the map.
def draw_map():
    #Global variables.
    global map_data
    global origin
    global resolution
    global width
    global height
    global robot1_goal_flag
    global robot1_goal
    global robot1_pose
    global robot2_goal_flag
    global robot2_goal
    global robot2_pose
    global counter

    #By calling this function for both robots, we increase the amount of times the map is sent.
    #That can cause the RabbitMQ channel to overload and close, so we decrease the times the function is called.    
    counter = counter + 1    

    if (map_data.size != 0 and counter % 5 == 1):

        #-------------------------------------
        #Draw the first robot.

        #Copy map data in order to draw robot.
        map_data_with_robot = np.copy(map_data)

        #Calculate robot's position in the array.
        cell_x = int(round((abs(origin[0]) + robot1_pose.position.x)/resolution))
        cell_y = int(round((abs(origin[1]) + robot1_pose.position.y)/resolution))

        #Mirror y coordinate.
        cell_y_mirrored = height - cell_y

        #Get angle from the quaternion.
        quaternion = robot1_pose.orientation
        sycp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cycp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        theta = math.atan2(sycp, cycp)

        #Draw the robot as a circle.
        for i in range(-7, 7):
            for j in range(-7, 7):
                if math.sqrt(pow(i, 2) + pow(j, 2)) <= 7:
                    map_data_with_robot[cell_y_mirrored + i, cell_x + j] = 0

        #Now draw a line to show the robot's orientation.
        #First find the endpoint of the line.
        if math.sqrt(pow(width, 2) + pow(height, 2)) < 100:
            line_length = 10
        else:
            line_length = 40

        x_end = int(abs(round(math.cos(theta) * line_length + cell_x)))
        y_end = int(abs(round(math.sin(theta) * line_length + cell_y)))

        #Get the indices of the pixels that form the line.
        y_values, x_values = line(cell_y, cell_x, y_end, x_end)

        #Mirror x coordinates.
        for i in range(len(y_values)):
            y_values[i] = height - y_values[i]

        #Draw the line.
        map_data_with_robot[y_values, x_values] = 0





        #If there is a navigation goal for the robot, draw it as well.
        if robot1_goal_flag:
            #Calculate the goal's position in the array.
            cell_x = int(round((abs(origin[0]) + robot1_goal.position.x)/resolution))
            cell_y = int(round((abs(origin[1]) + robot1_goal.position.y)/resolution))

            #Mirror y coordinate.
            cell_y_mirrored = height - cell_y

            #Get angle from the quaternion.
            quaternion = robot1_goal.orientation
            sycp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
            cycp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
            theta = math.atan2(sycp, cycp)

            #Draw the goal as a circle.
            for i in range(-4, 4):
                for j in range(-4, 4):
                    if math.sqrt(pow(i, 2) + pow(j, 2)) <= 4:
                        map_data_with_robot[cell_y_mirrored + i, cell_x + j] = 0

            #And draw a line for the orientation of the desired position.
            line_length = 15

            #Find the endpoint of the line.
            x_end = int(abs(round(math.cos(theta) * line_length + cell_x)))
            y_end = int(abs(round(math.sin(theta) * line_length + cell_y)))

            #Get the indices of the pixels that form the line.
            y_values, x_values = line(cell_y, cell_x, y_end, x_end)

            #Mirror x coordinates.
            for i in range(len(y_values)):
                y_values[i] = height - y_values[i]

            #Draw the line.
            map_data_with_robot[y_values, x_values] = 0

        #-------------------------------------

        #-------------------------------------
        #Draw the second robot.

        #Copy map data in order to draw robot.
        #map_data_with_robot = np.copy(map_data)

        #Calculate robot's position in the array.
        cell_x = int(round((abs(origin[0]) + robot2_pose.position.x)/resolution))
        cell_y = int(round((abs(origin[1]) + robot2_pose.position.y)/resolution))

        #Mirror y coordinate.
        cell_y_mirrored = height - cell_y

        #Get angle from the quaternion.
        quaternion = robot2_pose.orientation
        sycp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cycp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        theta = math.atan2(sycp, cycp)

        #Draw the robot as a circle.
        for i in range(-7, 7):
            for j in range(-7, 7):
                if math.sqrt(pow(i, 2) + pow(j, 2)) <= 7:
                    map_data_with_robot[cell_y_mirrored + i, cell_x + j] = 100

        #Now draw a line to show the robot's orientation.
        #First find the endpoint of the line.
        if math.sqrt(pow(width, 2) + pow(height, 2)) < 100:
            line_length = 10
        else:
            line_length = 40

        x_end = int(abs(round(math.cos(theta) * line_length + cell_x)))
        y_end = int(abs(round(math.sin(theta) * line_length + cell_y)))

        #Get the indices of the pixels that form the line.
        y_values, x_values = line(cell_y, cell_x, y_end, x_end)

        #Mirror x coordinates.
        for i in range(len(y_values)):
            y_values[i] = height - y_values[i]

        #Draw the line.
        map_data_with_robot[y_values, x_values] = 100





        #If there is a navigation goal for the robot, draw it as well.
        if robot2_goal_flag:
            #Calculate the goal's position in the array.
            cell_x = int(round((abs(origin[0]) + robot2_goal.position.x)/resolution))
            cell_y = int(round((abs(origin[1]) + robot2_goal.position.y)/resolution))

            #Mirror y coordinate.
            cell_y_mirrored = height - cell_y

            #Get angle from the quaternion.
            quaternion = robot2_goal.orientation
            sycp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
            cycp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
            theta = math.atan2(sycp, cycp)

            #Draw the goal as a circle.
            for i in range(-4, 4):
                for j in range(-4, 4):
                    if math.sqrt(pow(i, 2) + pow(j, 2)) <= 4:
                        map_data_with_robot[cell_y_mirrored + i, cell_x + j] = 100

            #And draw a line for the orientation of the desired position.
            line_length = 15

            #Find the endpoint of the line.
            x_end = int(abs(round(math.cos(theta) * line_length + cell_x)))
            y_end = int(abs(round(math.sin(theta) * line_length + cell_y)))

            #Get the indices of the pixels that form the line.
            y_values, x_values = line(cell_y, cell_x, y_end, x_end)

            #Mirror x coordinates.
            for i in range(len(y_values)):
                y_values[i] = height - y_values[i]

            #Draw the line.
            map_data_with_robot[y_values, x_values] = 100

        #-------------------------------------

        #-------------------------------------
        #Convert array into image and send to Nodered.

        #Convert to uint8.
        map_data_with_robot = map_data_with_robot.astype(np.uint8)

        #Form image.
        im = Image.fromarray(map_data_with_robot)

        #Convert the image to Base64 and send it to Nodered through the broker.
        buff = BytesIO()
        im.save(buff, format="PNG")
        new_image_string = base64.b64encode(buff.getvalue()).decode("utf-8")

        channel.basic_publish(exchange='amq.topic', routing_key='feed', body=new_image_string)





#This function subscribes to the necessary topics and waits for the messages.
def listener():

    rospy.init_node('navigation_map_multi', anonymous=True)

    #Subscriber nodes for robot pose messages.
    rospy.Subscriber("/robot1/robot_pose", Pose, robot1_pose_callback)
    rospy.Subscriber("/robot2/robot_pose", Pose, robot2_pose_callback)

    #Subscriber nodes for navigation goal messages.
    rospy.Subscriber("/robot1_send_goal_nodered", Pose, robot1_goal_set_callback)
    rospy.Subscriber("/robot2_send_goal_nodered", Pose, robot2_goal_set_callback)

    #Subscriber nodes for goal reached messages.
    rospy.Subscriber("/robot1_goal_reached", Bool, robot1_goal_reached_callback)
    rospy.Subscriber("/robot2_goal_reached", Bool, robot2_goal_reached_callback)

    rospy.spin()


#Call initialize function and then wait for messages.
if __name__ == '__main__':
    initialize()
    listener()
