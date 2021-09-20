#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
import numpy as np
from skimage.draw import line
from PIL import Image
import pika, base64
from io import BytesIO

#-----------------
#This node is responsible for forming the SLAM image and sending it
#to Node-RED through the RabbitMQ broker.

#It subscribes to the /map, /robot_pose and /scan topics to receive
#the relevant information. After drawing the map with the robot and
#the scan, it sends it to Node-RED.
#-----------------


#Establish connection with broker.
credentials = pika.PlainCredentials('bot', 'bot')
parameters = pika.ConnectionParameters('155.207.33.189', 8076, 'kostasgioko', credentials)

connection = pika.BlockingConnection(parameters)
channel = connection.channel()


#Initialize global variables.
origin = np.array([0, 0, 0])
resoluton = 0
map_data = np.array([])
map_data_with_robot = np.array([])
first_time = False
width = 0
height = 0
cell_x = 0
cell_y = 0
theta = 0


#This is the callback for the /map topic.
#It receives the map and converts it into an image.
def map_callback(data):
    #Global variables.
    global map_data
    global resolution
    global origin
    global first_time
    global width
    global height

    #Map dimensions.
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution

    
    #Get map origin pose.
    origin_pose = data.info.origin
    #Position.
    ox = origin_pose.position.x
    oy = origin_pose.position.y
    #Angle.
    quaternion = origin_pose.orientation
    sycp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cycp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    otheta = math.atan2(sycp, cycp)

    origin = np.array([ox, oy, otheta])
    


    #Map data.
    map_data = np.zeros(height * width)

    #Rescale values.
    for i in range(len(data.data)):
        if data.data[i] == -1:
            map_data[i] = 90
        elif data.data[i] == 100:
            map_data[i] = 0
        elif data.data[i] == 0:
            map_data[i] = 200

    #Reshape map array.
    map_data = np.reshape(map_data, (height,width))

    #Flip the array horizontally, as it is mirrored. 
    map_data = np.fliplr(map_data)

    #Convert to uint8.
    image_data = map_data.astype(np.uint8)


    #If this is the first time.
    if map_data.size == 0:
        #Raise flag.
        first_time = True
    
    if first_time:
        #Lower flag.
        first_time = False

        #Form image.
        im = Image.fromarray(image_data)

        #Convert the image to Base64 and send it to Node-RED through the broker.
        buff = BytesIO()
        im.save(buff, format="PNG")
        new_image_string = base64.b64encode(buff.getvalue()).decode("utf-8")

        channel.basic_publish(exchange='amq.topic', routing_key='feed', body=new_image_string)




#This is the callback for the /robot_pose topic.
#It draws the robot on the map.
def robot_pose_callback(data):
    #Global variables.
    global map_data
    global origin
    global resolution
    global width
    global height
    global map_data_with_robot
    global cell_x
    global cell_y
    global theta


    if map_data.size != 0:

        #Copy map data in order to draw robot.
        map_data_with_robot = np.copy(map_data)

        #Calculate robot's position in the array.
        cell_x = int(round((abs(origin[0]) + data.position.x)/resolution))
        cell_y = int(round((abs(origin[1]) + data.position.y)/resolution))

        #Mirror x coordinate.
        cell_x_mirrored = width - cell_x


        #Get angle from the quaternion.
        quaternion = data.orientation
        sycp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cycp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        theta = math.atan2(sycp, cycp)

        #Draw the robot as a circle.
        for i in range(-7, 7):
            for j in range(-7, 7):
                if math.sqrt(pow(i, 2) + pow(j, 2)) <= 7:
                    map_data_with_robot[cell_y + i, cell_x_mirrored + j] = 0

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
        for i in range(len(x_values)):
            x_values[i] = width - x_values[i]

        #Draw the line.
        map_data_with_robot[y_values, x_values] = 0



#This is the callback for the /scan topic.
#It receives the laser scan and draws it on the map.
def laser_scan_callback(data):
    #Global variables.
    global map_data_with_robot
    global resolution
    global cell_x
    global cell_y
    global theta


    if map_data_with_robot.size != 0:

        #Copy map data in order to draw scan.
        map_data_with_robot_and_scan = np.copy(map_data_with_robot)

        #For each scan value.
        for i in range(len(data.ranges)):

            #Calculate the angle and the distance.
            angle = theta + data.angle_min + i * data.angle_increment
            distance = data.ranges[i]

            #If it is a valid value.
            if distance >= data.range_min and distance <= data.range_max and not(np.isnan(distance)):
                #Calculate its coordinates.
                scan_x = int(abs(round(math.cos(angle) * (distance / resolution) + cell_x)))
                scan_y = int(abs(round(math.sin(angle) * (distance / resolution) + cell_y)))

                #Mirror the x coordinate.
                scan_x = width - scan_x

                #Draw the scan pixels.
                map_data_with_robot_and_scan[scan_y, scan_x] = 255

        #Convert to uint8.
        map_data_with_robot_and_scan = map_data_with_robot_and_scan.astype(np.uint8)

        #Form image.
        im = Image.fromarray(map_data_with_robot_and_scan)
       
        #Convert the image to Base64 and send it to Node-RED through the broker.
        buff = BytesIO()
        im.save(buff, format="PNG")
        new_image_string = base64.b64encode(buff.getvalue()).decode("utf-8")

        channel.basic_publish(exchange='amq.topic', routing_key='feed', body=new_image_string)



#This function subscribes to the necessary topics and waits for the messages.
def listener():

    rospy.init_node('SLAM_map', anonymous=True)

    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/robot_pose", Pose, robot_pose_callback)
    rospy.Subscriber("/scan", LaserScan, laser_scan_callback)

    rospy.spin()

#Wait for messages.
if __name__ == '__main__':
    listener()
