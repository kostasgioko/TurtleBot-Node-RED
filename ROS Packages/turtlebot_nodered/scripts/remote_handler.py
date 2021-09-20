#!/usr/bin/env python
import pika, sys, subprocess, signal, os, time

#-----------------
#This script is responsible for the communication between the remote PC and
#Node-RED.

#It receives the launch commands from Node-RED and launches the corresponding
#Launch file, or shuts it down.
#-----------------

#Global variables.
child_launch = None
child_broker = None

#Callback method for message handling.
def callback(ch, method, properties, body):
    global child_launch, child_broker

    #Split command.
    command = body.split()

    #Navigation command.
    if command[0] == "navigation": 
 
        #If nothing has been launched.
        if child_launch == None:

            #Form launch command.
            #If command is only 1 word, map is default.
            if len(command) == 1:
                launch_command = ['roslaunch', 'turtlebot_nodered', 'gazebo_amcl.launch']
            #Otherwise form command including map arguments.
            elif len(command) == 2:
                world_file_arg = 'world_file:=/home/giokotos/turtlebot_worlds/' + command[1] + '.world'
                map_file_arg = 'map_file:=/home/giokotos/turtlebot_maps/' + command[1] + '.yaml'
                map_arg = 'map:=' + command[1]
                launch_command = ['roslaunch', 'turtlebot_nodered', 'gazebo_amcl.launch', world_file_arg, map_file_arg, map_arg]

            #Launch the navigation module.
            child_launch = subprocess.Popen(launch_command)
            time.sleep(7)
            child_broker = subprocess.Popen(['python', '/home/giokotos/ros2broker/bin/ros2broker', 'run', '--model-file', '/home/giokotos/ros2broker/turtlebot/turtlebot_nodered.yaml'])


    #Navigation shutdown command.
    elif command[0] == "navigation_shutdown":

        #If we are in parent process.
        if child_launch > 0:
            #Shutdown launch file, and exit.
            child_launch.send_signal(signal.SIGINT)
	    child_broker.send_signal(signal.SIGINT)
            
            #Make child 'None' so we know nothing is launched.
            child_launch = None
            child_broker = None

    #Multi robot navigation command.
    elif command[0] == "multi":

        #If nothing has been launched.
        if child_launch == None:

            #Form launch command.
            #If command is only 1 word, map is default.
            if len(command) == 1:
                launch_command = ['roslaunch', 'multi_robot', 'multi.launch']
            #Otherwise form command including map arguments.
            elif len(command) == 2:
                world_file_arg = 'world_file:=/home/giokotos/turtlebot_worlds/' + command[1] + '.world'
                map_file_arg = 'map_file:=/home/giokotos/turtlebot_maps/' + command[1] + '.yaml'
                map_arg = 'map:=' + command[1]
                launch_command = ['roslaunch', 'multi_robot', 'multi.launch', world_file_arg, map_file_arg, map_arg]

            #Launch the navigation module.
            child_launch = subprocess.Popen(launch_command)
            time.sleep(7)
            child_broker = subprocess.Popen(['python', '/home/giokotos/ros2broker/bin/ros2broker', 'run', '--model-file', '/home/giokotos/ros2broker/turtlebot/turtlebot_nodered_multi.yaml'])

    #Multi robot navigation shutdown command.
    elif command[0] == "multi_shutdown":

        #If we are in parent process.
        if child_launch > 0:
            #Shutdown launch file, and exit.
            child_launch.send_signal(signal.SIGINT)
	    child_broker.send_signal(signal.SIGINT)
            
            #Make child 'None' so we know nothing is launched.
            child_launch = None
            child_broker = None

    #SLAM command.
    elif command[0] == "slam":

        #If nothing has been launched.
        if child_launch == None:

            #Form launch command.
            #If command is only 1 word, map is default.
            if len(command) == 1:
                launch_command = ['roslaunch', 'turtlebot_nodered', 'gazebo_slam.launch']
            #Otherwise form command including map arguments.
            elif len(command) == 2:
                world_file_arg = 'world_file:=/home/giokotos/turtlebot_worlds/' + command[1] + '.world'
                launch_command = ['roslaunch', 'turtlebot_nodered', 'gazebo_slam.launch', world_file_arg]

            #Launch the navigation module.
            child_launch = subprocess.Popen(launch_command)
            time.sleep(7)
            child_broker = subprocess.Popen(['python', '/home/giokotos/ros2broker/bin/ros2broker', 'run', '--model-file', '/home/giokotos/ros2broker/turtlebot/turtlebot_nodered.yaml'])

    #SLAM shutdown command.
    elif command[0] == "slam_shutdown":

        #If we are in parent process.
        if child_launch > 0:
            #Shutdown launch file, and exit.
            child_launch.send_signal(signal.SIGINT)
	    child_broker.send_signal(signal.SIGINT)
            
            #Make child 'None' so we know nothing is launched.
            child_launch = None
            child_broker = None

    #Save map command.
    elif command[0] == "save_map":

        #If we are in parent process.
        if child_launch > 0:
        
            #Form path.
            path = '/home/giokotos/turtlebot_maps/' + command[1]

            #Save map.
            subprocess.Popen(['rosrun', 'map_server', 'map_saver', '-f', path])

    #Launch Coffee Shop Application.
    elif command[0] == "cafe":

        #If nothing has been launched.
        if child_launch == None:

            #Form launch command.            
            world_file_arg = 'world_file:=/home/giokotos/turtlebot_worlds/cafe.world'
            map_file_arg = 'map_file:=/home/giokotos/turtlebot_maps/cafe.yaml'
            map_arg = 'map:=cafe'
            launch_command = ['roslaunch', 'turtlebot_nodered', 'gazebo_amcl.launch', world_file_arg, map_file_arg, map_arg]

            #Launch the navigation module.
            child_launch = subprocess.Popen(launch_command)
            time.sleep(7)
            child_broker = subprocess.Popen(['python', '/home/giokotos/ros2broker/bin/ros2broker', 'run', '--model-file', '/home/giokotos/ros2broker/turtlebot/turtlebot_nodered.yaml'])

    #Coffee Shop Application shutdown command.
    elif command[0] == "cafe_shutdown":

        #If we are in parent process.
        if child_launch > 0:
            #Shutdown launch file, and exit.
            child_launch.send_signal(signal.SIGINT)
	    child_broker.send_signal(signal.SIGINT)
            
            #Make child 'None' so we know nothing is launched.
            child_launch = None
            child_broker = None

    #Launch House Security Application.
    elif command[0] == "house":

        #If nothing has been launched.
        if child_launch == None:

            #Form launch command.            
            world_file_arg = 'world_file:=/home/giokotos/turtlebot_worlds/house.world'
            map_file_arg = 'map_file:=/home/giokotos/turtlebot_maps/house.yaml'
            map_arg = 'map:=house'
            launch_command = ['roslaunch', 'turtlebot_nodered', 'gazebo_amcl.launch', world_file_arg, map_file_arg, map_arg]

            #Launch the navigation module.
            child_launch = subprocess.Popen(launch_command)
            time.sleep(7)
            child_broker = subprocess.Popen(['python', '/home/giokotos/ros2broker/bin/ros2broker', 'run', '--model-file', '/home/giokotos/ros2broker/turtlebot/turtlebot_nodered.yaml'])

    #Coffee Shop Application shutdown command.
    elif command[0] == "house_shutdown":

        #If we are in parent process.
        if child_launch > 0:
            #Shutdown launch file, and exit.
            child_launch.send_signal(signal.SIGINT)
	    child_broker.send_signal(signal.SIGINT)
            
            #Make child 'None' so we know nothing is launched.
            child_launch = None
            child_broker = None

    #Launch Art Gallery Application.
    elif command[0] == "art":

        #If nothing has been launched.
        if child_launch == None:

            #Form launch command.
            world_file_arg = 'world_file:=/home/giokotos/turtlebot_worlds/museum.world'
            map_file_arg = 'map_file:=/home/giokotos/turtlebot_maps/museum.yaml'
            map_arg = 'map:=museum'
            launch_command = ['roslaunch', 'multi_robot', 'multi.launch', world_file_arg, map_file_arg, map_arg, 'robot1_x:=-0.5', 'robot1_y:=-14', 'robot1_a:=-1.57', 'robot2_x:=2', 'robot2_y:=-14', 'robot2_a:=-1.57']

            #Launch the navigation module.
            child_launch = subprocess.Popen(launch_command)
            time.sleep(7)
            child_broker = subprocess.Popen(['python', '/home/giokotos/ros2broker/bin/ros2broker', 'run', '--model-file', '/home/giokotos/ros2broker/turtlebot/turtlebot_nodered_multi.yaml'])

    #Multi robot navigation shutdown command.
    elif command[0] == "art_shutdown":

        #If we are in parent process.
        if child_launch > 0:
            #Shutdown launch file, and exit.
            child_launch.send_signal(signal.SIGINT)
	    child_broker.send_signal(signal.SIGINT)
            
            #Make child 'None' so we know nothing is launched.
            child_launch = None
            child_broker = None

#Establish connection with broker.
credentials = pika.PlainCredentials('bot', 'bot')
parameters = pika.ConnectionParameters('155.207.33.189', 8076, 'kostasgioko', credentials)

connection = pika.BlockingConnection(parameters)
channel = connection.channel()

result = channel.queue_declare('', exclusive=True)
queue_name = result.method.queue

channel.queue_bind(exchange='amq.topic', queue=queue_name, routing_key='remote_handler')
            
#Wait for messages to arrive.
channel.basic_consume(queue=queue_name, auto_ack=True, on_message_callback=callback)

print(' [*] Waiting for messages. To exit press CTRL+C')
channel.start_consuming()


