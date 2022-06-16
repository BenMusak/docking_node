# Import
import rclpy
import time
import cv2
import cv2.aruco as aruco
import numpy as np

# From import
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# Tracking variables
ownMarkerID = 1
arucoIDSearcher = 0
turnSpeed = 0
driveSpeed = 0

# Search for aruco marker variables
directionLookLeft = False
searchPeriod = 6
newTime = True
time_begin = time.time()
startSearch = True
targetDockingPos = [0, 0, 0.10] # given in meters
targetDockingAng = [0, 0, 0] # Given in degreess
completedDocking = False

# PID variables
integral, derivative, last_error = 0, 0, 0


class MinimalSubscriber(Node):

    """ Subscribes to the docking topic and listens for the keyword to start docking. """

    global completedDocking

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'docking',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.startDocking = False
        self.arucoMarkerID = None

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        splitMsg = msg.data.split()

        try:
            if splitMsg[0] == "start_docking" and int(splitMsg[1]) == ownMarkerID and splitMsg[2] is not None:
                self.startDocking = True
                self.arucoMarkerID = int(splitMsg[2])
                print("Start Docking")
                # ros2 topic pub /docking std_msgs/String '{data: start_docking}'
            elif msg.data == "stop_docking":
                completedDocking = False
                self.startDocking = False
            else:
                print("Something went wrong with the Aruco Marker ID.")
        except:
            print("Not enough parameters to unpack on topic. Are you sure you gave 3 parameters?")


class MinimalPublisher(Node):

    """ Publishes the angular and linear velocity to cmd_vel to control the robot. """

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.linearVec = (0.0, 0.0, 0.0)
        self.angularVec = (0.0, 0.0, 0.0)

    def timer_callback(self):
        msg = Twist()

        msg.linear.x = self.linearVec[0]
        msg.linear.y = self.linearVec[1]
        msg.linear.z = self.linearVec[2]
        msg.angular.x = self.angularVec[0]
        msg.angular.y = self.angularVec[1]
        msg.angular.z = self.angularVec[2]

        self.publisher_.publish(msg)


def findArucosMakers(img, makerSize=6, totalMarkers=250, draw=False):

    """ Finds aruco markers from a given key (markersize and totalmarkers). """

    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # We use the key 6x6_250 for the Aruco markers type
    key = getattr(aruco, f'DICT_{makerSize}X{makerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bbox, ids, __ = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img, bbox)

    return[bbox, ids]


def load_coefficients(path):
    
    """ Loads camera matrix and distortion coefficients. """

    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()

    return camera_matrix, dist_matrix


def searchForAruco(minimal_publisher):

    """ Makes the robot turn left and right to look for any aruco markers. """

    global newTime, directionLookLeft, time_begin, startSearch, searchPeriod

    if newTime:
        time_begin = time.time()
        newTime = False
    elif startSearch:
        searchPeriod = 3
    elif not startSearch:
        searchPeriod = 6

    if time.time() - time_begin > searchPeriod:
        # Reset clock
        newTime = True
        startSearch = False
        directionLookLeft = np.invert(directionLookLeft)
    else:
        if directionLookLeft:
            turnLeft(minimal_publisher)
        else:
            turnRight(minimal_publisher)


def turnRight(minimal_publisher):
    minimal_publisher.angularVec = (0.0, 0.0, -turnSpeed)
    minimal_publisher.linearVec = (0.0, 0.0, 0.0)
    rclpy.spin_once(minimal_publisher)
    print("Turning right with: " + str(-turnSpeed) + "rad/s.")


def turnLeft(minimal_publisher):
    minimal_publisher.angularVec = (0.0, 0.0, turnSpeed)
    minimal_publisher.linearVec = (0.0, 0.0, 0.0)
    rclpy.spin_once(minimal_publisher)
    print("Turning left with: " + str(turnSpeed) + "rad/s.")


# PID function
def PID(error, Kp, Ki, Kd):
    global integral, derivative, last_error
    integral = integral + error
    derivative = error - last_error
    last_error = error
    return Kp * error + Ki * integral + Kd * derivative


def controlDocking(minimal_publisher, rvecs, tvecs):

    #https://www.google.com/search?q=how+to+use+pid+control+system+with+motor&oq=how+to+use+pid+control+system+with+motor&aqs=edge..69i57.14481j0j1&sourceid=chrome&ie=UTF-8#kpvalbx=_6MipYuTPA-2mrgT-0aSQBw36

    """ Controls the docking of the robot from the found aruco marker and its transform and rotational vectors. """

    global angleAdjust, turnSpeed, driveSpeed, completedDocking

    arucoAng = [rvecs[0][0][0], rvecs[0][0][1], rvecs[0][0][2]]
    arucoPos = [tvecs[0][0][0], tvecs[0][0][1], tvecs[0][0][2]]

    # Adjust Z angle
    degrees = arucoAng[1] * (180.0/3.14159)
    angleDiff = degrees - targetDockingAng[2]
    pid_angleDiff = PID(angleDiff, 0.5, 0.1, 0.1)
    turnSpeedZ = pid_angleDiff

    # Adjust x position
    distanceX = arucoPos[0] - targetDockingPos[0]
    pid_distanceX = PID(distanceX, 0.3, 0.1, 0.1)
    driveSpeedX = pid_distanceX

    # Adjust z position 
    distanceZ = arucoPos[2] - targetDockingPos[2]
    pid_distanceZ = PID(distanceZ, 0.3, 0.1, 0.1)
    driveSpeedZ = pid_distanceZ

    if angleDiff > targetDockingAng[2] and pid_distanceX > targetDockingPos[0] and pid_distanceZ > targetDockingPos[2] and completedDocking == False:
        minimal_publisher.angularVec = (0.0, 0.0, -turnSpeedZ)
        minimal_publisher.linearVec = (driveSpeedX, 0.0, driveSpeedZ)
        rclpy.spin_once(minimal_publisher)
    else:
        completedDocking = True
        print("Completed docking d8)")
        startFeeder()


def startFeeder():
    print("Feeder Startet")


def main(args=None):

    global arucoMarkerID

    # Start ROS2 node
    rclpy.init(args=args)

    # Create subscriber
    minimal_subscriber = MinimalSubscriber()
    minimal_publisher = MinimalPublisher()

    # Start camera
    cap = cv2.VideoCapture(0)

    # Load camera parameters
    mtx, dist = load_coefficients("cali.yml")

    while True:

        # Check if we need to dock
        if not minimal_subscriber.startDocking:
            rclpy.spin_once(minimal_subscriber)
        elif minimal_subscriber.startDocking:

            __, img = cap.read()

            foundArucos = findArucosMakers(img)
            foundArucosMarkers = len(foundArucos[0])

            if foundArucosMarkers > 0:
                aruco.drawDetectedMarkers(img, foundArucos[0], foundArucos[1])
                counter = 0
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(foundArucos[0], 0.064, mtx, dist) # Aruco markers length are given in meters

                hasSeenAruco = False
                for __, id in zip(foundArucos[0], foundArucos[1]):
                    counter += 1
                    if id == minimal_subscriber.arucoMarkerID:
                        controlDocking(minimal_publisher, rvecs, tvecs)
                        hasSeenAruco = True
                    elif id is not minimal_subscriber.arucoMarkerID and not hasSeenAruco:
                        searchForAruco(minimal_publisher)
            else:
                print("No Aruco markers found")
                searchForAruco(minimal_publisher)

            cv2.waitKey(1)
        
        else:
            continue


    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
