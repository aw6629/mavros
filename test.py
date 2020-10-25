#!/usr/bin/env python

import rospy
import mavros
from sensor_msgs.msg import NavSatFix,NavSatStatus
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String

latitude = 0.0
longitude = 0.0
altitude = 0.0
last_waypoint = False

def waypoint_callback(data):
    global last_waypoint
    if len(data.waypoints) !=0:
        rospy.loginfo('is_current: %s',data.waypoints[len(data.waypoints)-1].is_current)
        last_waypoint =data.waypoints[len(data.waypoints)-1].is_current

def clear_pull():
	print('\n----------clear_pull----------')
	rospy.wait_for_service('/mavros/mission/clear')
	waypoint_clear = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
	resp = waypoint_clear()
	rospy.sleep(2)
	rospy.wait_for_service('/mavros/mission/pull')
	waypoint_pull = rospy.ServiceProxy('/mavros/mission/pull', WaypointPull)
	resp = waypoint_pull()
	rospy.sleep(2)
	return

def arming_call():
    print('\n----------arming_call----------')
    rospy.wait_for_service('/mavros/cmd/arming')
    drone_arm = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    resp = drone_arm(1)
    rospy.sleep(2)

def land__call():
	print('\n----------land----------')
	rospy.wait_for_service('/mavros/cmd/land')
	drone_land = rospy.ServiceProxy('/mavros/cmd/land',CommandTOL)
	resp = land()
	
def globalPosition_callback(data):
    global latitude
    global longitude
    global altitude
    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude

def takeoff_call(lat,lon,alt):
    print('\n----------takeoff----------')
    takeoff = rospy.ServiceProxy('/mavros/cmd/takeoffcur',CommandTOL)
    resp = takeoff(0,0,lat,lon,alt)
    rospy.sleep(5)
    return

def pushingWaypoints(position):
    print('\n----------pushingWaypoints----------')
    rospy.wait_for_service("/mavros/mission/push")
    waypoint_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
    resp = waypoint_push(position)
    rospy.sleep(2)
    return

def main():
    rospy.init_node('wayPoint')
    rospy.Subscriber('/mavros/mission/waypoints',WaypointList,waypoint_callback)
    rospy.Subscriber('/mavros/global_position/raw/fix',NavSatFix,globalPosition_callback)
    topic_flag = rospy.Publisher('/mavros/ugv/ready',String,queue_size=10)

    clear_pull()

    arming_call()
    takeoff_call(-1,-1,5)
    takeoff_call(-1,1,4)
    takeoff_call(0,0,0)
	land_call()
    # waypoints = [Waypoint(frame = 3, command = 16,is_current = True,autocontinue = True,param1 = 5,x_lat = -1,y_long = 1,z_alt = 3),
    #              Waypoint(frame =3 , command = 16,is_current = True,autocontinue = True,param1 = 5,x_lat = -2,y_long = 2,z_alt = 4),
    #              Waypoint(frame= 3 , command = 16,is_current = True,autocontinue = True,param1 = 5,x_lat = 0 ,y_long = 0 ,z_alt = 0)
    #              ]
    # pushingWaypoints(waypoints)

    print('Mission accomplished')
    rospy.spin()

if __name__ =='__main__':
    try:
        main()
    except  rospy.ROSInterruptException:
        pass
