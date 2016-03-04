#!/usr/bin/env python
import roslib
import rospy
import actionlib
from smart_battery_msgs.msg import SmartBatteryStatus #for netbook battery
from std_msgs.msg import Int32 #for netbook battery
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, SensorState

class chargingAction(): 

	def __init__(self):
		rospy.init_node("netbook_battery")		

		#monitor netbook's battery status.  Everytime anything changes call the call back function self.NetbookPowerEventCallback and pass the data regarding the current battery status
		self._client = actionlib.SimpleActionClient('/dock_drive_action', AutoDockingAction)
		rospy.loginfo("waiting for auto_docking server")
		self._client.wait_for_server()
		rospy.loginfo("auto_docking server found")
		goal = AutoDockingGoal()
		rospy.loginfo("Sending auto_docking goal and waiting for result (times out in 180 seconds and will try again if required)")
		self._client.send_goal(goal)

		#Give the auto docking script 180 seconds.  It can take a while if it retries.
		success = self._client.wait_for_result(rospy.Duration(180))

		if success:
			rospy.loginfo("Auto_docking succeeded")
			self.charging_at_dock_station = True #The callback which detects the docking status can take up to 3 seconds to update which was causing coffee bot to try and redock (presuming it failed) even when the dock was successful.  Therefore hardcoding this value after success.
			return True
		else:
			rospy.loginfo("Auto_docking failed")
			return False

if __name__ == '__main__':
	try:
		chargingAction()
	except rospy.ROSInterruptException:
		rospy.loginfo("exception")