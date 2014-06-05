#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import roslib
roslib.load_manifest("listen_and_grasp")

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
import genpy


## END_SUB_TUTORIAL

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import Grasp
from object_recognition_msgs.msg import RecognizedObjectArray
#from meldon_detection.msg import MarkerObjectArray, MarkerObject
from baxter_grasps_server.srv import GraspService

from visualization_msgs.msg import Marker

class Pick:
	def __init__(self):
		self.objects = []
		self.objectPoses = dict()
		self.graspService = rospy.ServiceProxy('grasp_service', GraspService)
		self.scene = moveit_commander.PlanningSceneInterface()
		self.robot = moveit_commander.RobotCommander()
		self.group = moveit_commander.MoveGroupCommander("left_arm")

		self.markers_publisher = rospy.Publisher("/grasp_markers", Marker)
		

	def addBoundingBox(self, points, name):
		minX = sys.float_info.max
		minY = sys.float_info.max
		minZ = sys.float_info.max
		maxX = -sys.float_info.max
		maxY = -sys.float_info.max
		maxZ = -sys.float_info.max

		for point in points:
			if (point.x() > maxX):
				maxX = point.x()
			if (point.y() > maxY):
				maxY = point.y()
			if (point.z() > maxZ):
				maxZ = point.z()
			if (point.x() < minX):
				minX = point.x()
			if (point.y() < minY):
				minY = point.y()
			if (point.z() < minZ):
				minZ = point.z()
		dim_x = maxX - minX
		dim_y = maxY - minY
		dim_z = maxZ - minZ

		pose = PoseStamped()
		pose.header.frame_id = "/base"
		pose.pose.position.x = (maxX + minX) / 2.0
		pose.pose.position.y = (maxY + minY) / 2.0
		pose.pose.position.z = (maxZ + minZ) / 2.0
		self.scene.add_box(name, pose, (dim_x, dim_y, dim_z))

	def addBoundingBoxAtPose(self, name):
		self.scene.add_box(name, self.objectPoses[name], (0.05, 0.05, 0.1))

	def getPoseStampedFromPoseWithCovariance(self, pose):
		pose_stamped = PoseStamped()
		pose_stamped.header= copy.deepcopy(pose.header)
		pose_stamped.pose.position = copy.deepcopy(pose.pose.position)
		pose_stamped.pose.orientation = copy.deepcopy(pose.pose.orientation)
		return pose_stamped

	def objectsCallback(self, msg):
		for object in self.objects:
			self.scene.remove_world_object(object)
		self.objects.clear()
		self.objectPoses.clear()
		for object in msg.objects:
			self.objects.add(object.type.key)
			self.objectPoses[object.type.key] = getPoseStampedFromPoseWithCovariance(object.pose)
			self.addBoundingBoxAtPose(object.type.key)

	def objectRequestCallback(self, msg):
		if msg.data not in self.objects:
			rospy.logerr("Object " + msg.data + " is not in detected objects")
			return
		
		graspResponse = self.graspService(msg.data)
		if not graspResponse.success:
			rospy.logerr("No grasps were found for object " + msg.data)
			return
		self.group.set_start_state_to_current_state()
		robot.left_arm.pick(msg.data, graspResponse.grasps)

	def addCylinder(self, x, y ,z, dim_x, dim_y, dim_z):
		scene = moveit_commander.PlanningSceneInterface()
		p = PoseStamped()
 		p.header.frame_id = "/base"
   		p.pose.position.x = 0.85
  		p.pose.position.y = 0.3
  		p.pose.position.z = -0.1
  		scene.add_box("cube", p, (0.05, 0.05, 0.05))
  
  		p.pose.position.y = 0.5
  		p.pose.position.z = -0.3
  		scene.add_box("table", p, (0.5, 1.5, 0.35))
		#pose = PoseStamped()
		#pose.header.frame_id = "/base"
		#pose.pose.position.x = x
		#pose.pose.position.y = y
		#pose.pose.position.z = z
		#self.scene.add_box("coconut", pose, (dim_x, dim_y, dim_z))

	def pick(self):
		graspResponse = self.graspService("coconut")
		if not graspResponse.success:
			rospy.logerr("No grasps were found for object coconut")
			return
		#self.addCylinder(0.8, 0, 0.3, 0.01, 0.01, 0.01)
		self.publishMarkers(graspResponse.grasps, "coconut")
		self.group.set_planning_time(20)
		rospy.loginfo("Current planning time " + str(self.group.get_planning_time()))
		rospy.loginfo("Robot planning time " + str(self.robot.left_arm.get_planning_time()))
		self.group.set_start_state_to_current_state()
		grasps = self.setGrasps("925c42faeac061f86fdbcf0b090efe57", graspResponse.grasps)
		self.group.pick("925c42faeac061f86fdbcf0b090efe57", grasps)
		rospy.sleep(10)

	def setGrasps(self, name, grasps):
		pose = self.objectPoses[name]
		correctedGrasps = []

		for grasp in grasps:
			newGrasp = copy.deepcopy(grasp)
			newGrasp.pre_grasp_posture.header.stamp = rospy.Time(0)
			newGrasp.grasp_posture.header.stamp = rospy.Time(0)
			newGrasp.grasp_pose.header.frame_id = 'world'
			newGrasp.grasp_pose.pose.position.x += pose.pose.position.x
			newGrasp.grasp_pose.pose.position.y += pose.pose.position.y
			newGrasp.grasp_pose.pose.position.z += pose.pose.position.z
			newGrasp.grasp_quality = 1.0
			correctedGrasps.append(newGrasp)
			rospy.loginfo(str(newGrasp))

		return correctedGrasps

	def publishMarkers(self, grasps, object_name):
		for grasp in grasps:
			marker = self.getMarker(grasp, object_name)
			#rospy.loginfo( genpy.message.strify_message(marker))
			self.markers_publisher.publish(marker)
		

	def getMarker(self, grasp, object_name):
		marker = Marker()
		marker.id = grasp.id
		marker.header = grasp.grasp_pose.header
		marker.pose = grasp.grasp_pose.pose
		marker.ns = object_name + "_grasp_"
		marker.lifetime.secs = 10.0
		marker.action = 0
		marker.color.r = 1
		marker.color.g = 1
		marker.color.b = 1
		marker.color.a = 1
		marker.scale.x = 1
		marker.scale.y = 1
		marker.scale.z = 1
		return marker


	def go(self, args):
		moveit_commander.roscpp_initialize(args)
		rospy.Subscriber("/recognized_object_array", RecognizedObjectArray, objectsCallback)
		#rospy.Subscriber("/labeled_objects", MarkerObjectArray, objectsCallback)
		
		#rospy.Service('/pick_place_server', String, objectRequestCallback)
		self.pick()
		#rospy.spin()

if __name__=='__main__':
	rospy.init_node("Pick_object")
	pick = Pick()
	pick.go(sys.argv)