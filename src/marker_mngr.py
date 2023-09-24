#!/usr/bin/env python3

from threading import Lock
from copy import deepcopy

import rospy
import pymap3d as pm

from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint

from marker_interfacing.msg import GeodeticMarker, ENUMarker, GeodeticMarkerList
from marker_interfacing.srv import \
	AddMarker, AddMarkerRequest, AddMarkerResponse, \
	ClearMarkers, ClearMarkersRequest, ClearMarkersResponse, \
	EditMarker, EditMarkerRequest, EditMarkerResponse, \
	InsertMarker, InsertMarkerRequest, InsertMarkerResponse, \
	PublishCurrentMarker, PublishCurrentMarkerRequest, PublishCurrentMarkerResponse, \
	PlanHomeMarker, PlanHomeMarkerRequest, PlanHomeMarkerResponse, \
	RemoveCurrentMarker, RemoveCurrentMarkerRequest, RemoveCurrentMarkerResponse, \
	RemoveMarker, RemoveMarkerRequest, RemoveMarkerResponse, \
	ReorderMarker, ReorderMarkerRequest, ReorderMarkerResponse, \
	ReorderMarkers, ReorderMarkersRequest, ReorderMarkersResponse

### main #####################################################################

def main():
	MarkerManager().loop()

class MarkerManager:
	def __init__(self):

		rospy.init_node("marker_manager")
		
		### local variables ##################################################

		self.global_origin = None
		self.global_origin_lock = Lock()

		self.marker_list = GeodeticMarkerList()
		self.marker_list_lock = Lock()

		self.next_marker_id = 0
		self.next_marker_id_lock = Lock()

		### connect to ROS ###################################################

		global_origin_topic = rospy.get_param("~global_origin_topic")
		marker_list_topic = rospy.get_param("~marker_list_topic")
		current_marker_topic = rospy.get_param("~current_marker_topic")
		current_waypoint_topic = rospy.get_param("~current_waypoint_topic")
		add_marker_service = rospy.get_param("~add_marker_service")
		clear_markers_service = rospy.get_param("~clear_markers_service")
		edit_marker_service = rospy.get_param("~edit_marker_service")
		insert_marker_service = rospy.get_param("~insert_marker_service")
		publish_current_marker_service = rospy.get_param("~publish_current_marker_service")
		plan_home_marker_service = rospy.get_param("~plan_home_marker_service")
		remove_current_marker_service = rospy.get_param("~remove_current_marker_service")
		remove_marker_service = rospy.get_param("~remove_marker_service")
		reorder_marker_service = rospy.get_param("~reorder_marker_service")
		reorder_markers_service = rospy.get_param("~reorder_markers_service")

		self.global_origin_sub = rospy.Subscriber(global_origin_topic, GeoPoint, self.global_origin_callback)
		self.marker_list_pub = rospy.Publisher(marker_list_topic, GeodeticMarkerList, queue_size=1)
		self.current_marker_pub = rospy.Publisher(current_marker_topic, ENUMarker, queue_size=1)
		self.current_waypoint_pub = rospy.Publisher(current_waypoint_topic, Point, queue_size=1)
		self.add_marker_srv = rospy.Service(add_marker_service, AddMarker, self.add_marker_callback)
		self.clear_markers_srv = rospy.Service(clear_markers_service, ClearMarkers, self.clear_markers_callback)
		self.edit_marker_srv = rospy.Service(edit_marker_service, EditMarker, self.edit_marker_callback)
		self.insert_marker_srv = rospy.Service(insert_marker_service, InsertMarker, self.insert_marker_callback)
		self.publish_current_marker_srv = rospy.Service(publish_current_marker_service, PublishCurrentMarker, self.publish_current_marker_callback)
		self.plan_home_marker_srv = rospy.Service(plan_home_marker_service, PlanHomeMarker, self.plan_home_marker_callback)
		self.remove_current_marker_srv = rospy.Service(remove_current_marker_service, RemoveCurrentMarker, self.remove_current_marker_callback)
		self.remove_marker_srv = rospy.Service(remove_marker_service, RemoveMarker, self.remove_marker_callback)
		self.reorder_marker_srv = rospy.Service(reorder_marker_service, ReorderMarker, self.reorder_marker_callback)
		self.reorder_markers_srv = rospy.Service(reorder_markers_service, ReorderMarkers, self.reorder_markers_callback)

		### end init #########################################################

	### local functions ######################################################

	def set_global_origin(self, gps_position: GeoPoint):
		with self.global_origin_lock:
			self.global_origin = gps_position

	def get_global_origin(self):
		with self.global_origin_lock:
			global_origin = deepcopy(self.global_origin)
		return global_origin

	def publish_markers(self):
		with self.marker_list_lock:
			self.marker_list_pub.publish(self.marker_list)

	def current_marker(self):
		with self.marker_list_lock:
			current_marker = deepcopy(self.marker_list.markers[0]) if self.marker_list.markers else None
		return current_marker

	def publish_marker(self, current_marker):
		global_origin = self.get_global_origin()
		if not global_origin:
			return False

		### translate geodetic coords to enu ###

		lat = current_marker.gps.latitude
		lon = current_marker.gps.longitude
		h = current_marker.gps.altitude
		lat0 = global_origin.latitude
		lon0 = global_origin.longitude
		h0 = global_origin.altitude

		e, n, u = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
		waypoint = Point(e, n, u)

		enu_marker = ENUMarker()
		enu_marker.waypoint_enu = waypoint
		enu_marker.waypoint_error = current_marker.waypoint_error
		enu_marker.marker_type = current_marker.marker_type
		enu_marker.aruco_id = current_marker.aruco_id
		enu_marker.aruco_id_2 = current_marker.aruco_id_2
		enu_marker.marker_id = current_marker.marker_id

		### publish enu_marker and waypoint ###

		self.current_marker_pub.publish(enu_marker)
		self.current_waypoint_pub.publish(waypoint)

		return True
	
	def make_marker(self, lat, lon, h, error, marker_type, aruco_id, aruco_id_2) -> GeodeticMarker:

		# get/update next marker id
		with self.next_marker_id_lock:
			next_marker_id = self.next_marker_id
			self.next_marker_id += 1

		gps = GeoPoint()
		gps.latitude = lat
		gps.longitude = lon
		gps.altitude = h

		marker = GeodeticMarker()
		marker.gps = gps
		marker.waypoint_error = error
		marker.marker_type = marker_type
		marker.aruco_id = aruco_id
		marker.aruco_id_2 = aruco_id_2
		marker.marker_id = next_marker_id

		return marker

	def add_marker(self, marker: GeodeticMarker):

		# update marker list
		with self.marker_list_lock:
			self.marker_list.markers.append(marker)
			self.marker_list.count += 1

			# publish updated marker list
			self.marker_list_pub.publish(self.marker_list)

	def edit_marker(self, lat, lon, h, error, marker_type, aruco_id, aruco_id_2, marker_id) -> bool:
		with self.marker_list_lock:
			if self.marker_list.markers and self.marker_list.count:

				# find marker
				marker_ids = [marker.marker_id for marker in self.marker_list.markers]
				try:
					marker_index = marker_ids.index(marker_id)
					marker = self.marker_list.markers[marker_index]
				except ValueError:
					return False

				# update marker
				marker.gps.latitude = lat
				marker.gps.longitude = lon
				marker.gps.altitude = h
				marker.waypoint_error = error
				marker.marker_type = marker_type
				marker.aruco_id = aruco_id
				marker.aruco_id_2 = aruco_id_2

			# publish updated marker list
			self.marker_list_pub.publish(self.marker_list)

		return True

	def insert_marker(self, new_following_marker_id, marker: GeodeticMarker):
		with self.marker_list_lock:

			# insert marker
			marker_ids = [marker.marker_id for marker in self.marker_list.markers]
			try:
				marker_index = marker_ids.index(new_following_marker_id)
				self.marker_list.markers.insert(marker_index, marker)
			except ValueError:
				# if marker was not found, insert marker at the end
				self.marker_list.markers.append(marker)
			finally:
				self.marker_list.count += 1

			# publish updated marker list
			self.marker_list_pub.publish(self.marker_list)

	def remove_marker_by_id(self, marker_id) -> GeodeticMarker:
		with self.marker_list_lock:
			if self.marker_list.markers and self.marker_list.count:

				# find marker
				marker_ids = [marker.marker_id for marker in self.marker_list.markers]
				try:
					marker_index = marker_ids.index(marker_id)
				except ValueError:
					return None

				# remove marker
				removed_marker = self.marker_list.markers.pop(marker_index)
				self.marker_list.count -= 1

			# publish updated marker list
			self.marker_list_pub.publish(self.marker_list)

		return removed_marker

	def clear_markers(self):
		with self.marker_list_lock:
			self.marker_list.markers.clear()
			self.marker_list.count = 0

			# publish updated marker list
			self.marker_list_pub.publish(self.marker_list)

	def get_marker_list(self):
		with self.marker_list_lock:
			return deepcopy(self.marker_list)
		
	def set_marker_list(self, marker_list):
		with self.marker_list_lock:
			self.marker_list = marker_list

			# publish updated marker list
			self.marker_list_pub.publish(self.marker_list)

	### callbacks ############################################################

	def global_origin_callback(self, gps_position: GeoPoint):
		# affects newly published markers
		self.set_global_origin(gps_position)

	def add_marker_callback(self, add_request: AddMarkerRequest) -> AddMarkerResponse:
		response = AddMarkerResponse()
		response.success = True
		self.add_marker(
			self.make_marker(
				add_request.lat,
				add_request.lon,
				add_request.alt,
				add_request.waypoint_error,
				add_request.marker_type,
				add_request.aruco_id,
				add_request.aruco_id_2,
			)
		)
		return response

	def insert_marker_callback(self, insert_request: InsertMarkerRequest) -> InsertMarkerResponse:
		response = InsertMarkerResponse()
		response.success = True
		self.insert_marker(
			insert_request.new_following_marker_id,
			self.make_marker(
				insert_request.lat,
				insert_request.lon,
				insert_request.alt,
				insert_request.waypoint_error,
				insert_request.marker_type,
				insert_request.aruco_id,
				insert_request.aruco_id_2,
			)
		)
		return response

	def clear_markers_callback(self, _: ClearMarkersRequest):
		response = ClearMarkersResponse()
		response.success = True
		self.clear_markers()
		return response

	def edit_marker_callback(self, edit_request: EditMarkerRequest):
		response = EditMarkerResponse()
		response.success = self.edit_marker(
			edit_request.lat,
			edit_request.lon,
			edit_request.alt,
			edit_request.waypoint_error,
			edit_request.marker_type,
			edit_request.aruco_id,
			edit_request.aruco_id_2,
			edit_request.marker_id
		)

		if response.success == False:
			rospy.logwarn(f"Marker manager could not find marker with {edit_request.marker_id}! Cannot edit!")

		return response

	def publish_current_marker_callback(self, _: PublishCurrentMarkerRequest):
		response = PublishCurrentMarkerResponse()
		response.success = True

		# check if no markers
		current_marker = self.current_marker()
		if not current_marker:
			rospy.logwarn("Marker manager has no markers! Cannot publish current marker!")
			response.success = False
			return response

		# try to publish current marker
		publish_current_marker_success = self.publish_marker(current_marker)
		if not publish_current_marker_success:
			rospy.logwarn("Marker manager has no global origin! Cannot publish waypoint!")
			response.success = False
			return response

		return response

	def plan_home_marker_callback(self, _: PlanHomeMarkerRequest):
		response = PlanHomeMarkerResponse()
		response.success = True

		global_origin = self.get_global_origin()
		lat0 = global_origin.latitude
		lon0 = global_origin.longitude
		h0 = global_origin.altitude
		error = 0
		marker_type = "no_marker"
		aruco_id = 0
		aruco_id_2 = 0

		self.clear_markers()
		new_marker = self.make_marker(lat0, lon0, h0, error, marker_type, aruco_id, aruco_id_2)
		self.add_marker(new_marker)
		self.publish_marker(self.current_marker())
		return response

	def remove_current_marker_callback(self, _: RemoveCurrentMarkerRequest):
		response = RemoveCurrentMarkerResponse()

		# check if no markers
		current_marker = self.current_marker()
		if not current_marker:
			rospy.logwarn("Marker manager has no markers! Cannot remove current marker!")
			response.success = False
			return response

		removed_marker = self.remove_marker_by_id(current_marker.marker_id)
		if removed_marker is None:
			rospy.logwarn(f"Marker manager could not remove current marker!")

		response.success = removed_marker is not None
		return response

	def remove_marker_callback(self, remove_request: RemoveMarkerRequest):
		response = RemoveMarkerResponse()

		removed_marker = self.remove_marker_by_id(remove_request.marker_id)
		if removed_marker is None:
			rospy.logwarn(f"Marker manager has no marker with id {remove_request.marker_id}! Cannot remove marker!")
		
		response.success = removed_marker is not None
		return response

	def reorder_marker_callback(self, reorder_request: ReorderMarkerRequest):
		response = ReorderMarkerResponse()
		response.success = True

		removed_marker = self.remove_marker_by_id(reorder_request.marker_id)
		if removed_marker is None:
			rospy.logwarn(f"Marker manager has no marker with id {reorder_request.marker_id}! Cannot reorder marker!")
			response.success = False
			return response
		
		self.insert_marker(reorder_request.new_following_marker_id, removed_marker)
		return response
	
	def reorder_markers_callback(self, reorder_request: ReorderMarkersRequest):
		response = ReorderMarkersResponse()
		response.success = True

		ordered_marker_ids = reorder_request.marker_ids

		# make dictionary from original markers
		original_marker_list = self.get_marker_list()
		markers_dict = {}
		for marker in original_marker_list.markers:
			markers_dict[marker.marker_id] = marker

		# initialize new order marker list
		new_order_marker_list = GeodeticMarkerList()
		new_order_markers = new_order_marker_list.markers
		new_order_marker_list.count = original_marker_list.count

		# arrange markers in correct order in new_order_markers
		for marker_id in ordered_marker_ids:
			if marker_id in markers_dict:
				marker = markers_dict[marker_id]
				new_order_markers.append(marker)

		# update marker list
		self.set_marker_list(new_order_marker_list)

		return response

	### loop #################################################################

	def loop(self):
		rospy.spin()

if __name__ == "__main__":
	main()
