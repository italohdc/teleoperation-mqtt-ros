import json
import paho.mqtt.client as mqtt
import rospy
import roslib
import tf
import geometry_msgs
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerInit
import math
from threading import Thread
import threading
import time

from moveit_ros_planning_interface._moveit_robot_interface import RobotInterface


mqtt_connected = False
ros_fixed_frame = "odom_combined"
client = None # MQTT client
mqtt_hostname = "localhost"
mqtt_port = 1883

class Buttons():
    def __init__(self):
        self.X = False
        self.Y = False
        self.A = False
        self.B = False
        self.left_thumb = False
        self.right_thumb = False
        self.right_index_trigger = 0
        self.left_index_trigger = 0
        self.right_hand_trigger = 0
        self.left_hand_trigger = 0
        
        self.left_thumbstick = []
        self.right_thumbstick = []

    def get_buttons_json(self, json_object):
        """ Finds the button information and creates a message """

        self.X = json_object["X"]
        self.Y = json_object["Y"]
        self.A = json_object["A"]
        self.B = json_object["B"]
        self.right_hand_trigger = json_object["right_hand_trigger"]
        self.right_index_trigger = json_object["right_index_trigger"]
        self.left_hand_trigger = json_object["left_hand_trigger"]
        self.left_index_trigger = json_object["left_index_trigger"]

        self.left_thumb = json_object["left_thumb"]
        self.right_thum = json_object["right_thumb"]

        self.left_thumbstick = json_object["left_thumbstick"]
        self.right_thumbstick = json_object["right_thumbstick"]

class StatusHistory():
  def __init__(self, max_length=10):
    self.max_length = max_length
    self.buffer = []
  def add(self, status):
    self.buffer.append(status)
    if len(self.buffer) > self.max_length:
      self.buffer = self.buffer[1:self.max_length+1]
  def all(self, proc):
    for status in self.buffer:
      if not proc(status):
        return False
    return True
  def latest(self):
    if len(self.buffer) > 0:
      return self.buffer[-1]
    else:
      return None
  def length(self):
    return len(self.buffer)
  def new(self, status, attr):
    if len(self.buffer) == 0:
      return getattr(status, attr)
    else:
      return getattr(status, attr) and not getattr(self.latest(), attr)

class TeleoperationOculus:
    def __init__(self):
        self.buttons = Buttons()
        self.pose_left = geometry_msgs.msg.Pose()
        self.pose_right = geometry_msgs.msg.Pose()
        self.pose_head = geometry_msgs.msg.Pose()

        self.initial_poses = {}
        self.planning_groups_tips = {}
        self.tf_listener = tf.TransformListener()
        self.marker_lock = threading.Lock()
        self.prev_time = rospy.Time.now()
        self.counter = 0
        self.history = StatusHistory(max_length=10)
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        self.current_planning_group_index = 0
        self.current_eef_index = 0
        self.initialize_poses = False
        self.initialized = False
        self.parseSRDF()
        self.plan_group_pub = rospy.Publisher('/rviz/moveit/select_planning_group', String, queue_size=5)
        self.updatePlanningGroup(0)
        self.updatePoseTopic(0, False)
        self.marker_lock = threading.Lock()

        self.plan_group_pub = rospy.Publisher('/rviz/moveit/select_planning_group', String, queue_size=5)
        self.plan_pub = rospy.Publisher("/rviz/moveit/plan", Empty, queue_size=5)
        self.execute_pub = rospy.Publisher("/rviz/moveit/execute", Empty, queue_size=5)
        self.update_start_state_pub = rospy.Publisher("/rviz/moveit/update_start_state", Empty, queue_size=5)
        self.update_goal_state_pub = rospy.Publisher("/rviz/moveit/update_goal_state", Empty, queue_size=5)
        self.interactive_marker_sub = rospy.Subscriber("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full",
                                                  InteractiveMarkerInit,
                                                  self.markerCB, queue_size=1)
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect
        self.client.connect(mqtt_hostname, mqtt_port, 60)
        self.client.subscribe("oculus_lucas_teste", 0)

        self.thread = Thread(target = self.threaded_spin_mqtt, args=[])
        self.thread.start()
        
    def waitForInitialPose(self, next_topic, timeout=None):
        counter = 0
        while not rospy.is_shutdown():
            counter = counter + 1
            if timeout and counter >= timeout:
                return False
            try:
                self.marker_lock.acquire()
                self.initialize_poses = True
                topic_suffix = next_topic.split("/")[-1]
                if self.initial_poses.has_key(topic_suffix):
                    self.pre_pose = PoseStamped(pose=self.initial_poses[topic_suffix])
                    self.initialize_poses = False
                    return True
                else:
                    rospy.logdebug(self.initial_poses.keys())
                    rospy.loginfo("Waiting for pose topic of '%s' to be initialized",
                                topic_suffix)
                    rospy.sleep(1)
            finally:
                self.marker_lock.release()

    def updatePlanningGroup(self, next_index):
        if next_index >= len(self.planning_groups_keys):
            self.current_planning_group_index = 0
        elif next_index < 0:
            self.current_planning_group_index = len(self.planning_groups_keys) - 1
        else:
            self.current_planning_group_index = next_index
        next_planning_group = None
        try:
            next_planning_group = self.planning_groups_keys[self.current_planning_group_index]
        except IndexError:
            msg = 'Check if you started movegroups. Exiting.'
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)
        rospy.loginfo("Changed planning group to " + next_planning_group)
        self.plan_group_pub.publish(next_planning_group)
    def updatePoseTopic(self, next_index, wait=True):
        planning_group = self.planning_groups_keys[self.current_planning_group_index]
        topics = self.planning_groups[planning_group]
        if next_index >= len(topics):
            self.current_eef_index = 0
        elif next_index < 0:
            self.current_eef_index = len(topics) - 1
        else:
            self.current_eef_index = next_index
        next_topic = topics[self.current_eef_index]

        rospy.loginfo("Changed controlled end effector to " + self.planning_groups_tips[planning_group][self.current_eef_index])
        self.pose_pub = rospy.Publisher(next_topic, PoseStamped, queue_size=5)
        if wait:
            self.waitForInitialPose(next_topic)
        self.current_pose_topic = next_topic


    def parseSRDF(self):
        ri = RobotInterface("/robot_description")
        planning_groups = {}
        for g in ri.get_group_names():
            self.planning_groups_tips[g] = ri.get_group_joint_tips(g)
            planning_groups[g] = ["/rviz/moveit/move_marker/goal_" + l
                                  for l in self.planning_groups_tips[g]]
        for name in planning_groups.keys():
            if len(planning_groups[name]) == 0:
                del planning_groups[name]
            else:
                print name, planning_groups[name]
        self.planning_groups = planning_groups
        self.planning_groups_keys = planning_groups.keys()   #we'd like to store the 'order'
        self.frame_id = ri.get_planning_frame()

    def markerCB(self, msg):
        try:
            self.marker_lock.acquire()
            if not self.initialize_poses:
                return
            self.initial_poses = {}
            for marker in msg.markers:
                if marker.name.startswith("EE:goal_"):
                    # resolve tf
                    if marker.header.frame_id != self.frame_id:
                        ps = PoseStamped(header=marker.header, pose=marker.pose)
                        try:
                            transformed_pose = self.tf_listener.transformPose(self.frame_id, ps)
                            self.initial_poses[marker.name[3:]] = transformed_pose.pose
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, e):
                            rospy.logerr("tf error when resolving tf: %s" % e)
                    else:
                        self.initial_poses[marker.name[3:]] = marker.pose   #tf should be resolved
        finally:
            self.marker_lock.release()


    def on_connect(self, mqttc, obj, flags, rc) :
        """Called when the client connects to the mqtt server"""
        global mqtt_connected
        print "Connected with result code "+str(rc)
        mqtt_connected = True

    def on_disconnect(self, client_mqtt, data, result_code):
        """Called when mqtt disconnects. Tries to reconnect to the server"""
        global mqtt_hostname, mqtt_port
        print "Disconnected from MQTT server. Trying to reconnect"
        client_mqtt.connect(mqtt_hostname, mqtt_port, 60)

        # The callback for when a PUBLISH message is received from the server.
    def on_message(self, client_mqtt, userdata, msg):
        """ receives data from the mqtt topic """
        #print msg.topic+" "+str(msg.payload)
        try: 
            json_object = json.loads(msg.payload)

            self.pose_left = self.get_pose_json(json_object, "left")
            self.pose_right = self.get_pose_json(json_object, "right")
            self.pose_head = self.get_pose_json(json_object, "head")
            self.buttons = self.get_buttons_json(json_object, "buttons")

            self.publish_transformations(self.pose_left, self.pose_right, self.pose_head)
            
            self.hand_tracking(self.buttons)

        except ValueError, e:
            print "Error while parsing message from mqtt server"
            print e

    def euclidian_distance(self, p1, p2):
        return math.sqrt(math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2) + math.pow(p1.z - p2.z, 2))

    def hand_tracking(self, button):
        if not self.initialized:
            # when not initialized, we will force to change planning_group
            while True:
                self.updatePlanningGroup(self.current_planning_group_index)
                planning_group = self.planning_groups_keys[self.current_planning_group_index]
                topics = self.planning_groups[planning_group]
                next_topic = topics[self.current_eef_index]
                if not self.waitForInitialPose(next_topic, timeout=3):
                    rospy.logwarn("Unable to initialize planning group " + planning_group + ". Trying different group.")
                    rospy.logwarn("Is 'Allow External Comm.' enabled in Rviz? Is the 'Query Goal State' robot enabled?")
                else:
                    rospy.loginfo("Initialized planning group")
                    self.initialized = True
                    self.updatePoseTopic(self.current_eef_index)
                    return
                # Try to initialize with different planning group
                self.current_planning_group_index += 1
                if self.current_planning_group_index >= len(self.planning_groups_keys):
                    self.current_planning_group_index = 0 # reset loop
        if self.history.new(button, "Y"):   #increment planning group
            self.updatePlanningGroup(self.current_planning_group_index + 1)
            self.current_eef_index = 0    # force to reset
            self.updatePoseTopic(self.current_eef_index)
            return
        if self.history.new(button, "X"):   #decrement planning group
            self.updatePlanningGroup(self.current_planning_group_index - 1)
            self.current_eef_index = 0    # force to reset
            self.updatePoseTopic(self.current_eef_index)
            return
        if self.history.new(button, "right_thumb"):
            self.updatePoseTopic(self.current_eef_index + 1)
            return
        if self.history.new(button, "left_thumb"):
            self.updatePoseTopic(self.current_eef_index - 1)
            return
        if self.history.new(button, "A"):   #plan
            rospy.loginfo("Plan")
            self.plan_pub.publish(Empty())
            return
        if self.history.new(button, "B"):   #execute
            rospy.loginfo("Execute")
            print "Execute"
            self.execute_pub.publish(Empty())
            return
        self.marker_lock.acquire()
        pre_pose = self.pre_pose
        new_pose = self.computePoseFromJoy(pre_pose, button, self.pose_left, self.pose_right)
        now = rospy.Time.from_sec(time.time())
        # placement.time_from_start = now - self.prev_time
        if (new_pose is not None) and ((now - self.prev_time).to_sec() > 1 / 30.0):
            # rospy.loginfo(new_pose)
            self.pose_pub.publish(new_pose)
            #self.joy_pose_pub.publish(new_pose)
            self.prev_time = now
        # sync start state to the real robot state
        self.counter = self.counter + 1
        if self.counter % 10:
            self.update_start_state_pub.publish(Empty())
        
        if(new_pose is not None):
            self.pre_pose = new_pose
        self.marker_lock.release()
        # update self.initial_poses
        self.marker_lock.acquire()
        self.initial_poses[self.current_pose_topic.split("/")[-1]] = new_pose.pose
        self.marker_lock.release()

    def computePoseFromJoy(self, pre_pose, button, pose_left, pose_right):
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame_id
        new_pose.header.stamp = rospy.Time(0.0)

        pose_hand = None

        diff_left = self.euclidian_distance(pre_pose.pose.position, pose_left.position)
        diff_right = self.euclidian_distance(pre_pose.pose.position, pose_right.position)

        #print diff_left, diff_right, button.left_index_trigger, button.right_index_trigger

        if(diff_left < 0.2): 
            if(button.left_index_trigger > 0.2):
                pose_hand = pose_left
                print "Copy pose"
        elif(diff_right < 0.2):
            if(button.right_index_trigger > 0.2):
                pose_hand = pose_right
                print "Copy pose"

        if(pose_hand == None):
            return pre_pose
        
        # move in local
        # dist = status.left_analog_y * status.left_analog_y + status.left_analog_x * status.left_analog_x
        # scale = 200.0
        # x_diff = signedSquare(status.left_analog_y) / scale
        # y_diff = signedSquare(status.left_analog_x) / scale
        # # z
        # if status.L2:
        #     z_diff = 0.005
        # elif status.R2:
        #     z_diff = -0.005
        # else:
        #     z_diff = 0.0
        # if self.history.all(lambda s: s.L2) or self.history.all(lambda s: s.R2):
        #     z_scale = 4.0
        # else:
        #     z_scale = 2.0
        # local_move = numpy.array((x_diff, y_diff,
        #                           z_diff * z_scale,
        #                           1.0))
        # q = numpy.array((pre_pose.pose.orientation.x,
        #                  pre_pose.pose.orientation.y,
        #                  pre_pose.pose.orientation.z,
        #                  pre_pose.pose.orientation.w))
        # xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q),
        #                  local_move)

        new_pose.pose.position.x = pose_hand.position.x
        new_pose.pose.position.y = pose_hand.position.y
        new_pose.pose.position.z = pose_hand.position.z
        # roll = 0.0
        # pitch = 0.0
        # yaw = 0.0
        # DTHETA = 0.005
        # if status.L1:
        #     if self.history.all(lambda s: s.L1):
        #         yaw = yaw + DTHETA * 2
        #     else:
        #         yaw = yaw + DTHETA
        # elif status.R1:
        #     if self.history.all(lambda s: s.R1):
        #         yaw = yaw - DTHETA * 2
        #     else:
        #         yaw = yaw - DTHETA
        # if status.up:
        #     if self.history.all(lambda s: s.up):
        #         pitch = pitch + DTHETA * 2
        #     else:
        #         pitch = pitch + DTHETA
        # elif status.down:
        #     if self.history.all(lambda s: s.down):
        #         pitch = pitch - DTHETA * 2
        #     else:
        #         pitch = pitch - DTHETA
        # if status.right:
        #     if self.history.all(lambda s: s.right):
        #         roll = roll + DTHETA * 2
        #     else:
        #         roll = roll + DTHETA
        # elif status.left:
        #     if self.history.all(lambda s: s.left):
        #         roll = roll - DTHETA * 2
        #     else:
        #         roll = roll - DTHETA
        # diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        # new_q = tf.transformations.quaternion_multiply(q, diff_q)

        new_pose.pose.orientation.x = pose_hand.orientation.x
        new_pose.pose.orientation.y = pose_hand.orientation.y
        new_pose.pose.orientation.z = pose_hand.orientation.z
        new_pose.pose.orientation.w = pose_hand.orientation.w

        print "Following hand"
        return new_pose

    def publish_tf_transformation(self, msg, frame, base_frame):
        """Publishes a transformation"""
        broadcaster = tf.TransformBroadcaster()
        broadcaster.sendTransform((msg.position.x, msg.position.y, msg.position.z),
                                (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                                rospy.Time.now(),
                                frame,
                                base_frame)

    def array_to_quat(self, quat):
        quatPose = geometry_msgs.msg.Quaternion()
        quatPose.x = quat[0]
        quatPose.y = quat[1]
        quatPose.z = quat[2]
        quatPose.w = quat[3]
        return quatPose

    def get_buttons_json(self, json_object, frame_name):
        """ Finds the button information and creates a message """

        buttons = Buttons()
        json_buttons = json_object[frame_name]
        buttons.X = json_buttons["X"]
        buttons.Y = json_buttons["Y"]
        buttons.A = json_buttons["A"]
        buttons.B = json_buttons["B"]
        buttons.right_hand_trigger = json_buttons["right_hand_trigger"]
        buttons.right_index_trigger = json_buttons["right_index_trigger"]
        buttons.left_hand_trigger = json_buttons["left_hand_trigger"]
        buttons.left_index_trigger = json_buttons["left_index_trigger"]

        buttons.left_thumbstick = json_buttons["left_thumbstick"]
        buttons.right_thumbstick = json_buttons["right_thumbstick"]

        return buttons


    def get_pose_json(self, json_object, frame_name):
        """Transforms a json string into a geometry_msgs.Pose message"""
        pose = geometry_msgs.msg.Pose()
        try:
            json_pose = json_object[frame_name]
            pose.position.y = - json_pose["pose"]["position"]["x"]
            pose.position.x = -json_pose["pose"]["position"]["z"]
            pose.position.z = json_pose["pose"]["position"]["y"] + 1.5

            pose.orientation.w = json_pose["pose"]["orientation"]["w"]
            pose.orientation.x = json_pose["pose"]["orientation"]["x"]
            pose.orientation.y = json_pose["pose"]["orientation"]["y"]
            pose.orientation.z = json_pose["pose"]["orientation"]["z"]

            # fuck transformations, doing transformations in Rambo mode
            quat = (
                -pose.orientation.z,  
                -pose.orientation.x,
                pose.orientation.y,
                pose.orientation.w)

            pose.orientation = self.array_to_quat(quat)
            return pose

        except ValueError, e:
            print e
            return None
    
    def threaded_spin_mqtt(self):
        while(True):
            self.client.loop()

    def publish_transformations(self, pose_left, pose_right, pose_head):

        transformation_camera1 = geometry_msgs.msg.Pose()
        transformation_camera1.position.x = 0
        transformation_camera1.position.y = -0.032
        transformation_camera1.position.z = 0 

        quat = (
            transformation_camera1.orientation.x,
            transformation_camera1.orientation.y,
            transformation_camera1.orientation.z,
            1)

        rot1 = tf.transformations.quaternion_about_axis(-math.pi/2, (0, 0, 1))
        rot2 = tf.transformations.quaternion_about_axis(-math.pi/2, (1, 0, 0))

        quat = tf.transformations.quaternion_multiply(quat, rot1)
        quat = tf.transformations.quaternion_multiply(quat, rot2)

        transformation_camera1.orientation = self.array_to_quat(quat)

        transformation_camera2 = geometry_msgs.msg.Pose()
        transformation_camera2.position.x = 0
        transformation_camera2.position.y = 0.032
        transformation_camera2.position.z = 0
        transformation_camera2.orientation = transformation_camera1.orientation

        self.publish_tf_transformation(pose_left, "left", ros_fixed_frame)
        self.publish_tf_transformation(pose_right, "right", ros_fixed_frame)
        self.publish_tf_transformation(pose_head, "head", ros_fixed_frame)

        self.publish_tf_transformation(transformation_camera1, "camera1", "head")
        self.publish_tf_transformation(transformation_camera2, "camera2", "head")

    

def threaded_spin():
    while(True):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('teleoperation_mqtt')
    rospy.sleep(2)

    teleoperation = TeleoperationOculus()

    thread = Thread(target = threaded_spin, args=[])
    thread.daemon = True
    thread.start()

    while(True):
        time.sleep(10)
