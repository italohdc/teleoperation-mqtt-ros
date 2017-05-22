import json
import paho.mqtt.client as mqtt
import rospy
import roslib
import tf
import geometry_msgs
import math
from threading import Thread

mqtt_connected = False
ros_fixed_frame = "world"
client = None # MQTT client
mqtt_hostname = "localhost"
mqtt_port = 1883

def on_connect(client_mqtt, userdata, result_code):
    """Called when the client connects to the mqtt server"""
    global mqtt_connected
    print "Connected with result code "+str(result_code)
    mqtt_connected = True

def on_disconnect(client_mqtt, data, result_code):
    """Called when mqtt disconnects. Tries to reconnect to the server"""
    global mqtt_hostname, mqtt_port
    print "Disconnected from MQTT server. Trying to reconnect"
    client_mqtt.connect(mqtt_hostname, mqtt_port, 60)

def publish_tf_transformation(msg, frame, base_frame):
    """Publishes a transformation"""
    broadcaster = tf.TransformBroadcaster()
    broadcaster.sendTransform((msg.position.x, msg.position.y, msg.position.z),
                              (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                              rospy.Time.now(),
                              frame,
                              base_frame)

def array_to_quat(quat):
    quatPose = geometry_msgs.msg.Quaternion()
    quatPose.x = quat[0]
    quatPose.y = quat[1]
    quatPose.z = quat[2]
    quatPose.w = quat[3]
    return quatPose

def get_pose_json(json_object, frame_name):
    """Transforms a json string into a geometry_msgs.Pose message"""
    pose = geometry_msgs.msg.Pose()
    try:
        json_pose = json_object[frame_name]
        pose.position.y = - json_pose["pose"]["position"]["x"]
        pose.position.x = -json_pose["pose"]["position"]["z"]
        pose.position.z = json_pose["pose"]["position"]["y"]

        pose.orientation.w = json_pose["pose"]["orientation"]["w"]
        pose.orientation.x = json_pose["pose"]["orientation"]["x"]
        pose.orientation.y = json_pose["pose"]["orientation"]["y"]
        pose.orientation.z = json_pose["pose"]["orientation"]["z"]

        quat = (
            -pose.orientation.z,
            -pose.orientation.x,
            pose.orientation.y,
            pose.orientation.w)


        rot1 = tf.transformations.quaternion_about_axis(-math.pi/2, (1, 0, 0))
        rot2 = tf.transformations.quaternion_about_axis(math.pi/2, (1, 0, 0))
        #rot3 = tf.transformations.quaternion_about_axis(math.pi/2, (0, 1, 0))

        #quat = tf.transformations.quaternion_multiply(quat, rot1)
        #quat = tf.transformations.quaternion_multiply(quat, rot2)
        #quat4 = tf.transformations.quaternion_multiply(quat3, rot3)

        pose.orientation = array_to_quat(quat)

        #print pose
        return pose

    except ValueError, e:
        print e
        return None

# The callback for when a PUBLISH message is received from the server.
def on_message(client_mqtt, userdata, msg):
    """ receives data from the mqtt topic """
    #print msg.topic+" "+str(msg.payload)
    try: 
        json_object = json.loads(msg.payload)
        pose_left = get_pose_json(json_object, "left")
        pose_right = get_pose_json(json_object, "right")
        pose_head = get_pose_json(json_object, "head")

        pose1 = geometry_msgs.msg.Pose()
        pose1.position.x = 0
        pose1.position.y = -0.032
        pose1.position.z = 0 

        quat = (
            pose1.orientation.x,
            pose1.orientation.y,
            pose1.orientation.z,
            1)

        rot1 = tf.transformations.quaternion_about_axis(-math.pi/2, (0, 0, 1))
        rot2 = tf.transformations.quaternion_about_axis(-math.pi/2, (1, 0, 0))
        #rot3 = tf.transformations.quaternion_about_axis(math.pi/2, (0, 0, 1))

        quat = tf.transformations.quaternion_multiply(quat, rot1)
        quat = tf.transformations.quaternion_multiply(quat, rot2)
        #quat = tf.transformations.quaternion_multiply(quat, rot3)

        pose1.orientation = array_to_quat(quat)
        
        pose2 = geometry_msgs.msg.Pose()
        pose2.position.x = 0
        pose2.position.y = 0.032
        pose2.position.z = 0
        pose2.orientation = pose1.orientation

        #rotation = geometry_msgs.msg.Pose()
        #rotation.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-math.pi/2, math.pi/2, 0))

        #rotation2 = geometry_msgs.msg.Pose()
        #rotation2.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(math.pi/2, 0, 0))

        #rotation3 =  geometry_msgs.msg.Pose()
        #rotation3.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, -math.pi/2))

        publish_tf_transformation(pose_left, "/left", ros_fixed_frame)
        publish_tf_transformation(pose_right, "/right", ros_fixed_frame)
        publish_tf_transformation(pose_head, "/head", ros_fixed_frame)
        #publish_tf_transformation(rotation, "/left_r1", "/left_oculus")
        #publish_tf_transformation(rotation, "/right_r1", "/right_oculus")
        #publish_tf_transformation(rotation, "/head_r1", "/head_oculus")
        #publish_tf_transformation(rotation2, "/left_r2", "/left_r1")
        #publish_tf_transformation(rotation2, "/right_r2", "/right_r1")
        #publish_tf_transformation(rotation2, "/head_r2", "/head_r1")
        #publish_tf_transformation(rotation3, "/left", "/left_r2")
        #publish_tf_transformation(rotation3, "/right", "/right_r2")
        #publish_tf_transformation(rotation3, "/head", "/head_r2")
        publish_tf_transformation(pose1, "/camera1", "/head")
        publish_tf_transformation(pose2, "/camera2", "/head")

    except ValueError, e:
        print "Error while parsing message from mqtt server"
        print e

def threaded_spin():
    while(True):
        rospy.spin()

if __name__ == '__main__':
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.connect(mqtt_hostname, mqtt_port, 60)
    client.subscribe("oculus_lucas_teste", 0)

    rospy.init_node('teleoperation_mqtt')
    rospy.sleep(2)

    head_pub = rospy.Publisher('/oculus/head', geometry_msgs.msg.Pose, queue_size=10)
    left_hand_pub = rospy.Publisher('/oculus/left_hand', geometry_msgs.msg.Pose, queue_size=10)
    right_hand_pub = rospy.Publisher('/oculus/right_hand', geometry_msgs.msg.Pose, queue_size=10)

    thread = Thread(target = threaded_spin, args=[])
    thread.start()

    while(True):
        client.loop()
