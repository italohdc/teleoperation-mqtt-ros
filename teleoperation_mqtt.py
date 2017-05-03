import json
import paho.mqtt.client as mqtt
import rospy
import roslib
import tf
import geometry_msgs
from threading import Thread


mqtt_connected = False
ros_fixed_frame = "odom_combined"
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
                              tf.transformations.quaternion_from_euler(0, 0, 90),
                              rospy.Time.now(),
                              frame,
                              base_frame)

def get_pose_json(json_object, frame_name):
    """Transforms a json string into a geometry_msgs.Pose message"""
    pose = geometry_msgs.msg.Pose()
    try:
        json_pose = json_object[frame_name]
        pose.position.x = json_pose["position"]["x"]
        pose.position.y = json_pose["position"]["y"]
        pose.position.z = json_pose["position"]["z"]
        return pose

    except ValueError, e:
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

        publish_tf_transformation(pose_left, "/left", ros_fixed_frame)
        publish_tf_transformation(pose_right, "/right", ros_fixed_frame)
        publish_tf_transformation(pose_head, "/head", ros_fixed_frame)

        pose1 = geometry_msgs.msg.Pose()
        pose1.position.x = 0.05
        pose1.position.y = 0
        pose1.position.z = 0
        
        pose2 = geometry_msgs.msg.Pose()
        pose2.position.x = -0.05
        pose2.position.y = 0
        pose2.position.z = 0


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
    client.subscribe("oculus_teste", 0)

    rospy.init_node('teleoperation_mqtt')
    rospy.sleep(2)

    head_pub = rospy.Publisher('/oculus/head', geometry_msgs.msg.Pose, queue_size=10)
    left_hand_pub = rospy.Publisher('/oculus/left_hand', geometry_msgs.msg.Pose, queue_size=10)
    right_hand_pub = rospy.Publisher('/oculus/right_hand', geometry_msgs.msg.Pose, queue_size=10)

    thread = Thread(target = threaded_spin, args=[])
    thread.start()

    while(True):
        client.loop()
