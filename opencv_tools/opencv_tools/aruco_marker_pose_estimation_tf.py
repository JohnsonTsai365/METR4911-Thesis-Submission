# Publishes a coordinate transformation between an ArUco marker and a camera
  
# Import the necessary ROS 2 libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import TransformStamped # Handles TransformStamped message
from sensor_msgs.msg import Image # Image is the message type
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
import time

# Import Python libraries
import cv2 # OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import faulthandler

SIMULATION = False

# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

faulthandler.enable()

# Sends a static transform from the aruco tag to the map
def aruco_to_map(node, x, y, z, roll, pitch, yaw, aruco_id):
    t = TransformStamped()
    t.child_frame_id = aruco_id
    t.header.frame_id = 'map'
    t.header.stamp = node.get_clock().now().to_msg()
    node.tf_static_broadcaster = StaticTransformBroadcaster(node)

    # Store the translation (i.e. position) information
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z 
    
    # Store the rotation information as radians in matrix
    r = R.from_rotvec(np.pi/2*np.array([roll,pitch,yaw]))
    quat = r.as_quat()

    # Quaternion format     
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    node.tf_static_broadcaster.sendTransform(t)

class ArucoNode(Node):
  """
  Create an ArucoNode class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('aruco_node')

    # Declare parameters
    self.declare_parameter("aruco_dictionary_name", "DICT_4X4_50")
    self.declare_parameter("aruco_marker_side_length", 0.2)
    self.declare_parameter("camera_calibration_parameters_filename", "/home/metr4810/ros2_ws/src/opencv_tools/opencv_tools/calibration_chessboard.yaml")
    self.declare_parameter("image_topic_real", "/image_raw")
    self.declare_parameter("aruco_marker_name", "aruco_marker")
    
    self.frame_rate = 2
    self.prev_capture_time = 0
    
    self.old_t = TransformStamped()
    
    self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
    
    # Read parameters
    aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
    self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value
    self.camera_calibration_parameters_filename = self.get_parameter(
      "camera_calibration_parameters_filename").get_parameter_value().string_value
    if SIMULATION:
      image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
    else:
      image_topic = self.get_parameter("image_topic_real").get_parameter_value().string_value
    self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value

    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
      self.get_logger().info("[INFO] ArUCo tag of '{}' is not supported".format(
        args["type"]))

    # Load the camera parameters from the saved file
    cv_file = cv2.FileStorage(
      self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
    self.mtx = cv_file.getNode('K').mat()
    self.dst = cv_file.getNode('D').mat()
    cv_file.release()

    aruco_to_map(self, 0.0, 0.0, 0.0, 0, 0, 0, "odom")
    
    # Load the ArUco dictionary
    self.get_logger().info("[INFO] detecting '{}' markers...".format(
  	  aruco_dictionary_name))
    self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])
    self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()

    # Initialize the transform broadcaster
    self.tfbroadcaster = TransformBroadcaster(self)

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      image_topic, 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()

    if not SIMULATION:
      self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
      assert self.cap.isOpened()
      frame_rate = 2
      prev = 0
      while(True):
        time_elapsed = time.time() - prev
        if time_elapsed > 1./frame_rate:
          prev = time.time()
          self.get_logger().info('Receiving video frame')
          ret, current_frame = self.cap.read()
          current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
          self.aruco_detect(current_frame)
        

  def publish_odometry(self, x, y, z, quat_x, quat_y, quat_z, quat_w):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = quat_x
        msg.pose.pose.orientation.y = quat_y
        msg.pose.pose.orientation.z = quat_z
        msg.pose.pose.orientation.w = quat_w
        msg.pose.covariance[0]  = 0.01
        msg.pose.covariance[7]  = 0.01
        msg.pose.covariance[14] = 0.01
        msg.pose.covariance[21] = 0.01
        msg.pose.covariance[28] = 0.01
        msg.pose.covariance[35] = 0.01
        self.odom_pub.publish(msg)
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    time_elapsed = time.time() - self.prev_capture_time + 0.01
    # small time difference for if camera refreshes at the same time as time_elapsed
    if time_elapsed > 1./self.frame_rate:
      self.prev_capture_time = time.time()
      # Display the message on the console
      self.get_logger().info('Receiving video frame')

      # Convert ROS Image message to OpenCV image
      current_frame = self.bridge.imgmsg_to_cv2(data, "rgb8")
      self.get_logger().info('2. I heard: "%s"' % data.encoding)
      self.aruco_detect(current_frame)
    
  def aruco_detect(self, current_frame):
    # Detect ArUco markers in the video frame
    (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)

    # Check that at least one ArUco marker was detected
    if marker_ids is not None:
    
      # Draw a square around detected markers in the video frame
      # cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)

      # Get the rotation and translation vectors
      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        self.aruco_marker_side_length,
        self.mtx,
        self.dst)
        
      # The pose of the marker is with respect to the camera lens frame.
      # Imagine you are looking through the camera viewfinder, 
      # the camera lens frame's:
      # x-axis points to the right
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera
      marker_id = marker_ids[0]

      # Create the coordinate transform
      t = TransformStamped()
      t.header.stamp = self.get_clock().now().to_msg()
      t.header.frame_id = 'odom'
      t.child_frame_id = 'base_link'
      t.transform.translation.z = 0.0
      r_transform = np.array(rvecs[0][0])

      match marker_id:
        # rotate all markers upside down
        # 0 at front of arena
        # 1 on right
        # 2 on bottom
        # 3 on left
        # 2m arena, tags placed 0.5 from the center of each edge of the arena, center is (0,0)
        case 0:
          # Store the translation (i.e. position) information
          t.transform.translation.x = -tvecs[0][0][0]
          t.transform.translation.y = -tvecs[0][0][2] + 1.5

          r_transform[1], r_transform[2] = r_transform[2], r_transform[1] - 1.5707
        
        case 1:
          t.transform.translation.y = tvecs[0][0][0]
          t.transform.translation.x = -tvecs[0][0][2] + 1.5

          r_transform[1], r_transform[2] = r_transform[2], r_transform[1] - 3.1415
        
        case 2:
          t.transform.translation.x = tvecs[0][0][0]
          t.transform.translation.y = tvecs[0][0][2] - 1.5

          r_transform[1], r_transform[2] = r_transform[2], r_transform[1] - 1.5707
          r_transform[0], r_transform[1] = r_transform[1], -r_transform[0]
          
        case 3:
          t.transform.translation.y = -tvecs[0][0][0]
          t.transform.translation.x = tvecs[0][0][2] - 1.5

          r_transform[1], r_transform[2] = r_transform[2], r_transform[1]
          
        case _:
          # Never supposed to be getting here
          # Send the previous valid location
          self.tfbroadcaster.sendTransform(self.old_t)
          return
      
      r_transform[0] = 0.0
      r_transform[1] = 0.0
          
      # Store the rotation information
      rotation_matrix = np.eye(4)
      rotation_matrix[0:3, 0:3] = cv2.Rodrigues(r_transform)[0]
      r = R.from_matrix(rotation_matrix[0:3, 0:3])
      quat = r.as_quat()
        
      # Quaternion format
      t.transform.rotation.x = quat[0]
      t.transform.rotation.y = quat[1] 
      t.transform.rotation.z = quat[2] 
      t.transform.rotation.w = quat[3]

      # Send the transform
      self.tfbroadcaster.sendTransform(t)
      # Publish the odometry alongside it
      self.publish_odometry(t.transform.translation.x, t.transform.translation.y, 0.0, quat[0], quat[1], quat[2], quat[3])

      print("transform_translation_x: {}".format(t.transform.translation.x))
      print("transform_translation_y: {}".format(t.transform.translation.y))
      print("transform_translation_z: {}".format(t.transform.translation.z))  
      print("{}".format(r_transform))
      
      # Store previous waypoint to be used in case of
      self.old_t = t
                  
      # Draw the axes on the marker
      if SIMULATION:
        cv2.drawFrameAxes(current_frame, self.mtx, self.dst, rvecs[0], tvecs[0], 0.05)        
              
    # Display image
    if SIMULATION:
      cv2.imshow("camera", current_frame)
    
    cv2.waitKey(1)    
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  aruco_node = ArucoNode()
  
  # Spin the node so the callback function is called.
  rclpy.spin(aruco_node)
  aruco_node.cap.release()
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  aruco_node.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
