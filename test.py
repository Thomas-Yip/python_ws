import cv2
import numpy as np
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image, Imu, Joy # Image is the message type
from std_msgs.msg import Float64MultiArray, Bool
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import math
from enum import Enum
def set_motion(motion,thrust):
    data = [0,0,0,0]
    if(motion ==0):
      data = [1*thrust,0.0,0.0,0.0]
    elif(motion == 1):
      data = [-1*thrust,0.0,0.0,0.0]
    elif(motion == 2):
      data = [0.0,1*thrust,0.0,0.0]
    elif(motion == 3):
      data = [0.0,-1*thrust,0.0,0.0]
    elif(motion == 4):
      data = [0,0,1*thrust,0]
    elif(motion == 5):
      data = [0,0,-1*thrust,0]
    elif(motion == 6):
      data = [0,0,0,1*thrust]
    elif(motion == 7):
      data = [0,0,0,-1*thrust]
    elif(motion == 8):
      data = [0,0,0,0]
    for i in range(4):
       data[i] = float(data[i])
    # print(data)
    return data
class Motion(Enum):
  FORWARD = 0
  BACKWARD = 1
  LEFT = 3
  RIGHT = 2
  UP = 4
  DOWN = 5
  ROTATE_LEFT = 7
  ROTATE_RIGHT = 6
# Initialize the webcam

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/cam2/image', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.sub_imu = self.create_subscription(Imu,'/rov/imu',self.imu_callback,10)
    self.sub_imu
    self.angle = 0
    self.sub_joy = self.create_subscription(
       Bool,
       '/rov/auto',
       self.auto_callback,
       10)
    self.target_angle = 0
    self.target_is_set = False
    self.pub_thrust = self.create_publisher(
       Float64MultiArray,
       '/rov/movement',
       10)
    self.timer = self.create_timer(0.05,self.timer_callback)
    self.flow = 0    
    self.AUTO = False
    self.arrived = False
    self.action = 0
    self.dist2centre = 0
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
  
  def auto_callback(self,msg):
     if( msg.data == True):
        self.AUTO = True
        self.target_is_set = True
        self.target_angle = self.angle
     else:
        self.AUTO = False
        self.target_is_set = False

  def timer_callback(self):
    thrust_msg = Float64MultiArray()
    thrust_msg.data = [0.0,0.0,0.0,0.0]
    opp_angle = 0
    thrust = 5
    if(self.target_angle < 0):
        opp_angle = self.target_angle + 2*math.pi -math.pi
    else:
        opp_angle = self.target_angle + math.pi - 2*math.pi
    if (self.flow == 0):
      self.flow = 1
      if(self.target_angle < 0):
          if(self.angle < opp_angle and self.angle > self.target_angle):
              thrust_msg.data = set_motion(7,1)
          else:
              thrust_msg.data = set_motion(6,1)
      else:
          if(self.angle < self.target_angle and self.angle > opp_angle):
              thrust_msg.data = set_motion(6,1)
          else:
              thrust_msg.data = set_motion(7,1)
    else:
       self.flow = 0
       thrust_msg.data = set_motion(0,1)

    des_msg = Float64MultiArray()
    des_msg.data = [0.0,0.0,0.0,0.0]
    # des_msg.data = set_motion(6,3)
    if(self.action ==0):
        des_msg.data = set_motion(5,thrust)
    elif(self.action == 1):
       if(self.flow == 0):
          des_msg.data = set_motion(2,thrust)
          self.flow += 1
       else:
          self.flow = 0
          des_msg.data = set_motion(0,thrust)
    elif(self.action == 2):
       if(self.flow == 0):
          des_msg.data = set_motion(0,thrust)
          self.flow += 1
       else:
          self.flow = 0
          des_msg.data = set_motion(3,thrust)
    elif(self.action == 3):
       if(self.flow == 0):
          self.flow += 1
          des_msg.data = set_motion(1,thrust)
       else:
          des_msg.data = set_motion(3,thrust)
          self.flow = 0
    elif(self.action == 4):
       if(self.flow == 0):
          des_msg.data = set_motion(1,thrust)
          self.flow += 1
       else:
          des_msg.data =set_motion(2,thrust)
          self.flow = 0

    if(self.AUTO == True):
         if(self.arrived == False):          
          self.pub_thrust.publish(thrust_msg)
         else:
          self.pub_thrust.publish(des_msg)
    # self.pub_thrust.publish(thrust_msg)

  def imu_callback(self, data):
    """     
    double quaternion_to_z_axis_angle(double q[4]){
    double X=(2*(q[0]*q[3]+q[1]*q[2]));
    double Y=(1-2*(q[2]*q[2]+q[3]*q[3]));
    return atan2(-Y,X);
    """
    q = [data.orientation.w,data.orientation.x,data.orientation.y,data.orientation.z]
    X=(2*(q[0]*q[3]+q[1]*q[2]))
    Y=(1-2*(q[2]*q[2]+q[3]*q[3]))
    self.angle =  np.arctan2(-Y,X)

   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    frame = self.br.imgmsg_to_cv2(data)
    ori = frame

    # lower_red = np.array([160,100,100])
    lower_red = np.array([100,100,100])
    upper_red = np.array([179,255,255])
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,lower_red,upper_red)    
    result = cv2.bitwise_and(frame,frame,mask = mask)
    frame = result
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image = frame
    target_x = -1
    target_y = -1
 
    # Iterate over the contours
    for i in contours:
        # Get the bounding rectangle of the contour
        approx = cv2.approxPolyDP(i, 0.04 * cv2.arcLength(i, True), True)
        if len(approx) == 4 and cv2.contourArea(approx) > 200:
        # Draw bounding rectangle around the contour
            # x, y, w, h = cv2.boundingRect(approx)
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)   
             M = cv2.moments(i)
             if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.drawContours(image, [i], -1, (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 7, (0, 255, 0), -1)
                cv2.putText(image, "center", (cx - 20, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(image, 'coor: '+str(cx-20)+','+str(cy-20), (cx-20,cy-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))   
                target_x = cx-20
                target_y = cy-20   
    if(target_x != -1 and target_y != -1):
       self.arrived = True
    frame = image
    height, width = frame.shape[:2]
    print("height is ",height)
    print("height is ",width)
    x, y, w, h = 0, 0, width, height
    l = 25
    color = (0, 255, 0)  # Rectangle color (green)
    thickness = 2  # Rectangle line thickness
    # centre =
    q1 = [w/2,0,w,h/2]
    q2 = [0,0,w/2,h/2]
    q3 = [0,h/2,w/2,h]
    q4 = [w/2,h/2,w,h]
    q5 = [w/2-l,h/2-l,w/2+l,h/2+l]
    self.dist2centre = math.sqrt((target_x - w/2)**2 + (target_y - h/2)**2)
    sector = [q1,q2,q3,q4,q5]
    for q in sector:
        for i in range(4):
            q[i] = int(q[i])
        cv2.rectangle(frame, (q[0],q[1]),(q[2],q[3]), color)
        # print(q[0],q[1],q[2],q[3])
    if(q5[0] <= target_x and target_x < q5[2] and q5[1] < target_y and target_y < q5[3]):
        print("descend")
        self.action = 0
    elif(q1[0] < target_x and target_x < q1[2] and q1[1] < target_y and target_y < q1[3]):
        print("move forward and right") 
        self.action = 1   
    elif(q2[0] < target_x and target_x < q2[2] and q2[1] < target_y and target_y < q2[3]):
        print("move forward and left")
        self.action = 2    
    elif(q3[0] < target_x and target_x < q3[2] and q3[1] < target_y and target_y < q3[3]):
        print("move backward and left")
        self.action = 3   
    elif(q4[0] < target_x and target_x < q4[2] and q4[1] < target_y and target_y < q4[3]):
        print("move backward and right")
        self.action = 4
    print(self.angle)
    print(self.target_angle)
    print('Auto is ',self.target_is_set)
    print(target_x)
    print(target_y)
    print('Arrived is ',self.arrived)
    print('action is ',self.action)
    print('distance to centre is ',self.dist2centre)
    # print(thrust)

    # Display image
    cv2.imshow("camera", frame)


    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
