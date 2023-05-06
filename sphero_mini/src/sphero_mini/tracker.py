#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String, Int64, Float64
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()
last_sent_heading = None

fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('Tracking.avi', fourcc, 30.0, (320, 240))

def euclid(center1, center2):
  dist = np.sqrt((center1[0]-center2[0])**2 + (center1[1]-center2[1])**2)
  dir = np.arctan2(center2[0]-center1[0], center2[1]-center1[1])

  return dist, dir

def image_callback(ros_image):
  print('got an image')
  global bridge
  global last_sent_heading
  global out

  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)

  #from now on, you can work exactly like with opencv
  rob_message = Point()
  targ_message = Point()
  yaw_message = Int64()
  dist_message = Int64()
  move_flag_message = Int64()
  velocity_message = Int64()

  loop_rate = rospy.Rate(10) # 10 Hz

  robot_pub = rospy.Publisher("sphero/robot_pose", Point, queue_size=1)
  target_pub = rospy.Publisher("sphero/target_pose", Point, queue_size=1)
  yaw_pub = rospy.Publisher("sphero/desired_yaw", Int64, queue_size=1)
  dist_pub = rospy.Publisher("sphero/desired_distance", Int64, queue_size=1)
  move_flag_pub = rospy.Publisher("sphero/move_flag", Int64, queue_size=1)
  velocity_pub = rospy.Publisher("/sphero/desired_velocity", Int64, queue_size=1)

  # resizing for faster processing
  img = cv2.resize(cv_image, (320,240), interpolation = cv2.INTER_LINEAR)
  # (1920.0 1080.0) or (1280.0 720.0)
  # creating window for mouse inputTypeError: Invalid number of arguments, args should be ['x', 'y', 'theta'] args are(['307', '160', '0'],)
  
  # result = cv2.VideoWriter('Tracking.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (320, 240))
  
  cv2.namedWindow("Robot Tracking")

  # converting to HSV
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # HSV range for color and defining mask
    # change these for respective colors
    # link for HSV stuff
    # https://www.google.com/imgres?imgurl=https%3A%2F%2Fi.stack.imgur.com%2FTSKh8.png&imgrefurl=https%3A%2F%2Fstackoverflow.com%2Fquestions%2F47483951%2Fhow-to-define-a-threshold-value-to-detect-only-green-colour-objects-in-an-image&tbnid=Jx2H1bjYvu6n_M&vet=12ahUKEwiA6p3xp-z3AhXUXM0KHcAtDh4QMygBegUIARDDAQ..i&docid=d4AswGhN6lbYWM&w=720&h=355&q=hsv%20range&ved=2ahUKEwiA6p3xp-z3AhXUXM0KHcAtDh4QMygBegUIARDDAQ
    # https://www.google.com/imgres?imgurl=https%3A%2F%2Fanswers.opencv.org%2Fupfiles%2F15181560142151344.png&imgrefurl=https%3A%2F%2Fanswers.opencv.org%2Fquestion%2F184281%2Fhow-are-hsv-values-interpreted-in-python%2F&tbnid=mpa5ObAswr1QPM&vet=12ahUKEwi2qKKx8oL4AhVaookEHZVJDLQQxiAoAXoECAAQGQ..i&docid=06ORPhgZpk_9yM&w=743&h=477&itg=1&q=hsv%20range&ved=2ahUKEwi2qKKx8oL4AhVaookEHZVJDLQQxiAoAXoECAAQGQ
  # green_lower = np.array([25, 52, 72])             # change upper and lower values for colors
  # green_upper = np.array([83, 255, 255])
  # green_mask = cv2.inRange(hsv, green_lower, green_upper)

  red_lower1 = np.array([0, 50, 50])      # 94       # change upper and lower values for colors
  red_upper1 = np.array([10, 255, 255])   # 120
  red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)

  red_lower2 = np.array([170, 50, 50])      # 94       # change upper and lower values for colors
  red_upper2 = np.array([180, 255, 255])   # 120
  red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)

  red_mask = cv2.bitwise_or(red_mask1, red_mask2)

  blue_lower = np.array([94, 80, 2])      # 94       # change upper and lower values for colors
  blue_upper = np.array([110, 255, 255])   # 120
  blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

    # tracking green color
  contours, hierarchy=cv2.findContours(red_mask,cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_TC89_L1)
  for pic, contour in enumerate(contours):
      area = cv2.contourArea(contour)
      maxArea = max(contours, key = cv2.contourArea)
        
        # only taking largest area
      if(area > 10):
            x, y, w, h = cv2.boundingRect(maxArea)
            # finding center of target
            xC = int(x + w/2)
            yC = int(y + h/2)
            red_center = [xC, yC]
            # print(center)
            
            # # plotting bounding box and center
            # img = cv2.rectangle(img, (x, y),
            #                         (x + w, y + h),
            #                         (0, 0, 255), 2)
            # cv2.circle(img, red_center, 10, (0, 0, 255), -1)
            # cv2.putText(img, "Target", (x, y),
            #             cv2.FONT_HERSHEY_SIMPLEX, 1.0,
            #             (0, 0, 255))

    # tracking blue color
  contours, hierarchy=cv2.findContours(blue_mask,cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_TC89_L1)
  for pic, contour in enumerate(contours):
      area = cv2.contourArea(contour)
      maxArea = max(contours, key = cv2.contourArea)
        
        # only taking largest area
      if(area > 10):
            x, y, w, h = cv2.boundingRect(maxArea)
            # finding center of target
            xC = int(x + w/2)
            yC = int(y + h/2)
            blue_center = [xC, yC]
            # print(center)
            
            # # plotting bounding box and center
            # img = cv2.rectangle(img, (x, y),
            #                         (x + w, y + h),
            #                         (255, 0, 0), 2)
            # cv2.circle(img, blue_center, 10, (255, 0, 0), -1)
            # cv2.putText(img, "Robot", (x, y),
            #             cv2.FONT_HERSHEY_SIMPLEX, 1.0,
            #             (255, 0, 0))

  out.write(img)

  l, theta = euclid(blue_center, red_center)

  if l <= 70:
    move_flag = 0
  elif l > 70:
    move_flag = 1

  rob_message.x = blue_center[0]
  rob_message.y = blue_center[1]
  rob_message.z = 0

  targ_message.x = red_center[0]
  targ_message.y = red_center[1]
  targ_message.z = 0

  yaw_message = (-theta * 180 / np.pi).astype(np.int)
  dist_message = l.astype(np.int)
  move_flag_message = move_flag

  kP = 0.85 
  desired_dist = 70
  velocity_message = kP * np.abs(dist_message - desired_dist)
  if velocity_message > 255: 
     velocity_message = 250
  velocity_message = velocity_message.astype(np.int)


  robot_pub.publish(rob_message)
  target_pub.publish(targ_message)
  dist_pub.publish(dist_message)
  move_flag_pub.publish(move_flag_message)
  velocity_pub.publish(velocity_message)

  # if yaw_message != last_sent_heading:
  yaw_pub.publish(yaw_message)
  # last_sent_heading = yaw_message

  # rospy.spin()

  cv2.imshow("Robot Tracking", img)
  cv2.waitKey(1)

def main(args):
  rospy.init_node('robot_tracking', anonymous=True)
  image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
  loop_rate = rospy.Rate(10) # 10 Hz

  # robot_pub = rospy.Publisher("robot_pose", Point, queue_size=10)
  # target_pub = rospy.Publisher("target_pose", Point, queue_size=10)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)