#!/usr/bin/env python
import rospy
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from std_msgs.msg import Int32


def image_callback(rawImage):
    print("callback executed")
    camera_image = CvBridge().imgmsg_to_cv2(rawImage, "bgr8")
    #cv2.Rect myROI(0, 0, 800, 300);
    crop_img = camera_image[0:300, 0:800]
    cv2.imshow("ahmet",crop_img)
    cv2.waitKey(0)

    lower_blue = np.array([90,80,50])
    upper_blue = np.array([110,255,255])

    print(crop_img.shape)

    frame_area=int(crop_img.shape[0]*crop_img.shape[1])

    hsv=cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3,3),np.uint8)
        # extract binary image with active blue regions
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
        # morphological operations
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    detectedTrafficSign = None
        
        # define variables to hold values during loop
    largestArea = 0
    largestRect = None
        
        # only proceed if at least one contour was found
    if len(cnts) > 0:
        for cnt in cnts:
            #print(cnt)
            # Rotated Rectangle. Here, bounding rectangle is drawn with minimum area,
            # so it considers the rotation also. The function used is cv2.minAreaRect().
            # It returns a Box2D structure which contains following detals -
            # ( center (x,y), (width, height), angle of rotation ).
            # But to draw this rectangle, we need 4 corners of the rectangle.
            # It is obtained by the function cv2.boxPoints()
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            # count euclidian distance for each side of the rectangle
            sideOne = np.linalg.norm(box[0]-box[1])
            sideTwo = np.linalg.norm(box[0]-box[3])
            # count area of the rectangle
            area = sideOne*sideTwo
            # find the largest rectangle within all contours
            if area > largestArea:
                largestArea = area
                largestRect = box
        
    if largestArea > frame_area*0.02:
        print(largestRect)
        # draw contour of the found rectangle on  the original image   
        cv2.drawContours(crop_img,[largestRect],0,(0,0,255),2)
        
        # cut and warp interesting area
        #warped = four_point_transform(mask, [largestRect][0])
    
    cv2.imshow("dayi", crop_img)
    cv2.waitKey(0)

    pts1=np.float32(largestRect)
    pts2=np.float32([[150,150], [0,150], [0,0], [150,0]])

    matrix=cv2.getPerspectiveTransform(pts1, pts2)
    result=cv2.warpPerspective(crop_img, matrix, (150,150))
    cv2.imshow("warp", result)
    cv2.waitKey(0)

    black_mask=np.full((150, 150), 255, dtype=int)

    black_min=np.array([0,0,0])
    black_max=np.array([10,10,10])

    mask2 = cv2.inRange(result, black_min, black_max)
    cv2.imshow("black", mask2)
    cv2.waitKey(0)

    black_mask = cv2.bitwise_not(mask2)
   # white_black=black_mask-mask2
    cv2.imshow("white_black", black_mask)
    cv2.waitKey(0)

    #img=cv2.cvtColor(black_mask, cv2.COLOR_HSV2BGR)

    #gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_img=cv2.blur(black_mask,(5,5))
    #canny = cv2.Canny(gray_img, 10, 20)   #100 200
    #ret, thresh = cv2.threshold(canny, 10, 20, 0)
    
    state=0

    #img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    corners= cv2.goodFeaturesToTrack(gray_img,15,0.01,40)
    corners = np.array(corners)
    if(len(corners) != 0):
        #cnt = contours[0]
        #perimeter = cv2.arcLength(cnt,True)
        #epsilon = 0.01*cv2.arcLength(cnt,True)
        #corners = cv2.approxPolyDP(cnt,epsilon,True)
        print(corners)
        if(len(corners) == 3):  #ucgen a
            print("ucgen")
            state=2
        elif(len(corners) == 6):  #altigen f
            print("altigen")
            state=3
        else:   # yildiz d
            print("yildiz")
            state=1

    state_publisher.publish(state)

if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('state_publisher', anonymous=True)

    state_publisher = rospy.Publisher('state', Int32, queue_size = 10)
    
    rospy.Subscriber("camera/rgb/image_raw", Image, image_callback, queue_size = 1)

    #SUBSCRIBE TO CAMERA RAW
    #rospy.Subscriber("chatter", String, callback)

    print("hello")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
