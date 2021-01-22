#!/usr/bin/env python
import rospy
import sys
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from std_msgs.msg import Int32


def image_callback(rawImage):

    state=0
    

    
    camera_image = CvBridge().imgmsg_to_cv2(rawImage, "bgr8")
   
    crop_img = camera_image[0:300, 0:800]
    


    min_blue = np.array([90,80,50])
    max_blue = np.array([110,255,255])

    print(crop_img.shape)

    frame_area=int(crop_img.shape[0]*crop_img.shape[1])

    crop_hsv=cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    kernel_matrix = np.ones((3,3),np.uint8)
        
    mask = cv2.inRange(crop_hsv, min_blue, max_blue) 
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_matrix)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_matrix)
    
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        

    largestArea = 0
    largestRect = np.zeros((4,2), dtype=int)                                        
        
    if len(contours) > 0:
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
           
            side1 = np.linalg.norm(box[0]-box[1])
            side2 = np.linalg.norm(box[0]-box[3])
            
            area = side1*side2
            
            if area > largestArea:
                largestArea = area
                largestRect = box
        y_values=[largestRect[0][1], largestRect[1][1], largestRect[2][1], largestRect[3][1]]
        x_values=[largestRect[0][0], largestRect[1][0], largestRect[2][0], largestRect[3][0]]

        min_yvalue=min(y_values)
        max_yvalue=max(y_values)
        min_xvalue=min(x_values)
        max_xvalue=max(x_values)        

    if largestArea > frame_area*0.04 and min_xvalue > 10 and min_yvalue > 10 and max_xvalue < 790 and max_yvalue < 290:
        print(largestRect)
        print(largestRect.shape)   
        cv2.drawContours(crop_img,[largestRect],0,(0,0,255),2)

        pts1=np.float32(largestRect)
        pts2=np.float32([[150,150], [0,150], [0,0], [150,0]])

        matrix=cv2.getPerspectiveTransform(pts1, pts2)
        result=cv2.warpPerspective(crop_img, matrix, (150,150))

        black_min=np.array([0,0,0])
        black_max=np.array([10,10,10])

        mask2 = cv2.inRange(result, black_min, black_max)

        black_mask=np.full((150, 150), 255, dtype=int)

        black_mask = cv2.bitwise_not(mask2)

        blur = cv2.GaussianBlur(black_mask,(5,5),0)
        gray_img=blur

        corners= cv2.goodFeaturesToTrack(gray_img,15,0.05,40)
        corners = np.array(corners)

        if(len(corners) != 0):
            if(len(corners) == 3):  
                print("ucgen")
                state=2
            elif(len(corners) == 6):  
                print("altigen")
                state=3
            elif(len(corners)==5):   
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


    print("hello")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
