#!/usr/bin/env python
import rospy

def callback(rawImage):
    print("callback executed")
    



if __name__ == '__main__':
        # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('state_publisher', anonymous=True)

    #SUBSCRIBE TO CAMERA RAW
    #rospy.Subscriber("chatter", String, callback)

    print("hello")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
