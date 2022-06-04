import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

cvb = CvBridge()

blueFrame = np.zeros((100,100,3), dtype=np.uint8)
yellowFrame = np.zeros((100,100,3), dtype=np.uint8)
frame = np.zeros((100,100,3), dtype=np.uint8)

def listener_callback_blue(msg):
    global blueFrame
    blueFrame = cvb.compressed_imgmsg_to_cv2(msg)
    #frame = cv2.resize(frame, (frame.shape[1] * 4, frame.shape[0] * 4), cv2.INTER_NEAREST)
    rospy.loginfo("recieved blue frame")

def listener_callback_yellow(msg):
    global yellowFrame
    yellowFrame = cvb.compressed_imgmsg_to_cv2(msg)
    #frame = cv2.resize(frame, (frame.shape[1] * 4, frame.shape[0] * 4), cv2.INTER_NEAREST)
    rospy.loginfo("recieved yellow frame")

def listener_callback_unfiltered(msg):
    global frame
    frame = cvb.imgmsg_to_cv2(msg)
    rospy.loginfo("recieved unfiltered frame")
    

def main():
    rospy.init_node('camera_viewer')

    rospy.Subscriber("blue_feed", CompressedImage,listener_callback_blue)
    rospy.Subscriber("yellow_feed", CompressedImage,listener_callback_yellow)
    rospy.Subscriber("unfiltered_feed", Image,listener_callback_unfiltered)

    while not rospy.is_shutdown():
        cv2.imshow("recieve_unfiltered", frame)
        cv2.imshow("recieve_yellow", yellowFrame)
        cv2.imshow("recieve_blue", blueFrame)
        cv2.waitKey(1)


if __name__ == '__main__':
    main()