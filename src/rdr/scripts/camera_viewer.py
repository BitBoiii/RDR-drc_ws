import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

cvb = CvBridge()

def listener_callback_blue(msg):
    frame = cvb.compressed_imgmsg_to_cv2(msg)
    #frame = cv2.resize(frame, (frame.shape[1] * 4, frame.shape[0] * 4), cv2.INTER_NEAREST)
    cv2.imshow("recieve_blue", frame)
    cv2.waitKey(1)
    rospy.loginfo("recieved blue frame")

def listener_callback_yellow(msg):
    frame = cvb.compressed_imgmsg_to_cv2(msg)
    #frame = cv2.resize(frame, (frame.shape[1] * 4, frame.shape[0] * 4), cv2.INTER_NEAREST)
    cv2.imshow("recieve_yellow", frame)
    cv2.waitKey(1)
    rospy.loginfo("recieved yellow frame")

def listener_callback_unfiltered(msg):
    frame = cvb.imgmsg_to_cv2(msg)
    cv2.imshow("recieve_unfiltered", frame)
    cv2.waitKey(1)
    rospy.loginfo("recieved unfiltered frame")

def main():
    rospy.init_node('camera_viewer')

    rospy.Subscriber("blue_feed", CompressedImage,listener_callback_blue)
    #rospy.Subscriber("yellow_feed", CompressedImage,listener_callback_yellow)
    #rospy.Subscriber("unfiltered_feed", Image,listener_callback_unfiltered)

    rospy.spin()

if __name__ == '__main__':
    main()