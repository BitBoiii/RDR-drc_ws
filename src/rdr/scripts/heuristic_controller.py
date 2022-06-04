import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist 
import sensor_msgs.msg
import cv2
import numpy as np
from simple_pid import PID

cvb = CvBridge()

pid = PID(1, 0.1, 0.05, setpoint=0)
pid.proportional_on_measurement = True

def main():
    global yellow_frame, base_throttle, blue_frame, midpoint_segments, use_polyfit, blue_is_left, track_width, pub_cmd_vel

    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('heuristic_controller', anonymous=True)
    rospy.Subscriber('yellow_feed', sensor_msgs.msg.CompressedImage, listener_callback_yellow)
    rospy.Subscriber('blue_feed', sensor_msgs.msg.CompressedImage, listener_callback_blue)

    yellow_frame = np.zeros((640,480))
    blue_frame = np.zeros((640,480))
    rospy.set_param('midpoint_segments',16)
    midpoint_segments = rospy.get_param('midpoint_segments')
    rospy.set_param('blue_is_left', True)
    blue_is_left = rospy.get_param('blue_is_left')
    rospy.set_param('use_polyfit', False)
    use_polyfit = rospy.get_param('use_polyfit')
    rospy.set_param('track_width', 400)
    track_width = rospy.get_param('track_width')
    rospy.set_param('base_throttle', 0.2)
    base_throttle =rospy.get_param('base_throttle')


    while not rospy.is_shutdown():
        cv2.imshow("recieve_yellow", yellow_frame)
        cv2.imshow("recieve_blue", blue_frame)
        cv2.imshow("frame", f)
        cv2.waitKey(1)

def listener_callback_blue(msg):
        global blue_frame
        blue_frame = cvb.compressed_imgmsg_to_cv2(msg)

def listener_callback_yellow(msg):
        global yellow_frame
        global base_throttle
        global pub_cmd_vel
        yellow_frame = cvb.compressed_imgmsg_to_cv2(msg)
        twist = Twist()
        twist.linear.x = base_throttle
        pidError = calculate_steering()
        twist.angular.z = pid(pidError)
        pub_cmd_vel.publish(twist)

def calculate_steering():
        global blue_frame
        global midpoint_segments
        global yellow_frame
        global blue_is_left
        global track_width
        global use_polyfit
        yellowDot = (125, 255, 255)
        blueDot = (255, 125, 125)
        centerDot = (255, 255, 255)
        centerLine = (0, 0, 255)

        imsizeX = blue_frame.shape[1]
        imsizeY = blue_frame.shape[0]

        midX = imsizeX//2

        predictionHeight = imsizeY//midpoint_segments

        midPoints = np.zeros(midpoint_segments)
        verticalPoints = np.linspace(imsizeY-predictionHeight//2, predictionHeight//2, num=midpoint_segments, dtype=np.int32)
        global f
        f = np.zeros((imsizeY, imsizeX, 3))

        for i in range(midpoint_segments):
            blue_cropped = blue_frame[imsizeY-predictionHeight*(i+1):imsizeY-predictionHeight*i].astype(np.uint8)
            yellow_cropped = yellow_frame[imsizeY-predictionHeight*(i+1):imsizeY-predictionHeight*i].astype(np.uint8)

            yellowCont, yellowHier = cv2.findContours(yellow_cropped, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            if len(yellowCont) > 0:
                yellowRet = True
                cont = max(yellowCont, key = cv2.contourArea)
                M = cv2.moments(cont)
                try:
                    yellowX = int(M['m10']/M['m00'])
                except:
                    yellowRet = False
            else:
                yellowRet = False

            blueCont, blueHier = cv2.findContours(blue_cropped, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            if len(blueCont) > 0:
                blueRet = True
                cont = max(blueCont, key = cv2.contourArea)
                M = cv2.moments(cont)
                try:
                    blueX = int(M['m10']/M['m00'])
                except:
                    blueRet = False
            else:
                blueRet = False

            cv2.line(f, (imsizeX//2, 0), (imsizeX//2, imsizeY), centerLine, 3)
            if (blue_is_left):
                addVal = 1
            else:
                addVal = -1

            circleY = verticalPoints[i]

            if (yellowRet and blueRet):
                midX = (yellowX+blueX)//2
                cv2.circle(f, (yellowX, circleY), 3, yellowDot, -1)
                cv2.circle(f, (blueX, circleY), 3, blueDot, -1)
                cv2.circle(f, (midX, circleY), 3, centerDot, -1)
            elif (yellowRet):
                midX = yellowX - (track_width//2) * addVal
                cv2.circle(f, (yellowX, circleY), 3, yellowDot, -1)            
                cv2.circle(f, (midX, circleY), 3, centerDot, -1)
            elif (blueRet):
                midX = blueX + (track_width//2) * addVal
                cv2.circle(f, (blueX, circleY), 3, blueDot, -1)
                cv2.circle(f, (midX, circleY), 3, centerDot, -1)

            midPoints[i] = midX 

        if (use_polyfit):
            polyfit = np.polyfit(midPoints, verticalPoints, 2)
            a = polyfit[0]
            b = polyfit[1]
            c = polyfit[2]
            for x in range(imsizeX):
                y = int(a * x**2  +  b * x  +  c)
                cv2.circle(f, (x, y), 3, (0, 255, 0), -1)
            
        else:
            cv2.circle(f, (int(midPoints[3]), verticalPoints[3]), 10, (0, 255, 0), -1)
            return midPoints[3]


if __name__ == '__main__':
    main()