import cv2
import torch
import segmentation_models_pytorch as smp
import albumentations as albu


class SegNet():
    def __init__(self, model_path, res=(384, 288), encoder='timm-mobilenetv3_small_minimal_100', encoder_weights='imagenet'):
        self.model = torch.load(model_path)
        self.res = res
        preprocessing_fn = smp.encoders.get_preprocessing_fn(encoder, encoder_weights)
        self.preprocessing  = self.get_preprocessing(preprocessing_fn)
        pass

    def to_tensor(self, x, **kwargs):
        return x.transpose(2, 0, 1).astype('float32')

    def get_preprocessing(self, preprocessing_fn):
        """Construct preprocessing transform
        
        Args:
            preprocessing_fn (callbale): data normalization function 
                (can be specific for each pretrained neural network)
        Return:
            transform: albumentations.Compose
        
        """
        
        _transform = [
            albu.Lambda(image=preprocessing_fn),
            albu.Lambda(image=self.to_tensor, mask=self.to_tensor),
        ]
        return albu.Compose(_transform)

    def detect_lines(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, self.res)
        # apply preprocessing
        sample = self.preprocessing(image=image)
        image = sample['image']

        x_tensor = torch.from_numpy(image).to('cuda').unsqueeze(0)
        pr_mask = self.model.predict(x_tensor)
        pr_mask = (pr_mask.squeeze().cpu().numpy().round())

        blue_mask = pr_mask[0]
        yellow_mask = pr_mask[1]
        return blue_mask, yellow_mask

import math
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from std_srvs.srv import Empty

yellow_hsv_vals = [25, 54, 60, 32, 255, 255]
blue_hsv_vals = [100, 120, 85, 122, 255, 255]

cap = cv2.VideoCapture(0)

cvb = CvBridge()

line_detector = SegNet(
    model_path='/home/nikita/RDR-drc_ws/src/rdr/scripts/best_model.pth',
    res=(384,288)
)

def generateFlatCorners():
    cornersFlat = np.zeros((70, 1, 2))

    for x in range (10):
        for y in range(7):
            i = y + x * 7
            cornersFlat[i][0][0] = x * 20
            cornersFlat[i][0][1] = y * 20
    return cornersFlat

def getPointRotation(pt1, pt2):
    slope = (pt1[1] - pt2[1]) / (pt1[0] - pt2[0])
    return math.degrees(math.atan(slope))

def calibrateWarp():
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    while (1):
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        retCorners, cornersReal = cv2.findChessboardCorners(gray, (7, 10))
        cornersFlat = generateFlatCorners()

        if(retCorners):
            corners2 = cv2.cornerSubPix(gray,cornersReal, (11,11), (-1,-1), criteria)
            H, _ = cv2.findHomography(corners2, cornersFlat)

            corners = np.array([
                [0, 0],
                [0, frame.shape[0] - 1],
                [frame.shape[1] - 1, frame.shape[0] -1],
                [frame.shape[1] - 1, 0]
            ])

            cornersFinal = cv2.perspectiveTransform(np.float32([corners]), H)[0]

            bx, by, bwidth, bheight = cv2.boundingRect(cornersFinal)

            angle = getPointRotation(cornersFinal[1], cornersFinal[2])
            print(angle)
            rotationMtx = cv2.getRotationMatrix2D((bwidth/2, bheight/2), angle, 1)

            cornersFlat = cv2.transform(cornersFlat, rotationMtx)
            H, _ = cv2.findHomography(corners2, cornersFlat)

            corners = np.array([
                [0, 0],
                [0, frame.shape[0] - 1],
                [frame.shape[1] - 1, frame.shape[0] -1],
                [frame.shape[1] - 1, 0]
            ])

            cornersFinal = cv2.perspectiveTransform(np.float32([corners]), H)[0]

            bx, by, bwidth, bheight = cv2.boundingRect(cornersFinal)

            th = np.array([
                [ 1, 0, -bx ],
                [ 0, 1, -by ],
                [ 0, 0,   1 ]
            ])

            pth = th.dot(H)
            
            calib_file_path = rospy.get_param('~warp_calib_save')
            np.savez(calib_file_path, homography=pth, width=bwidth, height=bheight)

            return pth, bwidth, bheight

def hsv_line_detect(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(hsv_image, 
        (
            blue_hsv_vals[0],
            blue_hsv_vals[1],
            blue_hsv_vals[2]
        ),
        (
            blue_hsv_vals[3],
            blue_hsv_vals[4],
            blue_hsv_vals[5]
            
        ))
    yellow_mask = cv2.inRange(hsv_image, 
        (
            yellow_hsv_vals[0],
            yellow_hsv_vals[1],
            yellow_hsv_vals[2]
        ),
        (
            yellow_hsv_vals[3],
            yellow_hsv_vals[4],
            yellow_hsv_vals[5]
            
        ))
    return blue_mask, yellow_mask
    
def nn_line_detect(image):
    blue_mask, yellow_mask = line_detector.detect_lines(image)
    return blue_mask, yellow_mask

def refresh_params_callback(request):
    rospy.loginfo('Refreshing the parameters')
    line_filter_mode = rospy.get_param('line_filter_mode', line_filter_mode)
    transmit_unfiltered = rospy.get_param('transmit_unfiltered', transmit_unfiltered)
    yellow_hsv_vals = rospy.get_param('yellow_hsv_vals', yellow_hsv_vals)
    blue_hsv_vals = rospy.get_param('blue_hsv_vals', blue_hsv_vals)

def calibrate_warp_callback(self, request, response):
    rospy.loginfo('Request to calibrate recieved')
    try:
        homography, bwidth, bheight = calibrateWarp()
    except Exception as e:
        rospy.loginfo('Calibration Failed')
        rospy.loginfo(str(e))
    else:
        rospy.loginfo('Calibration Succeeded')

def main():
    rospy.init_node('camera_processor')

    pub_blue_img = rospy.Publisher('blue_feed', CompressedImage, queue_size=10)
    pub_yellow_img = rospy.Publisher('yellow_feed', CompressedImage, queue_size=10)
    pub_img_unfiltered = rospy.Publisher('unfiltered_feed', Image, queue_size=10)

    refresh_params_srv = rospy.Service('refresh_params', Empty, refresh_params_callback)
    calibrate_warp_srv = rospy.Service('calibrate_warp', Empty, calibrate_warp_callback)

    line_filter_mode = rospy.get_param('~line_filter_mode', 'hsv')
    transmit_unfiltered = rospy.get_param('~transmit_unfiltered', True)

    try:
        calib_file_path =  rospy.get_param('~warp_calib_file')
        data = np.load(calib_file_path)
        homography = data['homography']
        bwidth = int(data['width'])
        bheight = int(data['height'])
    except Exception as e:
        rospy.logwarn("failed to read warp calibration file, no warp will be applied")

    while not rospy.is_shutdown():
        try:
            ret, frame = cap.read()
            image = frame
            
            if (ret):
                if (line_filter_mode == 'hsv'):
                    if ('homography' in locals() or 'homography' in globals()):
                        image = cv2.warpPerspective(image, homography, (bwidth, bheight))

                    blue_mask, yellow_mask = hsv_line_detect(image)

                elif (line_filter_mode == 'nn'):
                    blue_mask, yellow_mask = nn_line_detect(image)

                    if ('homography' in locals() or 'homography' in globals()):
                        blue_mask = cv2.warpPerspective(blue_mask, homography, (bwidth, bheight))
                        yellow_mask = cv2.warpPerspective(yellow_mask, homography, (bwidth, bheight))
                        
                else:
                    rospy.logerr("'" + line_filter_mode + "'" + "is not a valid mode, please select 'hsv' or 'nn'")

                pub_blue_img.publish(cvb.cv2_to_compressed_imgmsg(blue_mask))
                pub_yellow_img.publish(cvb.cv2_to_compressed_imgmsg(yellow_mask))

                if (transmit_unfiltered):
                    pub_img_unfiltered.publish(cvb.cv2_to_imgmsg(frame))

                cv2.waitKey(1)
        except Exception as e:
            rospy.logwarn(str(e)) 

if __name__ == '__main__':
    main()
