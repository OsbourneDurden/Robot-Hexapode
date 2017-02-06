from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError
import compute_cloud

class CameraManager:
    def __init__(self):
        self.camera = PiCamera()
        self.rawCapture = PiRGBArray(self.camera)
        self.bridge = CvBridge()
        self.MasterImage = None
        self.SlaveImage = None

        rospy.Subscriber("image_command", Int8, self.ImageCommandCallback)
        rospy.Subscriber("master_camera", Image, self.MasterImageCallback)

        self.CameraCommandPublisher = rospy.Publisher("image_command", Int8, queue_size=1)

        rospy.init_node('analyser', anonymous=True)
        
        self.CameraCommandPublisher.publish(0)

    def ImageCommandCallback(self, message):
        if message.data == 2:
            self.camera.capture(self.rawCapture, format="bgr")
            self.SlaveImage = self.rawCapture.array
            self.rawCapture.truncate(0)
            self.CameraCommandPublisher.publish(3)

    def MasterImageCallback(self, message):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(message, "bgr8")
            
            self.MasterImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            print (e)

C = CameraManager()
A = compute_cloud.Analyser(C, calibrated=True)

rospy.spin()
