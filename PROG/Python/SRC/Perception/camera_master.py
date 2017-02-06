from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError

class CameraManager:
    def __init__(self):
        self.camera = PiCamera()
        self.rawCapture = PiRGBArray(self.camera)
        self.bridge = CvBridge()
        self.MasterImage = None
        
        rospy.Subscriber("image_command", Int8, self.ImageCommandCallback)
        self.ImagePublisher = rospy.Publisher("master_camera", Image, queue_size = 1)
        
        self.CameraCommandPublisher = rospy.Publisher("image_command", Int8, queue_size=1)
        
        rospy.init_node('camera_manager_master', anonymous=True)
        
    def ImageCommandCallback(self, message):
        if message.data == 1:
            self.CameraCommandPublisher.publish(2)
            self.CaptureAndPublish()
        elif message.data == 4:
            self.CameraCommandPublisher.publish(0)
            self.CaptureAndPublish()
    
    def CaptureAndPublish(self):
        self.camera.capture(self.rawCapture, format="bgr")
        image = self.rawCapture.array
        print image
        self.rawCapture.truncate(0)
        
        try:
            self.ImagePublisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)

C = CameraManager()
rospy.spin()
