#!/usr/bin/python

import rospy
import matplotlib.pyplot as plt
import Tkinter
import time
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import Image as Im
import ImageTk
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

class GUI:
    def __init__(self, parent=None):

        self.master = parent
        
        
        self.master.title("Cornelius GUI")

        self.label = Tkinter.Label(self.master, text="Main commands")
        self.label.grid(row = 0, columnspan=5, sticky = 'EW')
        
        LegsWindow = Tkinter.Frame(self.master, borderwidth=2)
        LegsWindow.grid(row=1,  column = 0)
        self.LegsLabels = [Tkinter.Label(LegsWindow, text="{0}".format(n_leg), bg="red", fg="white") for n_leg in range(6)]
        for n_legLabel in range(len(self.LegsLabels)):
            self.LegsLabels[n_legLabel].grid(row = int(0+n_legLabel/3), column = n_legLabel%3)

        self.greet_button = Tkinter.Button(self.master, text="Plot", command=self.UpdatePlot)
        self.greet_button.grid(row=1, column=3, rowspan = 2)
        
        self.close_button = Tkinter.Button(self.master, text="Picture", command=self.UpdatePicture)
        self.close_button.grid(row=1, column=4, rowspan = 2)

        self.master.protocol("WM_DELETE_WINDOW", self.master.quit)

        self.PlotPlot = Figure(figsize=(5, 4), dpi=100)
        self.SubPlotPlot = self.PlotPlot.add_subplot(111)
        self.PlotCanvas = FigureCanvasTkAgg(self.PlotPlot, master=self.master)
        self.PlotCanvas.get_tk_widget().grid(row = 3, column = 0, columnspan = 4)
        
        f = Figure(figsize=(5, 4), dpi=100)
        self.SubPlotPicture = f.add_subplot(111)
        self.img = np.zeros([480,640,3])
        self.PictureCanvas = FigureCanvasTkAgg(f, master=self.master)
        self.PictureCanvas.show()
        self.PictureCanvas.get_tk_widget().grid(row = 3, column = 4, columnspan = 1)

        DirectionWindow = Tkinter.Frame(self.master, borderwidth=2)
        DirectionWindow.grid(row = 4, column = 0, columnspan = 4)
        left = Tkinter.PhotoImage(file='Icons/up_left.png')
        self.ButtonLeft = Tkinter.Button(DirectionWindow, width=50, height=50, image=left, bg='gray')
        self.ButtonLeft.image = left
        self.ButtonLeft.bind("<Button-1>", lambda event, d=1: self.SetDirection(d))
        self.ButtonLeft.bind("<ButtonRelease-1>", lambda event, d=0: self.SetDirection(d))
        self.ButtonLeft.pack(side = Tkinter.LEFT)
        front = Tkinter.PhotoImage(file='Icons/up.png')
        self.ButtonFront = Tkinter.Button(DirectionWindow, width=50, height=50, image=front, bg='gray')
        self.ButtonFront.image = front
        self.ButtonFront.bind("<Button-1>", lambda event, d=2: self.SetDirection(d))
        self.ButtonFront.bind("<ButtonRelease-1>", lambda event, d=0: self.SetDirection(d))
        self.ButtonFront.pack(side = Tkinter.LEFT)
        right = Tkinter.PhotoImage(file='Icons/up_right.png')
        self.ButtonRight = Tkinter.Button(DirectionWindow, width=50, height=50, image=right, bg='gray')
        self.ButtonRight.image = right
        self.ButtonRight.bind("<Button-1>", lambda event, d=3: self.SetDirection(d))
        self.ButtonRight.bind("<ButtonRelease-1>", lambda event, d=0: self.SetDirection(d))
        self.ButtonRight.pack(side = Tkinter.LEFT)

        print "Starting ROSThread class"

        self.RosWorker = ROSWorker(self)
        #self.RosProcess = Process(target = self.RosWorker.run(),  args=())


        self.N = 1
        self.UpdatePlot()
        self.UpdatePicture()
        

    def CameraDisplay(self):

        im = Im.fromarray(img)
        imgtk = ImageTk.PhotoImage(image=im) 

    def UpdatePlot(self):

        t = arange(0.0, 3.0, 0.01)
        s = sin(self.N*2*pi*t)

        self.SubPlotPlot.clear()
        self.SubPlotPlot.plot(t, s)
        self.PlotCanvas.show()
        self.N +=0.5
        
    def UpdatePicture(self):
        print "Updating Picture"
        self.SubPlotPicture.imshow(self.img)
        self.PictureCanvas.show()
        self.master.after(100, self.UpdatePicture)

    def SetDirection(self, directionNumber):
        if directionNumber == 0:
            self.RosWorker.DirPub.publish(np.array([0., 0., 1.], dtype = np.float32))
        elif directionNumber == 1:
            self.RosWorker.DirPub.publish(np.array([10., 10., 1.], dtype = np.float32))
        elif directionNumber == 2:
            self.RosWorker.DirPub.publish(np.array([10., 0., 1.], dtype = np.float32))
        elif directionNumber == 3:
            self.RosWorker.DirPub.publish(np.array([10., -10., 1.], dtype = np.float32))

class ROSWorker():
    
    def __init__(self, parent = None):
        

        self.WindowManager = parent

        rospy.init_node('GUI', anonymous=True)
        self.image_sub_right = rospy.Subscriber("/image_topic_2", Image, self.monitoringCallback)
        self.DirPub = rospy.Publisher("direction", numpy_msg(Floats),queue_size=1)
        self.bridge = CvBridge()

        self.lastPictureUpdate = time.time()
        
        #rospy.spin()
        
        print "Done with ROSWorker init"
    def monitoringCallback(self, data):  
        print "Callback"
        if time.time() - self.lastPictureUpdate > 0.1:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)        
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            print (np.array(cv_image)<256).all()        
            print (np.array(cv_image)>=0).all()
            
            self.WindowManager.img = cv_image
            self.lastPictureUpdate = time.time()
        
root = Tkinter.Toplevel()
Gui_Instance = GUI(root)
root.mainloop()
Gui_Instance.RosWorker.image_sub_right.unregister()
root.destroy()
