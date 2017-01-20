#!/usr/bin/python

import rospy
import matplotlib.pyplot as plt
import Tkinter
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import Image as Im
import ImageTk
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

class GUI:
    def __init__(self, parent=None):

        self.master = parent
        
        self.h = Tkinter.StringVar()
        self.h.set("14.1")
        self.speed = Tkinter.StringVar()
        self.speed.set("0.")
        self.command = "STOP"
        
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
        DirectionWindow.grid(row = 4, column = 0, columnspan = 3)
        self.master.bind("<KeyPress>", self.keyEventCallback)
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
        self.master.bind("<KeyRelease>", self.keyReleaseCallback)

        self.RosWorker = ROSWorker(self)

        ParametersWindow = Tkinter.Frame(self.master, borderwidth=2)
        ParametersWindow.grid(row = 4, column = 3)
        HLabel = Tkinter.Label(ParametersWindow, text="H = ")
        HLabel.grid(row=0, column = 0)
        self.HEntry = Tkinter.Entry(ParametersWindow, textvariable=self.h)
        self.HEntry.grid(row=0, column=1)
        self.HButton = Tkinter.Button(ParametersWindow, text="Set Height", command=self.RosWorker.SetH)
        self.HButton.grid(row=0, column=2)
        SpeedLabel = Tkinter.Label(ParametersWindow, text="Speed = ")
        SpeedLabel.grid(row=1, column = 0)
        self.SpeedEntry = Tkinter.Entry(ParametersWindow, textvariable=self.speed)
        self.SpeedEntry.grid(row=1, column=1)
        self.SpeedButton = Tkinter.Button(ParametersWindow, text="Set Speed", command=self.RosWorker.SetSpeed)
        self.SpeedButton.grid(row=1, column=2)

        CommandWindow = Tkinter.Frame(self.master, borderwidth=2)
        CommandWindow.grid(row = 4, column = 4)
        self.CommandLabel = Tkinter.Label(CommandWindow, text="Current command : " + self.command)
        self.CommandLabel.grid(row=0, column = 0, columnspan = 3)
        self.StopButton = Tkinter.Button(CommandWindow, text = "STOP", command = lambda event, d=0: self.SetCommand(d))
        self.StopButton.grid(row=1, column = 0)
        self.MoveButton = Tkinter.Button(CommandWindow, text = "Move", command = lambda event, d=1: self.SetCommand(d))
        self.MoveButton.grid(row=1, column = 1)
        self.SetHeightButton = Tkinter.Button(CommandWindow, text = "Set Height", command = lambda event, d=2: self.SetCommand(d))
        self.SetHeightButton.grid(row=2, column = 1)

        self.UpdateCommand()

        print "Starting ROSThread class"

        #self.RosProcess = Process(target = self.RosWorker.run(),  args=())


        self.N = 1
        self.UpdatePlot()
        self.UpdatePicture()
    
    def SetCommand(self, commandValue):
        None

    def UpdatePlot(self):

        t = arange(0.0, 3.0, 0.01)
        s = sin(self.N*2*pi*t)

        self.SubPlotPlot.clear()
        self.SubPlotPlot.plot(t, s)
        self.PlotCanvas.show()
        self.N +=0.5
        
    def UpdatePicture(self):
        print "Updating Picture"
        self.SubPlotPicture.clear()
        self.SubPlotPicture.imshow(self.img)
        self.PictureCanvas.show()
        self.master.after(50, self.UpdatePicture)

    def UpdateCommand(self):
        self.CommandLabel['text'] = "Current command : " + self.command
        self.master.after(50,  self.UpdateCommand)

    def keyReleaseCallback(self, event):
        self.KeyPressed = False
        self.SetDirection(0)
    def keyEventCallback(self, event):
        print event.type
        if event.keysym == 'Left':
            self.SetDirection(1)
        elif event.keysym == 'Right':
            self.SetDirection(3)
        elif event.keysym == 'Up':
            self.SetDirection(2)

    def SetDirection(self, directionNumber):
        print "Setting direction"
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
        self.image_sub_right = rospy.Subscriber("/stereo/right/image_raw", Image, self.PictureCallback)
        self.commandSubscriber = rospy.Subscriber('command', String, self.CommandCallback)
        self.DirPub = rospy.Publisher("direction", numpy_msg(Floats),queue_size=1)
        self.HeightPub = rospy.Publisher("height", Float64 ,queue_size=1)
        self.SpeedPub = rospy.Publisher("speed", Float64 ,queue_size=1)
        self.bridge = CvBridge()

        self.lastPictureUpdate = time.time()
        
        #rospy.spin()
        
        print "Done with ROSWorker init"
    def PictureCallback(self, data):  
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
    
    def CommandCallback(self,  data):
        self.WindowManager.command = data.data

    def SetH(self):
        self.HeightPub.publish(float(self.WindowManager.h.get()))
    def SetSpeed(self):
        if 0 < float(self.WindowManager.speed.get()) <=1:
            self.SpeedPub.publish(float(self.WindowManager.speed.get()))
        else:
            print "Wrong speed value"

root = Tkinter.Toplevel()
Gui_Instance = GUI(root)
root.mainloop()
Gui_Instance.RosWorker.image_sub_right.unregister()
root.destroy()
