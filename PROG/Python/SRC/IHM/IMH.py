#!/usr/bin/python

import rospy
import numpy as np
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
from std_msgs.msg import Float32
from std_msgs.msg import Bool
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
        self.speedSet = False
        self.command = "STOP"
        self.status = "Unknown"
        self.SetHeightButtonColor = 'orange'
        self.directionNumber = 0
        self.SonarValue = 0.
        self.Light = False

        self.commandDictionnary = {}
        self.commandDictionnary['STOP']=0
        self.commandDictionnary['MOVE']=1
        self.commandDictionnary['RESET']=2
        self.commandDictionnary['SETH']=3
        self.commandDictionnary['ERROR']=4

        self.position = np.array([0., 0., 0.])
        self.orientation = np.array([1., 0., 0.])
        self.master.title("Cornelius GUI")

        self.label = Tkinter.Label(self.master, text="Main commands")
        self.label.grid(row = 0, columnspan=5, sticky = 'EW')
        
        LegsWindow = Tkinter.Frame(self.master, borderwidth=2)
        LegsWindow.grid(row=1,  column = 0, rowspan = 2)
        self.LegsLabels = [Tkinter.Label(LegsWindow, text="  \n  {0}  \n  ".format(n_leg), bg="red", fg="white") for n_leg in range(6)]
        self.LegsStatus = []
        for n_legLabel in range(len(self.LegsLabels)):
            self.LegsStatus += [0]
            self.LegsLabels[n_legLabel].grid(row = int(0+n_legLabel/3), column = n_legLabel%3)

        self.master.protocol("WM_DELETE_WINDOW", self.master.quit)

        light = Tkinter.PhotoImage(file='Icons/light.png')
        self.ButtonLight = Tkinter.Button(self.master, width=50, height=50, image=light, bg='red')
        self.ButtonLight.image = light
        self.ButtonLight.grid(row = 1, column = 3, sticky='E')
        self.ButtonLight.bind("<Button-1>", self.SwitchLight)

        RobotDataWindow = Tkinter.Frame(self.master, borderwidth = 2)
        RobotDataWindow.grid(row = 1, column = 4)
        self.PositionLabel = Tkinter.Label(RobotDataWindow, text = "Position : ")
        self.PositionLabel.grid(row=0, column = 0)
        self.OrientationLabel = Tkinter.Label(RobotDataWindow, text = "Orientation : ")
        self.OrientationLabel.grid(row=1, column = 0)
        self.SonarLabel = Tkinter.Label(RobotDataWindow, text = "Sonar : ")
        self.SonarLabel.grid(row=2, column = 0)

        self.PlotPlot = Figure(figsize=(5, 4), dpi=100)
        self.SubPlotPlot = self.PlotPlot.add_subplot(111)
        self.PlotCanvas = FigureCanvasTkAgg(self.PlotPlot, master=self.master)
        self.PlotCanvas.get_tk_widget().grid(row = 3, column = 0, columnspan = 4)
        
        self.PlotPosition = Figure(figsize=(5, 4), dpi=100)
        self.SubPlotPosition = self.PlotPosition.add_subplot(111)
        self.PositionCanvas = FigureCanvasTkAgg(self.PlotPosition, master=self.master)
        self.PositionCanvas.get_tk_widget().grid(row = 4, column = 0, columnspan = 4)
        
        f = Figure(figsize=(5, 4), dpi=100)
        self.SubPlotPicture = f.add_subplot(111)
        self.img = np.zeros([480,640,3])
        self.PictureCanvas = FigureCanvasTkAgg(f, master=self.master)
        self.PictureCanvas.show()
        self.PictureCanvas.get_tk_widget().grid(row = 3, column = 4, columnspan = 1)

        self.PlotMap = Figure(figsize=(5, 4), dpi=100)
        self.SubPlotMap = self.PlotPosition.add_subplot(111)
        self.MapCanvas = FigureCanvasTkAgg(self.PlotPosition, master=self.master)
        self.MapCanvas.get_tk_widget().grid(row = 4, column = 4, columnspan = 1)
        
        DirectionWindow = Tkinter.Frame(self.master, borderwidth=2)
        DirectionWindow.grid(row = 5, column = 0, columnspan = 3)
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
        #self.master.bind("<KeyRelease>", self.keyReleaseCallback)

        print "Starting ROSWorker class"
        self.RosWorker = ROSWorker(self)

        ParametersWindow = Tkinter.Frame(self.master, borderwidth=2)
        ParametersWindow.grid(row = 5, column = 3)
        HLabel = Tkinter.Label(ParametersWindow, text="H = ")
        HLabel.grid(row=0, column = 0)
        self.HEntry = Tkinter.Entry(ParametersWindow, textvariable=self.h)
        self.HEntry.grid(row=0, column=1)
        self.HButton = Tkinter.Button(ParametersWindow, text="Set Height", background = 'gray', command=self.RosWorker.SetH)
        self.HButton.grid(row=0, column=2)
        SpeedLabel = Tkinter.Label(ParametersWindow, text="Speed = ")
        SpeedLabel.grid(row=1, column = 0)
        self.SpeedEntry = Tkinter.Entry(ParametersWindow, textvariable=self.speed)
        self.SpeedEntry.grid(row=1, column=1)
        self.SpeedButton = Tkinter.Button(ParametersWindow, text="Set Speed", background = 'orange', command=self.RosWorker.SetSpeed)
        self.SpeedButton.grid(row=1, column=2)

        CommandWindow = Tkinter.Frame(self.master, borderwidth=2)
        CommandWindow.grid(row = 5, column = 4)
        self.StatusLabel = Tkinter.Label(CommandWindow, text="Current status : " + self.status)
        self.StatusLabel.grid(row=0, column = 0, columnspan = 4)
        self.CommandLabel = Tkinter.Label(CommandWindow, text="Current command : " + self.command)
        self.CommandLabel.grid(row=1, column = 0, columnspan = 4)
        self.StopButton = Tkinter.Button(CommandWindow, text = "  \nSTOP\n  ", background = 'red', command = lambda: self.SetCommand(0))
        self.StopButton.grid(row=2, column = 0)
        self.MoveButton = Tkinter.Button(CommandWindow, text = "  \nMove\n  ", background = 'red', command = lambda: self.SetCommand(1))
        self.MoveButton.grid(row=2, column = 1)
        self.ResetButton = Tkinter.Button(CommandWindow, text = "  \nReset\n  ", background = 'red', command = lambda: self.SetCommand(2))
        self.ResetButton.grid(row=2, column = 2)
        self.SetHeightButton = Tkinter.Button(CommandWindow, text = "  \nSet Height\n  ", background = 'red', command = lambda: self.SetCommand(3))
        self.SetHeightButton.grid(row=2, column = 3)

        self.UpdateCommand()
        self.UpdateStatus()


        #self.RosProcess = Process(target = self.RosWorker.run(),  args=())

        self.N = 1
        self.UpdatePlot()
        self.UpdatePicture()
        self.UpdateRobotData()
        self.UpdatePosition()
        self.UpdateMap()
        self.UpdateLegs()
        self.UpdateSonar()
    
    def SwitchLight(self, event):
        self.Light = bool(1-self.Light)
        self.RosWorker.LightPub.publish(self.Light)
        if self.Light:
            self.ButtonLight.configure(background = 'green')
        else:
            self.ButtonLight.configure(background = 'red')

    def SetCommand(self, commandValue, from_outside = False):
        if self.speedSet:
            self.MoveButton.configure(background = 'gray')
            self.StopButton.configure(background = 'gray')
            self.SetHeightButton.configure(background = 'gray')
            self.ResetButton.configure(background = 'gray')
            if self.status == 'ERROR' and self.command !=  'RESET':
                None
                #print "Current command is ERROR. Filtring dangerous commands"
            else:
                self.SetHeightButton.configure(background = self.SetHeightButtonColor)
                if commandValue == 0:
                    if self.command != 'ERROR':
                        self.command = "STOP"
                        self.StopButton.configure(background = 'green')
                elif commandValue == 1:
                    if self.command != 'ERROR':
                        self.command = "MOVE"
                        self.MoveButton.configure(background = 'green')
                elif commandValue == 2:
                    self.command = "RESET"
                    self.ResetButton.configure(background = 'green')
                elif commandValue == 3:
                    if self.command != 'ERROR':
                        self.command = "SETH"
                        self.SetHeightButton.configure(background = 'green')
                elif commandValue == 4:
                    self.command = 'ERROR'
                    self.SetHeightButton.configure(background = 'red')
                    self.MoveButton.configure(background = 'red')
                    self.StopButton.configure(background = 'red')
                    
            if not from_outside:
                self.RosWorker.CommandPub.publish(self.command)
                if self.command == 'SETH':
                    self.SetHeightButtonColor = 'gray'

    def UpdatePlot(self):
        t = arange(0.0, 3.0, 0.01)
        s = sin(self.N*2*pi*t)

        self.SubPlotPlot.clear()
        self.SubPlotPlot.plot(t, s)
        self.PlotCanvas.show()
        self.N +=0.5
        
    def UpdateRobotData(self):
        self.PositionLabel['text'] = "Position : X = {0:.2}, Y = {1:.2}, Z = {2:.2}".format(self.position[0], self.position[1], self.position[2])
        self.OrientationLabel['text'] = "Orientation : Ux = {0:.2}, Uy = {1:.2}, Uz = {2:.2}".format(self.orientation[0], self.orientation[1], self.orientation[2])
        self.master.after(100, self.UpdateRobotData)

    def UpdateLegs(self):
        for n_leg in range(6):
            if self.LegsStatus[n_leg] == 1:
                self.LegsLabels[n_leg].configure(background = 'green')
            else:
                self.LegsLabels[n_leg].configure(background = 'red')
        self.master.after(50, self.UpdateLegs)

    def UpdatePicture(self):
        self.SubPlotPicture.clear()
        self.SubPlotPicture.imshow(self.img)
        self.PictureCanvas.show()
        self.master.after(50, self.UpdatePicture)

    def UpdateCommand(self):
        self.CommandLabel['text'] = "Current command : " + self.command
        self.SetCommand(self.commandDictionnary[self.command], from_outside=True)
        self.master.after(50,  self.UpdateCommand)

    def UpdateStatus(self):
        self.StatusLabel['text'] = "Current status : " + self.status
        self.master.after(50,  self.UpdateStatus)

    def UpdatePosition(self):
        self.SubPlotPosition.plot(self.position[0], self.position[1], 'xr')
        self.PositionCanvas.show()
        self.master.after(100, self.UpdatePosition)

    def UpdateMap(self):
        #self.SubPlotMap.plot(self.position[0], self.position[1])
        self.PositionCanvas.show()
        #self.master.after(1000, self.UpdatePosition)

    def UpdateSonar(self):
        self.SonarLabel['text'] = "Sonar : {0} cm".format(int(self.SonarValue))
        self.master.after(300, self.UpdateSonar)

    def keyReleaseCallback(self, event):
        self.KeyPressed = False
        self.SetDirection(0)
    def keyEventCallback(self, event):
        if event.keysym == 'Left':
            self.SetDirection(1)
        elif event.keysym == 'Right':
            self.SetDirection(3)
        elif event.keysym == 'Up':
            self.SetDirection(2)
        elif event.keysym == 'Down':
            self.SetDirection(0)

    def SetDirection(self, directionNumber):
        if self.directionNumber != directionNumber:
            self.directionNumber = directionNumber
            if directionNumber == 0:
                self.RosWorker.DirPub.publish(np.array([0., 0., 1.], dtype = np.float32))
                if self.status == 'MOVING':
                    self.SetCommand(0)
            elif directionNumber == 1:
                self.RosWorker.DirPub.publish(np.array([40., 40., 1.], dtype = np.float32))
                if self.command != 'MOVING':
                    self.SetCommand(1)
            elif directionNumber == 2:
                self.RosWorker.DirPub.publish(np.array([40., 0., 1.], dtype = np.float32))
                if self.command != 'MOVING':
                    self.SetCommand(1)
            elif directionNumber == 3:
                self.RosWorker.DirPub.publish(np.array([40., -40., 1.], dtype = np.float32))
                if self.command != 'MOVING':
                    self.SetCommand(1)

class ROSWorker():
    
    def __init__(self, parent = None):
        

        self.WindowManager = parent
        self.bridge = CvBridge()

        self.lastPictureUpdate = time.time()
        rospy.init_node('GUI', anonymous=True)
        self.image_sub_right = rospy.Subscriber("/stereo/right/image_raw", Image, self.PictureCallback)
        self.leg_contacts_subcriber = rospy.Subscriber("legs_contacts", String, self.ContactsCallback)
        self.commandSubscriber = rospy.Subscriber('command', String, self.CommandCallback)
        self.statusSubscriber = rospy.Subscriber('status', String, self.StatusCallback)
        self.positionSubscriber = rospy.Subscriber('position', numpy_msg(Floats), self.PositionCallback)
        self.orientationSubscriber = rospy.Subscriber('orientation', numpy_msg(Floats), self.OrientationCallback)
        self.SonarSubscriber = rospy.Subscriber('sonar_front', Float32, self.SonarCallback)
        self.DirPub = rospy.Publisher("direction", numpy_msg(Floats),queue_size=1)
        self.HeightPub = rospy.Publisher("height", Float64 ,queue_size=1)
        self.LightPub = rospy.Publisher("led", Bool ,queue_size=1)
        self.SpeedPub = rospy.Publisher("speed", Float64 ,queue_size=1)
        self.CommandPub = rospy.Publisher("command", String, queue_size=1)

        
        #rospy.spin()
        
        print "Done with ROSWorker init"

    def ContactsCallback(self, contactsMessage):
        for n_leg in range(6):
            self.WindowManager.LegsStatus[n_leg] = float(contactsMessage.data.split('&')[n_leg])

    def PictureCallback(self, data):  
        if time.time() - self.lastPictureUpdate > 0.1:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)        
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            self.WindowManager.img = cv_image
            self.lastPictureUpdate = time.time()
    
    def CommandCallback(self, data):
        self.WindowManager.command = data.data
    
    def StatusCallback(self, data):
        self.WindowManager.status = data.data

    def PositionCallback(self, data):
        self.WindowManager.position = data.data

    def OrientationCallback(self, data):
        self.WindowManager.orientation = data.data

    def SonarCallback(self, data):
        self.WindowManager.SonarValue = data.data

    def SetH(self):
        self.HeightPub.publish(float(self.WindowManager.h.get()))
        self.WindowManager.SetHeightButtonColor = 'orange'
        self.WindowManager.master.focus()
        if self.WindowManager.status != 'ERROR':
            self.WindowManager.SetHeightButton.configure(background = self.WindowManager.SetHeightButtonColor)

    def SetSpeed(self):
        if 0 < float(self.WindowManager.speed.get()) <=1:
            self.WindowManager.master.focus()
            self.WindowManager.speedSet = True
            self.WindowManager.SpeedButton.configure(background = 'gray')
            self.WindowManager.SetCommand(self.WindowManager.commandDictionnary[self.WindowManager.command], True) # Fake line to easily reset command buttons colors
            self.SpeedPub.publish(float(self.WindowManager.speed.get()))
        else:
            print "Wrong speed value"

root = Tkinter.Toplevel()
Gui_Instance = GUI(root)
root.mainloop()
Gui_Instance.RosWorker.image_sub_right.unregister()
root.destroy()
