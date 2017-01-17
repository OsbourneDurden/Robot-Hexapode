import numpy as np
from scipy import ndimage
import cv2

class Analyser:
    def __init__(self, calibrated = False):
        self.PointsList = []
        if calibrated:
            calibration_file = open("calibration.txt", 'r')
            self.LeftCameraPosition = [0,0,0]
            self.RightCameraPosition = [0,0,0]
            self.WindowMatchSize = None
            self.variance_threshold = 0
            for line_raw in calibration_file:
                line = line_raw[:-1]
                if "HHL" in line.split('&'):
                    self.HomographyHorizontalCameraLeft = np.array([float(value) for value in line.split('&')[1:]]).reshape((3,3))
                elif "HVL" in line.split('&'):
                    self.HomographyVerticalCameraLeft = np.array([float(value) for value in line.split('&')[1:]]).reshape((3,3))
                elif "HHR" in line.split('&'):
                    self.HomographyHorizontalCameraRight = np.array([float(value) for value in line.split('&')[1:]]).reshape((3,3))
                elif "HVR" in line.split('&'):
                    self.HomographyVerticalCameraRight = np.array([float(value) for value in line.split('&')[1:]]).reshape((3,3))
                elif "CLP" in line.split('&'): # Stands for Camera Left Position
                    self.LeftCameraPosition = np.array([float(value) for value in line.split('&')[1:]])
                elif "CRP" in line.split('&'): # Stands for Camera Right Position
                    self.RightCameraPosition = np.array([float(value) for value in line.split('&')[1:]])
                elif "WMS" in line.split('&'):
                    self.WindowMatchSize = int(line.split('&')[1])
                    self.HWMS = int((self.WindowMatchSize-1)/2.)
                elif "LWL" in line.split('&'):
                    self.LeftWindowLimits={}
                    for value in line.split('&')[1:]:
                        self.LeftWindowLimits[value.split('=')[0]] = int(value.split('=')[1])
                elif "RWL" in line.split('&'):
                    self.RightWindowLimits={}
                    for value in line.split('&')[1:]:
                        self.RightWindowLimits[value.split('=')[0]] = int(value.split('=')[1])
                elif "PP" in line.split('&'):
                    self.PicturesPath = line.split('&')[1]
                elif "VT" in line.split('&'):
                    self.variance_threshold= float(line.split('&')[1])
                else:
                    print "Unknown entry: {0}".format(line)
            calibration_file.close()
            if self.WindowMatchSize == None or (self.WindowMatchSize/2. == int(self.WindowMatchSize/2.)):
                print "Wrond WindowMatchSize (undefined or even number)."
        else:
            print "No correct calibration file found"

    def save_parameters(self, output):
        
        calibration_file = open(output, 'w')
        try:
            HHLstring = 'HHL'
            for value in self.HomographyHorizontalCameraLeft.reshape(9).tolist() :
                HHLstring += '&'+str(value)
            calibration_file.write(HHLstring+'\n')
        except:
            None

        try:
            HVLstring='HVL'
            for value in self.HomographyVerticalCameraLeft.reshape(9).tolist() :
                HVLstring += '&'+str(value)
            calibration_file.write(HVLstring+'\n')
        except:
            None

        try:
            HHRstring='HHR'
            for value in self.HomographyHorizontalCameraRight.reshape(9).tolist() :
                HHRstring += '&'+str(value)
            calibration_file.write(HHRstring+'\n')
        except:
            None

        try:
            HVRstring='HVR'
            for value in self.HomographyVerticalCameraRight.reshape(9).tolist() :
                HVRstring += '&'+str(value)
            calibration_file.write(HVRstring+'\n')
        except:
            None

        try:
            CLPstring='CLP'
            for value in self.LeftCameraPosition.tolist() :
                CLPstring += '&'+str(value)
            calibration_file.write(CLPstring+'\n')
        except:
            None

        try:
            CRPstring='CRP'
            for value in self.RightCameraPosition.tolist() :
                CRPstring += '&'+str(value)
            calibration_file.write(CRPstring+'\n')
        except:
            None

        try:
            WMSstring='WMS'
            WMSstring += '&'+str(self.WindowMatchSize)
            calibration_file.write(WMSstring+'\n')
        except:
            None

        try:
            LWLstring='LWL'
            for cle in self.LeftWindowLimits.keys() :
                LWLstring+='&'+str(cle)+'='+str(self.LeftWindowLimits[cle])
            calibration_file.write(LWLstring+'\n')
        except:
            None
        
        try:
            RWLstring='RWL'
            for cle in self.RightWindowLimits.keys() :
                RWLstring+='&'+str(cle)+'='+str(self.RightWindowLimits[cle])
            calibration_file.write(RWLstring+'\n')
        except:
            None

        try:
            VTstring = 'VT'
            VTstring += '&'+str(self.variance_threshold)
            calibration_file.write(VTstring+'\n')
        except:
            None

        calibration_file.close()

    def open_pictures(self, leftImagePath, rightImagePath):
        leftImageRaw = cv2.imread(leftImagePath)
        rightImageRaw = cv2.imread(rightImagePath)

        self.ImageLeft = cv2.cvtColor(leftImageRaw, cv2.COLOR_RGB2GRAY)
        self.ImageRight = cv2.cvtColor(rightImageRaw, cv2.COLOR_RGB2GRAY)

    def calibrate(self, leftImagePath, RightImagePath):
        self.open_pictures(leftImagePath, RightImagePath)

        ImageHorRawInput = raw_input("Please enter the coordinates of the calibration points for the HORIZONTAL landmark : \n")
        ImageVerRawInput = raw_input("Please enter the coordinates of the calibration points for the VERTICAL landmark : \n")

        self.ImageHorCoordinates = [[float(value.split(',')[0]), float(value.split(',')[1])] for value in ImageHorRawInput.split(';')]
        self.ImageVerCoordinates = [[float(value.split(',')[0]), float(value.split(',')[1])] for value in ImageVerRawInput.split(';')]

        cv2.namedWindow("ImageLeft", cv2.WINDOW_NORMAL)
        
        cv2.imshow("ImageLeft", self.ImageLeft)
        self.LeftImageHorPoints = []
        print "Recording point for Left Image, Horizontal landmark"
        cv2.setMouseCallback("ImageLeft", self.get_click_Left_Hor)
        k = cv2.waitKey()
        self.LeftImageVerPoints = []
        print "Recording point for Left Image, Vertical landmark"
        cv2.setMouseCallback("ImageLeft", self.get_click_Left_Ver)
        k = cv2.waitKey()
        cv2.destroyAllWindows()
        cv2.namedWindow("ImageRight", cv2.WINDOW_NORMAL)
        cv2.imshow("ImageRight", self.ImageRight)
        self.RightImageHorPoints = []
        print "Recording point for Right Image, Horizontal landmark"
        cv2.setMouseCallback("ImageRight", self.get_click_Right_Hor)
        k = cv2.waitKey()
        self.RightImageVerPoints = []
        print "Recording point for Right Image, Vertical landmark"
        cv2.setMouseCallback("ImageRight", self.get_click_Right_Ver)
        k = cv2.waitKey()
        cv2.destroyAllWindows()

        self.HomographyHorizontalCameraLeft = self.computeHomography(self.ImageHorCoordinates, self.LeftImageHorPoints)
        self.HomographyVerticalCameraLeft = self.computeHomography(self.ImageVerCoordinates, self.LeftImageVerPoints)
        self.HomographyHorizontalCameraRight = self.computeHomography(self.ImageHorCoordinates, self.RightImageHorPoints)
        self.HomographyVerticalCameraRight = self.computeHomography(self.ImageVerCoordinates, self.RightImageVerPoints)

    def get_click_Left_Hor(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.LeftImageHorPoints += [[x, y]]
            print "Added point {0} ({1}/{2})".format(self.LeftImageHorPoints[-1], len(self.LeftImageHorPoints), len(self.ImageHorCoordinates))
    def get_click_Left_Ver(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.LeftImageVerPoints += [[x, y]]
            print "Added point {0} ({1}/{2})".format(self.LeftImageVerPoints[-1], len(self.LeftImageVerPoints), len(self.ImageVerCoordinates))
    def get_click_Right_Hor(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.RightImageHorPoints += [[x, y]]
            print "Added point {0} ({1}/{2})".format(self.RightImageHorPoints[-1], len(self.RightImageHorPoints), len(self.ImageHorCoordinates))
    def get_click_Right_Ver(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.RightImageVerPoints += [[x, y]]
            print "Added point {0} ({1}/{2})".format(self.RightImageVerPoints[-1], len(self.RightImageVerPoints), len(self.ImageVerCoordinates))

    def computeHomography(self, ActualCoordinates, ImageCoordinates):
        CompleteMatrix = self.CreaM(ActualCoordinates[0], ImageCoordinates[0])
        for ActualCoordinate, ImageCoordinate in zip(ActualCoordinates[1:], ImageCoordinates[1:]):
            CompleteMatrix = np.array(CompleteMatrix.tolist() + self.CreaM(ActualCoordinate, ImageCoordinate).tolist())
        U,s,V = np.linalg.svd(CompleteMatrix)

        return np.array(V)[:,-1]

    def CreaM(self, Pt1, Pt2):
        x=Pt1[0]
        y=Pt1[1]
        xprime=Pt2[0]
        yprime=Pt2[1]
        matrix_elem=np.array([[0, 0, 0, -x, -y, -1, x*yprime, y*yprime, yprime],[x, y, 1, 0, 0, 0, -x*xprime, -y*xprime, -xprime],[-x*yprime, -y*yprime, -yprime, x*xprime, y*xprime, xprime, 0, 0, 0]])
        return matrix_elem

    def match_point(self, point, origin):
        '''Computes the position of a point in the other image.
        Input :
            - point : 2D list with the position of the point in the origin image
            - origin : String, "left" or "right" giving the original picture the point is from
        Returns a 2D list with the theoretical point in the other frame'''

        point = np.array(point + [1.])
        if origin == 'left':
            pointHorizontalFromLeft = np.dot(self.HomographyHorizontalCameraLeft, point)
            pointHorizontalValue = pointHorizontalFromLeft[0]/pointHorizontalFromLeft[2]

            pointVerticalFromLeft = np.dot(self.HomographyVerticalCameraLeft, point)
            pointVerticalValue = pointVerticalFromLeft[0]/pointVerticalFromLeft[2]

            neighbourhood_to_match = self.ImageLeft[point[0] - self.HWMS:point[0] + self.HWMS, point[1] - self.HWMS:point[1] + self.HWMS]

            epipolarLine = np.cross(np.dot(np.linalg.inv(self.HomographyHorizontalCameraRight), np.dot(self.HomographyHorizontalCameraLeft, point)),
                                    np.dot(np.linalg.inv(self.HomographyVerticalCameraRight), np.dot(self.HomographyVerticalCameraLeft, point)))
            distances = {}
            if epipolarLine[1] != 0:
                for x in range(self.RightWindowLimits['xmin'], self.RightWindowLimits['xmax']+1):
                    y = int((-epipolarLine[2] - epipolarLine[0]*x)/epipolarLine[1])
                    if self.RightWindowLimits['ymin'] <= y <= self.RightWindowLimits['ymax']:
                        distances[x] = np.sum(abs(neighbourhood_to_match - self.ImageRight[point[0] - self.HWMS:point[0] + self.HWMS, point[1] - self.HWMS:point[1] + self.HWMS]))

            else:
                x = int((-epipolarLine[2])/epipolarLine[1])
                if self.RightWindowLimits['xmin'] <= x <= self.RightWindowLimits['xmax']:
                    for y in range(self.RightWindowLimits['ymin'], self.RightWindowLimits['ymax']+1):
                        distances[x] = np.sum(abs(neighbourhood_to_match - self.ImageRight[point[0] - self.HWMS:point[0] + self.HWMS, point[1] - self.HWMS:point[1] + self.HWMS]))

        elif origin == 'right':
            pointHorizontalFromRight = np.dot(self.HomographyHorizontalCameraRight)
            pointHorizontalValue = pointHorizontalFromRight[0]/pointHorizontalFromRight[2]

            pointVerticalFromRight = np.dot(self.HomographyVerticalCameraRight)
            pointVerticalValue = pointVerticalFromRight[0]/pointVerticalFromRight[2]

            neighbour_to_match = self.ImageRight[point[0] - self.HWMS:point[0] + self.HWMS, point[1] - self.HWMS:point[1] + self.HWMS]

            epipolarLine = np.cross(np.dot(np.linalg.inv(self.HomographyHorizontalCameraLeft), np.dot(self.HomographyHorizontalCameraRight, point)),
                                    np.dot(np.linalg.inv(self.HomographyVerticalCameraLeft), np.dot(self.HomographyVerticalCameraRight, point)))
            distances = {}
            if epipolarLine[1] != 0:
                for x in range(self.LeftWindowLimits['xmin'], self.LeftWindowLimits['xmax']+1):
                    y = int((-epipolarLine[2] - epipolarLine[0]*x)/epipolarLine[1])
                    if self.LeftWindowLimits['ymin'] <= y <= self.LeftWindowLimits['ymax']:
                        distances[x] = np.sum(abs(neighbourhood_to_match - self.ImageLeft[point[0] - self.HWMS:point[0] + self.HWMS, point[1] - self.HWMS:point[1] + self.HWMS]))

            else:
                x = int((-epipolarLine[2])/epipolarLine[1])
                if self.LeftWindowLimits['xmin'] <= x <= self.LeftWindowLimits['xmax']:
                    for y in range(self.LeftWindowLimits['ymin'], self.LeftWindowLimits['ymax']+1):
                        distances[x] = np.sum(abs(neighbourhood_to_match - self.ImageLeft[point[0] - self.HWMS:point[0] + self.HWMS, point[1] - self.HWMS:point[1] + self.HWMS]))

        xFinal = distances.keys()[distances.values().index(min(distances.values()))]
        yFinal = int((-epipolarLine[2] - epipolarLine[0]*xFinal)/epipolarLine[1])
        
        return [xFinal, yFinal]

    def compute3DCoordinates(self, point_left, point_right):
        point_left = np.array(point_left+[1.])
        point_right = np.array(point_right+[1.])

        tmpPoint = np.dot(self.HomographyHorizontalCameraLeft, point_left)
        lineLeftPointHorizontal = np.array([0, tmpPoint[0]/tmpPoint[2], tmpPoint[1]/tmpPoint[2]])
        tmpPoint = np.dot(self.HomographyVerticalCameraLeft, point_left)
        lineLeftPointVertical = np.array([tmpPoint[0]/tmpPoint[2], tmpPoint[1]/tmpPoint[2], 0])

        tmpPoint = np.dot(self.HomographyHorizontalCameraRight, point_right)
        lineRightPointHorizontal = np.array([0, tmpPoint[0]/tmpPoint[2], tmpPoint[1]/tmpPoint[2]])
        tmpPoint = np.dot(self.HomographyVerticalCameraRight, point_right)
        lineRightPointVertical = np.array([tmpPoint[0]/tmpPoint[2], tmpPoint[1]/tmpPoint[2], 0])

        return self.findClosestPoint([lineLeftPointHorizontal, lineLeftPointVertical], [lineRightPointHorizontal, lineRightPointVertical])

    def findClosestPoint(self, lineLeft, lineRight):
        PLeft = lineLeft[0]
        PRight = lineRight[1]

        vLeft = lineLeft[1]-lineLeft[0]
        vRight = lineRight[1]-lineRight[0]

        a = np.dot(vLeft, vLeft)
        b = np.dot(vLeft, vRight)
        c = np.dot(vRight, vRight)
        d = np.dot(vLeft, (PLeft-PRight))
        e = np.dot(vRight, (PLeft-PRight))
        f = np.dot((PLeft-PRight), (PLeft-PRight))

        t = (b*e-c*d)/(a*c-b**2)
        return PLeft + t*vLeft

    def ComputeImages(self, origin = 'left'):
        
        points_to_compute = self.pointsOfInterest(origin)

        for original_point in points_to_compute:
            matched_point = self.match_point(original_point, origin)
            if origin == 'left':
                self.PointsList += self.compute3DCoordinates(point_left = original_point, point_right = matched_point)
            else:
                self.PointsList += self.compute3DCoordinates(point_left = original_point, point_right = matched_point)

    def pointsOfInterest(self, origin):
        ''' Function recuperation point of interest
        Input :
        - variance_threshold
        - origin
        Return a list of point (x,y)'''
        
        if origin == 'left' :
            xmin = self.LeftWindowLimits['xmin']
            ymin = self.LeftWindowLimits['ymin']
            xmax = self.LeftWindowLimits['xmax']
            ymax = self.LeftWindowLimits['ymax']
            Image = self.ImageLeft
            
        else :
            xmin = self.RightWindowLimits['xmin']
            ymin = self.RightWindowLimits['ymin']
            xmax = self.RightWindowLimits['xmax']
            ymax = self.RightWindowLimits['ymax']
            Image = self.ImageRight        
            
        points = []
        for x in range(xmin+self.HWMS, xmax-self.HWMS, self.WindowMatchSize) :
            for y in range(ymin+self.HWMS, ymax-self.HWMS, self.WindowMatchSize) :
                win = Image[x-self.HWMS : x+self.HWMS, y-self.HWMS : y+self.HWMS]
                variance = ndimage.variance(win)
                if variance > self.variance_threshold :
                    points += [[x,y]]
                    
        return points


