import sys
import cv2
import numpy as np
from scipy import optimize
from PyQt5.QtWidgets import QDialog, QApplication, QMainWindow, QLabel, QPushButton, QFrame, QCheckBox, QMessageBox
from PyQt5.uic import loadUi
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen
import os

# Config Variables - Enter their values according to your Checkerboard
no_of_columns = 9   # number of columns of your Checkerboard
no_of_rows = 6  # number of rows of your Checkerboard
square_size = 28.0  # size of square on the Checkerboard in mm

class MyGUI(QMainWindow):
    def __init__(self):
        super(MyGUI, self).__init__()
        self.initUI()  # initialize interface elements
        self.initEvents()  # initialize click events (listeners)
        self.cap = cv2.VideoCapture(0)  # webcam object
        self.pixmap = None
        self.firstPixmap = None  # first captured image. to be displayed at the end
        self.capturing = False
        self.confirmedImagesCounter = 0  # how many images are confirmed by user so far
        self.detectingCorners = False
        self.done_drawPredicted = False
        self.currentCorners = None  # array of last detected corners
        self.predicted2D = None
        self.homographies = []  # list of homographies of each captured image
        self.capturedImagePoints = {}  # dictionary of 2D points on captured image
        self.objectPoints = {}  # dictionary of 3D points on chessboard
        self.A = None  # intrinsic
        self.Rts = []  # extrinsic
        self.distCoeffs = None  # distortion coefficients
        self.points_in_row, self.points_in_column = no_of_rows, no_of_columns  # number of rows and columns of your Checkerboard
        x, y = square_size, square_size  # size of square on the Checkerboard in mm
        # points in 3D
        self.capturedObjectPointsLR = [[i * x, j * y, 0] for i in range(self.points_in_row, 0, -1) for j in range(self.points_in_column, 0, -1)]
        self.capturedObjectPointsRL = list(reversed(self.capturedObjectPointsLR))

    def initUI(self):
        # set window size & title
        self.setWindowTitle('Camera Calibration')
        self.setGeometry(20, 80, 660, 610)
        
        # initialize capture button
        self.captureButton = QPushButton('START', self)
        self.captureButton.resize(100, 40)
        self.captureButton.move(280, 10)
        
        # initialize checkbox
        self.detectCornersCheckbox = QCheckBox("Detect Corners", self)
        self.detectCornersCheckbox.move(10, 10)
        
        # initialize images counter label
        self.counterLabel = QLabel('Images taken: 0', self)
        self.counterLabel.resize(150, 40)
        self.counterLabel.move(430, 10)
        
        # initialize captured image label
        self.imageLabel = QLabel('Image will be displayed here', self)
        self.imageLabel.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.imageLabel.resize(640, 480)
        self.imageLabel.move(10, 60)
        self.imageLabel.setFrameShape(QFrame.Box)
        
        # initialize confirm button
        self.confirmButton = QPushButton('CONFIRM', self)
        self.confirmButton.resize(100, 30)
        self.confirmButton.move(60, 550)
        self.confirmButton.hide()
        
        # initialize ignore button
        self.ignoreButton = QPushButton('IGNORE', self)
        self.ignoreButton.resize(100, 30)
        self.ignoreButton.move(280, 550)
        self.ignoreButton.hide()
        
        # initialize done button
        self.doneButton = QPushButton('DONE', self)
        self.doneButton.resize(100, 30)
        self.doneButton.move(500, 550)
        
        # initialize signature label
        self.signatureLabel = QLabel('Coded by Obeida ElJundi & Mohammed Dhaybi', self)
        self.signatureLabel.resize(660, 20)
        self.signatureLabel.move(0, 590)
        self.signatureLabel.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)

    def initEvents(self):
        """ initialize click events (listeners) """
        self.captureButton.clicked.connect(self.captureClicked)
        self.ignoreButton.clicked.connect(self.ignoreClicked)
        self.confirmButton.clicked.connect(self.confirmClicked)
        self.doneButton.clicked.connect(self.doneClicked)
        self.detectCornersCheckbox.stateChanged.connect(self.detectCornersCheckboxClicked)
    
    def paintEvent(self, event):
        painter = QPainter(self)
        if self.pixmap:  # display image taken from webcam
            self.imageLabel.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            self.imageLabel.setText('')
            painter.drawPixmap(10, 60, self.pixmap)
        if self.capturing: self.captureImage()
        if self.done_drawPredicted:
            painter.drawPixmap(10, 60, self.firstPixmap)
            pen = QPen(Qt.white, 2, Qt.SolidLine)  # darkCyan
            painter.setPen(pen)
            for pixel in self.predicted2D:
                cx = int(pixel[0] + 10)
                cy = int(pixel[1] + 60)
                painter.drawEllipse(cx - 5, cy - 5, 10, 10)
                painter.drawLine(cx - 5, cy, cx + 5, cy)
                painter.drawLine(cx, cy - 5, cx, cy + 5)

    def captureClicked(self):
        if self.capturing == False:
            self.capturing = True
            self.captureButton.setText('STOP')
        else:
            self.capturing = False
            self.captureButton.setText('START')

    def detectCornersCheckboxClicked(self, state):
        self.detectingCorners = state == Qt.Checked

    def captureImage(self):
        """ captures frame from webcam & tries to detect corners on chess board """
        ret, frame = self.cap.read()  # read frame from webcam
        if ret:  # if frame captured successfully
            frame_inverted = cv2.flip(frame, 1)  # flip frame horizontally
            if self.detectingCorners:  # if detect corners checkbox is checked
                cornersDetected, corners, imageWithCorners = self.detectCorners(frame_inverted)  # detect corners on chess board
                if cornersDetected:  # if corners detected successfully
                    self.currentCorners = corners
                    self.frameWithCornersCaptured()
            self.pixmap = self.imageToPixmap(frame_inverted)
            self.update()
            
    def frameWithCornersCaptured(self):
        self.captureClicked()  # fix last frame
        self.toggleConfirmAndIgnoreVisibility(True)

    def confirmClicked(self):
        self.confirmedImagesCounter += 1
        if self.confirmedImagesCounter == 1: self.firstPixmap = self.pixmap
        self.counterLabel.setText('Images taken: ' + str(self.confirmedImagesCounter))
        self.capturedImagePoints[self.confirmedImagesCounter] = self.currentCorners
        if self.currentCorners[0, 0, 0] < self.currentCorners[-1, 0, 0]:
            capturedObjectPoints = self.capturedObjectPointsLR
        else:
            capturedObjectPoints = self.capturedObjectPointsRL
        self.objectPoints[self.confirmedImagesCounter] = capturedObjectPoints
        
        h = self.computeHomography(self.currentCorners, capturedObjectPoints)
        self.homographies.append(h)
        self.toggleConfirmAndIgnoreVisibility(False)
        self.captureClicked()  # continue capturing

    def ignoreClicked(self):
        self.captureClicked()  # continue capturing
        self.toggleConfirmAndIgnoreVisibility(False)

    def doneClicked(self):
        if self.confirmedImagesCounter < 3:
            rem = 3 - self.confirmedImagesCounter
            QMessageBox.question(self, 'Warning!', "The number of captured photos should be at least 3. Please take " + str(rem) + " more photos.", QMessageBox.Ok)
        else:
            self.captureClicked()  # stop capturing
            M = self.buildMmatrix()
            b = self.getMinimumEigenVector(M)
            v0, lamda, alpha, betta, gamma, u0, A = self.calculateIntrinsicParam(b)
            Rt = self.calculateExtrinsicParam(A)
            self.Rts = self.calculateAllExtrinsicParam(A, lamda)
            self.predicted2D = self.predict2Dpoints(self.objectPoints[1], A, Rt, lamda)
            self.done_drawPredicted = True
            self.update()

            # Calculate distortion coefficients using OpenCV's calibrateCamera function
            objpoints = np.array([np.array(p) for k in sorted(self.objectPoints.keys()) for p in self.objectPoints[k]], dtype=np.float32)
            imgpoints = np.array([np.squeeze(p) for k in sorted(self.capturedImagePoints.keys()) for p in self.capturedImagePoints[k]], dtype=np.float32)

            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera([objpoints], [imgpoints], (640, 480), A, None)

            self.distCoeffs = dist

            # Save distortion coefficients to file
            if not os.path.exists('./output'):
                os.mkdir('./output')
            with open('./output/distortion_coeffs.txt', 'w+') as f:
                f.write('Distortion Coefficients:\n{}'.format(dist))

            print('Done :)\nCheck intrinsic.txt, extrinsic.txt, distortion_coeffs.txt, & predicted VS actual.txt')
    
    def toggleConfirmAndIgnoreVisibility(self, visibility=True):
        if visibility:
            self.ignoreButton.show()
            self.confirmButton.show()
        else:
            self.ignoreButton.hide()
            self.confirmButton.hide()

    def detectCorners(self, image):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (no_of_columns, no_of_rows), cv2.CALIB_CB_FAST_CHECK)
        if ret:
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(image, (no_of_columns, no_of_rows), corners, ret)
        return ret, corners, image

    def imageToPixmap(self, image):
        qformat = QImage.Format_RGB888
        img = QImage(image, image.shape[1], image.shape[0], image.strides[0], qformat)
        img = img.rgbSwapped()  # BGR > RGB
        return QPixmap.fromImage(img)
    
    ####################### CALIBRATION STUFF ###########################
    
    def computeHomography(self, points2D, points3D):
        U = self.buildUmatrix(points2D, points3D)
        return self.getMinimumEigenVector(U)
    
    def buildUmatrix(self, points2D, points3D):
        rows = self.points_in_row * self.points_in_column * 2
        U = np.zeros((rows, 9))
        for i in range(len(points2D)):
            U[i * 2, 0] = points3D[i][0]  # Px
            U[i * 2, 1] = points3D[i][1]  # Py
            U[i * 2, 2] = 1
            U[i * 2, 3:6] = 0
            U[i * 2, 6:9] = U[i * 2, 0:3] * -points2D[i, 0, 0]
            
            U[i * 2 + 1, 0:3] = 0
            U[i * 2 + 1, 3:6] = U[i * 2, 0:3]
            U[i * 2 + 1, 6:9] = U[i * 2, 0:3] * -points2D[i, 0, 1]
        return U

    def calculateV(self, h1, h2):
        v = np.zeros((6, 1))
        v[0, 0] = h1[0] * h2[0]
        v[1, 0] = h1[0] * h2[1] + h1[1] * h2[0]
        v[2, 0] = h1[1] * h2[1]
        v[3, 0] = h1[2] * h2[0] + h1[0] * h2[2]
        v[4, 0] = h1[2] * h2[1] + h1[1] * h2[2]
        v[5, 0] = h1[2] * h2[2]
        return v

    def buildMmatrix(self):  # Build the matrix made by homographies to calculate B
        M = np.zeros((self.confirmedImagesCounter * 2, 6))
        for i in range(len(self.homographies)):
            h1 = self.homographies[i][::3]
            h2 = self.homographies[i][1::3]
            v12 = self.calculateV(h1, h2)  # 6X1
            v11 = self.calculateV(h1, h1)  # 6X1
            v22 = self.calculateV(h2, h2)  # 6X1
            M[2 * i, :] = v12.T  # 1X6
            M[2 * i + 1, :] = (v11 - v22).T  # 1X6
        return M

    def calculateIntrinsicParam(self, b):
        (B11, B12, B22, B13, B23, B33) = b
        v0 = (B12 * B13 - B11 * B23) / (B11 * B22 - B12 ** 2)
        lamda = B33 - (B13 ** 2 + v0 * (B12 * B13 - B11 * B23)) / B11
        alpha = np.sqrt(lamda / B11)
        betta = np.sqrt((lamda * B11) / (B11 * B22 - B12 ** 2))
        gamma = (-B12 * betta * alpha ** 2) / lamda
        u0 = (gamma * v0 / betta) - (B13 * alpha ** 2 / lamda)
        A = np.array([[alpha, gamma, u0], [0, betta, v0], [0, 0, 1]])
        # write intrinsic parameters to file
        if not os.path.exists('./output'):
            os.mkdir('./output')  # make output folder if not exists
        with open('./output/intrinsic.txt', 'w+') as f:
            f.write('A=\n{}'.format(A))
        return v0, lamda, alpha, betta, gamma, u0, A

    def calculateExtrinsicParam(self, A):
        h1 = self.homographies[0][::3]  # 1st column of 1st image homography
        h2 = self.homographies[0][1::3]  # 2nd column of 1st image homography
        h3 = self.homographies[0][2::3]  # 3rd column of 1st image homography
        A_inv = np.linalg.inv(A)
        Ah1 = np.dot(A_inv, h2)
        lamda = 1 / np.sqrt(np.dot(Ah1, Ah1))
        r1 = lamda * np.dot(A_inv, h1)  # 1st column or rotation matrix
        r2 = lamda * np.dot(A_inv, h2)  # 2nd column or rotation matrix
        r3 = np.cross(r1, r2)  # 3rd column or rotation matrix
        t = lamda * np.dot(A_inv, h3)  # translation vector
        Rt = np.array([r1.T, r2.T, r3.T, t.T]).T
        # write extrinsic parameters to file
        if not os.path.exists('./output'):
            os.mkdir('./output')  # make output folder if not exists
        with open('./output/extrinsic.txt', 'w+') as f:
            f.write('[R|t]=\n{}'.format(Rt))
        return Rt

    def calculateAllExtrinsicParam(self, A, lamda):
        Rts = []
        A_inv = np.linalg.inv(A)
        for homography in self.homographies:
            h1 = homography[::3]
            h2 = homography[1::3]
            h3 = homography[2::3]
            r1 = lamda * np.dot(A_inv, h1)
            r2 = lamda * np.dot(A_inv, h2)
            r3 = np.cross(r1, r2)
            t = lamda * np.dot(A_inv, h3)
            Rt = np.array([r1.T, r2.T, r3.T, t.T]).T
            Rts.append(Rt)
        return Rts

    def getMinimumEigenVector(self, U):
        """ return eigen vector of square matrix U with the minimum eigen value """
        P = np.dot(U.T, U)
        w, v = np.linalg.eig(P)
        i = np.where(w == np.min(w))
        e1 = v[:, i[0][0]]
        return e1

    def predict2Dpoints(self, points3D, A, Rt, lamda):
        Rt = np.delete(Rt, 2, axis=1)  # remove r3 column
        points3D = [[p[0], p[1], 1] for p in points3D]
        imgpoints = np.array([np.squeeze(p) for p in self.capturedImagePoints[1]])
        imgpoints = np.append(imgpoints, np.ones((imgpoints.shape[0], 1)), axis=1)  # append 1 to each 2D point
        pred = []
        if not os.path.exists('./output'):
            os.mkdir('./output')  # make output folder if not exists
        f = open('./output/predicted VS actual.txt', 'w+')
        f.write('predicted >> actual')
        for p3d, p2d in zip(points3D, imgpoints):
            p3d = np.array(p3d)
            tmp = np.dot(np.dot(A, Rt), p3d)
            if tmp[2] != 0:  # avoid division by zero
                tmp = tmp / tmp[2]
            else:
                tmp[:] = np.nan  # set to NaN if division by zero is detected

            if not np.any(np.isnan(tmp)):  # only use points that are not NaN
                pred.append(tmp)
                f.write('\n{} , {}  >>  {} , {}'.format(tmp[0], tmp[1], p2d[0], p2d[1]))
            else:
                f.write('\nNaN , NaN  >>  {} , {}'.format(p2d[0], p2d[1]))
                print("Warning: NaN value detected during prediction. This point will be skipped.")

        f.close()
        return pred


app = QApplication(sys.argv)
window = MyGUI()
window.show()
sys.exit(app.exec_())
q