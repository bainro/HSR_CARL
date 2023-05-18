from PyQt5.QtWidgets import QDialog, QGridLayout, QGroupBox, QPushButton, QLabel
from PyQt5 import QtGui, QtCore
import hand_cam_channels
import arm_control

class HandCam (QDialog):

    def __init__(self, parent = None):
        super(HandCam, self).__init__(parent)
        self.title = 'HSR Grasping Controls'
        self.left = 50
        self.top = 0
        self.width = 500
        self.height = 700
        self.cameraFeed = hand_cam_channels.rgbOut()
        self.armMove = arm_control.ArmControl()
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.setFixedSize(self.width, self.height)
        self.createGridLayout()
        self.show()

    def createGridLayout(self):
        self.createCameraGrid()
        self.createArmMoveGrid()
        self.createWristMoveGrid()
        self.createGraspGrid()
        gridLayout = QGridLayout()
        gridLayout.addWidget(self.cameraBox, 0, 0)
        gridLayout.addWidget(self.armMoveBox, 2, 0)
        gridLayout.addWidget(self.wristMoveBox, 1, 0)
        gridLayout.addWidget(self.graspBox, 3, 0)
        self.setLayout(gridLayout)

    @QtCore.pyqtSlot(QtGui.QImage)
    def setStream(self, image):
        output = QtGui.QTransform().rotate(-90)
        image = image.transformed(output)
        self.rgblabel.setPixmap(QtGui.QPixmap.fromImage(image))

    def createCameraGrid(self):
        self.cameraBox = QGroupBox('')
        cameraBoxLayout = QGridLayout()

        self.rgblabel = QLabel()
        self.cameraFeed.changePixmap.connect(self.setStream)
        self.rgblabel.setAlignment(QtCore.Qt.AlignCenter)
        self.rgblabel.adjustSize()
        cameraBoxLayout.addWidget(self.rgblabel)
        self.cameraBox.setLayout(cameraBoxLayout)

    def createArmMoveGrid(self):
        self.armMoveBox = QGroupBox('Arm Movement')
        armMoveBoxLayout = QGridLayout()

        #arm_roll_joint
        arm_rotate_right = QPushButton('Right')
        arm_rotate_right.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        armMoveBoxLayout.addWidget(arm_rotate_right, 1, 0)
        arm_rotate_right.pressed.connect(self.armMove.clicked_right_armRoll)
        arm_rotate_right.setAutoRepeat(True)

        arm_rotate_left = QPushButton('Left')
        arm_rotate_left.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        armMoveBoxLayout.addWidget(arm_rotate_left, 0, 0)
        arm_rotate_left.pressed.connect(self.armMove.clicked_left_armRoll)
        arm_rotate_left.setAutoRepeat(True)

        #arm_flex_joint (elbow)
        arm_flex_up = QPushButton('Elbow up')
        arm_flex_up.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        armMoveBoxLayout.addWidget(arm_flex_up, 0, 1)
        arm_flex_up.pressed.connect(self.armMove.clicked_up_armFlex)
        arm_flex_up.setAutoRepeat(True)

        arm_flex_down = QPushButton('Elbow down')
        arm_flex_down.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        armMoveBoxLayout.addWidget(arm_flex_down, 1, 1)
        arm_flex_down.pressed.connect(self.armMove.clicked_down_armFlex)
        arm_flex_down.setAutoRepeat(True)

        self.armMoveBox.setLayout(armMoveBoxLayout)
        self.armMoveBox.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')



    def createWristMoveGrid(self):
        self.wristMoveBox = QGroupBox('Hand')
        wristMoveBoxLayout = QGridLayout()

        #wrist_roll_joint
        wrist_rotate_right = QPushButton('>')
        wrist_rotate_right.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        wristMoveBoxLayout.addWidget(wrist_rotate_right, 1, 2)
        wrist_rotate_right.pressed.connect(self.armMove.clicked_right_wristRoll)
        wrist_rotate_right.setAutoRepeat(True)

        wrist_rotate_left = QPushButton('<')
        wrist_rotate_left.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        wristMoveBoxLayout.addWidget(wrist_rotate_left, 1, 0)
        wrist_rotate_left.pressed.connect(self.armMove.clicked_left_wristRoll)
        wrist_rotate_left.setAutoRepeat(True)

        #wrist_flex_joint
        wrist_rotate_up = QPushButton('Up')
        wrist_rotate_up.setStyleSheet("color: black; background-color: rgb(153, 204,255)")
        wristMoveBoxLayout.addWidget(wrist_rotate_up, 0, 1)
        wrist_rotate_up.pressed.connect(self.armMove.clicked_up_wristFlex)
        wrist_rotate_up.setAutoRepeat(True)

        wrist_rotate_down = QPushButton('Down')
        wrist_rotate_down.setStyleSheet("color: black; background-color: rgb(153, 204,255)")
        wristMoveBoxLayout.addWidget(wrist_rotate_down, 2, 1)
        wrist_rotate_down.pressed.connect(self.armMove.clicked_down_wristFlex)
        wrist_rotate_down.setAutoRepeat(True)



        self.wristMoveBox.setLayout(wristMoveBoxLayout)
        self.wristMoveBox.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')

    def createGraspGrid(self):
        self.graspBox = QGroupBox('')
        graspBoxLayout = QGridLayout()

        grasp = QPushButton('Grasp')
        grasp.setStyleSheet("color: white; background-color: rgb(64, 25, 255)")
        graspBoxLayout.addWidget(grasp, 0, 0)
        grasp.pressed.connect(self.armMove.clicked_hand_close)
        grasp.setAutoRepeat(True)

        release = QPushButton('Release')
        release.setStyleSheet("color: black; background-color: rgb(204, 153, 255)")
        graspBoxLayout.addWidget(release, 1, 0)
        release.pressed.connect(self.armMove.clicked_hand_open)
        release.setAutoRepeat(True)

        self.graspBox.setLayout(graspBoxLayout)
        self.graspBox.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')