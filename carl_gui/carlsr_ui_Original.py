#Author: H J Kashyap and T Hwu
import sys
from PyQt5.QtWidgets import QApplication, QDialog, QGridLayout, QGroupBox, QPushButton, QLabel
from PyQt5 import QtGui, QtCore
import os
import xtion_channels, button_control, arm_control, map_navigation, gripper_control, speech_control
import rospy
import yaml
import hsrb_interface
from get_click_xyz import Get3Dcoordinates
import math
import stereo_cam_channels
import hand_control_ui
import high_res_cam_ui
from hsrb_interface import geometry
import grasping_ui

class Thread(QtCore.QThread):
    def run(self):
        with open("param.yaml", "r") as f:
            param = yaml.load(f)
        QtCore.QThread.sleep(param["rotate_90"]["time_to_rotate"])


class ClientUI(QDialog):

    def __init__(self):
        super(ClientUI, self).__init__()
        self.title = 'CARL-SR Interface'
        self.robot = hsrb_interface.Robot()
        self.whole_body = self.robot.get('whole_body')
        self.omni_base = self.robot.get('omni_base')
        self.gripper = self.robot.get('gripper')

        self.left = 500
        self.top = 100
        self.width =1350
        self.height = 900
        self.active_reach_target = False
        self.current_cam = 0  # 0 for regular RGB camera and 1 for high resolution camera

        self.logopath = os.path.join(os.path.curdir, 'image', 'carl-logo.jpg')
        self.setWindowIcon(QtGui.QIcon(self.logopath))
        self.headMove = button_control.HeadControl()
        self.gripperMove = gripper_control.GripperControl()
        self.armMove = arm_control.ArmControl()
        self.xtionCameraFeed = xtion_channels.rgbOut()
        self.highresCameraFeed = stereo_cam_channels.rgbOut()
        self.baseMove = button_control.BaseControl()
        self.basePose = button_control.BaseTrajectoryControl()
        self.autodrive = map_navigation.AutoNavigate()
        self.find3D = Get3Dcoordinates()
        self.speech = speech_control.SpeechControl()
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.setFixedSize(self.width, self.height)
        self.createGridLayout()
        self.show()

    def createGridLayout(self):
        self.createCameraGrid()
        self.createBaseHeadMoveGrid()
        self.createArmTrunkMoveGrid()
        self.createMapGrid()
        self.createZoomPopGrid()
        self.createClick2ReachGrid()
        gridLayout = QGridLayout()
        gridLayout.addWidget(self.cameraBox, 0, 0)
        gridLayout.addWidget(self.zoomPopGrid, 2, 0)
        gridLayout.addWidget(self.click2reachGrid, 1, 0)
        gridLayout.addWidget(self.mapBox, 0, 1)
        gridLayout.addWidget(self.baseHeadMoveGrid, 1, 1)
        gridLayout.addWidget(self.armTrunkMoveGrid, 2, 1)
        self.setLayout(gridLayout)

    @QtCore.pyqtSlot(QtGui.QImage)
    def setXtionStream(self, image):
        #self.rgblabel.clear()
        self.rgblabel.setPixmap(QtGui.QPixmap.fromImage(image))

    @QtCore.pyqtSlot(QtGui.QImage)
    def setHighResStream(self, image):
        #self.rgblabel.clear()
        self.rgblabel.setPixmap(QtGui.QPixmap.fromImage(image))

    def createBaseHeadMoveGrid(self):
        self.baseHeadMoveGrid = QGroupBox('')
        baseHeadGridLayout = QGridLayout()
        self.createBaseMoveGrid()
        self.createHeadMoveGrid()
        baseHeadGridLayout.addWidget(self.baseMoveGrid, 0, 0)
        baseHeadGridLayout.addWidget(self.headMoveGrid, 0, 1)
        self.baseHeadMoveGrid.setLayout(baseHeadGridLayout)

    def createArmTrunkMoveGrid(self):
        self.armTrunkMoveGrid = QGroupBox('')
        armTrunkGridLayout = QGridLayout()
        self.createArmMoveGrid()
        self.createTrunkMoveGrid()
        armTrunkGridLayout.addWidget(self.trunkMoveGrid, 0, 1)
        armTrunkGridLayout.addWidget(self.armMoveGrid, 0, 0)
        self.armTrunkMoveGrid.setLayout(armTrunkGridLayout)

    def createTrunkMoveGrid(self):
        self.trunkMoveGrid = QGroupBox('Adjust Height')
        trunkMoveGridLayout = QGridLayout()

        trunk_raise_button = QPushButton('Raise Height')
        trunk_raise_button.setStyleSheet("color: black; background-color: rgb(204, 153, 255)")
        trunkMoveGridLayout.addWidget(trunk_raise_button, 0, 0)
        trunk_raise_button.pressed.connect(self.armMove.clicked_raise_trunk)
        trunk_raise_button.clicked.connect(self.headMove.clicked_home)
        trunk_raise_button.setAutoRepeat(True)

        trunk_lower_button = QPushButton('Lower Height')
        trunk_lower_button.setStyleSheet("color: black; background-color: rgb(204, 153, 255)")
        trunkMoveGridLayout.addWidget(trunk_lower_button, 1, 0)
        trunk_lower_button.pressed.connect(self.armMove.clicked_lower_trunk)
        trunk_lower_button.setAutoRepeat(True)

        self.trunkMoveGrid.setLayout(trunkMoveGridLayout)
        self.trunkMoveGrid.setStyleSheet('QGroupBox:title {'
                                        'subcontrol-origin: margin;'
                                        'subcontrol-position: top center;'
                                        'padding-left: 10px;'
                                        'padding-right: 10px; }')

    def createArmMoveGrid(self):
        self.armMoveGrid = QGroupBox('Move Arm')
        armMoveGridLayout = QGridLayout()

        arm_raise_button = QPushButton('Raise Hand')
        arm_raise_button.setStyleSheet("color: black; background-color: rgb(204, 153, 255)")
        armMoveGridLayout.addWidget(arm_raise_button, 0, 0)
        arm_raise_button.clicked.connect(self.gripperMove.clicked_hand_gripper)
        arm_raise_button.clicked.connect(self.basePose.clicked_left_45)
        arm_raise_button.clicked.connect(self.headMove.clicked_right_45)
        arm_raise_button.clicked.connect(self.armMove.clicked_arm_raise)

        # arm_lower_button = QPushButton('Lower Hand')
        # armMoveGridLayout.addWidget(arm_lower_button, 1, 0)
        # arm_lower_button.clicked.connect(self.gripperMove.clicked_hand_gripper)
        # arm_lower_button.clicked.connect(self.basePose.clicked_right_45)
        # arm_lower_button.clicked.connect(self.headMove.clicked_left_45)
        # arm_lower_button.clicked.connect(self.armMove.clicked_arm_lower)

        body_neutral_button = QPushButton('Neutral Arm Position')
        body_neutral_button.setStyleSheet("color: black; background-color: rgb(204, 153, 255)")
        armMoveGridLayout.addWidget(body_neutral_button, 1, 0)
        body_neutral_button.clicked.connect(self.armMove.clicked_arm_neutral)
        body_neutral_button.clicked.connect(self.gripperMove.clicked_gripper_close)


        self.armMoveGrid.setLayout(armMoveGridLayout)
        self.armMoveGrid.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')
    def createMapGrid(self):
        self.mapBox = QGroupBox('Auto Drive')
        mapBoxLayout = QGridLayout()
        mapBoxLayout.addWidget(self.autodrive)
        self.mapBox.setLayout(mapBoxLayout)
        self.mapBox.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')

    def createCameraGrid(self):
        self.cameraBox = QGroupBox('')
        cameraBoxLayout = QGridLayout()

        self.rgblabel = QLabel()
        self.xtionCameraFeed.changePixmap.connect(self.setXtionStream)
        self.rgblabel.setAlignment(QtCore.Qt.AlignCenter)
        self.rgblabel.adjustSize()
        self.rgblabel.mousePressEvent = self.register_click_to_reach
        cameraBoxLayout.addWidget(self.rgblabel)
        self.cameraBox.setLayout(cameraBoxLayout)

    def createZoomPopGrid(self):
        self.zoomPopGrid = QGroupBox('')
        zoomPopGridLayout = QGridLayout()
        self.createCameraZoomGrid()
        self.createPopGrid()
        zoomPopGridLayout.addWidget(self.cameraZoomGrid, 0, 0)
        zoomPopGridLayout.addWidget(self.popGrid, 0, 1)
        self.zoomPopGrid.setLayout(zoomPopGridLayout)

    def openGrip(self):
        try:
            self.gripper.command(1.0)
        except:
            print('Failed to open gripper properly!')

    def closeGrip(self):
        try:
            self.gripper.grasp(-0.1)
        except:
            print('Failed to close gripper properly!')

    @QtCore.pyqtSlot(QPushButton)
    def buildGraspingUIPopup(self):
        popup_win = grasping_ui.HandCam(self)
        popup_win.setGeometry(100, 200, 100, 100)
        popup_win.show()

    def change_camera_to_string(self):
        if self.current_cam == 0:
            return "High Resolution Camera"
        else:
            return "Regular Camera"

    def changeCamera(self):
        if self.current_cam == 0:
            self.current_cam = 1
            self.xtionCameraFeed.changePixmap.disconnect()
            self.highresCameraFeed.reset_zoom()
            self.highresCameraFeed.changePixmap.connect(self.setHighResStream)
            self.click2pointbutton.setEnabled(False)
            self.click2pointbutton.setStyleSheet("color: white; background-color: rgb(203, 203, 203)")
        else:
            self.current_cam = 0
            self.highresCameraFeed.changePixmap.disconnect()
            self.xtionCameraFeed.reset_zoom()
            self.xtionCameraFeed.changePixmap.connect(self.setXtionStream)
            self.click2pointbutton.setEnabled(True)
            self.click2pointbutton.setStyleSheet("color: white; background-color: rgb(0, 153, 153)")

        self.change_camera_button.setText(self.change_camera_to_string())

    def createPopGrid(self):
        self.popGrid = QGroupBox('')
        popGridLayout = QGridLayout()

        # close_grip_button = QPushButton('Close Gripper')
        # gripGridLayout.addWidget(close_grip_button, 0, 0)
        # close_grip_button.pressed.connect(self.closeGrip)
        #
        # open_grip_button = QPushButton('Open Gripper')
        # gripGridLayout.addWidget(open_grip_button, 1, 0)
        # open_grip_button.pressed.connect(self.openGrip)

        self.change_camera_button = QPushButton(self.change_camera_to_string())
        self.change_camera_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        popGridLayout.addWidget(self.change_camera_button, 0, 0)
        self.change_camera_button.pressed.connect(self.changeCamera)

        ######################################################################################################
        grasp_control_button = QPushButton('Grasp Controls')
        grasp_control_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        popGridLayout.addWidget(grasp_control_button, 1, 0)
        grasp_control_button.pressed.connect(self.buildGraspingUIPopup)
        ######################################################################################################


        # hand_controls_button = QPushButton('Open Hand Controls')
        # popGridLayout.addWidget(hand_controls_button, 1, 0)
        # hand_controls_button.pressed.connect(self.buildHandControlPopup)

        self.popGrid.setLayout(popGridLayout)
        self.popGrid.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')


    def createCameraZoomGrid(self):
        self.cameraZoomGrid = QGroupBox('Zoom')
        cameraZoomGridLayout = QGridLayout()

        plus_button = QPushButton('+')
        plus_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        cameraZoomGridLayout.addWidget(plus_button, 1, 0)
        plus_button.pressed.connect(self.xtionCameraFeed.clicked_zoom_in)
        plus_button.pressed.connect(self.highresCameraFeed.clicked_zoom_in)


        minus_button = QPushButton('-')
        minus_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        cameraZoomGridLayout.addWidget(minus_button, 2, 0)
        minus_button.pressed.connect(self.xtionCameraFeed.clicked_zoom_out)
        minus_button.pressed.connect(self.highresCameraFeed.clicked_zoom_out)

        self.cameraZoomGrid.setLayout(cameraZoomGridLayout)
        self.cameraZoomGrid.setStyleSheet('QGroupBox:title {'
                                       'subcontrol-origin: margin;'
                                       'subcontrol-position: top center;'
                                       'padding-left: 10px;'
                                       'padding-right: 10px; }')

    def register_click_to_reach(self, event):
        #self.whole_body.end_effector_frame = u'hand_palm_link'
        if self.active_reach_target:
            rgb_h = event.pos().x()
            rgb_v = event.pos().y()
            print(rgb_h, rgb_v)
            self.find3D.twoD_to_threeD(rgb_h, rgb_v)
            # while True:
            #     if finder3D.found_3d:
            #         break
            if self.find3D.found_3d:
                target_x = self.find3D.map_point.point.x
                target_y = self.find3D.map_point.point.y
                target_z = self.find3D.map_point.point.z
                print('rgbd frame: ', target_x, target_y, target_z)
            else:
                print('The target location could not be found by the RGBD sensor, please try again!!')

            if math.isnan(target_x) or math.isnan(target_y) or math.isnan(target_z):
                print('The target location is not found (nan) by the RGBD sensor, try again!!')

            else:
                #current_pose = self.omni_base.get_pose()

                #if current_pose.pos.x > target_x:
                if self.autodrive.global_pose[0] > target_x:
                    x_offset = 0.5
                else:
                    x_offset = -0.5

                #if current_pose.pos.y > target_y:
                if self.autodrive.global_pose[1] > target_y:
                    y_offset = 0.5
                else:
                    y_offset = -0.5

                try:
                    self.autodrive.go_to_mapXY(target_x, x_offset, target_y, y_offset)
                except:
                    rospy.logerr('fail to reach near the target')
                else:
                    self.speech.speak('I will reach and point to the target object.')
                    self.armMove.clicked_at_target(target_z)
                    self.gripperMove.clicked_gripper_open()
            self.active_reach_target = False

    def pointRandom(self):
        self.active_reach_target = True

    def createClick2ReachGrid(self):
        self.click2reachGrid = QGroupBox('')
        click2reachLayout = QGridLayout()

        self.click2pointbutton = QPushButton('Click at Target')
        self.click2pointbutton.setStyleSheet("color: white; background-color: rgb(0, 153, 153)")
        click2reachLayout.addWidget(self.click2pointbutton, 0, 0)
        self.click2pointbutton.pressed.connect(self.pointRandom)

        self.click2reachGrid.setLayout(click2reachLayout)
        self.click2reachGrid.setStyleSheet('QGroupBox:title {'
                                          'subcontrol-origin: margin;'
                                          'subcontrol-position: top center;'
                                          'padding-left: 10px;'
                                          'padding-right: 10px; }')


    def createHeadMoveGrid(self):
        self.headMoveGrid = QGroupBox('Move Head')
        headMoveGridLayout = QGridLayout()

        up_button = QPushButton('Up')
        up_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        headMoveGridLayout.addWidget(up_button, 0, 1)        
        up_button.pressed.connect(self.headMove.clicked_up)
        up_button.setAutoRepeat(True)

        left_button = QPushButton('<')
        left_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        headMoveGridLayout.addWidget(left_button, 1, 0)
        left_button.pressed.connect(self.headMove.clicked_left)
        left_button.setAutoRepeat(True)
        
        right_button = QPushButton('>')
        right_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        headMoveGridLayout.addWidget(right_button, 1, 2)
        right_button.pressed.connect(self.headMove.clicked_right)
        right_button.setAutoRepeat(True)
        
        down_button = QPushButton('Down')
        down_button.setStyleSheet("color: black; background-color: rgb(153, 204, 255)")
        headMoveGridLayout.addWidget(down_button, 2, 1)
        down_button.pressed.connect(self.headMove.clicked_down)
        down_button.setAutoRepeat(True)

        home_button = QPushButton('Reset')
        home_button.setStyleSheet("color: white; background-color: rgb(0, 0, 255)")
        headMoveGridLayout.addWidget(home_button, 1, 1)
        home_button.clicked.connect(self.headMove.clicked_home)
        
        self.headMoveGrid.setLayout(headMoveGridLayout)
        self.headMoveGrid.setStyleSheet('QGroupBox:title {'
                 'subcontrol-origin: margin;'
                 'subcontrol-position: top center;'
                 'padding-left: 10px;'
                 'padding-right: 10px; }')

    def clicked_left_90(self):
        self.basePose.clicked_left_90()
        if not self.rotate_90_thread.isRunning():
            self.left_90_button.setEnabled(False)
            self.right_90_button.setEnabled(False)
            self.rotate_90_thread.start()

    def clicked_right_90(self):
        self.basePose.clicked_right_90()
        if not self.rotate_90_thread.isRunning():
            self.left_90_button.setEnabled(False)
            self.right_90_button.setEnabled(False)
            self.rotate_90_thread.start()

    def clicked_reverse(self):
        reverse_result = self.basePose.clicked_reverse()
        if not reverse_result:
            self.speech.speak('Cannot go backward, will collide with obstacles.')
        else:
            self.speech.speak('Going backward.')

        # self.autodrive.go_to_relXY(-0.5, 0)

    def finish_90_thread(self):
        self.left_90_button.setEnabled(True)
        self.right_90_button.setEnabled(True)

    def createBaseMoveGrid(self):
        self.baseMoveGrid = QGroupBox('Move Base')
        baseMoveGridLayout = QGridLayout()

        forward_button = QPushButton('^')
        forward_button.setStyleSheet("color: black; background-color: rgb(153, 255, 153)")
        baseMoveGridLayout.addWidget(forward_button, 0, 1)
        forward_button.pressed.connect(self.baseMove.clicked_forward)
        forward_button.released.connect(self.baseMove.released)
        forward_button.setAutoRepeat(True)
        forward_button.setAutoRepeatDelay(0)

        left_button = QPushButton('<')
        left_button.setStyleSheet("color: black; background-color: rgb(153, 255, 153)")
        baseMoveGridLayout.addWidget(left_button, 1, 0)
        left_button.pressed.connect(self.baseMove.clicked_left)
        left_button.released.connect(self.baseMove.released)
        left_button.setAutoRepeat(True)

        right_button = QPushButton('>')
        right_button.setStyleSheet("color: black; background-color: rgb(153, 255, 153)")
        baseMoveGridLayout.addWidget(right_button, 1, 2)
        right_button.pressed.connect(self.baseMove.clicked_right)
        right_button.released.connect(self.baseMove.released)
        right_button.setAutoRepeat(True)

        self.rotate_90_thread = Thread()
        self.rotate_90_thread.finished.connect(self.finish_90_thread)

        self.left_90_button = QPushButton('<<')
        self.left_90_button.setStyleSheet("color: black; background-color: rgb(30,205,50)")
        baseMoveGridLayout.addWidget(self.left_90_button, 2, 0)
        self.left_90_button.clicked.connect(self.clicked_left_90)

        self.right_90_button = QPushButton('>>')
        self.right_90_button.setStyleSheet("color: black; background-color: rgb(30,205,50)")
        baseMoveGridLayout.addWidget(self.right_90_button, 2, 2)
        self.right_90_button.clicked.connect(self.clicked_right_90)

        self.reverse_button = QPushButton('v')
        self.reverse_button.setStyleSheet("color: black; background-color: rgb(30,205,50)")
        baseMoveGridLayout.addWidget(self.reverse_button, 3, 1)
        self.reverse_button.clicked.connect(self.clicked_reverse)

        self.baseMoveGrid.setLayout(baseMoveGridLayout)
        self.baseMoveGrid.setStyleSheet('QGroupBox:title {'
                 'subcontrol-origin: margin;'
                 'subcontrol-position: top center;'
                 'padding-left: 10px;'
                 'padding-right: 10px; }')

    def shutdown(self):
        rospy.loginfo("Stopping the robot.....")
        self.baseMove.shutdown()
        self.headMove.shutdown()
        self.basePose.shutdown()

def main():
    app = QApplication(sys.argv)
    ex = ClientUI()
    rospy.on_shutdown(ex.shutdown)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
