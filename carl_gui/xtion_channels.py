import cv2
import rospy
from PyQt5 import QtGui, QtCore
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage


class rgbOut(QtCore.QObject):
    """Get video data from the HSR"""
    changePixmap = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self):
        super(rgbOut, self).__init__()
        # used to be '/hsrb/head_rgbd_sensor/rgb/image_raw'
        # but it's too slow over UCI wifi, so doing a resize before sending :)
        topic_name = '/rgbd_resize/image_proc_resize/image/compressed'
        self._bridge = CvBridge()
        self._input_image = None
        self.zoom_state = 1
        self.maxzoom = 4
        self.minzoom = 1
        
        # Subscribe color image data from HSR
        self._image_sub = rospy.Subscriber(topic_name, CompressedImage, self._color_image_cb)
        # Wait until connection
        rospy.wait_for_message(topic_name, CompressedImage, timeout=15.0)

    def _color_image_cb(self, data):

        try:
            self._input_image = self._bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            rgbImage = cv2.cvtColor(self._input_image, cv2.COLOR_BGR2RGB)
            rgbImage = cv2.resize(rgbImage, (640, 480), interpolation=cv2.INTER_AREA)

            h = 480
            w = 640

            k = 2**(self.zoom_state - 1)
            wmin = ((k-1)*w)//(2*k)
            wmax = wmin + (w//k)

            hmin = ((k-1)*h)//(2*k)
            hmax = hmin + (h//k)

            cropped_img = rgbImage[hmin:hmax, wmin:wmax]
            dst = cv2.resize(cropped_img, None, fx=k, fy=k)

            cropped_img = QtGui.QImage(dst.data, dst.shape[1], dst.shape[0],
                                             QtGui.QImage.Format_RGB888)
            self.changePixmap.emit(cropped_img)

        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def clicked_zoom_in(self):
        if self.zoom_state < self.maxzoom:
            self.zoom_state += 1

    def clicked_zoom_out(self):
        if self.zoom_state > self.minzoom:
            self.zoom_state = self.zoom_state - 1

    def reset_zoom(self):
        self.zoom_state = 1
