import cv2
import rospy
import numpy as np
from PyQt5 import QtGui, QtCore
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

gamma = 1.3
invGamma = 1.0 / gamma
gamma_table = np.array([((i / 255.0) ** invGamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")


def adjust_gamma(image, gamma=1.0):
    # build a lookup table mapping the pixel values [0, 255] to
    # their adjusted gamma values

    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)


class rgbOut(QtCore.QObject):
    """Get video data from the HSR"""
    changePixmap = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self):
        super(rgbOut, self).__init__()
        # used to be '/hsrb/head_r_stereo_camera/image_rect_color'
        # but it's too slow over UCI wifi, so doing a resize before sending :)
        topic_name = '/stereo_resize/image_proc_resize/image/compressed' 
        self._bridge = CvBridge()
        self._input_image = None
        self.zoom_state = 1
        self.maxzoom = 4
        self.minzoom = 1

        # Subscribe color image data from HSR
        self._image_sub = rospy.Subscriber(topic_name, CompressedImage, self._color_image_cb)
        # Wait until connection
        rospy.wait_for_message(topic_name, CompressedImage, timeout=20.0)

    def _color_image_cb(self, data):

        try:
            self._input_image = self._bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            rgbImage = cv2.cvtColor(self._input_image, cv2.COLOR_BGR2RGB)
            rgbImage = cv2.resize(rgbImage, (640, 480), interpolation=cv2.INTER_AREA)

            # h = 960
            # w = 1280
            h = 480
            w = 640

            k = 2 ** (self.zoom_state - 1)
            wmin = ((k - 1) * w) / (2 * k)
            wmax = wmin + (w / k)

            hmin = ((k - 1) * h) / (2 * k)
            hmax = hmin + (h / k)

            if self.zoom_state == 1:
                cropped_img = rgbImage[20:460, 20:620]
                cropped_img = cv2.resize(cropped_img, (640, 480), interpolation=cv2.INTER_CUBIC)
                cropped_img = cv2.normalize(cropped_img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                                                                    dtype=cv2.CV_8UC1)
                cropped_img = cv2.LUT(cropped_img, gamma_table)
            else:
                cropped_img = rgbImage[hmin:hmax, wmin:wmax]
                cropped_img = cv2.normalize(cropped_img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                                                                    dtype=cv2.CV_8UC1)
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
