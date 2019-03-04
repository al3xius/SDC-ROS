#!/usr/bin/env python
import os
import os.path
import rospy
import glob
import datetime
import shutil
import cv2
from cv_bridge import CvBridge, CvBridgeError
from kivy.app import App
from kivy.clock import Clock
from kivy.core.window import Window
from kivy.lang import Builder
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.uix.camera import Camera
from kivy.uix.image import Image
from twisted.internet import task
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.properties import ObjectProperty
from kivy.graphics.vertex_instructions import Ellipse
from kivy.graphics.context_instructions import Color
from kivy.core.window import Window
from kivy.uix.screenmanager import SlideTransition
from kivy.garden.mapview import MapView
from kivy.graphics.texture import Texture
from kivy.graphics import Color, Rectangle

# ROS imports
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image as ROSImage
from sdc_msgs.msg import state
from rosgraph_msgs.msg import Log


# KV_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), 'ui'))

# Set Window size and other Variables
win_x = 1024
win_y = 600
Window.size = (win_x, win_y)

# MAP:
# Koordinaten (Weiz)
cur_lat = 47.224282
cur_lon = 15.6233008
zoomLevel = 18

# IMAGES:
camImgRos = 0
laneImgRos = 0
objImgRos = 0
showLanes = False
showObjects = False

# CAR DATA:
curspeed = 0

# Screen Manager
sm = ScreenManager()
Window.fullscreen = False


# ROS functions
def gpsCallback(msg):
    if msg.status > 0:
        cur_lat = msg.latitude
        cur_lon = msg.longitude


def laneCallback(msg):
    try:
        lane_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        global laneImgRos
        laneImgRos = lane_image
    except CvBridgeError as e:
        rospy.loginfo(e)


def camCallback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        global camImgRos
        camImgRos = cv_image
    except CvBridgeError as e:
        rospy.loginfo(e)


def objCallback(msg):
    try:
        obj_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        global objImgRos
        objImgRos = obj_img
    except CvBridgeError as e:
        rospy.loginfo(e)


def logCallback(msg):
    self.log = msg


def stateCallback(msg):
    state = msg
    global curspeed
    curspeed = state.velocity


class ScreenMAP(Screen):
    def on_enter(self):
        # Map with Zoom and Coordinates
        mapview = MapView(zoom=18, lat=cur_lat, lon=cur_lon)
                          # TODO: use ros variables for coordinates

        # Back to Menu Button
        def changeScreen(self):
            sm.transition = SlideTransition(direction='right')
            sm.current = "MNS"

        backBtn = Button(text="[b]BACK[/b]", font_size="20sp", pos=(0,
                         0), size_hint=(.2, .1), background_color=(1, 1, 1, 0.45), markup=True)
        backBtn.bind(on_press=changeScreen)

        # Add to Map layout
        self.add_widget(mapview)
        self.add_widget(backBtn)

    def on_leave(self):
        self.clear_widgets()


class ScreenCAV(Screen):
    def update(self, dt):
        self.clear_widgets()

        # Back to Menu Button
        def changeScreen(self):
            sm.transition = SlideTransition(direction='right')
            sm.current = "MNS"

        backBtn = Button(text="[b]BACK[/b]", font_size="20sp",
                         pos=(0, 0), size_hint=(.3, .12), markup=True)
        backBtn.bind(on_press=changeScreen)

        # Lanes Lines
        buf1 = cv2.flip(laneImgRos, 0)
        buf = buf1.tostring()
        lane_texture = Texture.create(
            size=(laneImgRos.shape[1], laneImgRos.shape[0]), colorfmt='bgr')
        lane_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
        # display image from the texture
        self.texture = lane_texture

        def toggleLanes(self):
            global showLanes
            showLanes = not showLanes

        laneBtn = Button(
            text="[b]LANE LINES[/b]", font_size="20sp", pos=(win_x/2-(win_x*0.15),
                                                             0), size_hint=(.3, .12), markup=True)
        laneBtn.bind(on_press=toggleLanes)

        # Objects
        buf1 = cv2.flip(objImgRos, 0)
        buf = buf1.tostring()
        object_texture = Texture.create(
            size=(objImgRos.shape[1], objImgRos.shape[0]), colorfmt='bgr')
        object_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
        # display image from the texture
        self.texture = object_texture

        def toggleObjects(self):
            global showObjects
            showObjects = not showObjects

        objButton = Button(
            text="[b]LANE LINES[/b]", font_size="20sp", pos=(win_x/2,
                                                             0), size_hint=(.3, .12), markup=True)
        objButton.bind(on_press=toggleObjects)

        # Cam Feed
        buf1 = cv2.flip(camImgRos, 0)
        buf = buf1.tostring()
        image_texture = Texture.create(
            size=(camImgRos.shape[1], camImgRos.shape[0]), colorfmt='bgr')
        image_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
        # display image from the texture
        self.texture = image_texture

        with self.canvas:
            Rectangle(texture=image_texture, pos=(0, 0), size=(win_x, win_y))
            global showLanes
            if showLanes:
                Rectangle(
                    texture=lane_texture, pos=(0, 0), size=(win_x, win_y))
            if showObjects:
                Rectangle(
                    texture=object_texture, pos=(0, 0), size=(win_x, win_y))

        # Add to Screen
        self.add_widget(backBtn)
        self.add_widget(laneBtn)

    def on_enter(self):
        t = 1.0
        fps = 10
        Clock.schedule_interval(self.update, t / fps)

    def on_leave(self):
        self.clear_widgets()


class ScreenMNS(Screen):
    def update(self, dt):
        # rospy.loginfo("tick")
        self.clear_widgets()

        curTime = datetime.datetime.now()

        timeString = curTime.strftime("%H:%M")
        timeLbl = Label(
            text='[color=111111] %s [/color]' % timeString, pos=(win_x/2-70, win_y/2-20), font_size='30dp', markup=True)

        dateString = curTime.strftime("%d, %b.")
        dateLbl = Label(text='[color=111111] %s [/color]' %
                        dateString, font_size='20dp', pos=(win_x/2-70, win_y/2-45), markup=True)

        # TODO: Ausgeben ans topic
        def decSpeed(self):
            global curspeed
            if curspeed > 0:
                curspeed -= 1

        def incSpeed(self):
            global curspeed
            if curspeed < 20:
                curspeed += 1

        # Change Speed
        speedLbl = Label(
            text="[color=111111][b]%d[/b][/color]" % curspeed,
                        pos_hint={'top': 1.1}, font_size="120dp", markup=True)

        speedMinus = Button(
            text="[color=111111][b]-[/b][/color]", pos=(330, 280),
                          size_hint=(.15, .2), font_size="100dp", markup=True, background_color=(0, 0, 0, 0))
        speedMinus.bind(on_press=decSpeed)

        speedPlus = Button(
            text="[color=111111][b]+[/b][/color]", pos=(550, 280),
                         size_hint=(.15, .2), font_size="80dp", markup=True, background_color=(0, 0, 0, 0))
        speedPlus.bind(on_press=incSpeed)

        einheitLbl = Label(text="[color=111111][b]km/h[/b][/color]", pos_hint={'top':
                                                                               0.97}, font_size="45dp", markup=True)

        # DrivingMode:
        scale = 60

        #TODO: change mode from topic input
        def changeMode(mode, curDir):
            actColor = "ffd700"
            stdColor = "111111"
            # Park
            if curspeed == 0:
                pColor = actColor
                rColor = stdColor
                mColor = stdColor
                jColor = stdColor
                cColor = stdColor
            # Return
            elif curDir == -1:
                pColor = stdColor
                rColor = actColor
                mColor = stdColor
                jColor = stdColor
                cColor = stdColor
            # Manual
            elif mode == "manual":
                pColor = stdColor
                rColor = stdColor
                mColor = actColor
                jColor = stdColor
                cColor = stdColor
            # Joy
            elif mode == "joy":
                pColor = stdColor
                rColor = stdColor
                mColor = stdColor
                jColor = actColor
                cColor = stdColor
            # Cruise Control
            elif mode == "cruise":
                pColor = stdColor
                rColor = stdColor
                mColor = stdColor
                jColor = actColor
                cColor = stdColor
            # Nothing active
            else:
                pColor = stdColor
                rColor = stdColor
                mColor = stdColor
                jColor = stdColor
                cColor = stdColor

            pLbl = Label(
                text="[color=%s]P[/color]" % pColor, pos=(win_x/2-120, win_y/2-130), font_size='45dp', markup=True)
            rLbl = Label(
                text="[color=%s]R[/color]" % rColor, pos=(win_x/2-120, win_y/2-130-(scale)), font_size='45dp', markup=True)
            mLbl = Label(
                text="[color=%s]M[/color]" % mColor, pos=(win_x/2-120, win_y/2-130-(scale*2)), font_size='45dp', markup=True)
            jLbl = Label(
                text="[color=%s]J[/color]" % jColor, pos=(win_x/2-120, win_y/2-130-(scale*3)), font_size='45dp', markup=True)
            cLbl = Label(
                text="[color=%s]C[/color]" % cColor, pos=(win_x/2-120, win_y/2-130-(scale*4)), font_size='45dp', markup=True)

            self.add_widget(pLbl)
            self.add_widget(rLbl)
            self.add_widget(mLbl)
            self.add_widget(jLbl)
            self.add_widget(cLbl)

        changeMode(state.mode, state.direction)

        # MenuButtons
        def screenMap(self):
            sm.transition = SlideTransition(direction='left')
            sm.current = "MAP"

        def screenCam(self):
            sm.transition = SlideTransition(direction='left')
            sm.current = "CAV"

        mapBtn = Button(
            text="[b]MAP[/b]", font_size="20sp", pos=(0,
                                                      0), size_hint=(.3, .12), markup=True)
        mapBtn.bind(on_press=screenMap)

        camBtn = Button(
            text="[b]CAM[/b]", font_size="20sp", pos=(win_x/2-(win_x*0.15),
                                                      0), size_hint=(.3, .12), markup=True)
        camBtn.bind(on_press=screenCam)

        # Lights
        def switchLights(left, right, light):
            # left = "left-arrow"
            leftPath = '../assets/data/%s.png' % left
            rightPath = '../assets/data/%s.png' % right
            lightPath = '../assets/data/%s.png' % light

            leftImg = Image(
                source=leftPath, size_hint=(.07, .07), pos=(win_x/2-120, win_y/2-130))
            rightImg = Image(
                source=rightPath, size_hint=(.07, .07), pos=(win_x/2+55, win_y/2-130))
            lightImg = Image(
                source=lightPath, size_hint=(.07, .07), pos=(win_x/2-30, win_y/2-130))

            self.add_widget(leftImg)
            self.add_widget(rightImg)
            self.add_widget(lightImg)

        switchLights("left-arrow", "right-arrow", "car-light-act")

        self.add_widget(speedMinus)
        self.add_widget(speedPlus)
        self.add_widget(mapBtn)
        self.add_widget(camBtn)
        self.add_widget(timeLbl)
        self.add_widget(speedLbl)
        self.add_widget(dateLbl)

    def on_enter(self):
        t = 1.0
        Clock.schedule_interval(self.update, t)

    def on_leave(self):
        self.clear_widgets()


sm.add_widget(ScreenMNS(name='MNS'))
sm.add_widget(ScreenCAV(name='CAV'))
sm.add_widget(ScreenMAP(name='MAP'))


# Run
class MyApp(App):

    def build(self):
        self.title = 'CONTROL PANEL | v 0.2'
        Window.clearcolor = (1, 1, 1, 1)
        # self.icon = 'assets/car.png'
        return sm

if __name__ == '__main__':
    # subscriber
    rospy.init_node('gui', anonymous=False)
    rospy.loginfo("GUI: Node started.")

    bridge = CvBridge()

    gps_sub = rospy.Subscriber('/gps', NavSatFix, gpsCallback)
    lane_sub = rospy.Subscriber('/lane/combinedImage', ROSImage, laneCallback)
    cam_sub = rospy.Subscriber('/usb_cam/image_raw', ROSImage, camCallback)
    obj_sub = rospy.Subscriber(
        '/objectDedector/overlayImage', ROSImage, objCallback)
    state_sub = rospy.Subscriber("/state", state, stateCallback)

    rosout_sub = rospy.Subscriber("/rosout_agg", Log, logCallback)

    state_pub = rospy.Publisher("/gui/state", state, queue_size=1)

    MyApp().run()

    try:
        MyApp().run()
    except rospy.ROSInterruptException:
        App.get_running_app().stop()
        import sys
        sys.exit()
