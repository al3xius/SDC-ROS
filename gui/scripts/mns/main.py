#!/usr/bin/env python
import os
import os.path
import rospy
import glob
import datetime
import shutil
from kivy.app import App
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

#ROS imports
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image as ROSImage

#TODO: sortieren / kommentieren / auslagern
#KV_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), 'ui'))

# Set Window size and other Variables
#TODO: 16:9 & touch mit display und rpi testen
win_x = 640
win_y = 480
Window.size = (win_x, win_y)
# MAP:
# Koordinaten (Weiz)
cur_lat = 47.224282
cur_lon = 15.6233008
zoomLevel = 18
# CAR DATA:
curr_speed = 12

# Screen Manager
sm = ScreenManager()
#Window.fullscreen = True


def gpsCallback(msg):
    if msg.status > 0:
        cur_lat = msg.latitude
        cur_lon = msg.longitude


def laneCallback(msg):
    lane_img = msg


def camCallback(msg):
    cam_img = msg


def objCallback(msg):
    obj_img = msg


class ScreenMAP(Screen):
    def on_enter(self):
        # Map with Zoom and Coordinates
        mapview = MapView(zoom=18, lat=cur_lat, lon=cur_lon)
                          #TODO: use ros variables for coordinates

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


class ScreenOPT(Screen):
    def on_enter(self):

        # Back to Menu Button
        def changeScreen(self):
            sm.transition = SlideTransition(direction='right')
            sm.current = "MNS"

        backBtn = Button(text="[b]BACK[/b]", font_size="20sp", pos=(0,
                         0), size_hint=(.2, .1), background_color=(1, 1, 1, 0.45), markup=True)
        backBtn.bind(on_press=changeScreen)


class ScreenCCD(Screen):
    pass


class ScreenCAV(Screen):
    def on_enter(self):
        # Back to Menu Button
        def changeScreen(self):
            sm.transition = SlideTransition(direction='right')
            sm.current = "MNS"

        backBtn = Button(text="[b]BACK[/b]", font_size="20sp", pos=(0,
                         0), size_hint=(.2, .1), background_color=(1, 1, 1, 0.45), markup=True)
        backBtn.bind(on_press=changeScreen)

        # Overlay
        def toggleOverlay(self):
            on = 0
            if on == 0:
                overlayImg = Image(
                    source="../assets/data/autobahn.jpg", width=win_x, height=win_y)
                self.add_widget(overlayImg)
                on = 1
            else:
                self.clear_widgets()
                on = 0

        overlayBtn = Button(
            text="[b]OVERLAY[/b]", font_size="20sp", pos=(200,
                                                          0), size_hint=(.2, .1), background_color=(1, 1, 1, 0.45), markup=True)
        overlayBtn.bind(on_press=toggleOverlay)

        # Add to Screen
        self.add_widget(backBtn)
        self.add_widget(overlayBtn)

    def on_leave(self):
        self.clear_widgets()


class ScreenMNS(Screen):
    def on_enter(self):
        # Get Current Time and Date
        curTime = datetime.datetime.now()

        timeString = curTime.strftime("%H:%M")
        timeLbl = Label(
            text='[color=111111] %s [/color]' % timeString, pos=(win_x/2-70, win_y/2-20), font_size='30dp', markup=True)

        dateString = curTime.strftime("%d, %b.")
        dateLbl = Label(text='[color=111111] %s [/color]' %
                        dateString, font_size='20dp', pos=(win_x/2-70, win_y/2-45), markup=True)

        # Show Speed
        speedLbl = Label(
            text="[color=111111][b]%d[/b][/color]" % curr_speed,
                         pos_hint={'top': 1.1}, font_size="90dp", markup=True)

        # DrivingMode:
        scale = 60

        def changeMode(mode):
            actColor = "ffd700"
            stdColor = "111111"
            if mode == "p":
                pColor = actColor
                rColor = stdColor
                nColor = stdColor
                dColor = stdColor
            elif mode == "r":
                pColor = stdColor
                rColor = actColor
                nColor = stdColor
                dColor = stdColor
            elif mode == "n":
                pColor = stdColor
                rColor = stdColor
                nColor = actColor
                dColor = stdColor
            elif mode == "d":
                pColor = stdColor
                rColor = stdColor
                nColor = stdColor
                dColor = actColor
            else:
                pColor = stdColor
                rColor = stdColor
                nColor = stdColor
                dColor = stdColor

            pLbl = Label(
                text="[color=%s]P[/color]" % pColor, pos=(win_x/2-120, win_y/2-130), font_size='45dp', markup=True)
            rLbl = Label(
                text="[color=%s]R[/color]" % rColor, pos=(win_x/2-120, win_y/2-130-(scale)), font_size='45dp', markup=True)
            nLbl = Label(
                text="[color=%s]N[/color]" % nColor, pos=(win_x/2-120, win_y/2-130-(scale*2)), font_size='45dp', markup=True)
            dLbl = Label(
                text="[color=%s]D[/color]" % dColor, pos=(win_x/2-120, win_y/2-130-(scale*3)), font_size='45dp', markup=True)

            self.add_widget(pLbl)
            self.add_widget(rLbl)
            self.add_widget(nLbl)
            self.add_widget(dLbl)

        changeMode("d")

        # MenuButtons
        def screenMap(self):
            sm.transition = SlideTransition(direction='left')
            sm.current = "MAP"

        def screenOpt(self):
            sm.transition = SlideTransition(direction='left')
            sm.current = "OPT"

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

        setBtn = Button(
            text="[b]SETTINGS[/b]", font_size="20sp", pos=(win_x-(win_x*0.3),
                                                           0), size_hint=(.3, .12), markup=True)
        setBtn.bind(on_press=screenOpt)

        # Lights
        def switchLights(left, right, light):
            #left = "left-arrow"
            leftPath = 'scripts/assets/data/%s.png' % left
            rightPath = 'scripts/assets/data/%s.png' % right
            lightPath = 'scripts/assets/data/%s.png' % light

            leftImg = Image(
                source=leftPath, size_hint=(.07, .07), pos=(win_x/2-100, win_y/2-100))
            rightImg = Image(
                source=rightPath, size_hint=(.07, .07), pos=(win_x/2+60, win_y/2-100))
            lightImg = Image(
                source=lightPath, size_hint=(.07, .07), pos=(win_x/2-20, win_y/2-100))

            self.add_widget(leftImg)
            self.add_widget(rightImg)
            self.add_widget(lightImg)

        switchLights("left-arrow", "right-arrow", "car-light-act")

        self.add_widget(dateLbl)
        self.add_widget(timeLbl)
        self.add_widget(speedLbl)
        self.add_widget(mapBtn)
        self.add_widget(camBtn)
        self.add_widget(setBtn)

    def on_leave(self):
        self.clear_widgets()


sm.add_widget(ScreenMNS(name='MNS'))
sm.add_widget(ScreenOPT(name='OPT'))
sm.add_widget(ScreenCCD(name='CCD'))
sm.add_widget(ScreenCAV(name='CAV'))
sm.add_widget(ScreenMAP(name='MAP'))


# Run
class MyApp(App):

    def build(self):
        self.title = 'CONTROL PANEL | v 0.2'
        Window.clearcolor = (1, 1, 1, 1)
        #self.icon = 'assets/car.png'
        return sm

if __name__ == '__main__':
    #subscriber
<<<<<< < HEAD
    gps_sub = rospy.Subscriber('/gps', NavSatFix, gpsCallback)
    lane_sub = rospy.Subscriber('/lane/combinedImage', ROSImage, laneCallback)
    cam_sub = rospy.Subscriber('/usb_cam/image_raw', ROSImage, camCallback)
    MyApp().run()
== == == =
    rospy.init_node('gui', anonymous=False)
    rospy.loginfo("GUI: Node started.")
    gps_sub = rospy.Subscriber('/gps', NavSatFix, gpsCallback)
    lane_sub = rospy.Subscriber('/lane/combinedImage', ROSImage, laneCallback)
    cam_sub = rospy.Subscriber('/usb_cam/image_raw', ROSImage, camCallback)
    obj_sub = rospy.Subscriber(
        '/objectDedector/overlayImage', ROSImage, objCallback)
    try:
        MyApp().run()
    except rospy.ROSInterruptException:
        #TODO: correct closing
        App.get_running_app().stop()
        import sys
        sys.exit()

>>>>>> > e720053911574f5f0c7325d407d8651b0c3faee0
