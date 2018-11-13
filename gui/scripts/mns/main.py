#!/usr/bin/env python
import os, os.path, rospy
import glob
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.uix.camera import Camera
from kivy.core.image import Image
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.properties import ObjectProperty
from kivy.graphics.vertex_instructions import Ellipse
from kivy.graphics.context_instructions import Color
from kivy.core.window import Window
from kivy.uix.screenmanager import SlideTransition
from kivy.garden.mapview import MapView
import shutil

#ROS imports
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image

#TODO: sortieren / kommentieren / auslagern
#KV_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), 'ui'))
#Menu Buttons !!!Bitte immer relativen Pfad angben!!!
Builder.load_file("mns.kv")
Builder.load_file("../opt/opt.kv")
Builder.load_file("../ccd/ccd.kv")

# Set Window size and other Variables
#TODO: 16:9 & touch mit display und rpi testen
win_x = 640
win_y = 480
Window.size = (win_x, win_y)
zoomLevel = 18
#Weiz
cur_lat = 47.224282
cur_lon = 15.6233008
cam = None

# Screen Manager
sm = ScreenManager()
Window.fullscreen = True

def gpsCallback(msg):
    if msg.status > 0:
        cur_lat = msg.latitude
        cur_lon = msg.longitude

def laneCallback(msg):
    lane_img = msg

def camCallback(msg):
    cam_img = msg

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
    """
    # Delete Map Cache
    def deleteCache(self):
        files = glob.glob(
            '/home/davidzechm/catkin_ws/src/gui/scripts/cache/*')
        i = 0
        for f in files:
            try:
                os.remove(f)
                i = i+1
                print("Deleting File: " + str(i))
            except IOError:
                print("Error")
    """
    pass


class ScreenCCD(Screen):
    pass


class ScreenCAV(Screen):
    def on_enter(self):
        """"
        #Get Camera image
        cam = Camera(
            play=False, index=0, resolution=(win_x, win_y), allow_stretch=True, keep_ratio=False)
            #TODO: Overlays
        cam.play = True"""
        cam = Image(source="26a7511794_18_142445_170178.png").texture

        # Back to Menu Button
        def changeScreen(self):
            sm.transition = SlideTransition(direction='right')
            sm.current = "MNS"

        backBtn = Button(text="[b]BACK[/b]", font_size="20sp", pos=(0,
                         0), size_hint=(.2, .1), background_color=(1, 1, 1, 0.45), markup=True)
        backBtn.bind(on_press=changeScreen)

        # Overlay
        overlayBtn = Button(
            text="[b]TOGGLE OVERLAY[/b]", font_size="20sp", pos=(200,
                                                                 0), size_hint=(.2, .1), background_color=(1, 1, 1, 0.45), markup=True)
        overlayBtn.bind(on_press=changeScreen)

        # Screenshot
        screenBtn = Button(
            text="[b]SCREENSHOT[/b]", font_size="20sp", pos=(400,
                                                             0), size_hint=(.2, .1), background_color=(1, 1, 1, 0.45), markup=True)
        screenBtn.bind(on_press=changeScreen)

        # Add to Screen
        self.add_widget(cam)
        self.add_widget(backBtn)
        self.add_widget(overlayBtn)
        self.add_widget(screenBtn)

    def on_leave(self):
        self.clear_widgets()


class ScreenMNS(Screen):
    pass


sm.add_widget(ScreenMNS(name='MNS'))
sm.add_widget(ScreenOPT(name='OPT'))
sm.add_widget(ScreenCCD(name='CCD'))
sm.add_widget(ScreenCAV(name='CAV'))
sm.add_widget(ScreenMAP(name='MAP'))


# Run
class MyApp(App):

    def build(self):
        self.title = 'CONTROL PANEL | v 0.2'
        #self.icon = 'assets/car.png'
        return sm

if __name__ == '__main__':
    #subscriber
    gps_sub = rospy.Subscriber('/gps', NavSatFix, gpsCallback)
    lane_sub = rospy.Subscriber('/lane/combinedImage', Image, laneCallbac)
    cam_sub = rospy.Subscriber('/usb_cam/image_raw', Image, camCallbac)
    MyApp().run()
