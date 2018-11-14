import os
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
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.properties import ObjectProperty
from kivy.graphics.vertex_instructions import Ellipse
from kivy.graphics.context_instructions import Color
from kivy.core.window import Window
from kivy.uix.screenmanager import SlideTransition
from kivy.garden.mapview import MapView
#TODO: sortieren / kommentieren / auslagern


#Menu Buttons
Builder.load_file("mns.kv")
Builder.load_file("/home/davidzechm/catkin_ws/src/gui/scripts/opt/opt.kv")
Builder.load_file("/home/davidzechm/catkin_ws/src/gui/scripts/ccd/ccd.kv")

# Set Window size and other Variables
#TODO: 16:9 & touch mit display und rpi testen
win_x = 1024
win_y = 600
Window.size = (win_x, win_y)
zoomLevel = 18
cur_lat = 47.224282
cur_lon = 15.6233008
cur_speed = 12

# Screen Manager
sm = ScreenManager()


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
        #Get Camera image
        cam = Camera(
            play=False, index=0, resolution=(win_x, win_y), allow_stretch=True, keep_ratio=False)
            #TODO: Overlays
        cam.play = True

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
    def on_enter(self):
        # Get Current Time and Date
        curTime = datetime.datetime.now()

        timeString = curTime.strftime("%H:%M")
        timeLbl = Label(
            text='[color=111111] %s [/color]' % timeString, pos=(win_x/2-70, win_y/2-40), font_size='30dp', markup=True)
        self.add_widget(timeLbl)

        dateString = curTime.strftime("%d, %b.")
        dateLbl = Label(text='[color=111111] %s [/color]' %
                        dateString, font_size='22dp', pos=(win_x/2-70, win_y/2-65), markup=True)
        self.add_widget(dateLbl)

        # Show Speed
        speedLbl = Label(
            text="[color=111111] %d [size=30]km/h[/size] [/color]" % cur_speed,
                         pos_hint={'top': 1.1}, font_size="80dp", markup=True)
        self.add_widget(speedLbl)

        # Battery


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
    MyApp().run()
