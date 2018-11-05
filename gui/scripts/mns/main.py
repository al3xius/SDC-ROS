import os
import glob
from kivy.app import App
from kivy.lang import Builder
from kivy.uix.widget import Widget
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.properties import ObjectProperty
from kivy.graphics.vertex_instructions import Ellipse
from kivy.graphics.context_instructions import Color
from kivy.core.window import Window
from kivy.garden.mapview import MapView
import shutil

#Menu Buttons
Builder.load_file("mns.kv")
Builder.load_file("/home/davidzechm/catkin_ws/src/gui/scripts/opt/opt.kv")
Builder.load_file("/home/davidzechm/catkin_ws/src/gui/scripts/ccd/ccd.kv")
Builder.load_file("/home/davidzechm/catkin_ws/src/gui/scripts/cav/cav.kv")
Builder.load_file("/home/davidzechm/catkin_ws/src/gui/scripts/map/map.kv")

# Set Window size and other Variables
Window.size = (950, 700)  # TODO: 16:9 & touch mit display und rpi testen
zoomLevel = 18
cur_lat = 47.224282
cur_lon = 15.6233008

# Screen Manager
sm = ScreenManager()


class ScreenMAP(Screen):
    def on_enter(self):
        # Map with Zoom and Coordinates
        mapview = MapView(zoom=zoomLevel, lat=cur_lat, lon=cur_lon)
                          #TODO: use ros variables for coordinates && implement zoom

        # Back to Menu Button
        def changeScreen(self):
            #TODO: change transition to "right"
            sm.current = "MNS"

        backBtn = Button(text="[b]BACK[/b]", font_size="20sp", pos=(0,
                         0), size_hint=(.2, .1), background_color=(1, 1, 1, 0.45), markup=True)
        backBtn.bind(on_press=changeScreen)
        #TODO: font-weight auf bold stellen

        # Add to Map layout
        self.add_widget(mapview)
        self.add_widget(backBtn)


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


class ScreenCCD(Screen):
    pass


class ScreenCAV(Screen):
    pass


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
        self.title = 'CONTROL PANEL | v 0.1'
        #self.icon = 'assets/car.png'
        return sm

if __name__ == '__main__':
    MyApp().run()
