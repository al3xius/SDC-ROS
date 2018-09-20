from kivy.app import App
from kivy.lang import Builder
from kivy.uix.widget import Widget
from kivy.uix.image import Image
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.properties import ObjectProperty
from kivy.core.window import Window

#Menu Buttons
Builder.load_file("mns.kv")
Builder.load_file("opt/opt.kv")
Builder.load_file("ccd/ccd.kv")
Builder.load_file("cav/cav.kv")

#Cam View
Builder.load_file("cav/cam_settings.kv")
Window.size = (700, 700) #16:9 testen

# Car Image - Screen
def drawCar():
    layout = BoxLayout()
    carImage = Image(source='assets/car.jpg')
    layout.add_widget(carImage)
    return layout

# Screen Manager
class ScreenOPT(Screen):
    pass

class ScreenCCD(Screen):
    pass

class ScreenCAV(Screen):
    pass
    
class ScreenMNS(Screen):
    pass


sm = ScreenManager()
sm.add_widget(ScreenMNS(name='MNS'))
sm.add_widget(ScreenOPT(name='OPT'))
sm.add_widget(ScreenCCD(name='CCD'))
sm.add_widget(ScreenCAV(name='CAV'))

class MyApp(App):

    def car(self):
        return drawCar

    def build(self):
        self.title = 'CONTROL PANEL | v 0.1'
        self.icon = 'assets/car.png'
        return sm

if __name__ == '__main__':
    MyApp().run()