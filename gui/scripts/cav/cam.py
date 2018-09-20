from kivy.app import App
from kivy.lang import Builder
from gi.repository import Gst
from picamera import PiCamera
import cv2

camera = '''
BoxLayout:
    orientation: 'vertical'
    Camera:
        id: Cam0
        resolution: (1280, 720)
        play: False
    ToggleButton:
        text: 'Toggle'
        on_press: Cam0.play = not Cam0.play
        size_hint_y: None
        height: '48dp'
'''

class CamApp(App):

    def build(self):
        return Builder.load_string(camera)

CamApp().run()