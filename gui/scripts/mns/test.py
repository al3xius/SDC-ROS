from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.app import App

import random


class MainView(BoxLayout):
    def __init__(self, **kwargs):
        super(MainView, self).__init__(**kwargs)

        self.main_text = Label()
        self.add_widget(self.main_text)

    def update(self):
        self.main_text.text = str(random.randint(0, 200))

    def on_touch_up(self, touch):
        self.update()


class MyApp(App):
    def build(self):
        return MainView()


MyApp().run()
