import kivy
kivy.require('1.0.6')

from kivy.app import App
from kivy.uix.button import Button
from kivy.logger import Logger

class CoolApp(App):
    icon = 'custom-kivy-icon.png'
    title = 'Basic Application'
    
    def build(self):
        return Button(text='Hello World')
    
    def on_start(self):
        Logger.info('App: I\'m alive!')
 
    def on_stop(self):
        Logger.critical('App: Aaaargh I\'m dying!')
 
if __name__ in ('__android__', '__main__'):
    CoolApp().run()