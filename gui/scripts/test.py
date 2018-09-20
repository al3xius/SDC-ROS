from kivy.app import App
from kivy.lang import Builder
from kivy.uix.image import Image
from kivy.uix.boxlayout import BoxLayout


# Car Image - Screen
def drawCar():
    layout = BoxLayout()
    carImage = Image(source='assets/car.png')
    layout.add_widget(carImage)
    return layout

class MyApp(App):

    def build(self):
        return drawCar()

if __name__ == '__main__':
    MyApp().run()