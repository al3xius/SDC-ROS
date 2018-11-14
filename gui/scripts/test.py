from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics.instructions import Instruction


class MyPaintApp(App):
    def build(self):
        blue = InstructionGroup()
        blue.add(Color(0, 0, 1, 0.2))
        blue.add(Rectangle(pos=self.pos, size=(100, 100)))

        green = InstructionGroup()
        green.add(Color(0, 1, 0, 0.4))
        green.add(Rectangle(pos=(100, 100), size=(100, 100)))

        # Here, self should be a Widget or subclass
        [self.canvas.add(group) for group in [blue, green]]

        return MyPaintWidget()


if __name__ == '__main__':
    MyPaintApp().run()