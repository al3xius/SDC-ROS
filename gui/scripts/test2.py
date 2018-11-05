from kivy.garden.mapview import MapView
from kivy.app import App


class MapViewApp(App):
    def build(self):
        mapview = MapView(zoom=17, lat=47.21, lon=15.62)
        return mapview

MapViewApp().run()