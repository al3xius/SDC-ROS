from kivy.garden.mapview import MapView
from kivy.app import App


class MapViewApp(App):
    def build(self):
        zoomLevel = 12
        mapview = MapView(zoom=zoomLevel, lat=47.21, lon=15.62)
        return mapview

MapViewApp().run()