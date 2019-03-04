import os
import random
from math import *
from mapview.utils import clamp
import time


from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout


from kivy.app import App
from kivy.clock import Clock
from kivy.graphics import Color, Line
from kivy.graphics.transformation import Matrix
from kivy.graphics.context_instructions import Translate, Scale
from mapview import MapView, MapLayer, MIN_LONGITUDE, MIN_LATITUDE, MAX_LATITUDE, MAX_LONGITUDE


class MapViewApp(App):
    mapview = None

    def __init__(self, **kwargs):
        super(MapViewApp, self).__init__(**kwargs)
        Clock.schedule_once(self.post, 0)

    def build(self):
        layout = BoxLayout(orientation='vertical')
        return layout

    def post(self, *args):
        layout = FloatLayout()
        self.mapview = MapView(zoom=9, lat=51.046284, lon=1.541179)
        line = LineMapLayer()
        self.mapview.add_layer(line, mode="scatter")  # window scatter
        layout.add_widget(self.mapview)

        self.root.add_widget(layout)
        b = BoxLayout(
            orientation='horizontal', height='32dp', size_hint_y=None)
        b.add_widget(Button(text="Zoom in", on_press=lambda a:
                     setattr(self.mapview, 'zoom', self.mapview.zoom+1)))
        b.add_widget(Button(text="Zoom out", on_press=lambda a:
                     setattr(self.mapview, 'zoom', self.mapview.zoom-1)))
        b.add_widget(
            Button(text="AddPoint", on_press=lambda a: line.add_point()))
        self.root.add_widget(b)


class LineMapLayer(MapLayer):
    def __init__(self, **kwargs):
        super(LineMapLayer, self).__init__(**kwargs)
        self.zoom = 0

        geo_dover = [51.126251, 1.327067]
        geo_calais = [50.959086, 1.827652]

        # NOTE: Points must be valid as they're no longer clamped
        self.coordinates = [geo_dover, geo_calais]
        for i in range(25000-2):
            self.coordinates.append(self.gen_point())

    def reposition(self):
        mapview = self.parent

        #: Must redraw when the zoom changes
        #: as the scatter transform resets for the new tiles
        if (self.zoom != mapview.zoom):
            self.draw_line()

    def gen_point(self):
        n = len(self.coordinates)
        dx, dy = random.randint(
            -100, 100)/10000.0, random.randint(0, 100)/10000.0
        c = (self.coordinates[-1][0]+dx,
             self.coordinates[-1][1]+dy)

        return c

    def add_point(self):
        #: Add a random point close to the previous one
        for i in range(len(self.coordinates)):
            self.coordinates.append(self.gen_point())
        self.draw_line()

    def get_x(self, lon):
        """Get the x position on the map using this map source's projection
        (0, 0) is located at the top left.
        """
        return clamp(lon, MIN_LONGITUDE, MAX_LONGITUDE)

    def get_y(self, lat):
        """Get the y position on the map using this map source's projection
        (0, 0) is located at the top left.
        """
        lat = clamp(-lat, MIN_LATITUDE, MAX_LATITUDE)
        lat = lat * pi / 180.
        return ((1.0 - log(tan(lat) + 1.0 / cos(lat)) / pi))

    def draw_line(self, *args):
        mapview = self.parent
        self.zoom = mapview.zoom

        # When zooming we must undo the current scatter transform
        # or the animation distorts it
        scatter = mapview._scatter
        map_source = mapview.map_source
        sx, sy, ss = scatter.x, scatter.y, scatter.scale
        vx, vy, vs = mapview.viewport_pos[0], mapview.viewport_pos[
            1], mapview.scale

        # Account for map source tile size and mapview zoom
        ms = pow(2.0, mapview.zoom) * map_source.dp_tile_size

        #: Since lat is not a linear transform we must compute manually
        line_points = []
        for lat, lon in self.coordinates:
            line_points.extend((self.get_x(lon), self.get_y(lat)))
            #line_points.extend(mapview.get_window_xy_from(lat,lon,mapview.zoom))

        with self.canvas:
            # Clear old line
            self.canvas.clear()

            # Undo the scatter animation transform
            Scale(1/ss, 1/ss, 1)
            Translate(-sx, -sy)

            # Apply the get window xy from transforms
            Scale(vs, vs, 1)
            Translate(-vx, -vy)

            # Apply the what we can factor out
            # of the mapsource long,lat to x,y conversion
            Scale(ms/360.0, ms/2.0, 1)
            Translate(180, 0)

            # Draw new
            Color(0, 0, 0, .6)
            Line(points=line_points, width=1)
                 #4/ms)#, joint="round",joint_precision=100)


MapViewApp().run()
