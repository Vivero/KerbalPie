#!/usr/bin/python

import math, pdb, time

from PyQt5.QtCore import QCoreApplication, QLineF, QPoint, QPointF, QRectF, Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QBrush, QColor, QConicalGradient, QLinearGradient
from PyQt5.QtGui import QPainter, QPainterPath, QPen, QPolygon, QPolygonF
from PyQt5.QtGui import QRadialGradient
from PyQt5.QtWidgets import QApplication, QWidget

class Plotter(QWidget):

    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, parent=None, yMin=0.0, yMax=1.0, timeSpan=60.0):
        super(Plotter, self).__init__(parent)
        self.parent = parent
        
        # validate inputs
        assert timeSpan > 0.0
        assert yMax > yMin
        
        # plot parameters
        self._current_time = time.time()
        self._previous_time = self._current_time
        
        self._timeSpan = timeSpan
        self._xMin = self._current_time - self._timeSpan
        self._xMax = self._current_time
        self._yMin = yMin
        self._yMax = yMax
        
        self._plots = {}
        self._plot_pens = {}
        self._plot_points = {}
        
        # initialize
        self._set_render_parameters()
        

    # O V E R R I D E   M E T H O D S 
    #===========================================================================
    def paintEvent(self, e):
        self._current_time = time.time()
        
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        
        self.setGeometry(self.parent.rect())
        
        self._set_render_parameters()
        self._assemble_plot(qp)
        
        self._previous_time = self._current_time
        
        

    # P U B L I C   M E T H O D S 
    #===========================================================================
    def updatePlot(self, plot_num, value):
        
        # define new plot point
        current_time = time.time()
        plot_point = (current_time, value)
    
        if not plot_num in self._plots.keys():
            self._plots[plot_num] = []
            self._plot_pens[plot_num] = QPen(Qt.blue, 3.0)
            
        self._plots[plot_num].append(plot_point)
        
    def setPlotPen(self, plot_num, pen):
        if plot_num in self._plots.keys():
            self._plot_pens[plot_num] = pen
            
            
    def clearPlots(self):
        for plot_key in self._plots.keys():
            self._plots[plot_key] = []
        
    

    # P R I V A T E   M E T H O D S 
    #===========================================================================
    def _set_render_parameters(self):
        delta_time = self._current_time - self._previous_time
    
        # get size of widget
        size = self.size()
        w = size.width()
        h = size.height()
        
        
        # starting coordinates of drawing
        # TODO: modify for horizontal/vertical alignment
        #-----------------------------------------------------------------------
        widget_x = 0.0
        widget_y = 0.0
        
        # axes
        #-----------------------------------------------------------------------
        self._axes_pen = QPen(Qt.black, 3.0)
        self._axes_brush = QBrush(Qt.NoBrush)
        self._axes_margin_pct = 0.06
        self._axes_margin = w * self._axes_margin_pct
        
        self._axes_tick_pen = QPen(Qt.black, 1.0, Qt.DashLine)
        self._axes_tick_brush = QBrush(Qt.NoBrush)
        
        self._axes_origin_pen = QPen(Qt.black, 2.0, Qt.DashDotLine)
        self._axes_origin_brush = QBrush(Qt.NoBrush)
        
        
        # x-axis
        self._xaxis_x0 = self._axes_margin
        self._xaxis_x1 = w
        self._xaxis_y = h - self._axes_margin
        
        self._xaxis_start = QPointF(self._xaxis_x0, self._xaxis_y)
        self._xaxis_stop = QPointF(self._xaxis_x1, self._xaxis_y)
        self._xaxis_line = QLineF(self._xaxis_start, self._xaxis_stop)
        
        
        # y-axis
        self._yaxis_y0 = 0.0
        self._yaxis_y1 = h - self._axes_margin
        self._yaxis_x = self._axes_margin
        
        self._yaxis_start = QPointF(self._yaxis_x, self._yaxis_y0)
        self._yaxis_stop = QPointF(self._yaxis_x, self._yaxis_y1)
        self._yaxis_line = QLineF(self._yaxis_start, self._yaxis_stop)
        
        
        # re-define axes
        self._xMin = self._current_time - self._timeSpan
        self._xMax = self._current_time
        
        # axis ticks
        self._xaxis_num_ticks = 6
        self._xaxis_ticks = []
        self._xaxis_ticks_text = []
        self._xaxis_ticks_text_rect = []
        self._xaxis_ticks_text_rect_w = (self._xaxis_x1 - self._xaxis_x0) / self._xaxis_num_ticks
        self._xaxis_ticks_text_rect_h = self._axes_margin
        for i in range(self._xaxis_num_ticks - 1):
            tick_x = Plotter.map_value_to_scale(float(i + 1) / self._xaxis_num_ticks, 0.0, 1.0, self._xaxis_x0, self._xaxis_x1)
            self._xaxis_ticks.append(QLineF(QPointF(tick_x, self._yaxis_y0), QPointF(tick_x, self._yaxis_y1)))
            
            self._xaxis_ticks_text.append("{:.1f}".format(float(self._xaxis_num_ticks - 1 - i) / self._xaxis_num_ticks * self._timeSpan))
            self._xaxis_ticks_text_rect.append(QRectF(tick_x - self._xaxis_ticks_text_rect_w / 2.0, self._xaxis_y, self._xaxis_ticks_text_rect_w, self._xaxis_ticks_text_rect_h))
            
            
        self._yaxis_num_ticks = 8
        self._yaxis_ticks = []
        self._yaxis_ticks_text = []
        self._yaxis_ticks_text_rect = []
        self._yaxis_ticks_text_rect_w = self._axes_margin
        self._yaxis_ticks_text_rect_h = (self._yaxis_y1 - self._yaxis_y0) / self._yaxis_num_ticks
        for i in range(self._yaxis_num_ticks - 1):
            tick_y = Plotter.map_value_to_scale(float(i + 1) / self._yaxis_num_ticks, 0.0, 1.0, self._yaxis_y1, self._yaxis_y0)
            self._yaxis_ticks.append(QLineF(QPointF(self._yaxis_x, tick_y), QPointF(self._yaxis_x, tick_y)))
            
            self._yaxis_ticks_text.append("{:.1f}".format(Plotter.map_value_to_scale(float(i + 1) / self._yaxis_num_ticks, 0.0, 1.0, self._yMin, self._yMax)))
            
            self._yaxis_ticks_text_rect.append(QRectF(0.0, tick_y - self._yaxis_ticks_text_rect_h / 2.0, self._yaxis_ticks_text_rect_w, self._yaxis_ticks_text_rect_h))
            
        self._yaxis_origin_y = Plotter.map_value_to_scale(0.0, self._yMin, self._yMax, self._yaxis_y1, self._yaxis_y0)
        self._yaxis_origin_line = QLineF(QPointF(self._xaxis_x0, self._yaxis_origin_y), QPointF(self._xaxis_x1, self._yaxis_origin_y))
        
        
        
        
        # define plot traces
        #-----------------------------------------------------------------------
        for plot_key in self._plots.keys():
            # delete old entries
            if len(self._plots[plot_key]) > 0:
                p = 0
                while self._plots[plot_key][p][0] < self._xMin:
                    p += 1
                self._plots[plot_key] = self._plots[plot_key][p:]
        
            # create plot points for trace
            self._plot_points[plot_key] = []
            for plot_value in self._plots[plot_key]:
                plot_value_x = Plotter.map_value_to_scale(plot_value[0], self._xMin, self._xMax, self._xaxis_x0, self._xaxis_x1)
                plot_value_y = Plotter.map_value_to_scale(plot_value[1], self._yMin, self._yMax, self._yaxis_y1, self._yaxis_y0)
                self._plot_points[plot_key].append(QPointF(plot_value_x, plot_value_y))
        
        
        
    
    def _assemble_plot(self, qpainter):
        qpainter.save()
        #---
        
        # draw axes
        qpainter.setPen(self._axes_pen)
        qpainter.setBrush(self._axes_brush)
        qpainter.drawLine(self._xaxis_line)
        qpainter.drawLine(self._yaxis_line)
        
        # draw ticks
        # x-axis
        qpainter.setPen(self._axes_tick_pen)
        qpainter.setBrush(self._axes_tick_brush)
        for tick_line in self._xaxis_ticks:
            qpainter.drawLine(tick_line)
            
        for i in range(len(self._xaxis_ticks_text)):
            qpainter.drawText(self._xaxis_ticks_text_rect[i], Qt.AlignHCenter, self._xaxis_ticks_text[i])
            
        # y-axis
        for tick_line in self._yaxis_ticks:
            qpainter.drawLine(tick_line)
            
        for i in range(len(self._yaxis_ticks_text)):
            qpainter.drawText(self._yaxis_ticks_text_rect[i], Qt.AlignVCenter | Qt.AlignLeft, self._yaxis_ticks_text[i])
            
        # y-axis origin
        qpainter.setPen(self._axes_origin_pen)
        qpainter.setBrush(self._axes_origin_brush)
        qpainter.drawLine(self._yaxis_origin_line)
        
        # draw plot traces
        for plot_key in self._plot_points.keys():
            qpainter.setPen(self._plot_pens[plot_key])
            qpainter.setBrush(QBrush(Qt.NoBrush))
            
            #print(len(self._plot_points[plot_key]))
            if len(self._plot_points[plot_key]) > 1:
                poly = QPolygonF(self._plot_points[plot_key])
                qpainter.drawPolyline(poly)
                
            
        
        #---
        qpainter.restore()
        
        

    # G E T T E R S   /   S E T T E R S 
    #===========================================================================
    def xMin(self):
        return self._xMin
        
    def setXMin(self, value):
        self._xMin = value
        self.update()
        
    def xMax(self):
        return self._xMax
        
    def setXMax(self, value):
        self._xMax = value
        self.update()
        
    def yMin(self):
        return self._yMin
        
    def setYMin(self, value):
        self._yMin = value
        self.update()
        
    def yMax(self):
        return self._yMax
        
    def setYMax(self, value):
        self._yMax = value
        self.update()
        
    

    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    @staticmethod
    def clamp(val_min, value, val_max):
        return max(val_min, min(val_max, value))
        
    @staticmethod
    def map_value_to_scale(value, val_min, val_max, scale_min, scale_max):
        value = Plotter.clamp(val_min, value, val_max)
        return (value - val_min) / (val_max - val_min) * (scale_max - scale_min) + scale_min
        
    @staticmethod
    def angle_to_point(from_point, to_point):
        # where x+ axis is 0 degrees, positive angle is counter-clockwise
        delta_x = to_point.x() - from_point.x()
        delta_y = from_point.y() - to_point.y()
        angle = math.atan2(delta_y, delta_x)
        #return angle if angle >= 0.0 else angle + (2.0 * math.pi)
        return angle
    
        