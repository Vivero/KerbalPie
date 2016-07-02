#!/usr/bin/python

import math, pdb, time

from kptools import *

from PyQt5.QtCore import QCoreApplication, QLineF, QPoint, QPointF, QRectF
from PyQt5.QtCore import QSize, QSizeF, Qt, QTimer
from PyQt5.QtGui import QBrush, QColor, QConicalGradient, QFont, QFontMetrics
from PyQt5.QtGui import QLinearGradient, QPainter, QPainterPath, QPen
from PyQt5.QtGui import QPolygon, QPolygonF, QRadialGradient
from PyQt5.QtWidgets import QApplication, QFrame, QHBoxLayout, QWidget

class Plot2D(QWidget):

    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self,
            xMin=0.0,
            xMax=1.0,
            yMin=0.0,
            yMax=1.0,
            xOriginValue=0.5,
            yOriginValue=0.5,
            xTickInterval=0.1,
            yTickInterval=0.1,
            labelFont=QFont("Segoe UI", 10),
            **kwds):
        super(Plot2D, self).__init__(**kwds)
        
        # create a frame to hold graphics
        self._plot_frame = QFrame()
        self._plot_layout = QHBoxLayout()
        self._plot_layout.addWidget(self._plot_frame)
        self.setMinimumSize(100,100)
        self.setLayout(self._plot_layout)
        
        # plot parameters
        self._xMin = xMin
        self._xMax = xMax
        self._yMin = yMin
        self._yMax = yMax
        self._xOriginValue = xOriginValue
        self._yOriginValue = yOriginValue
        self._xTickInterval = xTickInterval
        self._yTickInterval = yTickInterval
        self._labelFont = labelFont
        
        # structures to hold plot data
        self._plots = {}
        self._plot_pens = {}
        self._plot_points = {}
        self._plot_draw_method = {}
        
        # initialize
        self._debug = False
        self._set_render_parameters()
        

    # O V E R R I D E   M E T H O D S 
    #===========================================================================
    def paintEvent(self, e):
        qp = QPainter(self)
        #qp.setRenderHint(QPainter.Antialiasing)
        
        #self.setGeometry(self.parent.rect())
        
        self._set_render_parameters()
        self._assemble_plot(qp)
        
        

    # P U B L I C   M E T H O D S 
    #===========================================================================
    def updatePlot(self, plot_num, value_tuple):
        
        # check if we have an existing plot. if not, initialize list of plot
        # values.
        if not plot_num in self._plots.keys():
            self._plots[plot_num] = []
            
            # if no pen has been assigned to this plot, create default
            if not plot_num in self._plot_pens.keys():
                self._plot_pens[plot_num] = QPen(Qt.blue, 2.0)
            
            # if no draw method has been assigned to this plot, assign default
            if not plot_num in self._plot_draw_method.keys():
                self._plot_draw_method[plot_num] = 'scatter'
            
        # add value to plot points list
        self._plots[plot_num].append(value_tuple)
        
        self.update()
        
        
    def setPlotPen(self, plot_num, pen):
        self._plot_pens[plot_num] = pen
        
        
    def setPlotDrawMethod(self, plot_num, draw_method):
        self._plot_draw_method[plot_num] = draw_method
            
            
    def clearPlots(self):
        for plot_key in self._plots.keys():
            self._plots[plot_key] = []
        
    

    # P R I V A T E   M E T H O D S 
    #===========================================================================
    def _set_render_parameters(self):
        
        # get size of widget
        size = self.size()
        w = size.width()
        h = size.height()
        
        
        # starting coordinates of drawing
        # TODO: modify for horizontal/vertical alignment
        #-----------------------------------------------------------------------
        widget_x = 0.0
        widget_y = 0.0
        
        # x-axis tick labels
        #-----------------------------------------------------------------------
        labelFontMetrics = QFontMetrics(self._labelFont);
        
        xaxis_min = min(self._xMin, self._xMax)
        xaxis_max = max(self._xMin, self._xMax)
        
        self._xaxis_ticks_value = []
        self._xaxis_ticks_text_size = []
        tick_value = self._xOriginValue
        while tick_value < xaxis_max:
            if tick_value > xaxis_min:
                self._xaxis_ticks_value.append(tick_value)
                tick_text = "{:.1f}".format(tick_value)
                self._xaxis_ticks_text_size.append(labelFontMetrics.size(Qt.TextSingleLine, tick_text))
                
            tick_value += self._xTickInterval
            
        tick_value = self._xOriginValue - self._xTickInterval
        while tick_value > xaxis_min:
            if tick_value < xaxis_max:
                self._xaxis_ticks_value.append(tick_value)
                tick_text = "{:.1f}".format(tick_value)
                self._xaxis_ticks_text_size.append(labelFontMetrics.size(Qt.TextSingleLine, tick_text))
                
            tick_value -= self._xTickInterval
            
        xaxis_ticks_text_max_height = max([s.height() for s in self._xaxis_ticks_text_size])
        xaxis_ticks_text_max_width = max([s.width() for s in self._xaxis_ticks_text_size])
        
        
        # y-axis tick labels
        #-----------------------------------------------------------------------
        yaxis_min = min(self._yMin, self._yMax)
        yaxis_max = max(self._yMin, self._yMax)
        
        self._yaxis_ticks_value = []
        self._yaxis_ticks_text_size = []
        tick_value = self._yOriginValue
        while tick_value < yaxis_max:
            if tick_value > yaxis_min:
                self._yaxis_ticks_value.append(tick_value)
                tick_text = "{:.1f}".format(tick_value)
                self._yaxis_ticks_text_size.append(labelFontMetrics.size(Qt.TextSingleLine, tick_text))
                
            tick_value += self._yTickInterval
            
        tick_value = self._yOriginValue - self._yTickInterval
        while tick_value > yaxis_min:
            if tick_value < yaxis_max:
                self._yaxis_ticks_value.append(tick_value)
                tick_text = "{:.1f}".format(tick_value)
                self._yaxis_ticks_text_size.append(labelFontMetrics.size(Qt.TextSingleLine, tick_text))
                
            tick_value -= self._yTickInterval
            
        yaxis_ticks_text_max_height = max([s.height() for s in self._yaxis_ticks_text_size])
        yaxis_ticks_text_max_width = max([s.width() for s in self._yaxis_ticks_text_size]) * 1.1
        
        
        
        # axes
        #-----------------------------------------------------------------------
        self._axes_tick_pen = QPen(Qt.black, 1.0, Qt.DashLine)
        self._axes_tick_brush = QBrush(Qt.NoBrush)
        
        #self._axes_origin_pen = QPen(Qt.black, 2.0, Qt.DashDotLine)
        self._axes_origin_pen = QPen(Qt.black, 2.0, Qt.SolidLine)
        self._axes_origin_brush = QBrush(Qt.NoBrush)
        
        
        # bounding box for plot
        #-----------------------------------------------------------------------
        self._bbox_pen = QPen(Qt.black, 1.0)
        self._bbox_brush = QBrush(Qt.NoBrush)
        
        self._bbox_x0 = widget_x + yaxis_ticks_text_max_width
        self._bbox_x1 = w - 1
        self._bbox_y0 = widget_y
        self._bbox_y1 = h - xaxis_ticks_text_max_height - 1
        
        self._bbox_rect = QRectF(QPointF(self._bbox_x0, self._bbox_y0), QPointF(self._bbox_x1, self._bbox_y1))
        
        
        # origin lines
        #-----------------------------------------------------------------------
        self._xaxis_origin_x = map_value_to_scale(self._xOriginValue, self._xMin, self._xMax, self._bbox_x0, self._bbox_x1)
        self._xaxis_origin_line = QLineF(QPointF(self._xaxis_origin_x, self._bbox_y0), QPointF(self._xaxis_origin_x, self._bbox_y1))
        
        self._yaxis_origin_y = map_value_to_scale(self._yOriginValue, self._yMin, self._yMax, self._bbox_y1, self._bbox_y0)
        self._yaxis_origin_line = QLineF(QPointF(self._bbox_x0, self._yaxis_origin_y), QPointF(self._bbox_x1, self._yaxis_origin_y))
        
        
        # x-axis ticks
        #-----------------------------------------------------------------------
        self._xaxis_ticks = []
        self._xaxis_ticks_text = []
        self._xaxis_ticks_text_rect = []
        for tick_value in self._xaxis_ticks_value:
        
            # define the tick lines to draw
            tick_x = map_value_to_scale(tick_value, self._xMin, self._xMax, self._bbox_x0, self._bbox_x1)
            tick_line = QLineF(QPointF(tick_x, self._bbox_y0), QPointF(tick_x, self._bbox_y1))
            self._xaxis_ticks.append(tick_line)
            
            # define the tick labels to draw
            tick_text = "{:.1f}".format(tick_value)
            tick_size = labelFontMetrics.size(Qt.TextSingleLine, tick_text)
            tick_rect = QRectF(QPointF(tick_x - (tick_size.width() / 2.0), self._bbox_y1), QSizeF(tick_size))
            
            self._xaxis_ticks_text.append(tick_text)
            self._xaxis_ticks_text_rect.append(tick_rect)
        
        
        # y-axis ticks
        #-----------------------------------------------------------------------
        self._yaxis_ticks = []
        self._yaxis_ticks_text = []
        self._yaxis_ticks_text_rect = []
        for tick_value in self._yaxis_ticks_value:
        
            # define the tick lines to draw
            tick_y = map_value_to_scale(tick_value, self._yMin, self._yMax, self._bbox_y1, self._bbox_y0)
            tick_line = QLineF(QPointF(self._bbox_x0, tick_y), QPointF(self._bbox_x1, tick_y))
            self._yaxis_ticks.append(tick_line)
            
            # define the tick labels to draw
            tick_text = "{:.1f}".format(tick_value)
            tick_size = labelFontMetrics.size(Qt.TextSingleLine, tick_text)
            tick_rect = QRectF(QPointF(widget_x, tick_y - (tick_size.height() / 2.0)), QSizeF(yaxis_ticks_text_max_width, tick_size.height()))
            
            self._yaxis_ticks_text.append(tick_text)
            self._yaxis_ticks_text_rect.append(tick_rect)
            
        
        
        # define plot traces
        #-----------------------------------------------------------------------
        for plot_key in self._plots.keys():
            self._plot_points[plot_key] = []
            for plot_value in self._plots[plot_key]:
                plot_value_x = map_value_to_scale(plot_value[0], self._xMin, self._xMax, self._bbox_x0, self._bbox_x1)
                plot_value_y = map_value_to_scale(plot_value[1], self._yMin, self._yMax, self._bbox_y1, self._bbox_y0)
                self._plot_points[plot_key].append(QPointF(plot_value_x, plot_value_y))
        
        
    
    def _assemble_plot(self, qpainter):
        qpainter.save()
        #---
        
        # draw bounding box
        qpainter.setPen(self._bbox_pen)
        qpainter.setBrush(self._bbox_brush)
        qpainter.drawRect(self._bbox_rect)
        
        # x-axis
        qpainter.setFont(self._labelFont)
        qpainter.setPen(self._axes_tick_pen)
        qpainter.setBrush(self._axes_tick_brush)
        for tick_line in self._xaxis_ticks:
            qpainter.drawLine(tick_line)
            
        for i in range(len(self._xaxis_ticks_text)):
            qpainter.drawText(self._xaxis_ticks_text_rect[i], Qt.AlignVCenter | Qt.AlignHCenter, self._xaxis_ticks_text[i])
            
        # y-axis
        for tick_line in self._yaxis_ticks:
            qpainter.drawLine(tick_line)
            
        for i in range(len(self._yaxis_ticks_text)):
            qpainter.drawText(self._yaxis_ticks_text_rect[i], Qt.AlignVCenter | Qt.AlignHCenter, self._yaxis_ticks_text[i])
            
        # x-axis origin
        qpainter.setPen(self._axes_origin_pen)
        qpainter.setBrush(self._axes_origin_brush)
        if self._xaxis_origin_x > self._bbox_x0 and self._xaxis_origin_x < self._bbox_x1:
            qpainter.drawLine(self._xaxis_origin_line)
            
        # y-axis origin
        if self._yaxis_origin_y > self._bbox_y0 and self._yaxis_origin_y < self._bbox_y1:
            qpainter.drawLine(self._yaxis_origin_line)
            
        # draw plots
        for plot_key in self._plot_points.keys():
            qpainter.setPen(self._plot_pens[plot_key])
            qpainter.setBrush(QBrush(Qt.NoBrush))
            
            if self._plot_draw_method[plot_key] == 'line' and len(self._plot_points[plot_key]) > 1:
                qpainter.drawPolyline(QPolygonF(self._plot_points[plot_key]))
                    
            else:
                for plot_point in self._plot_points[plot_key]:
                    qpainter.drawPoint(plot_point)
                
        
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
        
    def xOriginValue(self):
        return self._xOriginValue
        
    def setXOriginValue(self, value):
        self._xOriginValue = value
        self.update()
        
    def yOriginValue(self):
        return self._yOriginValue
        
    def setYOriginValue(self, value):
        self._yOriginValue = value
        self.update()
        
    def xTickInterval(self):
        return self._xTickInterval
        
    def setXTickInterval(self, value):
        self._xTickInterval = value
        self.update()
        
    def yTickInterval(self):
        return self._yTickInterval
        
    def setYTickInterval(self, value):
        self._yTickInterval = value
        self.update()
        
    def labelFont(self):
        return self._labelFont
        
    def setLabelFont(self, font):
        self._labelFont = font
        self.update()
        

class Plot2DTime(Plot2D):

    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self,
            timeSpan=30.0,
            yMin=-1.0,
            yMax=1.0,
            yOriginValue=0.0,
            xTickInterval=5.0,
            yTickInterval=0.2,
            labelFont=QFont("Segoe UI", 10),
            refreshRate=0.1,
            **kwds):
            
        # validate inputs
        assert timeSpan > 0.0
        
        self._timeSpan = timeSpan
        
        self._lastUpdate = time.time()
        
        self._refreshRate = refreshRate
        
        self._refreshTimer = QTimer()
        self._refreshTimer.timeout.connect(self._refresh_plots)
        self._refreshTimer.start(self._refreshRate * 1000.0)
        
        super(Plot2DTime, self).__init__(
            xMin=timeSpan,
            xMax=0.0,
            yMin=yMin,
            yMax=yMax,
            xOriginValue=0.0,
            yOriginValue=yOriginValue,
            xTickInterval=xTickInterval,
            yTickInterval=yTickInterval,
            labelFont=labelFont,
            **kwds)
            
            
    # P U B L I C   M E T H O D S 
    #===========================================================================
    def updatePlot(self, plot_num, value):
        
        # define new plot point
        value_tuple = (0.0, value)
        
        super(Plot2DTime, self).updatePlot(plot_num, value_tuple)


    # P R I V A T E   M E T H O D S 
    #===========================================================================
    def _refresh_plots(self):
        current_time = time.time()
        delta_time = current_time - self._lastUpdate
        
        for plot_key in self._plots.keys():
            refreshed_plot_points = [(p[0] + delta_time, p[1]) for p in self._plots[plot_key] if (p[0] + delta_time) < self._timeSpan]
            self._plots[plot_key] = refreshed_plot_points
            
        self.update()
        
        self._lastUpdate = current_time
        
    
        