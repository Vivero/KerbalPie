#!/usr/bin/python

#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=  I M P O R T   #=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
import math, sys, time

if sys.version_info >= (3,0):
    isPython3 = True
else:
    isPython3 = False
    
from time import sleep

from lib.logger import Logger
from lib.kp_tools import PidController
from lib.widgets.QPlot2D import *

from PyQt5 import QtCore
from PyQt5.QtCore import QVariant, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel
from PyQt5.QtWidgets import QPushButton, QSizePolicy, QWidget

#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=

#--- Wrapper for PID Controller class augmented with Qt functionality
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class QPidController(QtCore.QObject):

    subsys = 'PID'
        
    # S I G N A L S 
    #===========================================================================
    kpChanged = pyqtSignal(float)
    kiChanged = pyqtSignal(float)
    kdChanged = pyqtSignal(float)
    setpointChanged = pyqtSignal(float)
    outputMinChanged = pyqtSignal(float)
    outputMaxChanged = pyqtSignal(float)
    outputChanged = pyqtSignal(float)
    isGainsEditableChanged = pyqtSignal(bool)
    isSetpointEditableChanged = pyqtSignal(bool)
        
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, kp, ki, kd, output_min, output_max, set_point, name="controller", **kwds):
        super(QPidController, self).__init__(**kwds)
        
        self._pid = PidController(kp, ki, kd, output_min, output_max, set_point)
        self.name = name
        
        
    # M E T H O D S 
    #===========================================================================
    def setProportionalGain(self, kp):
        self._pid.kp = kp
        self.kpChanged.emit(kp)
        
    def getProportionalGain(self):
        return self._pid.kp
        
    def setIntegralGain(self, ki):
        self._pid.ki = ki
        self.kiChanged.emit(ki)
        
    def getIntegralGain(self):
        return self._pid.ki
        
    def setDerivativeGain(self, kd):
        self._pid.kd = kd
        self.kdChanged.emit(kd)
        
    def getDerivativeGain(self):
        return self._pid.kd
        
    def setSetpoint(self, set_point):
        self._pid.set_point = set_point
        self.setpointChanged.emit(set_point)
        
    def getSetpoint(self):
        return self._pid.set_point
        
    def setOutputMin(self, output_min):
        self._pid.output_min = output_min
        self.outputMinChanged.emit(output_min)
        
    def getOutputMin(self):
        return self._pid.output_min
        
    def setOutputMax(self, output_max):
        self._pid.output_max = output_max
        self.outputMaxChanged.emit(output_max)
        
    def getOutputMax(self):
        return self._pid.output_max
        
    def setGainsEditable(self, is_editable):
        self._isGainsEditable = is_editable
        self.isGainsEditableChanged.emit(is_editable)
        
    def getGainsEditable(self):
        return self._isGainsEditable
        
    def setSetpointEditable(self, is_editable):
        self._isSetpointEditable = is_editable
        self.isSetpointEditableChanged.emit(is_editable)
        
    def getSetpointEditable(self):
        return self._isSetpointEditable
        
    def getProportionalValue(self):
        return self._pid._p_value
        
    def getIntegralValue(self):
        return self._pid._i_value
        
    def getDerivativeValue(self):
        return self._pid._d_value

    def getOutputValue(self):
        return self._pid._u
        
    def update(self, current_value):
        output = self._pid.update(current_value)
        self.outputChanged.emit(output)
        return output
    
    
    # S L O T S 
    #===========================================================================
    
    
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    def _log(self, log_message, log_type='info', log_data=None):
        Logger.log(QPidController.subsys, log_message, log_type, log_data)
        
    def _log_exception(self, log_message, log_exception):
        Logger.log_exception(QPidController.subsys, log_message, log_exception)
    
    
    
#--- Qt widget with editable panels for editing PID controller items
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class QPidControllerPanel(QWidget):

    subsys = 'PID_PANEL'
        
    # S I G N A L S 
    #===========================================================================
    
        
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, pid_controller, **kwds):
        super(QPidControllerPanel, self).__init__(**kwds)
        
        # PidControllerQ object
        self._pid           = pid_controller
        
        # setup widgets
        set_point_label     = QLabel("Ref.")
        kp_label            = QLabel("Kp")
        ki_label            = QLabel("Ki")
        kd_label            = QLabel("Kd")
        output_label        = QLabel("Out")
        
        set_point_editor    = QDoubleSpinBox()
        kp_editor           = QDoubleSpinBox()
        ki_editor           = QDoubleSpinBox()
        kd_editor           = QDoubleSpinBox()
        output_editor       = QDoubleSpinBox()
        
		# set labels' alignment
        self._labels = [set_point_label, kp_label, ki_label, kd_label, output_label]
        for label in self._labels:
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        
		# set spinbox values to gains
        set_point_editor.setValue(self._pid.getSetpoint())
        kp_editor.setValue(self._pid.getProportionalGain())
        ki_editor.setValue(self._pid.getIntegralGain())
        kd_editor.setValue(self._pid.getDerivativeGain())
        output_editor.setValue(self._pid.getOutputValue())
        output_editor.setEnabled(False)
        
		# spinbox settings
        self._editors = [set_point_editor, kp_editor, ki_editor, kd_editor, output_editor]
        for editor in self._editors:
            editor.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            editor.setDecimals(4)
            editor.setSingleStep(0.1)
            editor.setMinimum(-1000.0)
            editor.setMaximum(1000.0)
        
		# populate a grid with gain labels/editors
        grid_layout = QGridLayout()
        for idx in range(len(self._labels)):
            grid_layout.addWidget(self._labels[idx], idx, 0)
        for idx in range(len(self._editors)):
            grid_layout.addWidget(self._editors[idx], idx, 1)

        # create widget with gain controls
        gain_controls_widget = QWidget()
        gain_controls_widget.setLayout(grid_layout)

        # create widget for gain plotter
        self._pid_plotter = QPlot2DTime(
            timeSpan=30.0,
            yMin=-0.5,
            yMax=0.5,
            yOriginValue=0.0,
            xTickInterval=5.0,
            yTickInterval=0.2,
            labelFont=QFont("Segoe UI", 8),
            refreshRate=0.1)
        self._pid_plotter.setMinimumWidth(250)
        self._pid_plotter.setPlotPen(0, QPen(Qt.red, 1.0))
        self._pid_plotter.setPlotPen(1, QPen(Qt.green, 1.0))
        self._pid_plotter.setPlotPen(2, QPen(Qt.blue, 1.0))
        for i in range(3):
            self._pid_plotter.setPlotDrawMethod(i, 'line')
        
        # set the layout for the PID panel
        panel_layout = QHBoxLayout()
        panel_layout.addWidget(gain_controls_widget)
        panel_layout.addWidget(self._pid_plotter)
        
        self.setLayout(panel_layout)
        
        # connect signals
        self._pid.outputChanged.connect(self.pidOutputChanged)
        self._pid.outputChanged.connect(output_editor.setValue)
        self._pid.setpointChanged.connect(set_point_editor.setValue)
        self._pid.kpChanged.connect(kp_editor.setValue)
        self._pid.kiChanged.connect(ki_editor.setValue)
        self._pid.kdChanged.connect(kd_editor.setValue)
        self._pid.isGainsEditableChanged.connect(self.setControllerGainsEditable)
        self._pid.isSetpointEditableChanged.connect(self.setControllerSetpointEditable)
        
        set_point_editor.valueChanged.connect(self.setSetpoint)
        kp_editor.valueChanged.connect(self.setProportionalGain)
        ki_editor.valueChanged.connect(self.setIntegralGain)
        kd_editor.valueChanged.connect(self.setDerivativeGain)
        
    def update(self):
        self._pid_plotter.update()
        
        
    # S L O T S 
    #===========================================================================
    @pyqtSlot(bool)
    def setControllerGainsEditable(self, is_editable):
        for editor in self._editors[1:]:
            editor.setReadOnly(not is_editable)
            
    @pyqtSlot(bool)
    def setControllerSetpointEditable(self, is_editable):
        self._editors[0].setReadOnly(not is_editable)
        
    @pyqtSlot(float)
    def setSetpoint(self, set_point):
        self._pid._pid.set_point = set_point
        
    @pyqtSlot(float)
    def setProportionalGain(self, kp):
        self._pid._pid.kp = kp
        
    @pyqtSlot(float)
    def setIntegralGain(self, ki):
        self._pid._pid.ki = ki
        
    @pyqtSlot(float)
    def setDerivativeGain(self, kd):
        self._pid._pid.kd = kd
        
    @pyqtSlot(float)
    def pidOutputChanged(self, pidOutput):
        self._pid_plotter.updatePlot(0, self._pid.getProportionalValue())
        self._pid_plotter.updatePlot(1, self._pid.getIntegralValue())
        self._pid_plotter.updatePlot(2, self._pid.getDerivativeValue())
        