#!/usr/bin/python
 
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=  I M P O R T   #=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
import argparse, os, pdb, re, sys, threading, time

if sys.version_info >= (3,0):
    isPython3 = True
    import configparser
else:
    isPython3 = False
    import ConfigParser

from PyQt5 import QtCore, uic
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QColor, QPalette, QPen
from PyQt5.QtWidgets import QAbstractItemView, QApplication, QHeaderView
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget

from logger import KPLogger
from widgets.Plot2D import *
from widgets.PidControllerQ import PidControllerPanel
from flightcontrol import KPFlightDataModel, KPFlightController
from missioncontrol import MissionProgramsModel, MissionProgramsDatabase
from kptools import *

#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
    
class KerbalPie(QWidget):

    subsys = 'KERBALPIE'
    
    # configuration file named sections
    _CFG_GLOBALS_SECTION = 'GLOBALS'
    _CFG_KRPC_SECTION    = 'KRPC'
    
    # S I G N A L S 
    #===========================================================================
    krpc_client_begin_connect = pyqtSignal()
    krpc_client_begin_disconnect = pyqtSignal()
    
    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, parent=None, config_filename=os.path.join('..', 'data', 'kerbalpie.cfg'), debug_on=False):
        super(KerbalPie, self).__init__(parent)
        uic.loadUi(os.path.join('..', 'data', 'KerbalPie.ui'), self)
        
        # Set up KerbalPie application
        #-----------------------------------------------------------------------
        # parse configuration options
        self.config = self._parse_config(config_filename)
        
        # general message logging
        #-----------------------------------------------------------------------
        self._logger_thread = KPLogger(log_dir=self.config['logger_directory'], log_name=self.config['logger_filename'])
        self._logger_thread.start()
        
        # flight controller
        #-----------------------------------------------------------------------
        
        # controller thread
        self._flight_thread = QtCore.QThread()
        self._flight_ctrl = KPFlightController(krpc_address=self.config['krpc_address'], krpc_rpc_port=self.config['krpc_rpc_port'], krpc_stream_port=self.config['krpc_stream_port'], krpc_name=self.config['krpc_client_name'])
        self._flight_ctrl.moveToThread(self._flight_thread)
        
        # gui items
        
        # PID controller panels
        self.flightControl_tuningTabGroup = QTabWidget(parent=None)
        self.flightControl_pidControllerPanels = []
        for ctrl in self._flight_ctrl.controllers:
            ctrl_panel = PidControllerPanel(ctrl, parent=None)
            self.flightControl_tuningTabGroup.addTab(ctrl_panel, ctrl.name)
            self.flightControl_pidControllerPanels.append(ctrl_panel)
        self.flightControl_tuningGroup.layout().addWidget(self.flightControl_tuningTabGroup)
        
        # flight controller thread connections
        self._flight_thread.started.connect(self._flight_ctrl.process)
        self._flight_ctrl.finished.connect(self._flight_thread.quit)
        self._flight_ctrl.finished.connect(self._flight_ctrl.deleteLater)
        self._flight_thread.finished.connect(self._flight_thread.deleteLater)
        
        # flight controller connections
        self._flight_ctrl.telemetry_updated.connect(self.flight_telemetry_updated)
        self.krpc_client_begin_connect.connect(self._flight_ctrl.krpc_connect)
        self.krpc_client_begin_disconnect.connect(self._flight_ctrl.krpc_disconnect)
        self._flight_ctrl.krpc_connected.connect(self.krpc_client_connected)
        self._flight_ctrl.krpc_disconnected.connect(self.krpc_client_disconnected)
        self.krpc_connectionButton.clicked.connect(self.krpc_connectionButton_clicked)
        self.krpc_addressEdit.textChanged.connect(self.krpc_client_address_changed)
        self.krpc_rpcPortEdit.textChanged.connect(self.krpc_client_rpc_port_changed)
        self.krpc_streamPortEdit.textChanged.connect(self.krpc_client_stream_port_changed)
        
        self._flight_thread.start()
        
        
        # flight data display
        #-----------------------------------------------------------------------
        
        # gui items
        self.flightPlot_selection.addItems([
            "Vertical Speed",
            "Altitude"
        ])
        
        self.flight_data_model = KPFlightDataModel(parent=self)
        self.flightData_tableView.setModel(self.flight_data_model)
        self.flightData_tableView.verticalHeader().setVisible(False)
        self.flightData_tableView.verticalHeader().sectionResizeMode(QHeaderView.Fixed)
        self.flightData_tableView.resizeColumnsToContents()
        self.flightData_tableView.resizeRowsToContents()
        self.flightData_tableView.setColumnWidth(2, 120)
        
        
        # flight data plotter
        #-----------------------------------------------------------------------
        pal = QPalette(self.palette())
        pal.setColor(QPalette.Background, Qt.white)
        
        self.controllerPlotter = Plot2DTime(
            timeSpan=30.0,
            yMin=-2.0,
            yMax=2.0,
            yOriginValue=0.0,
            xTickInterval=5.0,
            yTickInterval=0.5,
            labelFont=QFont("Segoe UI", 10),
            refreshRate=0.1)
        self.controllerPlotter.setPlotDrawMethod(0, 'line')
        
        self.flightPlot_plotGroup.layout().addWidget(self.controllerPlotter)
        
        self.flightPlot_selection.currentTextChanged.connect(self.flightPlot_selection_changed)
        
        
        # mission programs table
        #-----------------------------------------------------------------------
        self.mission_program_db = MissionProgramsDatabase(parent=self)
        
        self.mission_programs_model = MissionProgramsModel(mp_database=self.mission_program_db, parent=self)
        self.mission_programTableView.setModel(self.mission_programs_model)
        self.mission_programTableView.verticalHeader().setVisible(False)
        self.mission_programTableView.verticalHeader().sectionResizeMode(QHeaderView.Fixed)
        self.mission_programTableView.resizeColumnsToContents()
        self.mission_programTableView.resizeRowsToContents()
        self.mission_programTableView.setSelectionMode(QAbstractItemView.SingleSelection)
        
        self.mission_program_db.current_program_updated.connect(self.mission_programs_model.set_active_program)
        self.mission_program_db.current_program_updated.connect(self._flight_ctrl.set_active_program)
        self.mission_activateButton.clicked.connect(self.mission_activateButton_clicked)
        
        self.mission_program_db.set_current_program_num(0)
        
        # automatically start KRPC connection
        QTimer.singleShot(200, self.krpc_connectionButton_clicked)
        
        
        # debug
        #-----------------------------------------------------------------------
        
        self.radarPlotter = Plot2D(
            xMin=-1.0,
            xMax=1.0,
            yMin=-1.0,
            yMax=1.0,
            xOriginValue=0.0,
            yOriginValue=0.0,
            xTickInterval=0.1,
            yTickInterval=0.1)
            
        radarLayout = QVBoxLayout()
        radarLayout.addWidget(self.radarPlotter)
        self.radarTab.setLayout(radarLayout)
        
        self._radarPlotColorBins = 60
        radarPlotColor = QColor()
        for i in range(self._radarPlotColorBins):
            hue = map_value_to_scale(float(i), 0.0, float(self._radarPlotColorBins), 0.0, 0.2) 
            radarPlotColor.setHsvF(hue, 1.0, 1.0)
            self.radarPlotter.setPlotPen(i, QPen(radarPlotColor, 15.0, Qt.SolidLine, Qt.RoundCap))
        self.radarPlotter.setPlotPen(self._radarPlotColorBins, QPen(Qt.blue, 15.0, Qt.SolidLine, Qt.RoundCap))
                
        self.radarPlotter.update()
           
    
    # S L O T S 
    #===========================================================================
    @pyqtSlot()
    def update_plots(self):
        pass
            
    
    @pyqtSlot()
    def krpc_client_connected(self):
        self.krpc_connectionLabel.setText("Status: Connected")
        self.krpc_connectionButton.setText("Disconnect")
        
    @pyqtSlot()
    def krpc_client_disconnected(self):
        self.krpc_connectionLabel.setText("Status: Disconnected")
        self.krpc_connectionButton.setText("Connect")
    
    @pyqtSlot()
    def krpc_client_address_changed(self, text):
        self._flight_ctrl.krpc_address = text
        
    @pyqtSlot()
    def krpc_client_rpc_port_changed(self, text):
        (port, ok) = text.toInt()
        if ok:
            self._flight_ctrl.krpc_rpc_port = port
        
    @pyqtSlot()
    def krpc_client_stream_port_changed(self, text):
        (port, ok) = text.toInt()
        if ok:
            self._flight_ctrl.krpc_stream_port = port
        
    @pyqtSlot()
    def krpc_connectionButton_clicked(self):
        if not self._flight_ctrl.krpc_is_connected:
            self.krpc_client_begin_connect.emit()
        else:
            self.krpc_client_begin_disconnect.emit()
            
    @pyqtSlot()
    def mission_activateButton_clicked(self):
        if self.mission_programTableView.currentIndex().column() == 0:
            current_selection = self.mission_programTableView.currentIndex().row()
            self.mission_program_db.set_current_program_num(current_selection)
                
        
    @pyqtSlot(dict)
    def flight_telemetry_updated(self, telemetry_dict):
        #self.log_logTextEdit.clear()
        
        for param in telemetry_dict.keys():
            if param in KPFlightDataModel.flight_data_lookup:
                self.flight_data_model.update_flight_data(param, telemetry_dict[param])
                
        # plotter
        plotter_current_selection = self.flightPlot_selection.currentText()
        if plotter_current_selection == 'Vertical Speed':
            self.controllerPlotter.updatePlot(0, telemetry_dict['vessel_vertical_speed'])
        elif plotter_current_selection == 'Altitude':
            self.controllerPlotter.updatePlot(0, telemetry_dict['vessel_mean_altitude'])
            
        # radar
        self.radarPlotter.clearPlots()
        
        #surface_height_at_vessel = telemetry_dict['vessel_surface_height']
        radar_altitude_map = telemetry_dict['surface_height_map']
        
        #print(radar_altitude_map)
        
        radar_altitude_map_min = min([min(a) for a in radar_altitude_map])
        radar_altitude_map_max = max([max(a) for a in radar_altitude_map])
        
        if radar_altitude_map_min == 0.0 and radar_altitude_map_max == 0.0:
            radar_altitude_map_min = -1.0
            radar_altitude_map_max = 1.0
        
        s = len(telemetry_dict['surface_height_map'][0])
        for y in range(s):
            for x in range(s):
                x_val = map_value_to_scale(float(x) + 0.5, 0.0, s, -1.0, 1.0)
                y_val = map_value_to_scale(float(y) + 0.5, 0.0, s, -1.0, 1.0)
                alt = radar_altitude_map[y][x]
                bin = map_value_to_scale(alt, radar_altitude_map_min, radar_altitude_map_max, 0.1, self._radarPlotColorBins - 0.1)
                #bin = map_value_to_scale(x_val, -1.0, 1.0, 0.1, 19.9)
                
                bin = int(bin)
                
                if alt > (telemetry_dict['vessel_surface_height'] - 1.0) and alt < (telemetry_dict['vessel_surface_height'] + 1.0):
                    self.radarPlotter.updatePlot(self._radarPlotColorBins, (x_val, y_val))
                else:
                    self.radarPlotter.updatePlot(bin, (x_val, y_val))
                
        self.radarPlotter.update()
        
        
    @pyqtSlot('QString')
    def flightPlot_selection_changed(self, text):
        self.controllerPlotter.clearPlots()
        if text == 'Vertical Speed':
            self.controllerPlotter.setYMin(-2.0)
            self.controllerPlotter.setYMax(2.0)
            self.controllerPlotter.setYTickInterval(0.5)
        elif text == 'Altitude':
            self.controllerPlotter.setYMin(70.0)
            self.controllerPlotter.setYMax(200.0)
            self.controllerPlotter.setYTickInterval(10.0)
            
    
    
    # O V E R R I D E   M E T H O D S 
    #===========================================================================
    
    
    # P R I V A T E   M E T H O D S 
    #===========================================================================
        
    def close(self):
        self._flight_ctrl.terminate = True
        
        self._logger_thread.terminate()
        self._logger_thread.join()
        
        
        
    def _parse_config(self, config_filename):
        if not os.path.isfile(config_filename):
            sys.stderr.write(
                'Config file not found: {:s}\nCWD: {:s}\n\n'.format(config_filename, os.getcwd()))
            
        if isPython3:
            cfg = configparser.ConfigParser()
        else:
            cfg = ConfigParser.SafeConfigParser()
        cfg.read(config_filename)
        
        config = {
            'logger_directory' : cfg.get(KerbalPie._CFG_GLOBALS_SECTION, 'logger_directory'),
            'logger_filename' : cfg.get(KerbalPie._CFG_GLOBALS_SECTION, 'logger_filename'),
            'krpc_address' : cfg.get(KerbalPie._CFG_KRPC_SECTION, 'krpc_address'),
            'krpc_client_name' : cfg.get(KerbalPie._CFG_KRPC_SECTION, 'krpc_client_name'),
            'krpc_rpc_port' : cfg.getint(KerbalPie._CFG_KRPC_SECTION, 'krpc_rpc_port'),
            'krpc_stream_port' : cfg.getint(KerbalPie._CFG_KRPC_SECTION, 'krpc_stream_port'),
        }
        
        return config
        
        
    def _clear_log_text(self):
        self.log_logTextEdit.clear()
    
    
    # G E T T E R S   /   S E T T E R S 
    #===========================================================================
    
    
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    
    
    
    
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#  E N T R Y   P O I N T   =#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=

if __name__ == '__main__':

    # parse command-line arguments
    #===========================================================================
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-c", "--config", 
        help="Specifies configuration file to use")
    arg_parser.add_argument("-d", "--debug", 
        help="Enable debug mode",
        action="store_true")
    args = arg_parser.parse_args()
    
    config_filename = args.config if args.config is not None else os.path.join('..', 'data', 'kerbalpie.cfg')
    
    
    # start a Qt GUI
    #===========================================================================
    app = QApplication(sys.argv)
    
    # start KerbalPie
    kerbalpie = KerbalPie(config_filename=config_filename, debug_on=args.debug)
    kerbalpie.show()
    
    # GUI event loop
    retval = app.exec_()
    
    # termination
    kerbalpie.close()
    sys.exit(retval)
    