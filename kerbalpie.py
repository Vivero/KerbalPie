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
from PyQt5.QtGui import QColor, QFont, QPalette, QPen
from PyQt5.QtWidgets import QAbstractItemView, QApplication, QHeaderView
from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget

from lib.logger import Logger
from lib.widgets.QPlot2D import QPlot2D, QPlot2DTime
from lib.widgets.QPidController import QPidControllerPanel
from lib.kp_flight_controller import KPFlightController
from lib.kp_flight_data import KPFlightDataModel
from lib.kp_mission_controller import KPMissionProgram, KPMissionProgramsModel, KPMissionProgramsDatabase
from lib.kp_serial_interface import KPSerialInterface
from lib.kp_tools import *

#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
    
class KerbalPie(QWidget):

    subsys = 'KERBALPIE'
    
    # configuration file named sections
    _CFG_GLOBALS_SECTION = 'GLOBALS'
    _CFG_KRPC_SECTION    = 'KRPC'
    _CFG_SERIAL_SECTION  = 'SERIAL'
    
    # S I G N A L S 
    #===========================================================================
    krpc_client_begin_connect = pyqtSignal()
    krpc_client_begin_disconnect = pyqtSignal()
    serial_iface_begin_connect = pyqtSignal()
    serial_iface_begin_disconnect = pyqtSignal()
    
    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, parent=None, config_filename=os.path.join('data', 'kerbalpie.cfg'), debug_on=False):
        super(KerbalPie, self).__init__(parent)
        uic.loadUi(os.path.join('data', 'KerbalPie.ui'), self)
        
        # Set up KerbalPie application
        #-----------------------------------------------------------------------
        
        # parse configuration options
        self.config = self._parse_config(config_filename)
        

        # general message logging
        #-----------------------------------------------------------------------
        self._logger_thread = Logger(
            log_dir=self.config['logger_directory'], 
            log_name=self.config['logger_filename'], 
            debug_on=True)
        self._logger_thread.start()
        

        # serial interface
        #-----------------------------------------------------------------------
        
        # interface thread
        self._serial_thread = QtCore.QThread()
        self._serial_iface = KPSerialInterface(
            serial_port=self.config['serial_port'], 
            serial_baudrate=self.config['serial_baudrate'])
        self._serial_iface.moveToThread(self._serial_thread)
        

        # flight controller
        #-----------------------------------------------------------------------
        
        # controller thread
        self._flight_thread = QtCore.QThread()
        self._flight_ctrl = KPFlightController(
            krpc_address=self.config['krpc_address'], 
            krpc_rpc_port=self.config['krpc_rpc_port'], 
            krpc_stream_port=self.config['krpc_stream_port'], 
            krpc_name=self.config['krpc_client_name'])
        self._flight_ctrl.moveToThread(self._flight_thread)
        

        # GUI elements
        #-----------------------------------------------------------------------

        # serial interface thread connections
        self._serial_thread.started.connect(self._serial_iface.process)
        self._serial_iface.finished.connect(self._serial_thread.quit)
        self._serial_iface.finished.connect(self._serial_iface.deleteLater)
        self._serial_thread.finished.connect(self._serial_thread.deleteLater)

        # serial interface connections
        self.serial_iface_begin_connect.connect(self._serial_iface.connect)
        self.serial_iface_begin_disconnect.connect(self._serial_iface.disconnect)
        self._serial_iface.connected.connect(self.serial_connected)
        self._serial_iface.disconnected.connect(self.serial_disconnected)
        self.serial_connectionButton.clicked.connect(self.serial_connectionButton_clicked)
        self.serial_portEdit.textChanged.connect(self.serial_port_changed)
        self.serial_baudRateEdit.textChanged.connect(self.serial_baudrate_changed)
        
        # PID controller panels
        self.flightControl_tuningTabGroup = QTabWidget(parent=None)
        self.flightControl_pidControllerPanels = []
        for controller in self._flight_ctrl.ctrl.values():
            ctrl_panel = QPidControllerPanel(controller, parent=None)
            self.flightControl_tuningTabGroup.addTab(ctrl_panel, controller.name)
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

        #self._serial_iface.rc_command.connect(self._flight_ctrl.rc_command_received)
        
        # start threads
        self._flight_thread.start()
        self._serial_thread.start()
        
        
        # flight data display
        #-----------------------------------------------------------------------
        
        # gui items
        self.flightPlot_selection.addItems([
            "Vertical Speed",
            "Altitude",
            "Attitude",
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
        
        self.controllerPlotter = QPlot2DTime(
            timeSpan=30.0,
            yMin=-6.0,
            yMax=6.0,
            yOriginValue=0.0,
            xTickInterval=5.0,
            yTickInterval=1.0,
            labelFont=QFont("Segoe UI", 10),
            refreshRate=0.1)
        self.controllerPlotter.setPlotDrawMethod(0, 'line')
        
        self.flightPlot_plotGroup.layout().addWidget(self.controllerPlotter)
        
        self.flightPlot_selection.currentTextChanged.connect(self.flightPlot_selection_changed)
        
        
        # mission programs table
        #-----------------------------------------------------------------------
        self.mission_program_db = self._flight_ctrl.mission_ctrl.mp_db
        
        self.mission_programs_model = KPMissionProgramsModel(mp_db=self.mission_program_db, parent=self)
        self.mission_programTableView.setModel(self.mission_programs_model)
        self.mission_programTableView.verticalHeader().setVisible(False)
        self.mission_programTableView.verticalHeader().sectionResizeMode(QHeaderView.Fixed)
        self.mission_programTableView.resizeColumnsToContents()
        self.mission_programTableView.resizeRowsToContents()
        self.mission_programTableView.setSelectionMode(QAbstractItemView.SingleSelection)

        # connect signals
        self._serial_iface.rc_command.connect(self._flight_ctrl.mission_ctrl.rc_command_update)
        self._flight_ctrl.mission_ctrl.active_mp_updated.connect(self.mission_program_changed)
        self.mission_activateButton.clicked.connect(self.mission_activateButton_clicked)
        
        # automatically start KRPC connection
        QTimer.singleShot(200, self.krpc_connectionButton_clicked)
        
        # automatically start serial port connection
        QTimer.singleShot(300, self.serial_connectionButton_clicked)
        
           
    
    # S L O T S 
    #===========================================================================
    @pyqtSlot()
    def serial_connected(self):
        self.serial_connectionLabel.setText("Status: Connected")
        self.serial_connectionButton.setText("Disconnect")

    @pyqtSlot()
    def serial_disconnected(self):
        self.serial_connectionLabel.setText("Status: Disconnected")
        self.serial_connectionButton.setText("Connect")
    
    @pyqtSlot()
    def serial_port_changed(self, text):
        self._serial_iface.port = text
        
    @pyqtSlot()
    def serial_baudrate_changed(self, text):
        (baudrate, ok) = text.toInt()
        if ok:
            self._serial_iface.baudrate = baudrate
        
    @pyqtSlot()
    def serial_connectionButton_clicked(self):
        if not self._serial_iface.is_connected:
            self.serial_iface_begin_connect.emit()
        else:
            self.serial_iface_begin_disconnect.emit()
    

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
            

    @pyqtSlot(KPMissionProgram)
    def mission_program_changed(self, mp):
        top_left  = self.mission_programs_model.index(0, 0)
        btm_right = self.mission_programs_model.index(
            self.mission_programs_model.rowCount(parent=self.mission_programs_model.parent), 
            self.mission_programs_model.columnCount(parent=self.mission_programs_model.parent))
        self.mission_programs_model.dataChanged.emit(top_left, btm_right)

    @pyqtSlot()
    def mission_activateButton_clicked(self):
        self._flight_ctrl.mission_ctrl.set_active_mp(self.mission_programTableView.currentIndex().row())

        
    @pyqtSlot(dict)
    def flight_telemetry_updated(self, telemetry_dict):
        #print("flight_telemetry_updated")

        # update the flight data table
        for param in telemetry_dict.keys():
            self.flight_data_model.update_flight_data(param, telemetry_dict[param])
                
        # plotter
        plotter_current_selection = self.flightPlot_selection.currentText()
        if plotter_current_selection == 'Vertical Speed':
            self.controllerPlotter.updatePlot(0, telemetry_dict['vessel_vertical_speed'])
        elif plotter_current_selection == 'Altitude':
            self.controllerPlotter.updatePlot(0, telemetry_dict['vessel_mean_altitude'])
            
        
    @pyqtSlot('QString')
    def flightPlot_selection_changed(self, text):
        self.controllerPlotter.clearPlots()
        if text == 'Vertical Speed':
            self.controllerPlotter.setYMin(-6.0)
            self.controllerPlotter.setYMax(6.0)
            self.controllerPlotter.setYTickInterval(1.0)
        elif text == 'Altitude':
            self.controllerPlotter.setYMin(70.0)
            self.controllerPlotter.setYMax(200.0)
            self.controllerPlotter.setYTickInterval(10.0)
            
    
    
    # O V E R R I D E   M E T H O D S 
    #===========================================================================
    
    
    # P R I V A T E   M E T H O D S 
    #===========================================================================
        
    def close(self):
        self._serial_iface.terminate = True
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
            'logger_directory'  : cfg.get(KerbalPie._CFG_GLOBALS_SECTION, 'logger_directory'),
            'logger_filename'   : cfg.get(KerbalPie._CFG_GLOBALS_SECTION, 'logger_filename'),
            'krpc_address'      : cfg.get(KerbalPie._CFG_KRPC_SECTION, 'krpc_address'),
            'krpc_client_name'  : cfg.get(KerbalPie._CFG_KRPC_SECTION, 'krpc_client_name'),
            'krpc_rpc_port'     : cfg.getint(KerbalPie._CFG_KRPC_SECTION, 'krpc_rpc_port'),
            'krpc_stream_port'  : cfg.getint(KerbalPie._CFG_KRPC_SECTION, 'krpc_stream_port'),
            'serial_port'       : cfg.get(KerbalPie._CFG_SERIAL_SECTION, 'serial_port'),
            'serial_baudrate'   : cfg.getint(KerbalPie._CFG_SERIAL_SECTION, 'serial_baudrate'),
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
    
    config_filename = args.config if args.config is not None else os.path.join('data', 'kerbalpie.cfg')
    
    
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
    