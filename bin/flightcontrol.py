import krpc, math, time

from time import sleep

from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication, Qt, QTimer, QVariant, pyqtSignal, pyqtSlot

from kptools import PidController, vector_dot_product, vector_length, vector_normalize, vector_project_onto_plane
from logger import KPLogger
from missioncontrol import MissionProgram, MissionProgramsDatabase
from widgets.PidControllerQ import PidControllerQ


#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=

    
#--- Flight controller
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPFlightController(QtCore.QObject):

    subsys = 'CONTROL'
    control_period = 0.100
        
    # S I G N A L S 
    #===========================================================================
    krpc_connected = pyqtSignal()
    krpc_disconnected = pyqtSignal()
    telemetry_updated = pyqtSignal(dict)
    finished = pyqtSignal()

    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, krpc_address="127.0.0.1", krpc_rpc_port=50000, krpc_stream_port=50001, krpc_name="KerbalPie", **kwds):
        super(KPFlightController, self).__init__(**kwds)
        
        # thread variables
        self.terminate = False
        self._current_time = time.time()
        self._previous_time = self._current_time
        
        # KRPC client
        self._krpc = None
        self.krpc_is_connected = False
        self.krpc_client_name = krpc_name
        self.krpc_address = krpc_address
        self.krpc_rpc_port = krpc_rpc_port
        self.krpc_stream_port = krpc_stream_port
        self._krpc_heartbeat_time = time.time()
        self._krpc_heartbeat_period = 10.0 # seconds
        
        # flight data
        self._telemetry = {}
        
        # flight controllers
        self.vertical_speed_ctrl = PidControllerQ(kp=0.181, ki=0.09, kd=0.005, output_min=0.0, output_max=1.0, set_point=0.0, name="Vertical Speed Controller", parent=self)
        self.altitude_ctrl = PidControllerQ(kp=1.0, ki=0.005, kd=0.005, output_min=-5.0, output_max=5.0, set_point=85.0, name="Altitude Controller", parent=self)
        self.controllers = []
        self.controllers.append(self.vertical_speed_ctrl)
        self.controllers.append(self.altitude_ctrl)
        
        # mission program
        self._mission_program = None
        
        
        
    # M E T H O D S 
    #===========================================================================
        
        
    # O V E R R I D E   M E T H O D S 
    #===========================================================================
        
        
    # P R I V A T E   M E T H O D S 
    #===========================================================================
    def _setup_telemetry(self):
        # obtain universal params
        self._space_g = self._krpc.space_center.g
        
        # obtain vessel params
        self._vessel            = self._krpc.space_center.active_vessel
        self._vessel_name       = self._vessel.name
        self._vessel_control    = self._vessel.control
        self._vessel_autopilot  = self._vessel.auto_pilot
        
        
        # add telemetry streams
        self._space_ut                  = self._krpc.add_stream(getattr, self._krpc.space_center, 'ut')
        self._vessel_orbit              = self._krpc.add_stream(getattr, self._vessel, 'orbit')
        self._vessel_body               = self._krpc.add_stream(getattr, self._vessel_orbit(), 'body')
        self._vessel_body_mass          = self._krpc.add_stream(getattr, self._vessel_body(), 'mass')
        self._vessel_body_name          = self._krpc.add_stream(getattr, self._vessel_body(), 'name')
        self._vessel_surface_ref        = self._krpc.add_stream(getattr, self._vessel, 'surface_reference_frame')
        self._vessel_surface_vel_ref    = self._krpc.add_stream(getattr, self._vessel, 'surface_velocity_reference_frame')
        self._vessel_body_ref           = self._krpc.add_stream(getattr, self._vessel_body(), 'reference_frame')
        self._vessel_flight_bdy         = self._krpc.add_stream(self._vessel.flight, self._vessel_body_ref())
        self._vessel_flight_srf         = self._krpc.add_stream(self._vessel.flight, self._vessel_surface_ref())
        
        self._vessel_position_bdy       = self._krpc.add_stream(self._vessel.position, self._vessel_body_ref())
        self._vessel_vertical_speed     = self._krpc.add_stream(getattr, self._vessel_flight_bdy(), 'vertical_speed')
        self._vessel_mass               = self._krpc.add_stream(getattr, self._vessel, 'mass')
        self._vessel_thrust             = self._krpc.add_stream(getattr, self._vessel, 'thrust')
        self._vessel_max_thrust         = self._krpc.add_stream(getattr, self._vessel, 'max_thrust')
        self._vessel_throttle           = self._krpc.add_stream(getattr, self._vessel_control, 'throttle')
        self._vessel_mean_altitude      = self._krpc.add_stream(getattr, self._vessel_flight_bdy(), 'mean_altitude')
        self._vessel_surface_altitude   = self._krpc.add_stream(getattr, self._vessel_flight_bdy(), 'surface_altitude')
        
        
        # draw vectors
        self._krpc.space_center.clear_drawing()
        #self._krpc.space_center.draw_direction((1.0, 0.0, 0.0), self._vessel_surface_vel_ref(), (255, 0, 0), 10.0)
        #self._krpc.space_center.draw_direction((0.0, 1.0, 0.0), self._vessel_surface_vel_ref(), (0, 255, 0), 10.0)
        #self._krpc.space_center.draw_direction((0.0, 0.0, 1.0), self._vessel_surface_vel_ref(), (0, 0, 255), 10.0)
        
        
        self._log('Tracking vessel "{:s}"'.format(self._vessel.name))
        
        
    def _telemetry_update(self):
        # obtain telemetry data
        self._telemetry['g']                    = self._space_g
        self._telemetry['ut']                   = self._space_ut()
        self._telemetry['vessel']               = self._vessel
        self._telemetry['vessel_name']          = self._vessel_name
        self._telemetry['vessel_control']       = self._vessel_control
        self._telemetry['vessel_autopilot']     = self._vessel_autopilot
        self._telemetry['vessel_orbit']         = self._vessel_orbit()
        self._telemetry['vessel_body']          = self._vessel_body()
        self._telemetry['vessel_body_mass']     = self._vessel_body_mass()
        self._telemetry['vessel_body_name']     = self._vessel_body_name()
        self._telemetry['vessel_surface_ref']   = self._vessel_surface_ref()
        self._telemetry['vessel_surface_vel_ref'] = self._vessel_surface_vel_ref()
        self._telemetry['vessel_body_ref']      = self._vessel_body_ref()
        self._telemetry['vessel_flight_bdy']    = self._vessel_flight_bdy()
        self._telemetry['vessel_flight_srf']    = self._vessel_flight_srf()
        
        self._telemetry['vessel_position_bdy']      = self._vessel_position_bdy()
        self._telemetry['vessel_vertical_speed']    = self._vessel_vertical_speed()
        self._telemetry['vessel_mass']              = self._vessel_mass()
        self._telemetry['vessel_thrust']            = self._vessel_thrust()
        self._telemetry['vessel_max_thrust']        = self._vessel_max_thrust()
        self._telemetry['vessel_throttle']          = self._vessel_throttle()
        self._telemetry['vessel_mean_altitude']     = self._vessel_mean_altitude()
        self._telemetry['vessel_surface_altitude']  = self._vessel_surface_altitude()
        
        # compute telemetry parameters
        body_to_vessel_distance_sq = self._telemetry['vessel_position_bdy'][0] ** 2 + self._telemetry['vessel_position_bdy'][1] ** 2 + self._telemetry['vessel_position_bdy'][2] ** 2
        
        self._telemetry['vessel_body_gravity'] = self._space_g * self._telemetry['vessel_body_mass'] / body_to_vessel_distance_sq
        self._telemetry['vessel_weight'] = self._telemetry['vessel_body_gravity'] * self._telemetry['vessel_mass']
        
        velocity_rel_to_surface = self._krpc.space_center.transform_direction((0,1.0,0), self._telemetry['vessel_surface_vel_ref'], self._telemetry['vessel_surface_ref'])
        srf_velocity_rel_to_surface = vector_project_onto_plane(velocity_rel_to_surface, (1.0, 0.0, 0.0))
        
        # draw vectors
        #self._krpc.space_center.clear_drawing()
        #self._krpc.space_center.draw_direction((1.0, 0.0, 0.0), self._telemetry['vessel_surface_vel_ref'], (255, 0, 0), 10.0)
        #self._krpc.space_center.draw_direction((0.0, 1.0, 0.0), self._telemetry['vessel_surface_vel_ref'], (0, 255, 0), 10.0)
        #self._krpc.space_center.draw_direction((0.0, 0.0, 1.0), self._telemetry['vessel_surface_vel_ref'], (0, 0, 255), 10.0)
        #self._krpc.space_center.draw_direction(velocity_rel_to_surface, self._telemetry['vessel_surface_ref'], (255,255,0), 10.0)
        
        #self._krpc.space_center.draw_direction(srf_velocity_rel_to_surface, self._telemetry['vessel_surface_ref'], (255,255,0), 10.0 * vector_length(srf_velocity_rel_to_surface))
    
        # finish telemetry update
        self.telemetry_updated.emit(self._telemetry)
        
        
        
    def _control_update(self):
        if self._mission_program is not None:
            mp_id = self._mission_program.id
    
            if mp_id == 'full_manual':
                return
                
            # Control vertical speed without automatic tuning of controller gains
            elif mp_id == 'vspeed_manual':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    throttle_cmd = self.vertical_speed_ctrl.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd
                
            # Control vertical speed with automatic tuning of controller gains
            elif mp_id == 'vspeed_auto':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    # set control gains
                    ku = self._telemetry['vessel_weight'] / self._telemetry['vessel_max_thrust']
                    #self.vertical_speed_ctrl.kp = ku * 0.70
                    #self.vertical_speed_ctrl.ki = ku / 3.0
                    #self.vertical_speed_ctrl.kd = ku / 50.0
                    self.vertical_speed_ctrl.setProportionalGain(ku * 0.70)
                    self.vertical_speed_ctrl.setIntegralGain(ku / 3.0)
                    self.vertical_speed_ctrl.setDerivativeGain(ku / 50.0)
                    
                    #throttle_cmd = self.vertical_speed_ctrl.update(self._telemetry['vessel_vertical_speed'])
                    throttle_cmd = self.vertical_speed_ctrl.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd
                    
            # Control altitude without automatic tuning of controller gains
            elif mp_id == 'altitude_manual':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    vspeed_cmd = self.altitude_ctrl.update(self._telemetry['vessel_mean_altitude'])
                    self.vertical_speed_ctrl.setSetpoint(vspeed_cmd)
                    
                    throttle_cmd = self.vertical_speed_ctrl.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd
                    
            # Control altitude with automatic tuning of speed controller gains
            elif mp_id == 'altitude_auto':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    vspeed_cmd = self.altitude_ctrl.update(self._telemetry['vessel_mean_altitude'])
                    
                    # set control gains
                    ku = self._telemetry['vessel_weight'] / self._telemetry['vessel_max_thrust']
                    self.vertical_speed_ctrl.setProportionalGain(ku * 0.70)
                    self.vertical_speed_ctrl.setIntegralGain(ku / 3.0)
                    self.vertical_speed_ctrl.setDerivativeGain(ku / 50.0)
                    self.vertical_speed_ctrl.setSetpoint(vspeed_cmd)
                    throttle_cmd = self.vertical_speed_ctrl.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd
                
            # Controlled descent that varies vertical speed according to altitude
            elif mp_id == 'controlled_descent':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    # set control gains
                    ku = self._telemetry['vessel_weight'] / self._telemetry['vessel_max_thrust']
                    self.vertical_speed_ctrl.setProportionalGain(ku * 0.70)
                    self.vertical_speed_ctrl.setIntegralGain(ku / 3.0)
                    self.vertical_speed_ctrl.setDerivativeGain(ku / 50.0)
                    self.vertical_speed_ctrl.setSetpoint(self._telemetry['vessel_surface_altitude'] / -12.0 - 1.0)
                
                    throttle_cmd = self.vertical_speed_ctrl.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd
                
                
    def _set_active_program_settings(self):
        self.vertical_speed_ctrl.setSetpointEditable(self._mission_program.settings['vertical_speed_controller_setpoint_editable'])
        self.vertical_speed_ctrl.setGainsEditable(self._mission_program.settings['vertical_speed_controller_gains_editable'])
        self.altitude_ctrl.setSetpointEditable(self._mission_program.settings['altitude_controller_setpoint_editable'])
        self.altitude_ctrl.setGainsEditable(self._mission_program.settings['altitude_controller_gains_editable'])
        
        
    
    def _krpc_heartbeat(self):
        if self._current_time > (self._krpc_heartbeat_time + self._krpc_heartbeat_period):
            self._krpc_heartbeat_time = self._current_time
            try:
                status = self._krpc.krpc.get_status().version
                
            except ConnectionAbortedError as ae:
                self._log_exception('KRPC connection aborted', ae)
                self.krpc_disconnect()
            except ConnectionResetError as re:
                self._log_exception('KRPC connection reset by host', re)
                self.krpc_disconnect()
    
    
    # S L O T S 
    #===========================================================================
    @pyqtSlot()
    def process(self):
        while not self.terminate:
            self._current_time = time.time()
            delta_t = self._current_time - self._previous_time
            
            # service KRPC
            telemetry_time = 0.0
            control_time = 0.0
            if self.krpc_is_connected:
                self._telemetry_update()
                self._control_update()
                self._krpc_heartbeat()
                
            # service Qt events
            QCoreApplication.processEvents()
            
            # sleep to next period
            self._previous_time = self._current_time
            processing_time = time.time() - self._current_time
            sleep_time = KPFlightController.control_period - processing_time
            
            #print("PROC: {:6.3f}, SLEEP: {:6.3f}, DT: {:6.3f}, TEL:{:6.3f}, CTL:{:6.3f}".format(processing_time, sleep_time, delta_t, telemetry_time, control_time))
            
            if sleep_time > 0.0:
                sleep(sleep_time)
            else:
                self._log_warning('Processing overrun: Control time = {:.1f} ms, overrun by {:.1f} ms'.format(processing_time * 1000.0, sleep_time * -1000.0))
            
            
        # thread termination
        self._log('Flight control thread terminating...')
        self.finished.emit()
        
    @pyqtSlot(MissionProgram)
    def set_active_program(self, program):
        self._mission_program = program
        self._set_active_program_settings()
                
        
    @pyqtSlot()
    def krpc_connect(self):
        try:
            # attempt to connect
            self._log('Connecting to KRPC at {:s}:{:d} ...'.format(self.krpc_address, self.krpc_rpc_port))
            self._krpc = krpc.connect(name=self.krpc_client_name, address=self.krpc_address, rpc_port=self.krpc_rpc_port, stream_port=self.krpc_stream_port)
            
            # perform initial setup
            self._setup_telemetry()
            
            # emit succesful connection signals
            self.krpc_is_connected = True
            self.krpc_connected.emit()
            
            self._log('Connected to KRPC version {:s}'.format(self._krpc.krpc.get_status().version))
            
            
        except krpc.error.NetworkError as e:
            self._log_exception('Unable to connect to KRPC server', e)
            
        
    @pyqtSlot()
    def krpc_disconnect(self):
        if self._krpc is not None:
            self._krpc.close()
            self.krpc_is_connected = False
            self.krpc_disconnected.emit()
            self._log('Disconnected from KRPC server')
            
            
    
        
        
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    def _log(self, log_message, log_type='info', log_data=None):
        KPLogger.log(KPFlightController.subsys, log_message, log_type, log_data)
        
    def _log_warning(self, log_message, log_data=None):
        KPLogger.log_warning(KPFlightController.subsys, log_message, log_data)
        
    def _log_exception(self, log_message, log_exception):
        KPLogger.log_exception(KPFlightController.subsys, log_message, log_exception)
    
    
    

#--- Qt Model for storing Flight data
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPFlightDataModel(QtCore.QAbstractTableModel):

    subsys = 'FLIGHT'
        
    flight_data_lookup = [
        'vessel_name',
        'ut',
        'vessel_body_name',
        'vessel_mass',
        'vessel_body_gravity',
        'vessel_weight',
        'vessel_forward_speed',
        'vessel_vertical_speed',
        'vessel_mean_altitude',
        'vessel_surface_altitude',
        'vessel_throttle',
        'vessel_thrust',
        'vessel_max_thrust',
    ]
        
    # S I G N A L S 
    #===========================================================================

    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, **kwds):
        super(KPFlightDataModel, self).__init__(**kwds)
        
        # set up model data
        self._flight_data_header = ['Parameter', 'Units', 'Value']
        
        self._flight_data = [
            ['Vessel Name',       'n/a',      ''],
            ['Universal Time',    's',        0.0],
            ['Planet Name',       'n/a',      ''],
            ['Mass',              'kg',       0.0],
            ['Gravity',           'm/s^2',    0.0],
            ['Weight',            'N',        0.0],
            ['Forward Speed',     'm/s',      0.0],
            ['Vertical Speed',    'm/s',      0.0],
            ['Altitude',          'm',        0.0],
            ['Radar Altitude',    'm',        0.0],
            ['Throttle',          'n/a',      0.0],
            ['Thrust',            'N',        0.0],
            ['Max Thrust',        'N',        0.0],
        ]
        
        
    # M E T H O D S 
    #===========================================================================
    def update_flight_data(self, parameter, value):
        if parameter in KPFlightDataModel.flight_data_lookup:
            row = KPFlightDataModel.flight_data_lookup.index(parameter)
            col = 2
            
            model_index = self.createIndex(row, col)
            
            self.setData(model_index, QVariant(value), Qt.EditRole)
        
        
    # O V E R R I D E   M E T H O D S 
    #===========================================================================
    def rowCount(self, parent):
        return len(self._flight_data)
            
    def columnCount(self, parent):
        return len(self._flight_data_header)
        
    def data(self, index, role):
        if role == Qt.DisplayRole:
            row = index.row()
            col = index.column()
        
            return QVariant(self._flight_data[row][col])
            
        return QVariant()
        
    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return QVariant(self._flight_data_header[section])
        
    def flags(self, index):
        flags = Qt.ItemIsEnabled
        return flags
        
        
    def setData(self, index, value, role):
        if index.isValid() and role == Qt.EditRole:
            row = index.row()
            col = index.column()
            
            self._flight_data[row][col] = value
            self.dataChanged.emit(index, index)
            
        
        
    # P R I V A T E   M E T H O D S 
    #===========================================================================
    
    
    # S L O T S 
    #===========================================================================
        
        
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    
    