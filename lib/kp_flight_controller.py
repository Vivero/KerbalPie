import collections, krpc, math, time

from time import sleep

from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication, Qt, QTimer, QVariant, pyqtSignal
from PyQt5.QtCore import pyqtSlot

from lib.kp_tools import *
from lib.kp_mission_controller import KPMissionController, KPMissionProgram, KPMissionProgramsDatabase
from lib.kp_serial_interface import KPSerialInterface
from lib.logger import Logger
from lib.widgets.QPidController import QPidController


#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=

    
#--- Flight controller
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPFlightController(QtCore.QObject):

    subsys = 'CONTROL'
    sts_period  =  0.006    # Short Term Scheduler period (seconds)
    lts_period  =  0.100    # Long Term Scheduler period (seconds)
    xlts_period = 10.000    # Extra-Long Term Scheduler period (seconds)
        
    # S I G N A L S 
    #===========================================================================
    krpc_connected      = pyqtSignal()
    krpc_disconnected   = pyqtSignal()
    telemetry_updated   = pyqtSignal(dict)
    finished            = pyqtSignal()

    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, 
            krpc_address="127.0.0.1", 
            krpc_rpc_port=50000, 
            krpc_stream_port=50001, 
            krpc_name="KerbalPie",
            vessel_name=None,
            **kwds):
        super(KPFlightController, self).__init__(**kwds)
        
        # thread variables
        #-------------------------------
        self.terminate = False
        
        # mission controller
        #-------------------------------
        self.mission_ctrl = KPMissionController(parent=self)
        
        # KRPC client
        #-------------------------------
        self._krpc                                  = None
        self.krpc_is_connected                      = False
        self.krpc_client_name                       = krpc_name
        self.krpc_address                           = krpc_address
        self.krpc_rpc_port                          = krpc_rpc_port
        self.krpc_stream_port                       = krpc_stream_port

        # flight data
        #-------------------------------
        self._telemetry                             = {}
        self._telemetry['scheduler_timings']        = {}
        self._telemetry['scheduler_timings']['sts'] = StateVariable(0.0, int(1.0 / KPFlightController.sts_period + 0.5))
        self._telemetry['scheduler_timings']['lts'] = StateVariable(0.0, int(1.0 / KPFlightController.lts_period + 0.5))
        self._telemetry['game_scene']               = None
        self._telemetry['space_g']                  = 0.0
        self._telemetry['space_ut']                 = 0.0
        self._telemetry['vessel_is_active']         = False
        self._telemetry['vessel_to_control']        = vessel_name
        self._telemetry['vessel']                   = None
        self._telemetry['vessel_name']              = None
        self._telemetry['vessel_control']           = None
        self._telemetry['vessel_autopilot']         = None
        self._telemetry['vessel_flight']            = None
        self._telemetry['vessel_orbit']             = None
        self._telemetry['vessel_srf_reff']          = None
        self._telemetry['vessel_srf_vel_reff']      = None
        self._telemetry['vessel_position']          = None
        self._telemetry['vessel_mass']              = 0.0
        self._telemetry['vessel_thrust']            = 0.0
        self._telemetry['vessel_max_thrust']        = 0.0
        self._telemetry['vessel_throttle']          = 0.0
        self._telemetry['vessel_vertical_speed']    = 0.0
        self._telemetry['vessel_rotation']          = None
        self._telemetry['vessel_mean_altitude']     = 0.0
        self._telemetry['vessel_surface_altitude']  = 0.0
        self._telemetry['vessel_latitude']          = None
        self._telemetry['vessel_longitude']         = None
        self._telemetry['vessel_weight']            = 0.0
        self._telemetry['body']                     = StateVariable()
        self._telemetry['body_name']                = None
        self._telemetry['body_mass']                = 0.0
        self._telemetry['body_reff']                = None
        self._telemetry['body_gravity']             = 0.0

        # define a dictionary of streams.
        #   Special Streams  -- add/remove when the KRPC client is connected/disconnected
        #   Standard Streams -- add/remove when the currently orbiting body changes
        self._streams                               = {}

        # Special Streams
        self._streams['game_scene']                 = None

        # Standard Streams
        self._streams['vessel_position']            = None
        self._streams['vessel_mass']                = None
        self._streams['vessel_thrust']              = None
        self._streams['vessel_max_thrust']          = None
        self._streams['vessel_throttle']            = None
        self._streams['vessel_vertical_speed']      = None
        self._streams['vessel_rotation']            = None
        self._streams['vessel_mean_altitude']       = None
        self._streams['vessel_surface_altitude']    = None
        self._streams['vessel_latitude']            = None
        self._streams['vessel_longitude']           = None
        
        # flight automatic controls
        #-------------------------------
        self.ctrl                                   = {}

        # define automatic controllers
        ctrl_vertical_speed = QPidController(
            kp=0.181, 
            ki=0.09, 
            kd=0.005, 
            output_min=0.0, 
            output_max=1.0, 
            set_point=0.0, 
            name="Vertical Speed Controller", 
            parent=self)

        ctrl_altitude = QPidController(
            kp=1.5, 
            ki=0.005, 
            kd=0.005, 
            output_min=-5.0, 
            output_max=5.0, 
            set_point=85.0, 
            name="Altitude Controller", 
            parent=self)

        ctrl_attitude = QPidController(
            kp=1.5, 
            ki=0.005, 
            kd=0.005, 
            output_min=0.0, 
            output_max=5.0, 
            set_point=0.0, 
            name="Attitude Controller", 
            parent=self)

        self.ctrl['vspeed']                         = ctrl_vertical_speed
        self.ctrl['altitude']                       = ctrl_altitude
        self.ctrl['attitude']                       = ctrl_attitude
        
        # thread schedulers
        #-------------------------------
        self._short_term_scheduler                  = QTimer()
        self._long_term_scheduler                   = QTimer()
        self._xlong_term_scheduler                  = QTimer()
        
        self._short_term_scheduler.timeout.connect(self.short_term_processing)
        self._short_term_scheduler.start(KPFlightController.sts_period * 1000.0)
        
        self._long_term_scheduler.timeout.connect(self.long_term_processing)
        self._long_term_scheduler.start(KPFlightController.lts_period * 1000.0)
        
        self._xlong_term_scheduler.timeout.connect(self.xlong_term_processing)
        self._xlong_term_scheduler.start(KPFlightController.xlts_period * 1000.0)
        
        
        
    # M E T H O D S 
    #===========================================================================
        
        
    # O V E R R I D E   M E T H O D S 
    #===========================================================================

        
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    def _log(self, log_message, log_type='info', log_data=None):
        Logger.log(KPFlightController.subsys, log_message, log_type, log_data)
        
    def _log_warning(self, log_message, log_data=None):
        Logger.log_warning(KPFlightController.subsys, log_message, log_data)
        
    def _log_exception(self, log_message, log_exception):
        Logger.log_exception(KPFlightController.subsys, log_message, log_exception)
        
        
    # P R I V A T E   M E T H O D S 
    #===========================================================================
    def _setup_telemetry(self):

        # set the active vessel
        vessel_found = False

        # if a vessel name was given to this Flight Controller, search for it
        if self._telemetry['vessel_to_control'] is not None:
            for vessel in self._krpc.space_center.vessels:
                if vessel.name == self._telemetry['vessel_to_control']:
                    self._telemetry['vessel'] = vessel
                    vessel_found = True
                    break

        # otherwise, set the vessel to be the currently active vessel in the game
        if not vessel_found:
            self._telemetry['vessel'] = self._krpc.space_center.active_vessel

        vessel = self._telemetry['vessel']
        self._telemetry['vessel_name']              = vessel.name
        self._telemetry['vessel_is_active']         = True

        # obtain universal params
        self._telemetry['space_g']                  = self._krpc.space_center.g

        # obtain the orbiting body params
        body = vessel.orbit.body
        self._telemetry['body'].update(body)
        self._telemetry['body_name']                = body.name
        self._telemetry['body_mass']                = body.mass
        self._telemetry['body_reff']                = body.reference_frame
        
        # obtain vessel params
        vessel_flight = vessel.flight(body.reference_frame)
        self._telemetry['vessel_control']           = vessel.control
        self._telemetry['vessel_autopilot']         = vessel.auto_pilot
        self._telemetry['vessel_flight']            = vessel_flight
        self._telemetry['vessel_orbit']             = vessel.orbit
        self._telemetry['vessel_srf_reff']          = vessel.surface_reference_frame
        self._telemetry['vessel_srf_vel_reff']      = vessel.surface_velocity_reference_frame
        
        # add telemetry streams
        self._streams['space_ut']                   = self._krpc.add_stream(getattr, self._krpc.space_center, 'ut')
        self._streams['vessel_position']            = self._krpc.add_stream(vessel.position, body.reference_frame)
        self._streams['vessel_mass']                = self._krpc.add_stream(getattr, vessel, 'mass')
        self._streams['vessel_thrust']              = self._krpc.add_stream(getattr, vessel, 'thrust')
        self._streams['vessel_max_thrust']          = self._krpc.add_stream(getattr, vessel, 'max_thrust')
        self._streams['vessel_throttle']            = self._krpc.add_stream(getattr, vessel.control, 'throttle')
        self._streams['vessel_vertical_speed']      = self._krpc.add_stream(getattr, vessel_flight, 'vertical_speed')
        self._streams['vessel_rotation']            = self._krpc.add_stream(getattr, vessel_flight, 'rotation')
        self._streams['vessel_mean_altitude']       = self._krpc.add_stream(getattr, vessel_flight, 'mean_altitude')
        self._streams['vessel_surface_altitude']    = self._krpc.add_stream(getattr, vessel_flight, 'surface_altitude')
        self._streams['vessel_latitude']            = self._krpc.add_stream(getattr, vessel_flight, 'latitude')
        self._streams['vessel_longitude']           = self._krpc.add_stream(getattr, vessel_flight, 'longitude')

        self._log('Tracking vessel "{:s}"'.format(self._telemetry['vessel_name']))


    def _remove_telemetry(self):
        self._streams['space_ut'].remove()
        self._streams['vessel_position'].remove()
        self._streams['vessel_mass'].remove()
        self._streams['vessel_thrust'].remove()
        self._streams['vessel_max_thrust'].remove()
        self._streams['vessel_throttle'].remove()
        self._streams['vessel_vertical_speed'].remove()
        self._streams['vessel_rotation'].remove()
        self._streams['vessel_mean_altitude'].remove()
        self._streams['vessel_surface_altitude'].remove()
        self._streams['vessel_latitude'].remove()
        self._streams['vessel_longitude'].remove()
        
        
    def _sts_telemetry_update(self):
        # obtain telemetry data
        for stream_key in self._streams:
            self._telemetry[stream_key] = self._streams[stream_key]()
        
        # compute telemetry parameters
        body_to_vessel_distance_sq = (
            self._telemetry['vessel_position'][0] ** 2
            + self._telemetry['vessel_position'][1] ** 2
            + self._telemetry['vessel_position'][2] ** 2)
        
        self._telemetry['body_gravity'] = self._telemetry['space_g'] * self._telemetry['body_mass'] / body_to_vessel_distance_sq
        self._telemetry['vessel_weight'] = self._telemetry['body_gravity'] * self._telemetry['vessel_mass']


    def _lts_telemetry_update(self):
        # check the current orbiting body
        self._telemetry['body'].update(self._telemetry['vessel'].orbit.body)
        body = self._telemetry['body'].get()

        # if orbiting body has changed, reset telemetry
        if self._telemetry['body'].has_changed():
            self._remove_telemetry()
            self._setup_telemetry()

    
    def _krpc_heartbeat(self):
        try:
            status = self._krpc.krpc.get_status().version
            
        except ConnectionAbortedError as abort:
            self._log_exception('KRPC connection aborted', abort)
            self.krpc_disconnect()
        except ConnectionResetError as reset:
            self._log_exception('KRPC connection reset by host', reset)
            self.krpc_disconnect()



    # S L O T S 
    #===========================================================================
    @pyqtSlot()
    def short_term_processing(self):
        start_time = time.time()
        #--
        
        if self.krpc_is_connected:

            # check the game scene
            self._telemetry['game_scene'] = self._streams['game_scene']()

            # if we are in the Flight scene, continue processing
            if self._telemetry['game_scene'] == self._krpc.krpc.GameScene.flight:

                # set up telemetry for the vessel if it hasn't already been done
                if not self._telemetry['vessel_is_active']:
                    self._setup_telemetry()

                else:
                    self._sts_telemetry_update()

            else:
                # otherwise, unset the active vessel and telemetry
                self._telemetry['vessel_is_active'] = False

            # emit telemetry updates
            self.telemetry_updated.emit(self._telemetry)
        
        #--
        process_time = time.time() - start_time
        self._telemetry['scheduler_timings']['sts'].update(process_time)
        if process_time > KPFlightController.sts_period:
            self._log_warning('STS overrun: {:.1f} ms, overrun by {:.3f} ms'.format(
                process_time * 1000.0, (process_time - KPFlightController.sts_period) * 1000.0))
        
            
        
    @pyqtSlot()
    def long_term_processing(self):
        start_time = time.time()
        #--
        
        if self.krpc_is_connected:

            # check the game scene
            self._telemetry['game_scene'] = self._streams['game_scene']()

            # if we are in the Flight scene, continue processing
            if self._telemetry['game_scene'] == self._krpc.krpc.GameScene.flight:

                # run LTS telemetry if vessel is currently active
                if self._telemetry['vessel_is_active']:
                    self._lts_telemetry_update()
        
        
        #--
        process_time = time.time() - start_time
        self._telemetry['scheduler_timings']['lts'].update(process_time)
        if process_time > KPFlightController.lts_period:
            self._log_warning('LTS overrun: {:.1f} ms, overrun by {:.1f} ms'.format(
                process_time * 1000.0, (process_time - KPFlightController.lts_period) * 1000.0))
        

    @pyqtSlot()
    def xlong_term_processing(self):
        if self.krpc_is_connected:
            self._krpc_heartbeat()
        
    
    @pyqtSlot()
    def process(self):
        while not self.terminate:

            # service Qt events
            QCoreApplication.processEvents()
            
        # thread termination
        self.krpc_disconnect()
        self._log('Flight controller thread terminating...')
        self.finished.emit()
        
        
    @pyqtSlot()
    def krpc_connect(self):
        try:
            # attempt to connect
            self._log('Connecting to KRPC at {:s}:{:d} ...'.format(self.krpc_address, self.krpc_rpc_port))
            self._krpc = krpc.connect(
                name=self.krpc_client_name, 
                address=self.krpc_address, 
                rpc_port=self.krpc_rpc_port, 
                stream_port=self.krpc_stream_port)
            
            # emit succesful connection signals
            self.krpc_is_connected = True
            self.krpc_connected.emit()
            self._log('Connected to KRPC version {:s}'.format(self._krpc.krpc.get_status().version))

            # connect Special Streams
            self._streams['game_scene'] = self._krpc.add_stream(getattr, self._krpc.krpc, 'current_game_scene')

            # Set the mission program
            self.mission_ctrl.set_default_mp()
            
            
            
        except krpc.error.NetworkError as e:
            self._log_exception('Unable to connect to KRPC server', e)
            
        
    @pyqtSlot()
    def krpc_disconnect(self):
        if self._krpc is not None:
            self._krpc.close()
            self.krpc_is_connected = False
            self.krpc_disconnected.emit()
            self._log('Disconnected from KRPC server')
