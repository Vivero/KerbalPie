import collections, krpc, math, time

from time import sleep

from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication, Qt, QTimer, QVariant, pyqtSignal
from PyQt5.QtCore import pyqtSlot

from lib.kp_tools import *
from lib.kp_mission_control import KPMissionProgram, KPMissionProgramsDatabase
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
    sts_period = 0.050      # Short Term Scheduler period (seconds)
    lts_period = 0.200      # Long Term Scheduler period (seconds)
    xlts_period = 10.0      # Extra-Long Term Scheduler period (seconds)
        
    # S I G N A L S 
    #===========================================================================
    krpc_connected = pyqtSignal()
    krpc_disconnected = pyqtSignal()
    telemetry_updated = pyqtSignal(dict)
    request_mp_change = pyqtSignal(str)
    finished = pyqtSignal()

    
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
        self.terminate = False
        self._timestamp = StateVariable(time.time())
        
        # KRPC client
        self._krpc = None
        self.krpc_is_connected = False
        self.krpc_client_name = krpc_name
        self.krpc_address = krpc_address
        self.krpc_rpc_port = krpc_rpc_port
        self.krpc_stream_port = krpc_stream_port
        self._krpc_game_scene = None
        self._vessel_to_control = vessel_name
        self._vessel_is_active = False

        # remote control data
        self._rc_master_switch_engine    = False
        self._rc_master_switch_autopilot = False
        self._rc_button_stabilize        = False
        self._rc_button_increment        = StateVariable(False)
        self._rc_button_decrement        = StateVariable(False)
        self._rc_joystick_x              = 0.0
        self._rc_joystick_y              = 0.0
        self._rc_joystick_z              = 0.0
        
        # flight data
        self._telemetry = {}
        
        # signals data
        self._signals = {}
        self._signals_task = 0
        self._signals_update_time = time.time()
        self._radar_resolution = 30
        
        # flight automatic controls
        self.ctrl_vertical_speed = QPidController(kp=0.181, ki=0.09, kd=0.005, output_min=0.0, output_max=1.0, set_point=0.0, name="Vertical Speed Controller", parent=self)
        self.ctrl_altitude = QPidController(kp=1.5, ki=0.005, kd=0.005, output_min=-5.0, output_max=5.0, set_point=85.0, name="Altitude Controller", parent=self)
        self.ctrl_attitude = QPidController(kp=1.5, ki=0.005, kd=0.005, output_min=0.0, output_max=5.0, set_point=0.0, name="Attitude Controller", parent=self)
        self.controllers = []
        self.controllers.append(self.ctrl_vertical_speed)
        self.controllers.append(self.ctrl_altitude)
        self.controllers.append(self.ctrl_attitude)
        self._kill_horizontal_velocity = StateVariable(False)
        
        # mission program
        self._mission_program = None
        
        # timing data
        self._scheduler_timings = {}
        self._scheduler_timings['sts'] = StateVariable(0.0, int(1.0 / KPFlightController.sts_period + 0.5))
        self._scheduler_timings['lts'] = StateVariable(0.0, int(1.0 / KPFlightController.lts_period + 0.5))
        
        # initialize data
        self._vessel_allow_engines   = StateVariable()
        self._vessel_allow_autopilot = StateVariable()
        self._vessel_body            = StateVariable()
        self._vessel_control_sas     = False
        self._telemetry['surface_height_map'] = [[0.0 for x in range(self._radar_resolution)] for y in range(self._radar_resolution)]
        
        # initialize control timers
        self._short_term_scheduler = QTimer()
        self._long_term_scheduler = QTimer()
        self._xlong_term_scheduler = QTimer()
        
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

    #=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%#
    #=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%#
    #                                                                          #
    #        ==#==  #====  |     #====  #\  /#  #====  ==#==  #==\  \  /       #
    #          |    #--    |     #--    # \/ #  #--      |    #--/   \/        #
    #          |    #====  #===  #====  #    #  #====    |    #  \   |         #
    #                                                                          #
    #=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%#
    #=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%#
    def _setup_telemetry(self):

        # set the active vessel
        vessel_found = False

        # if a vessel name was given to this Flight Controller, search for it
        if self._vessel_to_control is not None:
            for vessel in self._krpc.space_center.vessels:
                if vessel.name == self._vessel_to_control:
                    self._vessel = vessel
                    vessel_found = True
                    break

        # otherwise, set the vessel to be the currently active vessel in the game
        if not vessel_found:
            self._vessel = self._krpc.space_center.active_vessel

        self._vessel_name = self._vessel.name
        self._vessel_is_active = True

        # obtain universal params
        self._space_g = self._krpc.space_center.g
        
        # obtain vessel params
        self._vessel_control            = self._vessel.control
        self._vessel_autopilot          = self._vessel.auto_pilot
        self._vessel_orbit              = self._vessel.orbit
        self._vessel_body.update(self._vessel.orbit.body)
        self._vessel_body_mass          = self._vessel_body.get().mass
        self._vessel_body_name          = self._vessel_body.get().name
        self._vessel_surface_reff       = self._vessel.surface_reference_frame
        self._vessel_surface_vel_reff   = self._vessel.surface_velocity_reference_frame
        self._vessel_body_reff          = self._vessel_body.get().reference_frame
        self._vessel_flight_bdy         = self._vessel.flight(self._vessel_body_reff)
        self._vessel_flight_srf         = self._vessel.flight(self._vessel_surface_reff)

        
        # add telemetry streams
        self._space_ut                  = self._krpc.add_stream(getattr, self._krpc.space_center, 'ut')
        self._vessel_position_bdy       = self._krpc.add_stream(self._vessel.position, self._vessel_body_reff)
        self._vessel_mass               = self._krpc.add_stream(getattr, self._vessel, 'mass')
        self._vessel_thrust             = self._krpc.add_stream(getattr, self._vessel, 'thrust')
        self._vessel_max_thrust         = self._krpc.add_stream(getattr, self._vessel, 'max_thrust')
        self._vessel_throttle           = self._krpc.add_stream(getattr, self._vessel_control, 'throttle')
        self._vessel_vertical_speed     = self._krpc.add_stream(getattr, self._vessel_flight_bdy, 'vertical_speed')
        self._vessel_rotation           = self._krpc.add_stream(getattr, self._vessel_flight_bdy, 'rotation')
        self._vessel_mean_altitude      = self._krpc.add_stream(getattr, self._vessel_flight_bdy, 'mean_altitude')
        self._vessel_surface_altitude   = self._krpc.add_stream(getattr, self._vessel_flight_bdy, 'surface_altitude')
        self._vessel_latitude           = self._krpc.add_stream(getattr, self._vessel_flight_bdy, 'latitude')
        self._vessel_longitude          = self._krpc.add_stream(getattr, self._vessel_flight_bdy, 'longitude')

        # set up drawings
        '''
        body_to_zenith_vec = self._krpc.space_center.transform_direction((5,0,0), self._vessel.surface_reference_frame, self._vessel_body_reff)
        offset_vessel_to_zenith_vec = vector_add(self._vessel_position_bdy(), body_to_zenith_vec)
        self._dwg_dir_vessel_to_zenith = self._krpc.drawing.add_line(self._vessel_position_bdy(), offset_vessel_to_zenith_vec, self._vessel_body_reff)
        self._dwg_dir_vessel_to_zenith.color = (0.5, 0.0, 0.0)

        self._dwg_dir_vessel_to_north = self._krpc.drawing.add_line(self._vessel_position_bdy(), offset_vessel_to_zenith_vec, self._vessel_body_reff)
        self._dwg_dir_vessel_to_north.color = (0.0, 0.5, 0.0)

        self._dwg_dir_vessel_to_east = self._krpc.drawing.add_line(self._vessel_position_bdy(), offset_vessel_to_zenith_vec, self._vessel_body_reff)
        self._dwg_dir_vessel_to_east.color = (0.0, 0.0, 0.5)

        self._dwg_velocity = self._krpc.drawing.add_line((0,0,0), (0,5,0), self._vessel.surface_velocity_reference_frame, True)
        self._dwg_velocity.color = (1.0, 0.0, 1.0)

        self._dwg_counter_vel = self._krpc.drawing.add_line(self._vessel_position_bdy(), offset_vessel_to_zenith_vec, self._vessel_body_reff)
        self._dwg_counter_vel.color = (0.0, 0.6, 0.7)
        '''

        self._log('Tracking vessel "{:s}"'.format(self._vessel.name))


    def _remove_telemetry(self):
        self._space_ut.remove()
        self._vessel_position_bdy.remove()
        self._vessel_mass.remove()
        self._vessel_thrust.remove()
        self._vessel_max_thrust.remove()
        self._vessel_throttle.remove()
        self._vessel_vertical_speed.remove()
        self._vessel_rotation.remove()
        self._vessel_mean_altitude.remove()
        self._vessel_surface_altitude.remove()
        self._vessel_latitude.remove()
        self._vessel_longitude.remove()
        
        
    def _telemetry_update(self):
        # check the current orbiting body
        self._vessel_body.update(self._vessel.orbit.body)

        # if orbiting body has changed, reset telemetry
        if self._vessel_body.has_changed():
            self._remove_telemetry()
            self._setup_telemetry()

        # obtain telemetry data
        self._telemetry['g']                        = self._space_g
        self._telemetry['ut']                       = self._space_ut()
        self._telemetry['vessel']                   = self._vessel
        self._telemetry['vessel_name']              = self._vessel_name
        self._telemetry['vessel_control']           = self._vessel_control
        self._telemetry['vessel_autopilot']         = self._vessel_autopilot
        self._telemetry['vessel_orbit']             = self._vessel_orbit
        self._telemetry['vessel_body']              = self._vessel_body.get()
        self._telemetry['vessel_body_mass']         = self._vessel_body_mass
        self._telemetry['vessel_body_name']         = self._vessel_body_name
        
        self._telemetry['vessel_position_bdy']      = self._vessel_position_bdy()
        self._telemetry['vessel_vertical_speed']    = self._vessel_vertical_speed()
        self._telemetry['vessel_rotation']          = self._vessel_rotation()
        self._telemetry['vessel_mass']              = self._vessel_mass()
        self._telemetry['vessel_thrust']            = self._vessel_thrust()
        self._telemetry['vessel_max_thrust']        = self._vessel_max_thrust()
        self._telemetry['vessel_throttle']          = self._vessel_throttle()
        self._telemetry['vessel_mean_altitude']     = self._vessel_mean_altitude()
        self._telemetry['vessel_surface_altitude']  = self._vessel_surface_altitude()
        self._telemetry['vessel_latitude']          = self._vessel_latitude()
        self._telemetry['vessel_longitude']         = self._vessel_longitude()
        self._telemetry['vessel_surface_height']    = self._vessel_body.get().surface_height(
            self._telemetry['vessel_latitude'],
            self._telemetry['vessel_longitude'])
        
        # compute telemetry parameters
        body_to_vessel_distance_sq = self._telemetry['vessel_position_bdy'][0] ** 2 + self._telemetry['vessel_position_bdy'][1] ** 2 + self._telemetry['vessel_position_bdy'][2] ** 2
        
        self._telemetry['vessel_body_gravity'] = self._space_g * self._telemetry['vessel_body_mass'] / body_to_vessel_distance_sq
        self._telemetry['vessel_weight'] = self._telemetry['vessel_body_gravity'] * self._telemetry['vessel_mass']

        # compute horizontal stabilization vectors
        body_to_vessel_norm = vector_normalize(self._telemetry['vessel_position_bdy'])
        body_to_vessel_mag = vector_length(self._telemetry['vessel_position_bdy'])
        offset_vessel_to_zenith_mag = body_to_vessel_mag + 5.0
        offset_vessel_to_zenith_vec = vector_scale(body_to_vessel_norm, offset_vessel_to_zenith_mag)

        vessel_to_north_vec = self._krpc.space_center.transform_direction((0,5,0), self._vessel.surface_reference_frame, self._vessel_body_reff)
        offset_vessel_to_north_vec = vector_add(self._telemetry['vessel_position_bdy'], vessel_to_north_vec)

        vessel_to_east_vec = self._krpc.space_center.transform_direction((0,0,5), self._vessel.surface_reference_frame, self._vessel_body_reff)
        offset_vessel_to_east_vec = vector_add(self._telemetry['vessel_position_bdy'], vessel_to_east_vec)

        vessel_velocity_rel_to_body = self._vessel.velocity(self._vessel_body_reff)
        hrz_velocity_north = vector_dot_product(vessel_velocity_rel_to_body, vector_normalize(vessel_to_north_vec))
        hrz_velocity_east = vector_dot_product(vessel_velocity_rel_to_body, vector_normalize(vessel_to_east_vec))
        hrz_velocity = vector_project_onto_plane(vessel_velocity_rel_to_body, self._telemetry['vessel_position_bdy'])
        hrz_velocity_mag = vector_length(hrz_velocity)
        hrz_velocity_norm = vector_normalize(hrz_velocity)

        hrz_counter_velocity_norm = vector_scale(hrz_velocity_norm, -1.0)
        hrz_counter_velocity_mag = hrz_velocity_mag if (hrz_velocity_mag < 5.0) else 5.0
        hrz_counter_velocity = vector_scale(hrz_counter_velocity_norm, hrz_counter_velocity_mag)
        counter_direction_zenith = 28.36
        zenith_counter_velocity = vector_scale(body_to_vessel_norm, counter_direction_zenith)
        self._counter_direction = vector_add(zenith_counter_velocity, hrz_counter_velocity)

        angle = math.atan2(hrz_counter_velocity_mag, counter_direction_zenith)

        # update drawings
        '''
        self._dwg_dir_vessel_to_zenith.start = self._telemetry['vessel_position_bdy']
        self._dwg_dir_vessel_to_zenith.end = offset_vessel_to_zenith_vec
        self._dwg_dir_vessel_to_north.start = self._telemetry['vessel_position_bdy']
        self._dwg_dir_vessel_to_north.end = offset_vessel_to_north_vec
        self._dwg_dir_vessel_to_east.start = self._telemetry['vessel_position_bdy']
        self._dwg_dir_vessel_to_east.end = offset_vessel_to_east_vec
        self._dwg_counter_vel.start = self._telemetry['vessel_position_bdy']
        self._dwg_counter_vel.end = vector_add(self._telemetry['vessel_position_bdy'], self._counter_direction)
        print("N: {:7.3f}, E: {:7.3f}, a = {:7.3f}".format(hrz_velocity_north, hrz_velocity_east, math.degrees(angle)))
        '''
        
        # finish update
        self.telemetry_updated.emit(self._telemetry)
        

    #=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%#
    #=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%#
    #                                                                          #
    #               #===  #===#  #\  #  ==#==  #==\  #===#  |                  #
    #               |     #   #  # \ #    |    #--/  #   #  |                  #
    #               #===  #===#  #  \#    |    #  \  #===#  #===               #
    #                                                                          #
    #=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%#
    #=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%=%#
        
        
    def _control_update(self):

        # determine master engine control
        self._vessel_allow_engines.update(self._rc_master_switch_engine)

        if self._vessel_allow_engines.has_changed():
            for engine in self._vessel.parts.engines:
                engine.active = self._vessel_allow_engines.get()

        if not self._vessel_allow_engines.get():
            self._vessel_control.throttle = 0.0


        # determine master autopilot control
        self._vessel_allow_autopilot.update(self._rc_master_switch_autopilot)

        if self._vessel_allow_autopilot.has_changed(to_value=False):
            self.request_mp_change.emit('full_manual')


        # determine mission program control
        if self._vessel_allow_autopilot.get() and self._mission_program is not None:
            mp_id = self._mission_program.id

    
            if mp_id == 'full_manual':
                pass
                
            # Control vertical speed without automatic tuning of controller gains
            elif mp_id == 'vspeed_manual':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    throttle_cmd = self.ctrl_vertical_speed.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd
                
            # Control vertical speed with automatic tuning of controller gains
            elif mp_id == 'vspeed_auto':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    # set control gains
                    ku = self._telemetry['vessel_weight'] / self._telemetry['vessel_max_thrust']
                    self.ctrl_vertical_speed.setProportionalGain(ku * 0.70)
                    self.ctrl_vertical_speed.setIntegralGain(ku / 3.0)
                    self.ctrl_vertical_speed.setDerivativeGain(ku / 50.0)
                    
                    throttle_cmd = self.ctrl_vertical_speed.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd
                    
            # Control altitude without automatic tuning of controller gains
            elif mp_id == 'altitude_manual':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    vspeed_cmd = self.ctrl_altitude.update(self._telemetry['vessel_mean_altitude'])
                    self.ctrl_vertical_speed.setSetpoint(vspeed_cmd)
                    
                    throttle_cmd = self.ctrl_vertical_speed.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd
                    
            # Control altitude with automatic tuning of speed controller gains
            elif mp_id == 'altitude_auto':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    vspeed_cmd = self.ctrl_altitude.update(self._telemetry['vessel_mean_altitude'])
                    
                    # set control gains
                    ku = self._telemetry['vessel_weight'] / self._telemetry['vessel_max_thrust']
                    self.ctrl_vertical_speed.setProportionalGain(ku * 0.70)
                    self.ctrl_vertical_speed.setIntegralGain(ku / 3.0)
                    self.ctrl_vertical_speed.setDerivativeGain(ku / 50.0)
                    self.ctrl_vertical_speed.setSetpoint(vspeed_cmd)
                    throttle_cmd = self.ctrl_vertical_speed.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd
                
            # Controlled descent that varies vertical speed according to altitude
            elif mp_id == 'controlled_descent':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    # set control gains
                    ku = self._telemetry['vessel_weight'] / self._telemetry['vessel_max_thrust']
                    self.ctrl_vertical_speed.setProportionalGain(ku * 0.70)
                    self.ctrl_vertical_speed.setIntegralGain(ku / 3.0)
                    self.ctrl_vertical_speed.setDerivativeGain(ku / 50.0)
                    self.ctrl_vertical_speed.setSetpoint(self._telemetry['vessel_surface_altitude'] / -12.0 - 1.0)
                
                    throttle_cmd = self.ctrl_vertical_speed.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd

            # Control vertical speed while killing horizontal speed
            elif mp_id == 'hrz_stabilize':
                if self._telemetry['vessel_max_thrust'] > 0.0:
                    # set control gains
                    ku = self._telemetry['vessel_weight'] / self._telemetry['vessel_max_thrust']
                    self.ctrl_vertical_speed.setProportionalGain(ku * 0.70)
                    self.ctrl_vertical_speed.setIntegralGain(ku / 3.0)
                    self.ctrl_vertical_speed.setDerivativeGain(ku / 50.0)
                    
                    throttle_cmd = self.ctrl_vertical_speed.update(self._telemetry['vessel_vertical_speed'])
                    self._vessel_control.throttle = throttle_cmd

                    self._kill_horizontal_velocity.update(True)

        # engage horizontal stabilization
        self._kill_horizontal_velocity.update(self._rc_button_stabilize)

        if self._kill_horizontal_velocity.has_changed(to_value=True):
            self._vessel_autopilot.engage()
        elif self._kill_horizontal_velocity.has_changed(to_value=False):
            self._vessel_autopilot.disengage()
            self._vessel_control.sas = self._vessel_control_sas

        if self._kill_horizontal_velocity.get():
            self._vessel_autopilot.reference_frame = self._vessel_body_reff
            self._vessel_autopilot.target_direction = self._counter_direction
            self._vessel_autopilot.target_roll = math.nan
        else:
            self._vessel_control_sas = self._vessel_control.sas

        # issue remote control commands
        self._vessel_control.yaw   = self._rc_joystick_x
        self._vessel_control.pitch = self._rc_joystick_y
        self._vessel_control.roll  = self._rc_joystick_z

                    
                    
                    
    def _signals_update(self):
    
        if 'vessel_latitude' in self._telemetry.keys() and 'vessel_longitude' in self._telemetry.keys():
            
            '''
            for y in range(s):
                longitude = self._telemetry['vessel_longitude'] + float(y - s / 2) * delta_lat
                for x in range(s):
                    latitude = self._telemetry['vessel_latitude'] + float(x - s / 2) * delta_lat
                    #self._telemetry['surface_height_map'][x][y] = \
                    #    self._vessel_body().surface_height(latitude, longitude) - self._telemetry['vessel_surface_height']
                    self._telemetry['surface_height_map'][x][y] = 0.0
            '''
            
            
            #radar_element_spacing = 60000.0 # meters  60000.0 = can see whole map
            radar_element_spacing = 0.5
            
            body_to_vessel_distance = math.sqrt( \
                self._telemetry['vessel_position_bdy'][0] ** 2 + \
                self._telemetry['vessel_position_bdy'][1] ** 2 + \
                self._telemetry['vessel_position_bdy'][2] ** 2 )
            delta_lat = math.degrees(radar_element_spacing / body_to_vessel_distance)
            #print("delta_lat = {:.20f}".format(delta_lat))
            
            num_radar_tasks = self._radar_resolution * self._radar_resolution
            num_radar_tasks_to_execute = 60
            
            for i in range(num_radar_tasks_to_execute):
            
                if self._signals_task >= 0 and self._signals_task < num_radar_tasks:
                    x_idx = int(self._signals_task % self._radar_resolution)
                    y_idx = int(self._signals_task / self._radar_resolution)
                    
                    latitude = self._telemetry['vessel_latitude'] + float(y_idx - self._radar_resolution / 2) * delta_lat
                    longitude = self._telemetry['vessel_longitude'] + float(x_idx - self._radar_resolution / 2) * delta_lat
                    
                    latitude = clamp(-89.9, latitude, 89.9)
                    
                    self._telemetry['surface_height_map'][y_idx][x_idx] = \
                        self._telemetry['vessel_surface_height'] - self._vessel_body.get().surface_height(latitude, longitude)
                        
                self._signals_task += 1
                if self._signals_task >= num_radar_tasks:
                    self._signals_task = 0
                    
                    
                    #print("Span: {:.3f} m  ({:.6f} deg)".format(radar_element_spacing * self._radar_resolution,
                    #    delta_lat * self._radar_resolution))
                    for y in range(len(self._telemetry['surface_height_map']), 0, -1):
                        #print("{:d}".format(y - 1))
                        alt_line = []
                        for x in range(len(self._telemetry['surface_height_map'][y - 1])):
                            alt_line.append('{:5.1f}'.format(self._telemetry['surface_height_map'][y - 1][x]))
                        #print(' '.join(alt_line))
                    
                    
                    #for a in self._telemetry['surface_height_map'][int(self._radar_resolution / 2)]:
                        #print("{:.3f}".format(a))
                
        
    
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
            self._krpc_game_scene = self._krpc.krpc.current_game_scene

            # if we are in the Flight scene, continue processing
            if self._krpc_game_scene == self._krpc.krpc.GameScene.flight:

                # set up telemetry for the vessel if it hasn't already been done
                if not self._vessel_is_active:
                    self._setup_telemetry()

                else:
                    self._telemetry_update()
                    self._control_update()
                    pass

            else:
                # otherwise, unset the active vessel and telemetry
                self._vessel_is_active = False
        
        #--
        process_time = time.time() - start_time
        self._scheduler_timings['sts'].update(process_time)
        self._telemetry['sts_time'] = self._scheduler_timings['sts'].get_mean()
        if process_time > KPFlightController.sts_period:
            self._log_warning('STS overrun: {:.1f} ms, overrun by {:.1f} ms'.format(
                process_time * 1000.0, (process_time - KPFlightController.sts_period) * 1000.0))
        
            
        
    @pyqtSlot()
    def long_term_processing(self):
        start_time = time.time()
        #--
        
        if self.krpc_is_connected and self._vessel_is_active:
            self._signals_update()
        
        #--
        process_time = time.time() - start_time
        self._scheduler_timings['lts'].update(process_time)
        self._telemetry['lts_time'] = self._scheduler_timings['lts'].get_mean()
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

            # record the time
            self._timestamp.update(time.time())
            
            # service Qt events
            QCoreApplication.processEvents()
            
            
        # thread termination
        self.krpc_disconnect()
        self._log('Flight control thread terminating...')
        self.finished.emit()
        

    @pyqtSlot(KPMissionProgram)
    def set_active_program(self, program):

        # set the new active mission program
        self._mission_program = program
        
        # set mission program settings
        self.ctrl_vertical_speed.setSetpointEditable(self._mission_program.settings['vertical_speed_controller_setpoint_editable'])
        self.ctrl_vertical_speed.setGainsEditable(self._mission_program.settings['vertical_speed_controller_gains_editable'])
        self.ctrl_altitude.setSetpointEditable(self._mission_program.settings['altitude_controller_setpoint_editable'])
        self.ctrl_altitude.setGainsEditable(self._mission_program.settings['altitude_controller_gains_editable'])
                
        
    @pyqtSlot()
    def krpc_connect(self):
        try:
            # attempt to connect
            self._log('Connecting to KRPC at {:s}:{:d} ...'.format(self.krpc_address, self.krpc_rpc_port))
            self._krpc = krpc.connect(name=self.krpc_client_name, address=self.krpc_address, rpc_port=self.krpc_rpc_port, stream_port=self.krpc_stream_port)
            
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


    @pyqtSlot(KPRemoteControlState)
    def rc_command_received(self, rc_cmd):

        # register master switches
        self._rc_master_switch_engine    = rc_cmd.btn_switch_red
        self._rc_master_switch_autopilot = rc_cmd.btn_switch_blue

        # map joystick values to [-1.0, 1.0]
        joystick_x = (float(rc_cmd.joystick['x']) / 1023.0 - 0.5) * 2.0
        joystick_y = (float(rc_cmd.joystick['y']) / 1023.0 - 0.5) * -2.0
        joystick_z = (float(rc_cmd.joystick['z']) / 1023.0 - 0.5) * -2.0

        # filter out dead-zone
        self._rc_joystick_x = 0.0 if (joystick_x > -0.04 and joystick_x < 0.04) else joystick_x
        self._rc_joystick_y = 0.0 if (joystick_y > -0.04 and joystick_y < 0.04) else joystick_y
        self._rc_joystick_z = 0.0 if (joystick_z > -0.04 and joystick_z < 0.04) else joystick_z

        self._rc_button_stabilize = rc_cmd.btn_joystick
        self._rc_button_increment.update(rc_cmd.btn_rocker_up)
        self._rc_button_decrement.update(rc_cmd.btn_rocker_down)

        # control vertical speed
        if self._rc_button_decrement.has_changed(to_value=True):
            self.ctrl_vertical_speed.setSetpoint(self.ctrl_vertical_speed._pid.set_point - 0.5)

        if self._rc_button_increment.has_changed(to_value=True):
            self.ctrl_vertical_speed.setSetpoint(self.ctrl_vertical_speed._pid.set_point + 0.5)
            

