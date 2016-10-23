import krpc, math, time

from time import sleep

from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication, Qt, QTimer, QVariant, pyqtSignal
from PyQt5.QtCore import pyqtSlot

from lib.logger import Logger
from lib.kp_tools import *


#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=


#--- Mission Program definition
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPMissionProgram(QtCore.QObject):

    subsys = 'MP'
    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, id, name, description, settings, **kwds):
        super(KPMissionProgram, self).__init__(**kwds)

        self.id = id
        self.name = name
        self.description = description
        self.settings = settings
        self.state = 'disabled'



#--- Mission Program controller
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPMissionController(QtCore.QObject):

    subsys = 'MP_CTRL'
        
    # S I G N A L S 
    #===========================================================================
    active_mp_updated = pyqtSignal(KPMissionProgram)

    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, **kwds):
        super(KPMissionController, self).__init__(**kwds)

        # create mission programs database
        self.mp_db = KPMissionProgramsDatabase(parent=self)
        
        # set the default mission program
        self.active_mp = self.mp_db.get_default_mp()

        # remote control states
        #-------------------------------
        self._rc_joystick_x = 0.0
        self._rc_joystick_y = 0.0
        self._rc_joystick_z = 0.0

        # control parameters
        #-------------------------------
        self._control               = {}
        self._control['yaw']        = 0.0
        self._control['pitch']      = 0.0
        self._control['roll']       = 0.0

        # flight data
        #-------------------------------
        self._telemetry             = {}


    # M E T H O D S 
    #===========================================================================
    def set_active_mp(self, mp_id):
        # try to get the Mission Program, given the id
        mp = self.mp_db.get_mp(mp_id)
        if mp is not None:

            # set as the active MP
            self.active_mp = mp

            # update the state of all MP's in db
            for mp in self.mp_db._database:
                mp.state = 'enabled' if (mp.id == self.active_mp.id) else 'disabled'

            # notify MP has changed
            self.active_mp_updated.emit(self.active_mp)
            
            self._log("Activated program: {:s}".format(self.active_mp.name))


    def set_default_mp(self):
        self.set_active_mp(self.mp_db._default_mp_id)


    def get_controls(self):
        self._control['yaw']        = self._rc_joystick_x
        self._control['pitch']      = self._rc_joystick_y
        self._control['roll']       = self._rc_joystick_z

        return self._control
    
    
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    def _log(self, log_message, log_type='info', log_data=None):
        Logger.log(KPMissionController.subsys, log_message, log_type, log_data)
    
    
    # S L O T S 
    #===========================================================================
    pyqtSlot(dict)
    def telemetry_update(self, telemetry):
        self._telemetry = telemetry


    pyqtSlot(KPRemoteControlState)
    def rc_command_update(self, rc_cmd):
        # register master switches
        #self._rc_master_switch_engine    = rc_cmd.btn_switch_red
        #self._rc_master_switch_autopilot = rc_cmd.btn_switch_blue

        # map joystick values to [-1.0, 1.0]
        joystick_x = (float(rc_cmd.joystick['x']) / 1023.0 - 0.5) * 2.0
        joystick_y = (float(rc_cmd.joystick['y']) / 1023.0 - 0.5) * -2.0
        joystick_z = (float(rc_cmd.joystick['z']) / 1023.0 - 0.5) * -2.0

        # filter out dead-zone
        self._rc_joystick_x = 0.0 if (joystick_x > -0.04 and joystick_x < 0.04) else joystick_x
        self._rc_joystick_y = 0.0 if (joystick_y > -0.04 and joystick_y < 0.04) else joystick_y
        self._rc_joystick_z = 0.0 if (joystick_z > -0.04 and joystick_z < 0.04) else joystick_z

        #self._rc_button_stabilize = rc_cmd.btn_joystick
        #self._rc_button_increment.update(rc_cmd.btn_rocker_up)
        #self._rc_button_decrement.update(rc_cmd.btn_rocker_down)

        # control vertical speed
        '''
        if self._rc_button_decrement.has_changed(to_value=True):
            self.ctrl_vertical_speed.setSetpoint(self.ctrl_vertical_speed._pid.set_point - 0.5)

        if self._rc_button_increment.has_changed(to_value=True):
            self.ctrl_vertical_speed.setSetpoint(self.ctrl_vertical_speed._pid.set_point + 0.5)
        '''



#--- Mission Programs database
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPMissionProgramsDatabase(QtCore.QObject):

    subsys = 'MP_DB'
        
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, **kwds):
        super(KPMissionProgramsDatabase, self).__init__(**kwds)

        self._default_mp_id = 'full_manual'
        
        mp_full_manual = KPMissionProgram(
            id='full_manual',
            name='Manual Flight Control',
            description="No controls will be applied to the vessel.",
            settings={
                'vertical_speed_controller_setpoint_editable': True,
                'vertical_speed_controller_gains_editable': True,
                'altitude_controller_setpoint_editable': True,
                'altitude_controller_gains_editable': True,
            },
        )
        
        mp_vspeed_manual = KPMissionProgram(
            id='vspeed_manual',
            name='Vertical Speed Control - Manual',
            description="Vessel will maintain a specified vertical (relative to the planet surface) speed. PID controller gains are editable.",
            settings={
                'vertical_speed_controller_setpoint_editable': True,
                'vertical_speed_controller_gains_editable': True,
                'altitude_controller_setpoint_editable': False,
                'altitude_controller_gains_editable': False,
            },
        )
        
        mp_vspeed_auto = KPMissionProgram(
            id='vspeed_auto',
            name='Vertical Speed Control - Auto',
            description="Vessel will maintain a specified vertical (relative to the planet surface) speed. PID controller gains are automatically adjusted according to the vessel's thrust-to-weight ratio.",
            settings={
                'vertical_speed_controller_setpoint_editable': True,
                'vertical_speed_controller_gains_editable': False,
                'altitude_controller_setpoint_editable': True,
                'altitude_controller_gains_editable': True,
            },
        )
        
        mp_altitude_manual = KPMissionProgram(
            id='altitude_manual',
            name='Altitude Control - Manual',
            description="Vessel will maintain a specified mean altitude (relative to the planet sea-level). PID controller gains are editable.",
            settings={
                'vertical_speed_controller_setpoint_editable': False,
                'vertical_speed_controller_gains_editable': False,
                'altitude_controller_setpoint_editable': True,
                'altitude_controller_gains_editable': True,
            },
        )
        
        mp_altitude_auto = KPMissionProgram(
            id='altitude_auto',
            name='Altitude Control - Auto',
            description="Vessel will maintain a specified mean altitude (relative to the planet sea-level). PID controller gains are automatically adjusted.",
            settings={
                'vertical_speed_controller_setpoint_editable': False,
                'vertical_speed_controller_gains_editable': False,
                'altitude_controller_setpoint_editable': True,
                'altitude_controller_gains_editable': False,
            },
        )
        
        mp_controlled_descent = KPMissionProgram(
            id='controlled_descent',
            name='Controlled Descent',
            description="Vessel will descend to the planet's surface in a controlled manner.",
            settings={
                'vertical_speed_controller_setpoint_editable': False,
                'vertical_speed_controller_gains_editable': False,
                'altitude_controller_setpoint_editable': False,
                'altitude_controller_gains_editable': False,
            },
        )
        
        mp_horizontal_stabilize = KPMissionProgram(
            id='hrz_stabilize',
            name='Horizontal Stabilize',
            description="Vessel will cancel all horizontal (relative to the planet's surface) speed.",
            settings={
                'vertical_speed_controller_setpoint_editable': True,
                'vertical_speed_controller_gains_editable': False,
                'altitude_controller_setpoint_editable': False,
                'altitude_controller_gains_editable': False,
            },
        )
        
        self._database = [
            mp_full_manual,
            mp_vspeed_manual,
            mp_vspeed_auto,
            mp_altitude_manual,
            mp_altitude_auto,
            mp_controlled_descent,
            mp_horizontal_stabilize,
        ]
        
    
    # M E T H O D S 
    #===========================================================================
    def get_default_mp(self):
        return self.get_mp(self._default_mp_id)

    def get_mp(self, mp_id):
        mission_program = None
        if isinstance(mp_id, int) and mp_id < len(self._database):
            mission_program = self._database[mp_id]

        elif isinstance(mp_id, str):
            for mp in self._database:
                if mp.id == mp_id:
                    mission_program = mp
                    break

        if mission_program is None:
            self._log_warning("Could not find Mission Program ID='{:s}'".format(str(mp_id)))

        return mission_program
                    
    
    
    # O V E R R I D E   M E T H O D S 
    #===========================================================================
    
    
    # P R I V A T E   M E T H O D S 
    #===========================================================================
    
    
    # S L O T S 
    #===========================================================================
    
    
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    def _log(self, log_message, log_type='info', log_data=None):
        Logger.log(KPMissionProgramsDatabase.subsys, log_message, log_type, log_data)
        
    def _log_warning(self, log_message, log_data=None):
        Logger.log_warning(KPMissionProgramsDatabase.subsys, log_message, log_data)
    
    

#--- Qt Model for storing Mission Programs
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPMissionProgramsModel(QtCore.QAbstractTableModel):

    subsys = 'MP_MODEL'
    
    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, mp_db, **kwds):
        super(KPMissionProgramsModel, self).__init__(**kwds)
        
        # set up model data
        self._mp_table_header = ['Program Name', 'State']
        self._mp_database = mp_db
        
        
    # M E T H O D S 
    #===========================================================================
        
        
    # O V E R R I D E   M E T H O D S 
    #===========================================================================
    def rowCount(self, parent):
        return len(self._mp_database._database)
            
            
    def columnCount(self, parent):
        return len(self._mp_table_header)
        
        
    def data(self, index, role):
        if role == Qt.DisplayRole:
            row = index.row()
            col = index.column()
            
            if col == 0:
                return QVariant(self._mp_database.get_mp(row).name)
            elif col == 1:
                return QVariant(self._mp_database.get_mp(row).state)
            
        return QVariant()


    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return QVariant(self._mp_table_header[section])
                
        return QVariant()


    def flags(self, index):
        col = index.column()
        flags = Qt.ItemIsEnabled
        if col == 0:
            flags = flags | Qt.ItemIsSelectable
        return flags

    '''
    def setData(self, index, value, role):
        if index.isValid() and role == Qt.EditRole:
            row = index.row()
            col = index.column()
            
            if col == 0 or col == 1:
                if col == 0:
                    self._mp_database.get_mp(row).name = value
                elif col == 1:
                    self._mp_database.get_mp(row).state = value
            
                self.dataChanged.emit(index, index)
    '''
