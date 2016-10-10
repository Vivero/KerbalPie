import krpc, math, time

from time import sleep

from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication, Qt, QTimer, QVariant, pyqtSignal
from PyQt5.QtCore import pyqtSlot

from lib.logger import Logger


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



#--- Mission Programs database
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPMissionProgramsDatabase(QtCore.QObject):

    subsys = 'MP_DB'
        
    # S I G N A L S 
    #===========================================================================
    current_program_updated = pyqtSignal(KPMissionProgram)
    
    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, **kwds):
        super(KPMissionProgramsDatabase, self).__init__(**kwds)
        
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
        
        mp_forward_stabilize = KPMissionProgram(
            id='fwd_stabilize',
            name='Forward Stabilize',
            description="Vessel will cancel all horizontal (relative to the planet's surface) speed in the forward/aft direction.",
            settings={
                'vertical_speed_controller_setpoint_editable': True,
                'vertical_speed_controller_gains_editable': False,
                'altitude_controller_setpoint_editable': True,
                'altitude_controller_gains_editable': False,
            },
        )
        
        self.db = [
            mp_full_manual,
            mp_vspeed_manual,
            mp_vspeed_auto,
            mp_altitude_manual,
            mp_altitude_auto,
            mp_controlled_descent,
            mp_forward_stabilize,
        ]
        
        self._current_program = self.db[0]
        
    
    # M E T H O D S 
    #===========================================================================
    def set_current_program_num(self, program_num):
        if program_num < len(self.db):
            self._current_program = self.db[program_num]
            self.current_program_updated.emit(self._current_program)
            
            self._log("Activated program {:2d}: {:s}".format(program_num, self._current_program.name))
                    
    
    
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
    
    

#--- Qt Model for storing Mission Programs
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPMissionProgramsModel(QtCore.QAbstractTableModel):

    subsys = 'MP_MODEL'
    
    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, mp_database, **kwds):
        super(KPMissionProgramsModel, self).__init__(**kwds)
        
        # set up model data
        self._mp_table_header = ['Program Name', 'State']
        self._mp_database = mp_database
        
        
    # M E T H O D S 
    #===========================================================================
        
        
    # O V E R R I D E   M E T H O D S 
    #===========================================================================
    def rowCount(self, parent):
        return len(self._mp_database.db)
            
            
    def columnCount(self, parent):
        return len(self._mp_table_header)
        
        
    def data(self, index, role):
        if role == Qt.DisplayRole:
            row = index.row()
            col = index.column()
            
            if col == 0:
                return QVariant(self._mp_database.db[row].name)
            elif col == 1:
                return QVariant(self._mp_database.db[row].state)
            
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
        
        
    def setData(self, index, value, role):
        if index.isValid() and role == Qt.EditRole:
            row = index.row()
            col = index.column()
            
            if col == 0 or col == 1:
                if col == 0:
                    self._mp_database.db[row].name = value
                elif col == 1:
                    self._mp_database.db[row].state = value
            
                self.dataChanged.emit(index, index)
            
        
        
    # P R I V A T E   M E T H O D S 
    #===========================================================================
    def _set_program_state(self, program_num, state):
        if program_num < len(self._mp_database.db):
            row = program_num
            col = 1
            
            model_index = self.createIndex(row, col)
            
            self.setData(model_index, QVariant(state), Qt.EditRole)
    
    
    # S L O T S 
    #===========================================================================
    @pyqtSlot(KPMissionProgram)
    def set_active_program(self, program):
        for mp_idx in range(len(self._mp_database.db)):
            if program.id == self._mp_database.db[mp_idx].id:
                self._set_program_state(mp_idx, 'enabled')
            else:
                self._set_program_state(mp_idx, 'disabled')
        
        
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    
    
    