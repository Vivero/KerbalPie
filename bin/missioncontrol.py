import krpc, math, time

from time import sleep

from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication, Qt, QTimer, QVariant, pyqtSignal, pyqtSlot

from logger import KPLogger


#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=

#--- Mission Programs database
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPMissionProgramsDatabase(QtCore.QObject):

    subsys = 'MISSION_DB'
    
    mission_program_database = [
        ('full_manual',             'Manual Flight Control'),
        ('vspeed_ctrl_manual',      'Vertical Speed Control - Manual'),
        ('vspeed_ctrl_auto',        'Vertical Speed Control - Auto'),
        ('altitude_ctrl_manual',    'Altitude Control - Manual'),
        ('altitude_ctrl_auto',      'Altitude Control - Auto'),
        ('controlled_descent',      'Controlled Descent'),
        ('forward_ctrl_manual',     'Forward Stabilizer'),
    ]
        
    mission_program_id_lookup, mission_program_title_lookup = \
        zip(*mission_program_database)
        
    num_mission_programs = len(mission_program_id_lookup)
        
    # S I G N A L S 
    #===========================================================================
    current_program_updated = pyqtSignal(int)
    
    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, **kwds):
        super(KPMissionProgramsDatabase, self).__init__(**kwds)
        
        self._current_program_num = 0
        self._current_program_id = KPMissionProgramsDatabase.mission_program_id_lookup[self._current_program_num]
        
    
    # M E T H O D S 
    #===========================================================================
    def set_current_program_num(self, program_num):
        if program_num < KPMissionProgramsDatabase.num_mission_programs:
            self._current_program_num = program_num
            self.current_program_updated.emit(program_num)
            
            self._log("Activated program {:2d}: {:s}".format(program_num, KPMissionProgramsDatabase.mission_program_title_lookup[program_num]))
                    
    
    
    # O V E R R I D E   M E T H O D S 
    #===========================================================================
    
    
    # P R I V A T E   M E T H O D S 
    #===========================================================================
    
    
    # S L O T S 
    #===========================================================================
    
    
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    def _log(self, log_message, log_type='info', log_data=None):
        KPLogger.log(KPMissionProgramsDatabase.subsys, log_message, log_type, log_data)
    
    

#--- Qt Model for storing Mission Programs
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPMissionProgramsModel(QtCore.QAbstractTableModel):

    subsys = 'MISSION'
        
    # S I G N A L S 
    #===========================================================================

    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, **kwds):
        super(KPMissionProgramsModel, self).__init__(**kwds)
        
        # set up model data
        self._mission_programs_header = ['Program Name', 'Active']
        
        num_missions = len(KPMissionProgramsDatabase.mission_program_title_lookup)
        self._mission_programs = [[program_title, False] for program_title in KPMissionProgramsDatabase.mission_program_title_lookup]
        
        
    # M E T H O D S 
    #===========================================================================
    def update_program_status(self, program_id, is_active):
        if program_id in KPMissionProgramsDatabase.mission_program_id_lookup:
            row = KPMissionProgramsDatabase.mission_program_id_lookup.index(program_id)
            col = 1
            
            model_index = self.createIndex(row, col)
            
            self.setData(model_index, QVariant(is_active), Qt.EditRole)
        
        
    # O V E R R I D E   M E T H O D S 
    #===========================================================================
    def rowCount(self, parent):
        return len(self._mission_programs)
            
    def columnCount(self, parent):
        return len(self._mission_programs_header)
        
    def data(self, index, role):
        if role == Qt.DisplayRole:
            row = index.row()
            col = index.column()
        
            return QVariant(self._mission_programs[row][col])
            
        return QVariant()
        
    def headerData(self, section, orientation, role):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return QVariant(self._mission_programs_header[section])
        
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
            
            self._mission_programs[row][col] = value
            self.dataChanged.emit(index, index)
            
        
        
    # P R I V A T E   M E T H O D S 
    #===========================================================================
    
    
    # S L O T S 
    #===========================================================================
    @pyqtSlot(int)
    def set_active_program(self, program_num):
        if program_num < KPMissionProgramsDatabase.num_mission_programs:
            for program_id in KPMissionProgramsDatabase.mission_program_id_lookup:
                self.update_program_status(program_id, KPMissionProgramsDatabase.mission_program_id_lookup[program_num] == program_id)
        
        
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    
    
    