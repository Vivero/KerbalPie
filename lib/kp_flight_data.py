import collections, krpc, math, re, time

from time import sleep

from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication, Qt, QTimer, QVariant, pyqtSignal
from PyQt5.QtCore import pyqtSlot

from lib.kp_tools import *
from lib.logger import Logger
from lib.widgets.QPidController import QPidController


#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=

#--- Qt Model for storing a flight parameter
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPFlightParameter(QtCore.QObject):

    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, data_name, display_name, unit_name, value, **kwds):
        super(KPFlightParameter, self).__init__(**kwds)




#--- Qt Model for storing Flight data
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPFlightDataModel(QtCore.QAbstractTableModel):

    subsys = 'FLIGHT'
        
    flight_data_lookup = [
        'game_scene',
        'vessel_name',
        'space_ut',
        'body_name',
        'body_gravity',
        'vessel_mass',
        'vessel_weight',
        'vessel_forward_speed',
        'vessel_vertical_speed',
        'vessel_rotation',
        'vessel_mean_altitude',
        'vessel_surface_altitude',
        'vessel_throttle',
        'vessel_thrust',
        'vessel_max_thrust',
        'vessel_latitude',
        'vessel_longitude',
        'vessel_surface_height',
        'sts_time',
        'lts_time',
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
            ['Game Scene',        'n/a',      ''],
            ['Vessel Name',       'n/a',      ''],
            ['Universal Time',    's',        0.0],
            ['Planet Name',       'n/a',      ''],
            ['Gravity',           'm/s^2',    0.0],
            ['Mass',              'kg',       0.0],
            ['Weight',            'N',        0.0],
            ['Forward Speed',     'm/s',      0.0],
            ['Vertical Speed',    'm/s',      0.0],
            ['Rotation',          'n/a',      0.0],
            ['Altitude',          'm',        0.0],
            ['Radar Altitude',    'm',        0.0],
            ['Throttle',          'n/a',      0.0],
            ['Thrust',            'N',        0.0],
            ['Max Thrust',        'N',        0.0],
            ['Latitude',          'deg',      0.0],
            ['Longitude',         'deg',      0.0],
            ['Surface Height',    'm',        0.0],
            ['STS Timing',        's',        0.0],
            ['LTS Timing',        's',        0.0],
        ]
        
        
    # M E T H O D S 
    #===========================================================================
    def update_flight_data(self, parameter, value):
        col = 2 # table column to update

        if parameter in KPFlightDataModel.flight_data_lookup:
            row = KPFlightDataModel.flight_data_lookup.index(parameter)
            
            model_index = self.createIndex(row, col)

            if value is None:
                value = "n/a"

            else:
                # handle special formatting
                if parameter == 'game_scene':
                    try:
                        value = re.search(r'GameScene\.(.*)', str(value)).group(1)
                    except AttributeError:
                        value = "?"

                elif parameter == 'vessel_rotation':
                    value = "({:.3f},{:.3f},{:.3f},{:.3f})".format(value[0], value[1], value[2], value[3])

            self.setData(model_index, QVariant(value), Qt.EditRole)

        elif parameter == 'scheduler_timings':
            # update the STS mean time
            row = KPFlightDataModel.flight_data_lookup.index('sts_time')
            model_index = self.createIndex(row, col)
            sts_timing = "{:.3f}".format(value['sts'].get_mean())
            self.setData(model_index, QVariant(sts_timing), Qt.EditRole)

            # update the LTS mean time
            row = KPFlightDataModel.flight_data_lookup.index('lts_time')
            model_index = self.createIndex(row, col)
            lts_timing = "{:.3f}".format(value['lts'].get_mean())
            self.setData(model_index, QVariant(lts_timing), Qt.EditRole)


        
        
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
    
    