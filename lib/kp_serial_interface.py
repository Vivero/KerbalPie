import collections, krpc, math, time

from time import sleep

from PyQt5 import QtCore
from PyQt5.QtCore import QCoreApplication, QIODevice, Qt, QTimer, QVariant
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtSerialPort import QSerialPort

from lib.kp_tools import *
from lib.logger import Logger


#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=

    
#--- Serial port interface
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPSerialInterface(QtCore.QObject):

    subsys = 'SERIAL'
        
    # S I G N A L S 
    #===========================================================================
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    finished = pyqtSignal()


    
    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, serial_port="COM4", serial_baudrate=250000, **kwds):
        super(KPSerialInterface, self).__init__(**kwds)
        
        # thread variables
        self.terminate = False
        self._current_time = time.time()
        self._previous_time = self._current_time
        
        # serial interface
        self._serial = None
        self.is_connected = False
        self.port = serial_port
        self.baudrate = serial_baudrate
        self._rx_buffer = collections.deque(maxlen=512)
        

        
    # M E T H O D S 
    #===========================================================================
    def connect(self):
        if self._serial is None:
            self._serial = QSerialPort()
            self._serial.readyRead.connect(self.serial_read_bytes)

        self._serial.setPortName(self.serial_port)
        self._serial.setBaudRate(self.serial_baudrate)

        try:
            # attempt to connect
            self._log('Connecting serial port {:s} ...'.format(self.serial_port))
            self._serial.open(QIODevice.ReadWrite)
            
            # emit succesful connection signals
            self.serial_is_connected = True
            self.serial_connected.emit()
            
            self._log('Connected serial port!')
            
        except Exception as e:
            self._log_exception('Unable to open serial port', e)
        
    
    # S L O T S 
    #===========================================================================
    
    @pyqtSlot()
    def process(self):
        while not self.terminate:
            #self._current_time = time.time()
            #delta_t = self._current_time - self._previous_time
            
            # service Qt events
            QCoreApplication.processEvents()
            
            # sleep to next period
            #self._previous_time = self._current_time
            #processing_time = time.time() - self._current_time
            #sleep_time = KPFlightController.control_period - processing_time
            
            
            #if sleep_time > 0.0:
            #    sleep(sleep_time)
            #else:
            #    self._log_warning('Processing overrun: Control time = {:.1f} ms, overrun by {:.1f} ms'.format(processing_time * 1000.0, sleep_time * -1000.0))
            
            
        # thread termination
        self.disconnect()
        self._log('Serial interface thread terminating...')
        self.finished.emit()


    @pyqtSlot()
    def connect(self):
        if self._serial is None:
            self._serial = QSerialPort()
            self._serial.readyRead.connect(self.read_data)

        self._serial.setPortName(self.port)
        self._serial.setBaudRate(self.baudrate)

        try:
            # attempt to connect
            self._log('Connecting serial port {:s} ...'.format(self.port))
            self._serial.open(QIODevice.ReadWrite)
            
            # emit succesful connection signals
            self.is_connected = True
            self.connected.emit()
            
            self._log('Connected serial port!')
            
        except Exception as e:
            self._log_exception('Unable to open serial port', e)


    @pyqtSlot()
    def read_data(self):
        rx_bytes = self._serial.readAll()

        print("len = {:3d}".format(len(self._rx_buffer)))
        print(rx_bytes.data())

        for b in rx_bytes.data():
             self._rx_buffer.append(b)

        
        try:
            print(self._rx_buffer)
        except Exception as e:
            self._log_exception('EXCEPTION! {:s}'.format(str(e)), e)

        
    @pyqtSlot()
    def disconnect(self):
        if self._serial is not None:
            self._serial.readyRead.disconnect()
            self._serial.close()
            self.is_connected = False
            self.disconnected.emit()
            self._log('Disconnected serial port')
        
    # H E L P E R   F U N C T I O N S 
    #===========================================================================
    def _log(self, log_message, log_type='info', log_data=None):
        Logger.log(KPSerialInterface.subsys, log_message, log_type, log_data)
        
    def _log_warning(self, log_message, log_data=None):
        Logger.log_warning(KPSerialInterface.subsys, log_message, log_data)
        
    def _log_exception(self, log_message, log_exception):
        Logger.log_exception(KPSerialInterface.subsys, log_message, log_exception)
