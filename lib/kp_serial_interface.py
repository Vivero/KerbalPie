import collections, krpc, math, struct, time

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
    msg_parser_period = 0.005

        
    # S I G N A L S 
    #===========================================================================
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    finished = pyqtSignal()
    rc_command = pyqtSignal(KPRemoteControlState)

    
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

        # initialize control timers
        self._message_parser_timer = QTimer()
        
        self._message_parser_timer.timeout.connect(self.parse_rx_buffer)
        self._message_parser_timer.start(KPSerialInterface.msg_parser_period * 1000.0)

        # data variables
        self._rc_cmd = KPRemoteControlState()
        

        
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
    
    
    # P R I V A T E   M E T H O D S 
    #===========================================================================
    def _parse_message(self, message):
        msg_bytes = ''
        for b in message:
            msg_bytes += hex(b) + " "

        #print("Message received! {:s}".format(msg_bytes))

        message_type = message[0]

        if message_type == 0x40:
            button_state = message[1]
            message_bytes = bytearray(message)
            joystick_x = struct.unpack("H", message_bytes[2:4])[0]
            joystick_y = struct.unpack("H", message_bytes[4:6])[0]
            joystick_z = struct.unpack("H", message_bytes[6:8])[0]

            #print("State message: {:4s} {:4d} {:4d} {:4d}".format(hex(button_state), joystick_x, joystick_y, joystick_z))

            # construct remote control command object
            #rc_cmd = KPRemoteControlState(button_state, joystick_x, joystick_y, joystick_z)
            self._rc_cmd.set_button_states(button_state)
            self._rc_cmd.joystick['x'] = joystick_x
            self._rc_cmd.joystick['y'] = joystick_y
            self._rc_cmd.joystick['z'] = joystick_z

            self.rc_command.emit(self._rc_cmd)
        
    
    # S L O T S 
    #===========================================================================
    
    @pyqtSlot()
    def process(self):
        while not self.terminate:
            
            # service Qt events
            QCoreApplication.processEvents()
            
            
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

        #print("len = {:3d}".format(len(self._rx_buffer)))
        #print(rx_bytes.data())

        for b in rx_bytes.data():
             self._rx_buffer.append(b)

        '''
        try:
            print(self._rx_buffer)
        except Exception as e:
            self._log_exception('EXCEPTION! {:s}'.format(str(e)), e)
        '''

    @pyqtSlot()
    def parse_rx_buffer(self):
        msg_size = 0

        if len(self._rx_buffer) >= 1:
            byte1 = self._rx_buffer[0]

            if byte1 != ord('$'):
                self._rx_buffer.popleft()
                return

        if len(self._rx_buffer) >= 2:
            byte2 = self._rx_buffer[1]

            if byte2 != ord('$'):
                self._rx_buffer.popleft()
                self._rx_buffer.popleft()
                return

        if len(self._rx_buffer) >= 3:
            msg_size = self._rx_buffer[2]

        if len(self._rx_buffer) >= (3 + msg_size):

            # pop the header
            self._rx_buffer.popleft()
            self._rx_buffer.popleft()
            self._rx_buffer.popleft()

            # pop the message
            message = []
            for i in range(msg_size):
                message.append(self._rx_buffer.popleft())

            self._parse_message(message)





        
    @pyqtSlot()
    def disconnect(self):
        if self._serial is not None:
            #self._serial.readyRead.disconnect(self)
            self._serial.close()
            self._rx_buffer.clear()
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
