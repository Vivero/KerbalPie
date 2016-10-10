#!/usr/bin/python

#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=  I M P O R T   #=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
import json, os, sys, threading, time

if sys.version_info >= (3,0):
    isPython3 = True
    import queue
    from queue import Queue
else:
    isPython3 = False
    import Queue
    from Queue import Queue

from time import sleep
from threading import Lock


#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=

#--- General logging functionality
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class Logger(threading.Thread):

    subsys = 'LOGGER'
    
    log_queue = Queue(500)
    log_lock = Lock()
    debug = False

    def __init__(self, log_dir=os.getcwd(), log_name="kerbalpie.log", debug_on=False):
        threading.Thread.__init__(self)
        self.log_start_time = time.time()
        self._log_sleep_time = 0.050 # seconds
        self.stop = threading.Event()
        self.stop.clear()
        
        self.log_full_filename = os.path.join(log_dir, log_name)
        
        debug = debug_on
        
        
    def run(self):
    
        # check if the log file already exists
        log_file_exists = os.path.isfile(self.log_full_filename)
    
        # log a start message
        Logger.log(Logger.subsys,
            'Logger initialized, {:s}: "{:s}"'.format(
                "appending to" if log_file_exists else "created", 
                self.log_full_filename))
    
        # start main logging loop
        while not self.is_terminated():
            
            # check if any messages to log
            self.flush_queue()
                        
            sleep(self._log_sleep_time)
            
        # before terminating, log a message, flush the queue
        Logger.log(self.subsys, 'Logger terminating ...')
        self.flush_queue()
        
                    
    @staticmethod
    def log(log_subsys, log_message, log_type='info', log_data=None):
        current_time = time.time()
        
        # form log entry dictionary
        log_entry = {
            'time'      : current_time,
            'subsys'    : log_subsys,
            'type'      : log_type,
            'message'   : log_message,
        }
        if log_data is not None:
            log_dict = dict(log_entry, **log_data)
        else:
            log_dict = log_entry
            
        if Logger.debug:
            print("LOG {:s} | {:s}".format(time.strftime("%H:%M:%S", time.localtime(current_time)), log_message))
        
        # attempt to place in queue
        try:
            Logger.log_queue.put(log_dict)
        except Queue.Full as e:
            sys.stderr.write('Warning: log queue full, discarding message: "{:s}"\n'.format(log_message))
        
        
                    
    @staticmethod
    def log_error(log_subsys, log_message, log_data=None):
        Logger.log(log_subsys, log_message, 'error', log_data)
                    
    @staticmethod
    def log_warning(log_subsys, log_message, log_data=None):
        Logger.log(log_subsys, log_message, 'warning', log_data)
        
                    
    @staticmethod
    def log_exception(log_subsys, log_message, log_exception):
        log_type = 'exception'
        log_data = {
            'exception_type' : log_exception.__class__.__name__,
            'exception_msg'  : str(log_exception)
        }
        Logger.log(log_subsys, log_message, log_type, log_data)
    
        
        
    def flush_queue(self):
        # check number of messages in queue
        num_log_entries = self.log_queue.qsize()
        
        if num_log_entries > 0:
        
            # open the log file
            with open(self.log_full_filename, 'ab') as log_file:
        
                for i in range(num_log_entries):
                    log_entry = self.log_queue.get()
                    
                    # append extra log information
                    current_time = log_entry['time']
                    current_time_str = time.asctime(time.localtime(current_time))
                    log_entry['localtime'] = current_time_str
                    
                    # log the message as a JSON string
                    if isPython3:
                        log_file.write(bytes(json.dumps(log_entry) + "\n", 'UTF-8'))
                    else:
                        log_file.write(json.dumps(log_entry) + "\n")
    
    
    @staticmethod
    def clear_queue():
        while not Logger.log_queue.empty():
            Logger.log_queue.get()
        
        
    def is_terminated(self):
        return self.stop.is_set()
    
    def terminate(self):
        self.stop.set()
        