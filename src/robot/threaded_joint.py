import odrive
import odrive.enums
import math
import time

# the functions in Threaded_Joint get_command function may have to be moved to be defined here if they are not picklable as-is
def set_state_to_halt(axis, output_dict):
    axis.requested_state = odrive.enums.AXIS_STATE_IDLE

def start_calibrate_joint(axis, output_dict):
    print('start calibrate')
    axis.clear_errors()
    if axis.error:
        print('axis has existing errors')
        print(f'error: {hex(axis.error)}')
    axis.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    time.sleep(1)
    output_dict['curr_state'] = axis.current_state

def check_curr_state(axis, output_dict):
    if axis.error:
        print('axis error occured')
        print(f'error: {hex(axis.error)}')
    output_dict['curr_state'] = axis.current_state

def enable_axis(axis, output_dict):
    if axis.error:
        print('axis error occured')
        print(f'error: {hex(axis.error)}')
    axis.clear_errors()
    axis.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
    output_dict['curr_state'] = axis.current_state

def disable_axis(axis, output_dict):
    axis.requested_state = odrive.enums.AXIS_STATE_IDLE
    output_dict['curr_state'] = axis.current_state

def configure_axis(axis, output_dict):
    axis.encoder.config.mode = 257
    axis.encoder.config.cpr = 16384
    #self.odrive_axis.encoder.config.abs_spi_cs_gpio_pin = encoder_cs_pin
    #gotta figure out how to do GPIO config. should come from odrive controller. new class for odrive functions needed
    axis.motor.config.pole_pairs = 20
    axis.motor.config.torque_constant = 8.27/160
    axis.motor.config.current_lim = 20.0
    axis.motor.config.requested_current_range = 25.0
    axis.motor.config.current_lim_margin = 1000
    axis.motor.config.torque_lim = 10000
    axis.controller.config.enable_vel_limit = True
    axis.controller.config.control_mode = 3
    axis.controller.config.pos_gain = 25.0
    axis.controller.config.vel_gain = 0.11
    axis.controller.config.vel_integrator_gain = .33
    axis.controller.config.vel_limit = 10.0
    axis.controller.config.vel_limit_tolerance = 999999999
    axis.controller.config.vel_ramp_rate = 2.5
    axis.controller.config.torque_ramp_rate = 0.01
    axis.controller.config.inertia = 0.0
    axis.controller.config.homing_speed = 0.25
    axis.min_endstop.config.is_active_high = False
    axis.min_endstop.config.enabled = False
    axis.min_endstop.config.offset = -0.5
    axis.min_endstop.config.pullup = True
    output_dict['config_complete'] = True

def get_errors(axis, output_dict):
    output_dict['encoder'] = axis.encoder.error
    output_dict['axis'] = axis.error
    output_dict['motor'] = axis.motor.error
    output_dict['controller'] = axis.controller.error

def home_axis(axis, output_dict):
    if axis.error:
        print('axis error occured')
        print(f'error: {hex(axis.error)}')
    axis.clear_errors()
    axis.requested_state = odrive.enums.AXIS_STATE_HOMING
    if axis.error:
        print('axis error occured')
        print(f'error: {hex(axis.error)}')
    output_dict['home_started'] = True

def set_axis_zero(axis, output_dict):
    axis.clear_errors()
    axis.requested_state = odrive.enums.AXIS_STATE_IDLE
    axis.encoder.set_linear_count(0)

def check_home(axis, output_dict):
    if axis.error:
        print('axis error occured')
        print(f'error: {hex(axis.error)}')
    if axis.current_state == odrive.enums.AXIS_STATE_IDLE:
        #add encoder rounding logic here
        output_dict['home_complete'] = True
    else:
        output_dict['home_complete'] = False

def show_errors(axis, outptu_dict):
        if axis.error:
            print('axis error occured')
            print(f'error: {hex(axis.error)}')

class Threaded_Joint:
    def __init__(self, gear_ratio, torque_constant):
        self.gear_ratio = gear_ratio
        self.torque_constant = torque_constant
        self.state = 'halt'
        self.pos_command = 0.0
        self.torque_limit = 10.0
        self.max_current = 20.0
        self.curr_current = 0.0
        self.curr_pos = 0.0
        self.curr_vel = 0.0
        self.last_error_status = {'axis':0, 'motor':0, 'controller':0, 'encoder':0}
    
    def angle_to_motor(self, angle):
        return angle * self.gear_ratio / 2 / math.pi
    
    def torque_to_current(self, torque):
        current = abs(torque / self.torque_constant / 8.8507 / self.gear_ratio)
        if current > self.max_current:
            current = self.max_current
        return current
    
    def get_curr_torque(self):
        return self.curr_current * -1.0 * self.torque_constant * 8.8507 * self.gear_ratio
    
    def get_curr_position(self):
        return self.curr_pos / self.gear_ratio * 2 * math.pi
    
    def get_curr_velocity(self):
        return self.curr_vel / self.gear_ratio * 2 * math.pi
    
    def set_torque(self, torque):
        self.torque_limit = torque
    
    def set_setpoint(self, position):
        self.pos_command = position

    def get_command(self):
        #this is called every loop to send data to motor worker. form packet in command_dict
        command_dict = {}
        if self.state == 'halt':


            command_dict['command'] = set_state_to_halt
            command_dict['pos_command'] = 0.0
            command_dict['curr_command'] = 0.0
        
        if self.state == 'set_zero':
            command_dict['command'] = set_axis_zero
            command_dict['pos_command'] = 0.0
            command_dict['curr_command'] = 0.0
            self.state = 'halt'

        if self.state == 'wait_calibrate':

            
            command_dict['command'] = check_curr_state
            command_dict['pos_command'] = 0.0
            command_dict['curr_command'] = 0.0

        if self.state == 'start_calibrate':
            print('start calibrate command')

            command_dict['command'] = start_calibrate_joint
            command_dict['pos_command'] = 0.0
            command_dict['curr_command'] = 0.0
            self.state = 'wait_calibrate'

        if self.state == 'run':
            command_dict['command'] = show_errors
            command_dict['pos_command'] = self.angle_to_motor(self.pos_command)
            command_dict['curr_command'] = self.torque_to_current(self.torque_limit)
        
        if self.state == 'enable':


            command_dict['command'] = enable_axis
            command_dict['pos_command'] = self.angle_to_motor(self.pos_command)
            command_dict['curr_command'] = self.torque_to_current(self.torque_limit)

        if self.state == 'disable':


            command_dict['command'] = disable_axis
            command_dict['pos_command'] = self.angle_to_motor(self.pos_command)
            command_dict['curr_command'] = self.torque_to_current(self.torque_limit)
        
        if self.state == 'configure':


            command_dict['command'] = configure_axis
            command_dict['pos_command'] = 0.0
            command_dict['curr_command'] = 0.0
        
        if self.state == 'get_errors':

            
            command_dict['command'] = get_errors
            command_dict['pos_command'] = self.angle_to_motor(self.pos_command)
            command_dict['curr_command'] = self.torque_to_current(self.torque_limit)
        
        if self.state == 'home':
            self.pos_command = 0.0 #don't yeet the arm
            
            command_dict['command'] = home_axis
            command_dict['pos_command'] = self.angle_to_motor(self.pos_command)
            command_dict['curr_command'] = self.torque_to_current(self.torque_limit)
        
        if self.state == 'wait_home':
            self.pos_command = 0.0 #don't yeet the arm
            
            command_dict['command'] = check_home
            command_dict['pos_command'] = self.angle_to_motor(self.pos_command)
            command_dict['curr_command'] = self.torque_to_current(self.torque_limit)

        return command_dict

    def recieve_data(self, data_dict):
        #called when valid data comes back. index checked so guaranteed most recent

        #always update these values because they are coming in
        self.curr_current = data_dict['data']['current']
        self.curr_pos = data_dict['data']['pos']
        self.curr_vel = data_dict['data']['vel']

        if self.state == 'halt':
            #current readings will freeze if motor is not enabled
            self.curr_current = 0.0
            return

        if self.state == 'wait_calibrate':
            if 'curr_state' in data_dict and data_dict['curr_state'] == odrive.enums.AXIS_STATE_IDLE:
                print('motor idle')
                self.state = 'halt'
                #motor needs to be enabled after calibration
            return

        if self.state == 'run':
            return

        if self.state == 'enable':
            if 'curr_state' in data_dict:
                if data_dict['curr_state'] == odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL:
                    self.state = 'run'
                if data_dict['curr_state'] == odrive.enums.AXIS_STATE_IDLE:
                    self.state = 'halt'
                    #how to tell if errored out?
            return

        if self.state == 'disable':
            if data_dict['curr_state'] == odrive.enums.AXIS_STATE_IDLE:
                self.state = 'halt'
            return

        if self.state == 'configure':
            if 'config_complete' in data_dict and data_dict['config_complete'] == True:
                self.state = 'halt'
        
        if self.state == 'get_errors':
            if data_dict['axis']:
                self.last_error_status['axis'] = data_dict['axis']
                self.last_error_status['motor'] = data_dict['motor']
                self.last_error_status['controller'] = data_dict['controller']
                self.last_error_status['encoder'] = data_dict['encoder']
                self.state = 'run' #don't stop running if checking errors. 
        
        if self.state == 'home':
            if 'home_started' in data_dict and data_dict['home_started'] == True:
                self.state = 'wait_home'
        
        if self.state == 'wait_home':
            if 'home_complete' in data_dict and data_dict['home_complete'] == True:
                self.state = 'halt'

    def calibrate(self):
        #only calibrate if in correct state to
        if self.state == 'halt' or self.state == 'run':
            self.state = 'start_calibrate'

    def configure(self):
        self.state = 'configure'

    def get_curr_state(self):
        return self.state

    def enable(self):
        if self.state == 'halt':
            self.state = 'enable'
    
    def disable(self):
        self.state = 'disable'

    def get_torque_limit(self):
        return self.max_current * self.torque_constant * 8.8507 * self.gear_ratio

    def get_setpoint(self):
        return self.pos_command
    
    def poll_errors(self):
        self.state = 'get_errors'

    def get_errors(self):
        return {
            'axis': self.last_error_status['axis'],
            'motor': self.last_error_status['motor'],
            'encoder': self.last_error_status['encoder'],
            'controller': self.last_error_status['controller']
        }

    def is_calibration_complete(self):
        return self.state == 'halt'
    
    def home(self):
        self.state = 'home'
    
    def is_home_complete(self):
        return self.state == 'halt'
    
    def set_zero(self):
        self.state = 'set_zero'

