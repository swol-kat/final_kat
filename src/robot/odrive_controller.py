def set_odrive_params_func(drive, out_dict):
    drive.axis0.encoder.config.abs_spi_cs_gpio_pin = 4
    drive.axis1.encoder.config.abs_spi_cs_gpio_pin = 5
    drive.axis0.min_endstop.config.gpio_num = 6
    drive.axis1.min_endstop.config.gpio_num = 7
    drive.config.max_regen_current = 120.0
    drive.config.dc_bus_overvoltage_trip_level = 45.0
    drive.config.dc_max_positive_current = 120.0
    drive.config.dc_max_negative_current = -120.0
    drive.save_configuration()

class Odrive_Controller:
    def __init__(self, odrive_pipe, joint0, joint1):
        self.joint0 = joint0
        self.joint1 = joint1
        self.odrive_pipe = odrive_pipe
        self.state = 'ready'
        self.next_index = 1

    def send_packet(self):

        if self.state == 'wait_for_response':
            print('you done fucked up')
            return

        if self.state == 'set_params':
            self.joint0.configure()
            self.joint1.configure()
            odrive_command = {}
            odrive_command['axis_0'] = self.joint0.get_command()
            odrive_command['axis_1'] = self.joint1.get_command()
            odrive_command['command'] = set_odrive_params_func
            odrive_command['index'] = self.next_index
            self.next_index = self.next_index + 1
            self.state = 'wait_for_response'
            self.odrive_pipe.send(odrive_command)

        if self.state == 'ready':
            #make packet to send to odrive
            odrive_command = {}
            odrive_command['axis_0'] = self.joint0.get_command()
            odrive_command['axis_1'] = self.joint1.get_command()
            odrive_command['index'] = self.next_index
            self.next_index = self.next_index + 1
            self.state = 'wait_for_response'
            self.odrive_pipe.send(odrive_command)

    
    def set_odrive_params(self):
        self.state = 'set_params'
    
    def block_for_response(self):
        if self.state != 'wait_for_response':
            #catch command not sent so not infinite block
            return
        index = 0
        data = {}
        while (index + 1) != self.next_index:
            data_in = self.odrive_pipe.recv()
            index = data_in['index']

        self.joint0.recieve_data(data_in['axis_0'])
        self.joint1.recieve_data(data_in['axis_1'])
        self.state = 'ready'

        
