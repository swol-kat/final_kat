import odrive
import odrive.enums

odrives = odrive.find_any(find_multiple = int(input("how many odrives? ")))

jointList = []

   
def motor_configuration(axis):
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
    
def drive_configuration(drive):
    drive.axis0.encoder.config.abs_spi_cs_gpio_pin = 4
    drive.axis1.encoder.config.abs_spi_cs_gpio_pin = 5
    drive.axis0.min_endstop.config.gpio_num = 6
    drive.axis1.min_endstop.config.gpio_num = 7
    drive.config.max_regen_current = 120.0
    drive.config.dc_bus_overvoltage_trip_level = 45.0
    drive.config.dc_max_positive_current = 120.0
    drive.config.dc_max_negative_current = -120.0

def main():

    for od in odrives:
        try:
            motor_configuration(od.axis0)
            motor_configuration(od.axis1)
            drive_configuration(od)
            od.save_configuration()
            print(od.serial_number)
            od.reboot()
        except:
            pass
    
if __name__ == "__main__":
    main()