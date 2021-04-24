import odrive
import odrive.enums
import time
from multiprocessing import Process, Pipe
from copy import deepcopy
from collections import namedtuple
import json
from robot import Robot, Arm, VirtualJoint, Threaded_Joint, Odrive_Controller
from robot.gaits import Wiggle, OpenWalk
from robot.util import swing_pos, ground_pos
from math import tau
import numpy as np
import random
odrive_pipe = namedtuple('odrive_pipe', ['to_worker', 'to_main', 'axis_0_name', 'axis_1_name'])

process_list = []
odrive_controllers = []
joint_dict = {}
axis_dict = {}
serials = []
arm_dict = {}
arm_variables = {'D1': 3.319+.35, 'D2': 3.124, 'A2': 7.913, 'A3': 7.913} #some default variables
robot = None

# setup 
def setup():
    load_axis_data()
    find_odrives() 
    load_arm_vars()
    create_arms()
    flash_drive_params()
    calibrate_joints()
    get_errors()
    print('Waiting 10 seconds before Zeroing Joints')
    time.sleep(10)
    zero_joints()
    enable_joints()
    return create_robot()
    
            
def odrive_worker(serial, conn):
    search_serial = format(int(serial), 'x').upper()
    print(f'searching for odrive {search_serial}')
    od = odrive.find_any(serial_number = search_serial)
    print(f'found odrive {search_serial}')
    conn.send(0)
    while True:
        command = conn.recv()
        
        out_data = {}
        out_data['axis_0'] = {}
        out_data['axis_1'] = {}
        out_data['odrive'] = {}

        #run given functions on assigned axes if available
        if 'command' in command['axis_0']:
            #print('axis 0 commandflash_drive_params')
            command['axis_0']['command'](od.axis0, out_data['axis_0'])
        if 'command' in command['axis_1']: 
            #print('axis 1 command')
            command['axis_1']['command'](od.axis1, out_data['axis_1'])
        if 'command' in command:
            #print('odrive command')
            command['command'](od, out_data['odrive'])

        #index command is necessary. added in odrive handler
        out_data['index'] = command['index']
        
        #pos, curr commands automatically done. must be sent in control packets
        od.axis0.controller.input_pos = command['axis_0']['pos_command']
        od.axis1.controller.input_pos = command['axis_1']['pos_command']
        # od.axis0.motor.config.current_lim = command['axis_0']['curr_command']
        # od.axis1.motor.config.current_lim = command['axis_1']['curr_command']

        out_data['axis_0']['data'] = {'pos':od.axis0.encoder.pos_estimate, 'vel':od.axis0.encoder.vel_estimate, 'current':od.axis0.motor.current_control.Iq_measured}
        out_data['axis_1']['data'] = {'pos':od.axis1.encoder.pos_estimate, 'vel':od.axis1.encoder.vel_estimate, 'current':od.axis1.motor.current_control.Iq_measured}

        conn.send(out_data)

def find_odrives():
    print("Attempting To Find Odrives")
    for serial in serials:
        to_worker, to_main = Pipe()
        joint_dict[axis_dict[serial]['axis0']['name']] = Threaded_Joint(axis_dict[serial]['axis0']['ratio'], 8.27 / 160)
        joint_dict[axis_dict[serial]['axis1']['name']] = Threaded_Joint(axis_dict[serial]['axis1']['ratio'], 8.27 / 160)
        odrive_controllers.append(Odrive_Controller(to_main, joint_dict[axis_dict[serial]['axis0']['name']], joint_dict[axis_dict[serial]['axis1']['name']]))
        process_list.append(Process(target=odrive_worker, args=(serial, to_worker, )))
        process_list[-1].start()
        good = to_main.recv() #just wait for thread to respond so we know that it found the odrive
    print("Odrives Found")

def load_axis_data():
    print("Attempting To Load Axis Data")
    global axis_dict, serials
    axis_dict = json.loads(open('axis_config_pack1.json', "r").read())
    serials = list(axis_dict.keys())
    print("Axis Data Loaded")

def load_arm_vars():
    global arm_variables
    print("Attempting To Load Arm Params")
    #initialize arm variables - move to JSON file eventually
    arm_variables = {'D1': 3.319, 'D2': 3.125, 'A2': 7.913, 'A3': 7.913}
    print("Arm Params Loaded")

def create_arms():
    print("Attempting To Create Arm Objects")
    global arm_dict, arm_variables
    arm_dict = {}
    arm_dict['front_right'] = Arm(joint_dict['1 lower'], joint_dict['1 upper'], joint_dict['1 shoulder'], arm_variables, 1)
    arm_dict['back_right'] = Arm(VirtualJoint(-9,8.27 / 160), VirtualJoint(-9,8.27 / 160), VirtualJoint(-9,8.27 / 160), arm_variables, 4)
    arm_dict['front_left'] = Arm(joint_dict['2 lower'], joint_dict['2 upper'], joint_dict['2 shoulder'], arm_variables, 2)
    arm_dict['back_left'] = Arm(VirtualJoint(-9,8.27 / 160), VirtualJoint(-9,8.27 / 160), VirtualJoint(-9,8.27 / 160), arm_variables, 3)
    print("Arm Objects Created")

def flash_drive_params():
    print('Attempting To Flash Controllers') 
    global odrive_controllers
    if odrive_controllers:
        for controller in odrive_controllers:
            controller.set_odrive_params()

        for controller in odrive_controllers:
            controller.send_packet()

        for controller in odrive_controllers:
            controller.block_for_response()
        
        print('Controllers Flashed')
    else:
        print('No Controllers')

def calibrate_joints():#just calibrate one arm for arjun
    print('Attempting To Calibrating Joints')
    global joint_dict, odrive_controllers
    for joint in joint_dict.values():
        joint.calibrate()

    cal_incomplete = True

    while cal_incomplete:
        for controller in odrive_controllers:
            controller.send_packet()
        for controller in odrive_controllers:
            controller.block_for_response()
        for joint in joint_dict.values():
            cal_incomplete &= joint.is_calibration_complete()
            
        cal_incomplete = not cal_incomplete
    print('Calibration Complete')

def home_joints():
    global joint_dict, odrive_controllers
    print('Attempting To Homing Joints')
    for joint in joint_dict.values():
        joint.home()

    cal_incomplete = True

    while cal_incomplete:
        for controller in odrive_controllers:
            controller.send_packet()
        for controller in odrive_controllers:
            controller.block_for_response()
        for joint in joint_dict.values():
            cal_incomplete &= joint.is_home_complete()
            
        cal_incomplete = not cal_incomplete
    print('Joints Homed')

def zero_joints():
    print('Attempting To Zeroing Joints')
    global joint_dict, odrive_controllers
    for joint in joint_dict.values():
        joint.set_zero()

    for controller in odrive_controllers:
        controller.send_packet()
    for controller in odrive_controllers:
        controller.block_for_response()
    print('Joints Zerod')

def enable_joints():
    print('Attempting To Enable Joints')
    global joint_dict, odrive_controllers
    for joint in joint_dict.values():
        joint.enable()

    for controller in odrive_controllers:
        controller.send_packet()
    for controller in odrive_controllers:
        controller.block_for_response()
    print('Joints Enabled')

def disable_joints():
    print('Attempting To Disable Joints')
    global joint_dict, odrive_controllers
    for joint in joint_dict.values():
        joint.disable()

    for controller in odrive_controllers:
        controller.send_packet()
    for controller in odrive_controllers:
        controller.block_for_response()
    print('Joints Disabled')

def kill_process():
    global odrive_controllers
    for controller in odrive_controllers:
        controller.send_packet()
    for controller in odrive_controllers:
        controller.block_for_response()
    for process in process_list:
        process.terminate()

#------------------------------------arjun do stuff here ---------------------------------------------
def create_robot():
    global arm_dict, robot
    robot = Robot(arm_dict)
    robot.boot()
    return robot

def loop():
    global robot, odrive_controllers
    robot.loop()
    for controller in odrive_controllers:
        controller.send_packet()
    for controller in odrive_controllers:
        controller.block_for_response()
        
def loop2():
    global robot, odrive_controllers
    for controller in odrive_controllers:
        controller.send_packet()
    for controller in odrive_controllers:
        controller.block_for_response()

def set_movement_vector(x=0,y=0,z=0,a=0,b=0,g=0):
    movement_dict = {'x': x, 'y': y,'z':z, 'alpha':a,'beta':b,'gamma':g}
    robot.movement_vector = movement_dict
    
def run(seconds = 10):
    start_time = time.time()
    while time.time() - start_time < seconds:
        loop()
   
def get_errors():
    global joint_dict, odrive_controllers
    for joint in joint_dict.values():
        joint.poll_errors()

    for controller in odrive_controllers:
        controller.send_packet()
    for controller in odrive_controllers:
        controller.block_for_response()  
        
    for name,joint in joint_dict.items():
        output = joint.get_errors()

        print(f'{name}')
        print()
        print(output)
        print()
        


def ikin_test():
    a =arm_dict['front_left']
    #testing x,y,z
    pos_to_test = list(np.linspace([14,0,0],[5,0,0],100))
    pos_to_test += list(np.linspace([5,0,0],[14,0,0],100))
    zero = np.array([0,0,0])
    
    a.target_pos = (pos_to_test[0])
    a.update()
    loop2()
    time.sleep(3)
    
    
    for p in pos_to_test:
        print(p)
        a.target_pos = (p)
        a.update() 
        loop2()



def joint_range_test(points_to_gen=100):
    arms = [arm_dict['front_right'],arm_dict['front_left']]
    zero = np.array([0,0,0])
    angles_to_test = list(np.linspace(zero,[tau/16,tau/16,tau/2],points_to_gen))
    angles_to_test += list(np.linspace([tau/16,tau/16,tau/2],[-tau/16,-tau/16,-tau/2],points_to_gen))
    angles_to_test += list(np.linspace([-tau/16,-tau/16,-tau/2],zero,points_to_gen))
    
    for angle in angles_to_test:
        for arm in arms:
            arm.go_to_thetas(angle)
            loop2()
    



def go_home():
    for a in arm_dict.values():
        a.go_to_thetas(np.array([0,0,0]))
    loop()

if __name__ =="__main__":
    robot = setup()
    robot.gait = OpenWalk()
