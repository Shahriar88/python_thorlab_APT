# -*- coding: utf-8 -*-
"""
Created on Tue Nov  2 12:23:39 2021
@author: Casper
"""

import thorlabs_apt_protocol as apt
import serial
import pandas as pd
import numpy as np
from time import sleep
from tkinter import *

#%%
class thorlab_lin_stage_LTS_150:
    def __init__(self, portn="COM5", baud=115200, dest=0x50, source=1, chan_ident=1):
        self.portn=portn
        self.baud=baud
        self.source=source
        self.dest=dest
        self.chan_ident=chan_ident

    def initialize(self):
        port = serial.Serial(self.portn, self.baud, rtscts=True, timeout=0.1)
        port.rts = True
        port.reset_input_buffer()
        port.reset_output_buffer()
        port.rts = False
        # This message is sent on start up to notify the controller of the 
        # source and destination addresses. A client application must send 
        # this message as part of its initialization process.
        port.write(apt.hw_no_flash_programming(self.dest,self.source))
        self.port=port
        print('Device Initialized')
        return port

#%%
    def prnt(self,msg_list):
            unpacker = apt.Unpacker(self.port)
            all_msg=[]
            for msg in unpacker:
                all_msg.append(msg)
                #print(msg)
            all_msg=np.array(all_msg,dtype=object)
            all_msg=pd.DataFrame(all_msg,columns=msg_list)
            return all_msg
    
#%%
#(msg='mot_move_completed', msgid=1124, dest=1, source=80, chan_ident=1, position=14279384, velocity=0, forward_limit_switch=False, reverse_limit_switch=False, moving_forward=True, moving_reverse=True, jogging_forward=False, jogging_reverse=False, motor_connected=False, homing=True, homed=False, tracking=False, interlock=False, settled=False, motion_error=False, motor_current_limit_reached=False, channel_enabled=False)

    def msgp_jog(self):
        msg_list=[
         'msg', 'msgid', 'dest', 'source', 'chan_ident', 'position', 'velocity',
         'forward_limit_switch', 'reverse_limit_switch', 'moving_forward',
         'moving_reverse', 'jogging_forward', 'jogging_reverse',
         'motor_connected', 'homing', 'homed', 'tracking', 'interlock',
         'settled', 'motion_error', 'motor_current_limit_reached', 'channel_enabled']
        # for jog
        try:
            p_jog=self.prnt(msg_list)
            return p_jog
        except:
            print('Error reading fun: msgp_jog')
            return 0
        
#%%

# hw_get_info(msg='hw_get_info', msgid=6, dest=1, source=80, serial_number=45218804, model_number=b'LTS150\x00\x00', type=16, firmware_version=[3, 0, 10], hw_version=3, mod_state=0, nchs=1)
#Sent to request hardware information from the controller

    def msgp_hardware(self):
        msg_list2=[
        'msg', 'msgid', 'dest', 'source', 'serial_number', 'model_number', 'type', 
        'firmware_version', 'hw_version', 'mod_state', 'nchs']
        
        # for hardwareinfo
        self.port.write(apt.hw_req_info(self.dest, self.source))
        try:
            p_hardware=self.prnt(msg_list2)
            return p_hardware
        except:
            print('Error setting fun: msgp_hardware')
            return 0

#%% MGMSG_MOT_SET_VELPARAMS
#[mot_get_velparams(msg='mot_get_velparams', msgid=1045, dest=1, source=80, chan_ident=1, min_velocity=0, acceleration=90121, max_velocity=439804651)]



    def msgp_velparam(self):
        msg_list3=[
        'msg', 'msgid', 'dest', 'source', 'chan_ident', 'min_velocity',
        'acceleration', 'max_velocity']
        # for hardwareinfo
        self.port.write(apt.mot_req_velparams(self.dest,self.source, self.chan_ident))
        try:
            p_velparam=self.prnt(msg_list3)
            return p_velparam
        except:
            print('Error reading fun: msgs_velparam')
            return 0
        
    # Used to set the trapezoidal velocity parameters for the specified
    # motor channel. For DC servo controllers, the velocity is set in
    # encoder counts/sec and acceleration is set in encoder counts/sec/sec.
    # For stepper motor controllers the velocity is set in microsteps/sec
    # and acceleration is set in microsteps/sec/sec.

    def msgs_velparam(self, min_velocity, acceleration, max_velocity):
        # for hardwareinfo
        try:
            self.port.write(apt.mot_set_velparams(self.dest, self.source,
                                             self.chan_ident,
                                             min_velocity=min_velocity,
                                             acceleration=acceleration,
                                             max_velocity=max_velocity))
            print('Done')
        except:
            print('Error setting fun: msgs_velparam')
        return 0

#%% MGMSG_MOT_SET_JOGPARAMS

# mot_get_jogparams(msg='mot_get_jogparams', msgid=1048, dest=1, source=80, chan_ident=1, jog_mode=2, step_size=2048000, min_velocity=0, acceleration=45061, max_velocity=219902326, stop_mode=2)

    def msgp_jogparam(self):
        msg_list4=[
            'msg', 'msgid', 'dest', 'source', 'chan_ident', 'jog_mode', 'step_size', 
            'min_velocity', 'acceleration', 'max_velocity', 'stop_mode']
        # for hardwareinfo
        self.port.write(apt.mot_req_jogparams(self.dest,self.source, self.chan_ident))
        try:
            p_jogparam=self.prnt(msg_list4)
            return p_jogparam
        except:
            print('Error reading fun: msgp_jogparam')
            return 0
    

# Used to set the velocity jog parameters for the specified motor
# channel, For DC servo controllers, values set in encoder counts.
# For stepper motor controllers the values is set in microsteps

    def msgs_jogparam(self, step_size, min_velocity, acceleration,
                      max_velocity,jog_mode=2, stop_mode=2):
        # for hardwareinfo
        try:
            self.port.write(apt.mot_set_jogparams(
                                             self.dest, self.source,
                                             self.chan_ident,
                                             jog_mode=jog_mode,
                                             step_size=step_size,
                                             stop_mode=stop_mode,
                                             min_velocity=min_velocity,
                                             acceleration=acceleration,
                                             max_velocity=max_velocity))
            
            
            print('Done')
        except:
            print('Error setting fun: msgs_jogparam')
        return 0

#%%  MGMSG_MOT_SET_GENMOVEPARAMS

# [mot_get_genmoveparams(msg='mot_get_genmoveparams', msgid=1084, dest=1, source=80, chan_ident=1, backlash_distance=20480)]

    def msgp_backlash(self):
        msg_list5=['msg', 'msgid', 'dest', 'source',
               'chan_ident', 'backlash_distance']
        # for hardwareinfo
        self.port.write(apt.mot_req_genmoveparams(self.dest,self.source, self.chan_ident))
        try:
            p_backlash=self.prnt(msg_list5)
            return p_backlash
        except:
            print('Error reading fun: msgp_backlash')
            return 0
        
# Used to set the general move parameters for the specified motor
#channel. At this time this refers specifically to the backlash settings
    def msgs_backlash(self, backlash_distance):
        # for hardwareinfo
        try:
            self.port.write(apt.mot_set_genmoveparams(
                                             self.dest, self.source,
                                             self.chan_ident,
                                             backlash_distance=backlash_distance))                                         
            print('Done backlash_distance')
        except:
            print('Error setting fun: msgs_backlash')
        return 0

#%% MGMSG_MOT_SET_MOVERELPARAMS
# [mot_get_moverelparams(msg='mot_get_moverelparams', msgid=1095, dest=1, source=80, chan_ident=1, relative_distance=0)]

    def msgp_relmov(self):
        msg_list7=['msg', 'msgid', 'dest', 'source',
               'chan_ident', 'relative_distance']
        # for hardwareinfo
        self.port.write(apt.mot_req_moverelparams(self.dest,self.source, self.chan_ident))
        try:
            p_relmov=self.prnt(msg_list7)
            return p_relmov
        except:
            print('Error reading fun: msgp_relmov')
            return 0
        
# Used to set the general move parameters for the specified motor
#channel. At this time this refers specifically to the backlash settings
    def msgs_relmov(self, relative_distance= 0):
        # for hardwareinfo
        try:
            self.port.write(apt.mot_set_moverelparams(
                                             self.dest, self.source,
                                             self.chan_ident,
                                             relative_distance=relative_distance))                                         
            print('Done relative_distance')
        except:
            print('Error setting fun: msgs_relmov')
        return 0


#%% MGMSG_MOT_SET_MOVEABSPARAMS

    def msgp_absmov(self):
        msg_list8=['msg', 'msgid', 'dest', 'source',
               'chan_ident', 'absolute_position']
        # for hardwareinfo
        self.port.write(apt.mot_req_moveabsparams(self.dest,self.source, self.chan_ident))
        try:
            p_absmov=self.prnt(msg_list8)
            return p_absmov
        except:
            print('Error reading fun: msgp_absmov')
            return 0
    
   
    def msgs_absmov(self, absolute_position= 0):
        # for hardwareinfo
        try:
            self.port.write(apt.mot_set_moveabsparams(
                                             self.dest, self.source,
                                             self.chan_ident,
                                             absolute_position=absolute_position))                                         
            print('Done absolute_position')
        except:
            print('Error setting fun: msgs_absmov')
        return 0
#%% MGMSG_MOT_SET_HOMEPARAMS


    def msgp_homeparam(self):
        
        msg_list9=['msg', 'msgid', 'dest', 'source','chan_ident', 
               'home_dir', 'limit_switch', 'home_velocity', 'offset_distance']
        
        # for hardwareinfo
        self.port.write(apt.mot_req_homeparams(self.dest,self.source, self.chan_ident))
        try:
            p_homeparam=self.prnt(msg_list9)
            return p_homeparam
        except:
            print('Error reading fun: msgp_homeparam')
            return 0


    def msgs_homeparam(self, home_velocity, offset_distance,
                       home_dir=2, limit_switch=1, ):
        try:
            self.port.write(apt.mot_set_homeparams(
                                             self.dest, self.source,
                                             self.chan_ident,
                                              home_dir=home_dir, 
                                              limit_switch=limit_switch, 
                                              home_velocity=home_velocity,
                                              offset_distance=offset_distance))                                         
            print('Done homeparam')
        except:
            print('Error setting fun: msgs_homeparam')
        return 0


#%% MGMSG_MOT_SET_LIMSWITCHPARAMS
# mot_get_limswitchparams(msg='mot_get_limswitchparams', msgid=1061, dest=1, source=80, chan_ident=1, cw_hardlimit=2, ccw_hardlimit=2, cw_softlimit=1228800, ccw_softlimit=409600, soft_limit_mode=0)

    def msgp_limitsw(self):
        msg_list10=['msg', 'msgid', 'dest', 'source',
               'chan_ident','cw_hardlimit', 'ccw_hardlimit', 'cw_softlimit',
               'ccw_softlimit', 'soft_limit_mode']
        
        self.port.write(apt.mot_req_limswitchparams(self.dest,self.source, self.chan_ident))
        try:
            p_limitsw=self.prnt(msg_list10)
            return p_limitsw
        except:
            print('Error reading fun: msgp_limitsw')
            return 0

    def msgs_limitsw(self, cw_softlimit, ccw_softlimit, cw_hardlimit=2,
                     ccw_hardlimit=2, soft_limit_mode=0):
        try:
            self.port.write(apt.mot_set_limswitchparams(
                                             self.dest, self.source,
                                             self.chan_ident,                                        
                                             cw_hardlimit=cw_hardlimit, 
                                             ccw_hardlimit=ccw_hardlimit, 
                                             cw_softlimit=cw_softlimit, 
                                             ccw_softlimit=ccw_softlimit, 
                                             soft_limit_mode=soft_limit_mode))                                         
            print('Done limitsw')
        except:
            print('Error setting fun: msgs_limitsw')
        return 0
#%%

    def update(self):
        
        p_hardware=self.msgp_hardware()
        self.var_serial_number=p_hardware['serial_number']
        self.var_model_number=p_hardware['model_number']
        self.var_type=p_hardware['type']
        self.var_firmware_version=p_hardware['firmware_version']
        self.var_hw_version=p_hardware['hw_version']
        self.var_no_channels=p_hardware['nchs']
        
        p_velparam=self.msgp_velparam()
        self.var_min_velocity=p_velparam['min_velocity']
        self.var_acceleration=p_velparam['acceleration']
        self.var_max_velocity=p_velparam['max_velocity']
    
        p_jogparam=self.msgp_jogparam()
        self.var_jog_mode=p_jogparam['jog_mode']
        self.var_jog_step_size=p_jogparam['step_size']
        self.var_jog_min_velocity=p_jogparam['min_velocity']
        self.var_jog_acceleration=p_jogparam['acceleration']
        self.var_jog_max_velocity=p_jogparam['max_velocity']
        self.var_jog_stop_mode=p_jogparam['stop_mode']
        
        p_backlash=self.msgp_backlash()
        self.var_backlash_distance=p_backlash['backlash_distance']
        
        p_relmov=self.msgp_relmov()
        self.var_relative_distance=p_relmov['relative_distance']
        
        p_absmov=self.msgp_absmov()
        self.var_absolute_position=p_absmov['absolute_position']
        
        p_homeparam=self.msgp_homeparam()
        self.var_home_direction=p_homeparam['home_dir']
        self.var_home_limit_switch=p_homeparam['limit_switch']
        self.var_home_velocity=p_homeparam['home_velocity']
        self.var_home_offset_distance=p_homeparam['offset_distance']
        
        
        self.work_jog_f()
        sleep(2)
        self.work_jog_r()
        sleep(2)
        p_jog=self.msgp_jog()
        sleep(2)
        p_jogparam=self.msgp_jogparam()
        self.var_jog_step_size=p_jogparam['step_size']
        self.var_jog_position=p_jog['position']
        self.var_jog_velocity=p_jog['velocity']
        self.var_jog_homed=p_jog['homed']
        print('Data Updated >>>>>>>>>>>>>>>>>>>')
#%% 
#%%
#%% Work
#port.write(apt.mot_move_velocity(source=1, dest=0x50, chan_ident=1, direction=2))
#port.write(apt.mot_move_stop(source=1, dest=0x50, chan_ident=1, stop_mode=2))
#port.write(apt.mot_move_stop(source=1, dest=0x50, chan_ident=1, stop_mode=1))

    def work_back(self):
        self.port.write(apt.mot_move_home(self.dest,self.source, self.chan_ident))
        self.port.write(apt.mot_move_absolute(self.dest,self.source, self.chan_ident))
    
    def work_abspos(self, position=1): # 0 to 67
        self.position=position*1_000_000
        self.port.write(apt.mot_move_absolute(self.dest,self.source, self.chan_ident, position=position))
    
    def work_reldis(self, relative_distance=1): # 0 to 67
        self.relative_distance=int(float(relative_distance)*1_000_000/150*66) #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        self.msgs_relmov(relative_distance=self.relative_distance)
        self.port.write(apt.mot_move_absolute(self.dest,self.source, self.chan_ident))
               
    def work_jog_f(self):
        self.port.write(apt.mot_move_jog(self.dest,self.source, self.chan_ident, direction=1)) # to 150
            
        
    def work_jog_r(self):
        self.port.write(apt.mot_move_jog(self.dest,self.source, self.chan_ident, direction=2)) # to Home
    
        
    def stop_hard(self):
        self.port.write(apt.mot_move_stop(self.dest,self.source, self.chan_ident, stop_mode=1))
    
    def stop_soft(self):
        self.port.write(apt.mot_move_stop(self.dest,self.source, self.chan_ident, stop_mode=2))
    
    
    def disconnect(self):
        self.port.close()
        print('Device disconnected')
        
    def work_home(self):
        self.work_reldis(relative_distance=0)
        
if __name__ == '__main__':
    global device1, fun_var_list
    
    #device1.disconnect()
    
    # msgp--> get data, msgs--> set data, work--> movement >>>functions 
    # and var--> variables
    
    
    def device1_initialize():
            global device1,fun_var_list1
            portn=(port_10.get())
            baud=int(port_11.get())
            dest=int(port_12.get())
            source=int(port_13.get())
            chan_ident=int(port_14.get())
            
            device1=thorlab_lin_stage_LTS_150(portn=portn, baud=baud, dest=dest, source=source,  chan_ident=chan_ident)
            device1.initialize()
            device1.update()
            fun_var_list=dir(device1)
    
    
    #%%
    
    #%% tkinter initialize
    root=Tk()
    root.title("Casper-ThorlabsLTS150/M_Control_Work")
    root.iconbitmap("icons\\me.ico")
    h=300
    w=1000
    root.geometry(str(w)+'x'+str(h))
    #%%
    #%% Initial Inputs
    row=0
    Label(root,text="Port",bg='white',fg='green').grid(row=row,column=0)
    Label(root,text="Baud",bg='white',fg='green').grid(row=row,column=1)
    Label(root,text="Dest",bg='white',fg='green').grid(row=row,column=2)
    Label(root,text="Source",bg='white',fg='green').grid(row=row,column=3)
    Label(root,text="Channel_Ident",bg='white',fg='green').grid(row=row,column=4)
    Label(root,text="Select_Position",bg='white',fg='green').grid(row=row,column=5)
    
    
    
    row+=1
    port_10=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
    port_10.grid(row=row,column=0,columnspan=1, padx=1,pady=2)
    port_10.insert(0, 'COM5')

    port_11=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
    port_11.grid(row=row,column=1,columnspan=1, padx=1,pady=2)
    port_11.insert(0, '115200')
    
    port_12=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
    port_12.grid(row=row,column=2,columnspan=1, padx=1,pady=2)
    port_12.insert(0, '1')    

    port_13=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
    port_13.grid(row=row,column=3,columnspan=1, padx=1,pady=2)
    port_13.insert(0, '80')
    
    port_14=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
    port_14.grid(row=row,column=4,columnspan=1, padx=1,pady=2)
    port_14.insert(0, '1')
    
    port_15=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
    port_15.grid(row=row,column=5,columnspan=1, padx=1,pady=2)
    port_15.insert(0, '1')
    
    #%%
    #%% Button
    row+=1
    button10=Button(root,text="Device1 Initialize",padx=20, pady=20, bg='goldenrod',command=(lambda:device1_initialize())).grid(row=row,column=0)
    button11=Button(root,text="Device1 Disconnect",padx=20, pady=20, bg='red', command=(lambda:device1.disconnect())).grid(row=row,column=1)
    button12=Button(root,text="Move_Jog_Home",padx=20, pady=20, bg='forestgreen', command=(lambda: device1.work_home())).grid(row=row,column=2)
    button13=Button(root,text="Move_Jog_Forward >>>",padx=20, pady=20, bg='aquamarine2', command=(lambda: device1.work_jog_f())).grid(row=row,column=4)
    button14=Button(root,text="<<<Move_Jog_Reverse",padx=20, pady=20, bg='aquamarine2', command=(lambda: device1.work_jog_r())).grid(row=row,column=3)
    button15=Button(root,text="Move_Jog_Position",padx=20, pady=20, bg='maroon3', command=(lambda: device1.work_reldis((port_15.get())))).grid(row=row,column=5)
    
    #button15=Button(root,text="Move_Jog_Position",padx=20, pady=20, command=(lambda: device1.work_reldis(int(port_15.get())*1_000_000))).grid(row=row,column=5)
    #%%
    row+=1
    button16=Button(root,text="Setting",padx=20, pady=20, bg='peachpuff',command=(lambda:new_root())).grid(row=row,column=0)
    button17=Button(root,text="Data Update",padx=20, pady=20, bg='cyan',command=(lambda: device1.update())).grid(row=row,column=1)
    button18=Button(root,text="Hard Stop",padx=20, pady=20, bg='red',command=(lambda: device1.stop_hard())).grid(row=row,column=2)
    button19=Button(root,text="Soft Stop",padx=20, pady=20, bg='deeppink',command=(lambda: device1.stop_soft())).grid(row=row,column=3)
    def new_root():
        global device1
        h=300
        w=1000
        root=Tk()
        root.title("Casper-ThorlabsLTS150/M_Control_Setting")
        root.iconbitmap("icons\\me.ico")
        root.geometry(str(w)+'x'+str(h))
        row=0
        
        Label(root,text="serial_number",bg='white',fg='green').grid(row=row,column=0)
        Label(root,text="Jog Position",bg='white',fg='green').grid(row=row,column=1)
        
        row+=2
        Label(root,text="jog_step_size",bg='white',fg='green').grid(row=row,column=0)
        Label(root,text="jog_min_velocity",bg='white',fg='green').grid(row=row,column=1)
        Label(root,text="jog_acceleration",bg='white',fg='green').grid(row=row,column=2)
        Label(root,text="jog_max_velocity",bg='white',fg='green').grid(row=row,column=3)
        
        
        Label(root,text="<<<>>>",bg='white',fg='green').grid(row=row,column=4)
        Label(root,text="min_velocity",bg='white',fg='green').grid(row=row,column=5)
        Label(root,text="acceleration",bg='white',fg='green').grid(row=row,column=6)
        Label(root,text="max_velocity",bg='white',fg='green').grid(row=row,column=7)
        
        
        
        
        
        
        #>>>>
        
        row-=1
        port_0=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
        port_0.grid(row=row,column=0,columnspan=1, padx=1,pady=2)
        port_0.insert(0, device1.var_serial_number[0])
        
        port_00=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
        port_00.grid(row=row,column=1,columnspan=1, padx=1,pady=2)
        port_00.insert(0, device1.var_jog_position[0])
        
        #>>>>>>>>>>
        row+=2
        port_1=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
        port_1.grid(row=row,column=0,columnspan=1, padx=1,pady=2)
        port_1.insert(0, device1.var_jog_step_size[0])
        
        port_2=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
        port_2.grid(row=row,column=1,columnspan=1, padx=1,pady=2)
        port_2.insert(0, device1.var_jog_min_velocity[0])
        
        port_3=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
        port_3.grid(row=row,column=2,columnspan=1, padx=1,pady=2)
        port_3.insert(0, device1.var_jog_acceleration[0])
        
        port_4=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
        port_4.grid(row=row,column=3,columnspan=1, padx=1,pady=2)
        port_4.insert(0, device1.var_jog_max_velocity[0])
        #>>>>>>>>>>>>
        port_5=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
        port_5.grid(row=row,column=5,columnspan=1, padx=1,pady=2)
        port_5.insert(0, device1.var_min_velocity[0])
        
        port_6=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
        port_6.grid(row=row,column=6,columnspan=1, padx=1,pady=2)
        port_6.insert(0, device1.var_acceleration[0])
        
        port_7=Entry(root, width=10,bg='lemon chiffon', borderwidth=10)
        port_7.grid(row=row,column=7,columnspan=1, padx=1,pady=2)
        port_7.insert(0, device1.var_max_velocity[0])
        
        def jog_update():
            step_size=int(port_1.get())
            min_velocity=int(port_2.get())
            acceleration=int(port_3.get())
            max_velocity=int(port_4.get())
            device1.msgs_jogparam(step_size, min_velocity, acceleration, max_velocity)
            device1.update()
            print('Jog Updated')
        
        def move_update():
            min_velocity=int(port_5.get())
            acceleration=int(port_6.get())
            max_velocity=int(port_7.get())
            device1.msgs_velparam(min_velocity, acceleration, max_velocity)
            device1.update()
            print('Move Updated')
            
        row+=1
        button10=Button(root,text="Jog Update",padx=20, pady=20, bg='aquamarine2',command=(lambda: jog_update())).grid(row=row,column=2)
        button11=Button(root,text="Move Update",padx=20, pady=20, bg='maroon3',command=(lambda: move_update())).grid(row=row,column=5)
        
        
        root.mainloop()
    
    
    root.mainloop()
    print('End of Progam')
    
"""

a=device1.msgp_backlash()

a=device1.msgp_homeparam()

device1.disconnect()

create an object
device=class
then initialize it
then update it , so that it can keep all the ncessary data into variables
"""