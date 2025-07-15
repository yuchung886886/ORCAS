import orcas_serial
import orcas_radar

calibration = {'camera_fov': {'width': 48.8, 'height': 62.2},                     
               'parm_B': -14.0,
               'parm_D': 7.9,               
               'parm_E': 3.7,                     
               'camera_tilt_angle': [(39.2, 40.6),(-13.8, -10.77)], #[(aeg_tilt_angle1, camera_tilt_angle1),...]
               'camera_pan_angle': 2.24} 
status = {'scenario': 'interacting', # interacting, calibrating, tracking
          'fire_mode': 'SAFE',
          'aeg_pan_angle': 0,
          'aeg_tilt_angle': 0,
          'aeg_semi_fire_duty': 70,
          'aeg_auto_fire_duty': 100,
          'aeg_piston_released_timeout': 100,
          'feeder_motor_steps_per_shot': 210,
          'aiming_distance': 0,
          'ammo_loading_en': 0,
          'ammo_unloading_en': 0,
          'searchlight_en': 0,
          'searchlight_duty': 100,
          'camera_zoom_level': 1,          
          'received_frame_size': {'width': 0, 'height': 0},
          'frame_receiving_period': 0.1,
          'displayed_frame_size': {'width': 480, 'height': 640},
          'displayed_frame_origin_coor': {'x': 0, 'y': 0},
          'aim_icon_coor_update_req': 0,
          'aim_icon_coor': {'x': 0, 'y': 0},
          'aim_icon_size': 60,
          'static_icon_size': 30}              
cmds = {'pan_angle': 0,
        'tilt_angle': 0,
        'touch_coor': {'x': 0, 'y': 0},
        'touch_holding': 0,
        'clear_coor': 0,
        'direct_fire': 0,
        'cease_fire': 0,
        'enable_safety': 0,
        'disable_safety': 0,        
        'load_ammo': 0,
        'unload_ammo': 0,
        'enable_searchlight': 0,
        'disable_searchlight': 0,
        'enable_radar': 0,
        'disable_radar': 0}  

def init():
    ret = 0
    if (ret == 0):    
        ret = orcas_serial.get_pan_tilt_step_angle()
    if (ret == 0):
        ret = orcas_serial.set_fire_ctrl_config(status['aeg_semi_fire_duty'], status['aeg_auto_fire_duty'], status['aeg_piston_released_timeout'])
    if (ret == 0):
        ret = orcas_serial.set_ammo_feeder_config(status['feeder_motor_steps_per_shot'])
    if (ret == 0):
        ret = orcas_serial.set_searchlight_pwm(0)
        status['searchlight_en'] = 0
    if (ret == 0):
        ret = orcas_serial.set_fire_safety(1)
        status['fire_mode'] = 'SAFE'
    if (ret == 0):
        ret = orcas_radar.init()
    return ret