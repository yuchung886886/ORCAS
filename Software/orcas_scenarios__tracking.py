import cv2
import numpy as np
import math
import orcas_attributes
import orcas_radar

status = {'state': 'standby', # 'standby', 'tilt_homing', 'scanning', 'panning', 'searching', 'analyzing'
          'standby_timeout': 5,
          'searching_period': 0.4,
          'searching_timeout': 2.0,          
          'firing_period': 1.6,
          'saved_fire_mode': 'SAFE',
          'saved_aim_icon_size': 0,
          'saved_static_icon_size': 0,
          'proximity_warning_angle': 0}
          
target_color = {'hue_max': 99,
                'hue_min': 62,
                'sat_max': 240,
                'sat_min': 113,
                'val_max': 170,
                'val_min': 104}
                
timer1, timer2 = 0, 0

def init():
    global timer1
       
    status['saved_fire_mode'] = orcas_attributes.status['fire_mode']
    if (status['saved_fire_mode'] == 'SAFE'):
        fire_mode_button_img = "FireMode_Safe.bmp"
    elif (status['saved_fire_mode'] == 'SEMI'):
        fire_mode_button_img = "FireMode_Semi.bmp"
    else:
        fire_mode_button_img = "FireMode_Burst.bmp"
    orcas_attributes.cmds['enable_safety'] = 1
    orcas_attributes.status['fire_mode'] = 'SAFE'        
    
    if(orcas_attributes.status['camera_zoom_level'] == 1): 
        zoom_button_img = "Zoom_x1.bmp"
    elif(orcas_attributes.status['camera_zoom_level'] == 2): 
        zoom_button_img = "Zoom_x2.bmp"
       
    orcas_attributes.cmds['disable_searchlight'] = 1
    searchlight_button_img = "Searchlight_disable.bmp"    
    
    orcas_attributes.cmds['disable_radar'] = 1
    radar_button_img = "Radar_enable.bmp"
    
    status['saved_aim_icon_size'] = orcas_attributes.status['aim_icon_size']
    status['saved_static_icon_size'] = orcas_attributes.status['static_icon_size']
    orcas_attributes.status['aim_icon_size'] = 40
    orcas_attributes.status['static_icon_size'] = 20            
    status['state'] = 'standby'
    timer1 = 0
    
    return fire_mode_button_img, zoom_button_img, searchlight_button_img, radar_button_img
        
def de_init():
    orcas_attributes.cmds['disable_searchlight'] = 1
    orcas_attributes.cmds['disable_radar'] = 1
    orcas_attributes.status['fire_mode'] = status['saved_fire_mode']
    if (orcas_attributes.status['fire_mode'] == 'SAFE'):
        orcas_attributes.cmds['enable_safety'] = 1
    else:
        orcas_attributes.cmds['disable_safety'] = 1
    orcas_attributes.status['aim_icon_size'] = status['saved_aim_icon_size']
    orcas_attributes.status['static_icon_size'] = status['saved_static_icon_size']
    
def toggle_analysis():
    if (status['state'] == 'analyzing'):
        init()
    else:
        orcas_attributes.cmds['disable_radar'] = 1
        orcas_attributes.cmds['enable_safety'] = 1
        orcas_attributes.cmds['enable_searchlight'] = 1
        status['state'] = 'analyzing'

def analyze_target(frame):
    height, width, _ = frame.shape
    aim_icon_coor_x = orcas_attributes.status['aim_icon_coor']['x'] - orcas_attributes.status['displayed_frame_origin_coor']['x']
    aim_icon_coor_y = orcas_attributes.status['aim_icon_coor']['y'] - orcas_attributes.status['displayed_frame_origin_coor']['y']
    
    start_x, start_y = aim_icon_coor_x - orcas_attributes.status['static_icon_size'] // 2, aim_icon_coor_y - orcas_attributes.status['static_icon_size'] // 2
    end_x, end_y = aim_icon_coor_x + orcas_attributes.status['static_icon_size'], aim_icon_coor_y + orcas_attributes.status['static_icon_size']
    start_x, start_y = max(0, start_x), max(0, start_y)
    end_x, end_y = min(width, end_x), min(height, end_y)

    center_region = frame[start_y:end_y, start_x:end_x]
    hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_region)
    '''
    h_gt_85 = h[h > 85]
    h_lt_85 = h[h <= 85]
    if (h_gt_85.size > 0):    
        hue_gt_85_min, hue_gt_85_max = np.min(h_gt_85), np.max(h_gt_85)
    else:
        hue_gt_85_min = 85
        hue_gt_85_max = 85
    if (h_lt_85.size > 0):
        hue_lt_85_min, hue_lt_85_max = np.min(h_lt_85), np.max(h_lt_85)
    else:
        hue_lt_85_min = 85
        hue_lt_85_max = 85 
    '''
    hue_min, hue_max = np.min(h), np.max(h)
    sat_min, sat_max = np.min(s), np.max(s)
    val_min, val_max = np.min(v), np.max(v)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color = (0, 255, 0) 
    thickness = 2
    
    cv2.rectangle(frame, (start_x, start_y), (end_x, end_y), color, 1)
    # text = f"hue: {hue_lt_85_min},{hue_lt_85_max} ~ {hue_gt_85_min},{hue_gt_85_max}"            
    text = f"hue: {hue_min} ~ {hue_max}"            
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
    text_x_coor = 10
    text_y_coor = 50
    cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness) 
    text_y_coor += text_size[1] + 10
    
    text = f"sat: {sat_min} ~ {sat_max}"            
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]    
    cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness) 
    text_y_coor += text_size[1] + 10
    
    text = f"val: {val_min} ~ {val_max}"            
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]        
    cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness)     

def search_target(frame):

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   
    lower = np.array([target_color['hue_min'], target_color['sat_min'], target_color['val_min']])
    higher = np.array([target_color['hue_max'], target_color['sat_max'], target_color['val_max']])
    mask = cv2.inRange(hsv_frame, lower, higher)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center_x, center_y = -1, -1
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if (w > orcas_attributes.status['static_icon_size']) and (h > orcas_attributes.status['static_icon_size']):
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            center_x = x + w // 2
            center_y = y + h // 2
            break
            
    return frame, center_x, center_y 
    
def attach_info(frame):
    global timer1, timer2, timer3
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 4
    color = (0, 255, 0) 
    thickness = 4 
    
    if (status['state'] == 'standby'):
        timer1 = timer1 + 1
        if (timer1 > status['standby_timeout'] // orcas_attributes.status['frame_receiving_period']):
            status['state'] = 'tilt_homing'
            orcas_attributes.cmds['tilt_angle'] = -orcas_attributes.status['aeg_tilt_angle']
        else:
            text = f"{status['standby_timeout'] - int(timer1 * orcas_attributes.status['frame_receiving_period'])}"
            text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
            text_x_coor = frame.shape[1] // 2 - text_size[0] // 2 
            text_y_coor = frame.shape[0] // 2 + text_size[1] // 2 
            cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness)                
    elif (status['state'] == 'tilt_homing'):
        if (abs(orcas_attributes.status['aeg_tilt_angle']) < 1):
            if (orcas_radar.status['en'] == 0):
                orcas_attributes.cmds['enable_radar'] = 1
            status['state'] = 'scanning'          
        else:
            orcas_attributes.cmds['tilt_homing'] = 1
    elif (status['state'] == 'scanning'):    
        orcas_radar.append_radar_chart(frame)
        ret, pan_offset = orcas_radar.get_proximity_warning_pan_offset()
        if (ret >= 0):
            orcas_attributes.cmds['pan_angle'] = pan_offset
            status['proximity_warning_angle'] = orcas_attributes.status['aeg_pan_angle'] + pan_offset
            if (status['proximity_warning_angle'] > 180):
                status['proximity_warning_angle'] -= 360
            elif (status['proximity_warning_angle'] < -180):
                status['proximity_warning_angle'] += 360
            status['state'] = 'panning'
    elif (status['state'] == 'panning'):
        orcas_radar.append_radar_chart(frame)
        pan_offset = abs(status['proximity_warning_angle'] - orcas_attributes.status['aeg_pan_angle'])
        if (pan_offset > 180):
            pan_offset = pan_offset - 180
        if (pan_offset < orcas_radar.status['sectors_angle']):
            orcas_attributes.cmds['enable_searchlight'] = 1
            status['state'] = 'searching'
            timer1, timer2, timer3 = 0, 0, 0
    elif (status['state'] == 'searching'):                
        timer1 = timer1 + 1
        if (timer1 >= status['searching_period'] // orcas_attributes.status['frame_receiving_period']):
            timer1 = 0
            frame, target_coor_x, target_coor_y = search_target(frame)                       
            if ((target_coor_x >= 0) and (target_coor_y >= 0)):               
                timer2 = 0
                if (status['saved_fire_mode'] != 'SAFE'):
                    orcas_attributes.status['fire_mode'] = status['saved_fire_mode'] 
                    orcas_attributes.cmds['disable_safety'] = 1
                orcas_attributes.cmds['touch_coor']['x'] = target_coor_x
                orcas_attributes.cmds['touch_coor']['y'] = target_coor_y         
                orcas_attributes.cmds['clear_coor'] = 1 
                if ((abs(target_coor_x - (orcas_attributes.status['aim_icon_coor']['x'] - orcas_attributes.status['displayed_frame_origin_coor']['x'])) < orcas_attributes.status['aim_icon_size']) and 
                    (abs(target_coor_y - (orcas_attributes.status['aim_icon_coor']['y'] - orcas_attributes.status['displayed_frame_origin_coor']['y'])) < orcas_attributes.status['aim_icon_size'])):
                    if (timer3 == status['firing_period'] // status['searching_period']):
                        orcas_attributes.cmds['direct_fire'] = 1
                        timer3 = 0
            else:                
                timer2 = timer2 + 1
                if (timer2 > status['searching_timeout'] // status['searching_period']):
                    status['state'] = 'tilt_homing'
                    orcas_attributes.status['fire_mode'] = 'SAFE' 
                    orcas_attributes.cmds['enable_safety'] = 1   
                    orcas_attributes.cmds['tilt_angle'] = -orcas_attributes.status['aeg_tilt_angle']   
                    orcas_attributes.cmds['disable_searchlight'] = 1
            
            if (timer3 < status['firing_period'] // status['searching_period']):
                timer3 = timer3 + 1            
    elif (status['state'] == 'analyzing'):
        frame = analyze_target(frame)
        #search_target(frame)
        
    return frame
    
    