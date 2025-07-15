import math
import cv2
import orcas_attributes
import orcas_camera
import orcas_radar

status = {'state': 'horizontal', # 'horizontal', 'tilt_up', 'tilt_down'
          'switch_state_request': 0,
          'tilt_up_angle': 39,
          'tilt_down_angle': -14,
          'tilt_adjust_period': 1.0,
          'saved_fire_mode': 'SAFE'}        
timer = 0          

def init():
    global timer

    orcas_attributes.cmds['pan_angle'] = -orcas_attributes.status['aeg_pan_angle']
    orcas_attributes.cmds['tilt_angle'] = -orcas_attributes.status['aeg_tilt_angle']
    
    status['saved_fire_mode'] = orcas_attributes.status['fire_mode']
    if(orcas_attributes.status['fire_mode'] != 'SEMI'):        
        orcas_attributes.status['fire_mode'] = 'SEMI'
        orcas_attributes.cmds['disable_safety'] = 1    
    fire_mode_button_img = "FireMode_Semi.bmp"
    
    if(orcas_attributes.status['camera_zoom_level'] != 2):    
        orcas_camera.switch_cam_zoom_level(2)
    zoom_button_img = "Zoom_x2.bmp"

    orcas_attributes.cmds['disable_searchlight'] = 1
    searchlight_button_img = "Searchlight_disable.bmp"    
        
    orcas_attributes.cmds['disable_radar'] = 1
    radar_button_img = "Radar_disable.bmp"
        
    status['state'] = 'horizontal'
    status['switch_state_request'] = 0    
    timer = 0
    return fire_mode_button_img, zoom_button_img, searchlight_button_img, radar_button_img
    
def de_init():   
    orcas_attributes.cmds['pan_angle'] = -orcas_attributes.status['aeg_pan_angle']
    orcas_attributes.cmds['tilt_angle'] = -orcas_attributes.status['aeg_tilt_angle']

    if (status['saved_fire_mode'] != 'SEMI'):
        orcas_attributes.status['fire_mode'] = status['saved_fire_mode']
        if (orcas_attributes.status['fire_mode'] == 'SAFE'):
            orcas_attributes.cmds['enable_safety'] = 1
    orcas_camera.switch_cam_zoom_level(1)

def interpolate(x, x1, y1, x2, y2):
	return y1 + (x - x1) * (y2 - y1) / (x2 - x1) 

def get_camera_tilt_angle(parm_A, aeg_tilt_angle, coor_offset):
    parm_C = interpolate(coor_offset, 
                         0, 0, 
                         orcas_attributes.status['received_frame_size']['height'] // 2, 
                         parm_A * math.tan(math.radians(orcas_attributes.calibration['camera_fov']['height'] / 2)))      
    angle = math.degrees(math.asin( math.sin(math.radians(aeg_tilt_angle)) + (orcas_attributes.calibration['parm_E'] + orcas_attributes.calibration['parm_D'] * math.tan(math.radians(aeg_tilt_angle)) - parm_C) / parm_A))    
    return angle

def get_camera_pan_angle(parm_A, coor_offset):
    parm_C = interpolate(coor_offset, 
                         0, 0, 
                         orcas_attributes.status['received_frame_size']['width'] // 2, 
                         parm_A * math.tan(math.radians(orcas_attributes.calibration['camera_fov']['width'] / 2)))      
    angle = math.degrees(math.atan((orcas_attributes.calibration['parm_B'] - parm_C) / (parm_A - orcas_attributes.calibration['parm_D'])))
    return angle
    
def attach_info(frame):
    global timer
    
    timer = timer + 1
    if (timer == (status['tilt_adjust_period'] // orcas_attributes.status['frame_receiving_period'])):
        timer = 0
        status['tilt_adjust_period'] = 0.6
        if (status['state'] == 'horizontal'):
            orcas_attributes.cmds['pan_angle'] = -orcas_attributes.status['aeg_pan_angle']
            if (abs(-orcas_attributes.status['aeg_tilt_angle']) > 0.3):
                orcas_attributes.cmds['tilt_angle'] = -orcas_attributes.status['aeg_tilt_angle']            
        elif (status['state'] == 'tilt_up'):
            orcas_attributes.cmds['pan_angle'] = -orcas_attributes.status['aeg_pan_angle']
            if (abs(status['tilt_up_angle'] - orcas_attributes.status['aeg_tilt_angle']) > 0.3):
                orcas_attributes.cmds['tilt_angle'] = status['tilt_up_angle'] - orcas_attributes.status['aeg_tilt_angle']              
        elif (status['state'] == 'tilt_down'):
            orcas_attributes.cmds['pan_angle'] = -orcas_attributes.status['aeg_pan_angle']
            if (abs(status['tilt_down_angle'] - orcas_attributes.status['aeg_tilt_angle']) > 0.3):
                orcas_attributes.cmds['tilt_angle'] = status['tilt_down_angle'] - orcas_attributes.status['aeg_tilt_angle']
    else:
        if (status['switch_state_request'] == 1):
            status['switch_state_request'] = 0
            if (status['state'] == 'horizontal'):
                status['state'] = 'tilt_up'
                orcas_attributes.cmds['pan_angle'] = -orcas_attributes.status['aeg_pan_angle']
                orcas_attributes.cmds['tilt_angle'] = status['tilt_up_angle'] - orcas_attributes.status['aeg_tilt_angle']                
            elif (status['state'] == 'tilt_up'):
                status['state'] = 'tilt_down'
                orcas_attributes.cmds['pan_angle'] = -orcas_attributes.status['aeg_pan_angle']
                orcas_attributes.cmds['tilt_angle'] = status['tilt_down_angle'] - orcas_attributes.status['aeg_tilt_angle']                
            elif (status['state'] == 'tilt_down'):
                status['state'] = 'horizontal'
                orcas_attributes.cmds['pan_angle'] = -orcas_attributes.status['aeg_pan_angle']
                orcas_attributes.cmds['tilt_angle'] = -orcas_attributes.status['aeg_tilt_angle']
            timer = 0
            status['tilt_adjust_period'] = 1.0
                               
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color = (0, 255, 0) 
    thickness = 2  

    text = f"{orcas_attributes.status['aeg_tilt_angle']}"
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
    text_x_coor = frame.shape[1] - text_size[0] - 10  
    text_y_coor = frame.shape[0] // 2 + text_size[1] // 2 
    cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness)                      

    auxiliary_line_start = (10, frame.shape[0] // 2)
    auxiliary_line_end = ((frame.shape[1] // 2), frame.shape[0] // 2)
    cv2.line(frame, auxiliary_line_start, auxiliary_line_end, color, 1)
    auxiliary_line_start = (frame.shape[1] // 2, frame.shape[0] // 4)
    auxiliary_line_end = ((frame.shape[1] // 2), frame.shape[0] // 4 * 3)
    cv2.line(frame, auxiliary_line_start, auxiliary_line_end, color, 1)            
        
    if (orcas_attributes.status['aiming_distance'] != 0):    
        font_scale = 0.5
        thickness = 1                         
        for i in range (0, frame.shape[0] // 4, 20): 
            auxiliary_line_start = (frame.shape[1] // 2 - 10, frame.shape[0] // 2 + i)
            auxiliary_line_end = ((frame.shape[1] // 2 + 10), frame.shape[0] // 2 + i)
            cv2.line(frame, auxiliary_line_start, auxiliary_line_end, color, thickness)
            
            angle = get_camera_tilt_angle(orcas_attributes.status['aiming_distance'], 
                                          orcas_attributes.status['aeg_tilt_angle'], 
                                          -i)
            text = f"{angle:.2f}"
            text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
            text_x_coor = auxiliary_line_end[0] + 10
            text_y_coor = auxiliary_line_end[1] + text_size[1] // 2 
            cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness) 
            
            if (i > 0):
                j = -i
                auxiliary_line_start = (frame.shape[1] // 2, frame.shape[0] // 2 + j)
                auxiliary_line_end = ((frame.shape[1] // 2 + 10), frame.shape[0] // 2 + j)
                cv2.line(frame, auxiliary_line_start, auxiliary_line_end, color, thickness)
                
                angle = get_camera_tilt_angle(orcas_attributes.status['aiming_distance'], orcas_attributes.status['aeg_tilt_angle'], -j)
                text = f"{angle:.2f}"
                text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
                text_x_coor = auxiliary_line_end[0] + 10
                text_y_coor = auxiliary_line_end[1] + text_size[1] // 2 
                cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness) 

        for i in range (30, (frame.shape[1] // 2 - 10), 30):
            angle = get_camera_pan_angle(orcas_attributes.status['aiming_distance'], -i)
            text = f"{angle:.2f}"
            text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
            if ((i / 30) % 2 == 0):
                auxiliary_line_start = (frame.shape[1] // 2 - i, frame.shape[0] // 2)
                auxiliary_line_end = (frame.shape[1] // 2 - i, frame.shape[0] // 2 - 50)                     
                cv2.line(frame, auxiliary_line_start, auxiliary_line_end, color, thickness)
                text_x_coor = auxiliary_line_end[0] - text_size[0] // 2                                          
                text_y_coor = auxiliary_line_end[1] - 10
            else:
                auxiliary_line_start = (frame.shape[1] // 2 - i, frame.shape[0] // 2)
                auxiliary_line_end = (frame.shape[1] // 2 - i, frame.shape[0] // 2 + 50)                     
                cv2.line(frame, auxiliary_line_start, auxiliary_line_end, color, thickness)                    
                text_x_coor = auxiliary_line_end[0] - text_size[0] // 2                                          
                text_y_coor = auxiliary_line_end[1] + 10
            cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness)     
            
    return frame