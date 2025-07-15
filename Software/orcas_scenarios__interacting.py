import math
import cv2
import orcas_attributes
import orcas_radar

def init():
    if (orcas_attributes.status['fire_mode'] == 'SAFE'):
        fire_mode_button_img = "FireMode_Safe.bmp"
    elif (orcas_attributes.status['fire_mode'] == 'SEMI'):
        fire_mode_button_img = "FireMode_Semi.bmp"
    elif (orcas_attributes.status['fire_mode'] == 'BURST'):
        fire_mode_button_img = "FireMode_Burst.bmp"
    elif (orcas_attributes.status['fire_mode'] == 'AUTO'):
        fire_mode_button_img = "FireMode_Auto.bmp"        
        
    if(orcas_attributes.status['camera_zoom_level'] == 1): 
        zoom_button_img = "Zoom_x1.bmp"
    elif(orcas_attributes.status['camera_zoom_level'] == 2): 
        zoom_button_img = "Zoom_x2.bmp"

    orcas_attributes.cmds['disable_searchlight'] = 1
    searchlight_button_img = "Searchlight_disable.bmp"    
        
    orcas_attributes.cmds['disable_radar'] = 1
    radar_button_img = "Radar_disable.bmp"    
        
    return fire_mode_button_img, zoom_button_img, searchlight_button_img, radar_button_img
    
def de_init():
    orcas_attributes.cmds['disable_searchlight'] = 1    
    orcas_attributes.cmds['disable_radar'] = 1
  
def attach_info(frame):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color = (0, 255, 0) 
    thickness = 2  
    
    aim_icon_coor_x = orcas_attributes.status['aim_icon_coor']['x'] - orcas_attributes.status['displayed_frame_origin_coor']['x']
    aim_icon_coor_y = orcas_attributes.status['aim_icon_coor']['y'] - orcas_attributes.status['displayed_frame_origin_coor']['y']
    
    text = f"{orcas_attributes.status['aeg_pan_angle']}"
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
    text_x_coor = aim_icon_coor_x - text_size[0] // 2
    text_y_coor = text_size[1] + 10               
    cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness)                

    text = f"{orcas_attributes.status['aeg_tilt_angle']}"
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
    text_x_coor = orcas_attributes.status['displayed_frame_size']['width'] - text_size[0] - 10  
    text_y_coor = aim_icon_coor_y + text_size[1] // 2 
    cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness)    
    
    cv2.rectangle(frame,
                  (aim_icon_coor_x - orcas_attributes.status['static_icon_size'] // 2, aim_icon_coor_y - orcas_attributes.status['static_icon_size'] // 2), 
                  (aim_icon_coor_x + orcas_attributes.status['static_icon_size'] // 2, aim_icon_coor_y + orcas_attributes.status['static_icon_size'] // 2), 
                  color, 
                  1)                  

    cv2.rectangle(frame,
                  (aim_icon_coor_x - orcas_attributes.status['aim_icon_size'] // 2, aim_icon_coor_y - orcas_attributes.status['aim_icon_size'] // 2), 
                  (aim_icon_coor_x + orcas_attributes.status['aim_icon_size'] // 2, aim_icon_coor_y + orcas_attributes.status['aim_icon_size'] // 2), 
                  color, 
                  1)            

    text = f"{orcas_attributes.status['aiming_distance']}"            
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
    text_x_coor = aim_icon_coor_x - text_size[0] // 2
    text_y_coor = aim_icon_coor_y + orcas_attributes.status['aim_icon_size'] // 2 + text_size[1] + 10
    cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness)                   

    if (orcas_radar.status['en'] == 1):  
        orcas_radar.append_radar_chart(frame)
    
    