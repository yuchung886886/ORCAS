import os
import time
import threading
from flask import Flask, render_template, request, jsonify, Response
import cv2
import math
import orcas_serial
import orcas_camera
import orcas_attributes
import orcas_scenarios__interacting
import orcas_scenarios__calibrating
import orcas_scenarios__tracking
import orcas_radar
     
def interpolate(x, x1, y1, x2, y2):
	return y1 + (x - x1) * (y2 - y1) / (x2 - x1)     

def get_aim_icon_coor():
    if ((orcas_attributes.status['scenario'] != 'calibrating') and 
        (orcas_attributes.status['aiming_distance'] != 0) and 
        (orcas_attributes.status['received_frame_size']['width'] != 0) and 
        (orcas_attributes.status['received_frame_size']['height'] != 0)):     

        max_opposite_length_camera = orcas_attributes.status['aiming_distance'] * math.tan(math.radians(orcas_attributes.calibration['camera_fov']['width'] / 2))        
        opposite_length_camera = (orcas_attributes.status['aiming_distance'] - orcas_attributes.calibration['parm_D']) * math.tan(math.radians(orcas_attributes.calibration['camera_pan_angle']))        
        coor_offset = interpolate(orcas_attributes.calibration['parm_B'] - opposite_length_camera,
                                  0, 0,
                                  max_opposite_length_camera, orcas_attributes.status['received_frame_size']['width'] // 2)
        orcas_attributes.status['aim_icon_coor']['x'] = int(orcas_attributes.status['received_frame_size']['width'] // 2 + coor_offset)                

        camera_tilt_angle = interpolate(orcas_attributes.status['aeg_tilt_angle'], 
                                        orcas_attributes.calibration['camera_tilt_angle'][0][0], orcas_attributes.calibration['camera_tilt_angle'][0][1],
                                        orcas_attributes.calibration['camera_tilt_angle'][1][0], orcas_attributes.calibration['camera_tilt_angle'][1][1])                
        neighbor_length_camera = orcas_attributes.status['aiming_distance'] * math.cos(math.radians(camera_tilt_angle))
        neighbor_length_aeg = neighbor_length_camera + orcas_attributes.calibration['parm_D']
        opposite_length_aeg = neighbor_length_aeg * math.tan(math.radians(orcas_attributes.status['aeg_tilt_angle']))
        coor_offset_in_dist = (opposite_length_aeg + orcas_attributes.calibration['parm_E']) - (orcas_attributes.status['aiming_distance'] * math.sin(math.radians(camera_tilt_angle)))              
        max_opposite_length_camera = orcas_attributes.status['aiming_distance'] * math.tan(math.radians(orcas_attributes.calibration['camera_fov']['height'] / 2))
        coor_offset = interpolate(coor_offset_in_dist,
                                  0, 0,
                                  max_opposite_length_camera, orcas_attributes.status['received_frame_size']['height'] // 2)
        orcas_attributes.status['aim_icon_coor']['y'] = int(orcas_attributes.status['received_frame_size']['height'] // 2 - coor_offset)        
    else:
        orcas_attributes.status['aim_icon_coor']['x'] = orcas_attributes.status['received_frame_size']['width'] // 2
        orcas_attributes.status['aim_icon_coor']['y'] = orcas_attributes.status['received_frame_size']['height'] // 2       
    
def generate_frames():
    while True:
        frame = orcas_camera.capture()
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)       
        if ((orcas_attributes.status['received_frame_size']['width'] != frame.shape[1]) or 
            (orcas_attributes.status['received_frame_size']['height'] != frame.shape[0])):
            orcas_attributes.status['received_frame_size']['width'] = frame.shape[1]
            orcas_attributes.status['received_frame_size']['height'] = frame.shape[0]   
            orcas_attributes.status['aim_icon_coor_update_req'] = 1            
        if (orcas_attributes.status['aim_icon_coor_update_req'] == 1):
            get_aim_icon_coor()
            orcas_attributes.status['aim_icon_coor_update_req'] = 0

        # Crop the frame to 'displayed_frame_size' centered at the aim icon               
        if ((orcas_attributes.status['received_frame_size']['width'] != orcas_attributes.status['displayed_frame_size']['width']) or
            (orcas_attributes.status['received_frame_size']['height'] != orcas_attributes.status['displayed_frame_size']['height'])):
            orcas_attributes.status['displayed_frame_origin_coor']['x'] = orcas_attributes.status['aim_icon_coor']['x'] - (orcas_attributes.status['displayed_frame_size']['width'] // 2)
            if (orcas_attributes.status['displayed_frame_origin_coor']['x'] < 0):
                orcas_attributes.status['displayed_frame_origin_coor']['x'] = 0
            orcas_attributes.status['displayed_frame_origin_coor']['y'] = orcas_attributes.status['aim_icon_coor']['y'] - (orcas_attributes.status['displayed_frame_size']['height'] // 2)
            if (orcas_attributes.status['displayed_frame_origin_coor']['y'] < 0):
                orcas_attributes.status['displayed_frame_origin_coor']['y'] = 0
            frame = frame[orcas_attributes.status['displayed_frame_origin_coor']['y'] : orcas_attributes.status['displayed_frame_origin_coor']['y'] + orcas_attributes.status['displayed_frame_size']['height'],            
                          orcas_attributes.status['displayed_frame_origin_coor']['x'] : orcas_attributes.status['displayed_frame_origin_coor']['x'] + orcas_attributes.status['displayed_frame_size']['width']]
        else:
            orcas_attributes.status['displayed_frame_origin_coor']['x'] = 0
            orcas_attributes.status['displayed_frame_origin_coor']['y'] = 0            
        
        # Apply scenario interface
        if (orcas_attributes.status['scenario'] == 'interacting'):     
            orcas_scenarios__interacting.attach_info(frame)
        elif (orcas_attributes.status['scenario'] == 'calibrating'):
            orcas_scenarios__calibrating.attach_info(frame)
        elif (orcas_attributes.status['scenario'] == 'tracking'):
            orcas_scenarios__tracking.attach_info(frame)
            
        _, jpeg = cv2.imencode('.jpg', frame)
        frame_data = jpeg.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')        
               
        time.sleep(orcas_attributes.status['frame_receiving_period'])    
        
web_server = Flask(__name__)        
@web_server.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@web_server.route('/update_touch', methods=['POST'])
def update_touch():
    data = request.get_json()
    if ((orcas_attributes.status['received_frame_size']['width'] != 0) and 
        (orcas_attributes.status['received_frame_size']['height'] != 0)):
        if (orcas_attributes.status['scenario'] == 'calibrating'):
            orcas_scenarios__calibrating.status['switch_state_request'] = 1
        else:
            orcas_attributes.cmds['touch_coor']['x'] = interpolate(data.get('x', 0),
                                                                   0, 0,
                                                                   data.get('w', 0), orcas_attributes.status['displayed_frame_size']['width']) 
            orcas_attributes.cmds['touch_coor']['y'] = interpolate(data.get('y', 0),
                                                                   0, 0,
                                                                   data.get('h', 0), orcas_attributes.status['displayed_frame_size']['height'])    
    return jsonify({'status': 'success'})
    
@web_server.route('/clear_touch', methods=['POST'])
def clear_touch():    
    orcas_attributes.cmds['clear_coor'] = 1   
    return jsonify({'status': 'success'})

@web_server.route('/triger_homing', methods=['POST'])
def homing_route():
    orcas_attributes.cmds['pan_angle'] = -orcas_attributes.status['aeg_pan_angle']
    orcas_attributes.cmds['tilt_angle'] = -orcas_attributes.status['aeg_tilt_angle']
    return jsonify({'status': 'success'})  

@web_server.route('/switch_scenario', methods=['POST'])
def switch_scenario_route():
    orcas_attributes.status['aim_icon_coor_update_req'] = 1
    if (orcas_attributes.status['scenario'] == 'interacting'):
        orcas_attributes.status['scenario'] = 'calibrating'
        orcas_scenarios__interacting.de_init()
        fire_mode_button_img, zoom_button_img, searchlight_button_img, radar_button_img = orcas_scenarios__calibrating.init()
        return jsonify({'status': 'success', 
                        'scenario_button_img': "Scenario_calibrating.bmp",
                        'fire_mode_button_img': fire_mode_button_img,
                        'zoom_button_img': zoom_button_img,
                        'searchlight_button_img': searchlight_button_img,
                        'radar_button_img': radar_button_img})
    elif (orcas_attributes.status['scenario'] == 'calibrating'):
        orcas_scenarios__calibrating.de_init()
        orcas_attributes.status['scenario'] = 'tracking'
        fire_mode_button_img, zoom_button_img, searchlight_button_img, radar_button_img = orcas_scenarios__tracking.init()
        return jsonify({'status': 'success', 
                        'scenario_button_img': "Scenario_tracking.bmp",
                        'fire_mode_button_img': fire_mode_button_img,
                        'zoom_button_img': zoom_button_img,
                        'searchlight_button_img': searchlight_button_img,
                        'radar_button_img': radar_button_img})
    elif (orcas_attributes.status['scenario'] == 'tracking'):
        orcas_scenarios__tracking.de_init()
        orcas_attributes.status['scenario'] = 'interacting'
        fire_mode_button_img, zoom_button_img, searchlight_button_img, radar_button_img = orcas_scenarios__interacting.init()
        return jsonify({'status': 'success', 
                        'scenario_button_img': "Scenario_interacting.bmp",
                        'fire_mode_button_img': fire_mode_button_img,
                        'zoom_button_img': zoom_button_img,
                        'searchlight_button_img': searchlight_button_img,
                        'radar_button_img': radar_button_img})

@web_server.route('/switch_fire_mode', methods=['POST'])
def switch_fire_mode_route():
    if (orcas_attributes.status['scenario'] == 'interacting'):
        if (orcas_attributes.status['fire_mode'] == 'SAFE'):
            orcas_attributes.status['fire_mode'] = 'SEMI'
            orcas_attributes.cmds['disable_safety'] = 1
            return jsonify({'status': 'success', 'fire_mode_button_img': "FireMode_Semi.bmp"})                
        elif (orcas_attributes.status['fire_mode'] == 'SEMI'):
            orcas_attributes.status['fire_mode'] = 'BURST'
            orcas_attributes.cmds['disable_safety'] = 1
            return jsonify({'status': 'success', 'fire_mode_button_img': "FireMode_Burst.bmp"})        
        elif (orcas_attributes.status['fire_mode'] == 'BURST'):
            orcas_attributes.status['fire_mode'] = 'AUTO'
            orcas_attributes.cmds['disable_safety'] = 1
            return jsonify({'status': 'success', 'fire_mode_button_img': "FireMode_Auto.bmp"})        
        elif (orcas_attributes.status['fire_mode'] == 'AUTO'):
            orcas_attributes.status['fire_mode'] = 'SAFE'
            orcas_attributes.cmds['enable_safety'] = 1
            return jsonify({'status': 'success', 'fire_mode_button_img': "FireMode_Safe.bmp"})
    else:
        if (orcas_attributes.status['fire_mode'] == 'SAFE'):  
            return jsonify({'status': 'success', 'fire_mode_button_img': "FireMode_Safe.bmp"})
        elif (orcas_attributes.status['fire_mode'] == 'SEMI'):
            return jsonify({'status': 'success', 'fire_mode_button_img': "FireMode_Semi.bmp"})  
        elif (orcas_attributes.status['fire_mode'] == 'BURST'):
            return jsonify({'status': 'success', 'fire_mode_button_img': "FireMode_Burst.bmp"})  
        elif (orcas_attributes.status['fire_mode'] == 'AUTO'):
            return jsonify({'status': 'success', 'fire_mode_button_img': "FireMode_Auto.bmp"})
            
@web_server.route('/toggle_loading', methods=['POST'])  
def toggle_loading_route():
    if (orcas_attributes.status['ammo_unloading_en'] == 1):
        return jsonify({'status': 'fail', 'load_button_img': "Load_disable.bmp"})
    else:        
        if (orcas_attributes.status['ammo_loading_en'] == 1):
            orcas_attributes.status['ammo_loading_en'] = 0
            return jsonify({'status': 'success', 'load_button_img': "Load_disable.bmp"})
        else:        
            orcas_attributes.status['ammo_loading_en'] = 1
            return jsonify({'status': 'success', 'load_button_img': "Load_enable.bmp"})
            
@web_server.route('/toggle_unloading', methods=['POST'])  
def toggle_unloading_route():
    if (orcas_attributes.status['ammo_loading_en'] == 1):
        return jsonify({'status': 'fail', 'unload_button_img': "Unload_disable.bmp"})
    else:        
        if (orcas_attributes.status['ammo_unloading_en'] == 1):
            orcas_attributes.status['ammo_unloading_en'] = 0
            return jsonify({'status': 'success', 'unload_button_img': "Unload_disable.bmp"})
        else:        
            orcas_attributes.status['ammo_unloading_en'] = 1
            return jsonify({'status': 'success', 'unload_button_img': "Unload_enable.bmp"})                          
        
@web_server.route('/switch_zoom_level', methods=['POST'])             
def switch_zoom_route():         
    if (orcas_attributes.status['camera_zoom_level'] == 1):
        orcas_camera.switch_cam_zoom_level(2)
        return jsonify({'status': 'success', 'zoom_button_img': "Zoom_x2.bmp"})   
    elif (orcas_attributes.status['camera_zoom_level'] == 2):
        orcas_camera.switch_cam_zoom_level(1)
        return jsonify({'status': 'success', 'zoom_button_img': "Zoom_x1.bmp"})

@web_server.route('/toggle_searchlight', methods=['POST'])  
def toggle_searchlight_route():
    if (orcas_attributes.status['searchlight_en'] == 1):
        orcas_attributes.cmds['disable_searchlight'] = 1
        return jsonify({'status': 'success', 'searchlight_button_img': "Searchlight_disable.bmp"})
    else:        
        orcas_attributes.cmds['enable_searchlight'] = 1
        return jsonify({'status': 'success', 'searchlight_button_img': "Searchlight_enable.bmp"})

@web_server.route('/toggle_radaar', methods=['POST'])  
def toggle_radaar_route():
    if (orcas_radar.status['en'] == 1):
        orcas_attributes.cmds['disable_radar'] = 1
        return jsonify({'status': 'success', 'radar_button_img': "Radar_disable.bmp"})
    else:        
        orcas_attributes.cmds['enable_radar'] = 1
        return jsonify({'status': 'success', 'radar_button_img': "Radar_enable.bmp"})
        
@web_server.route('/press_f1', methods=['POST'])
def press_f1_route():
    if (orcas_attributes.status['scenario'] == 'interacting'):
        orcas_attributes.cmds['direct_fire'] = 1
    elif (orcas_attributes.status['scenario'] == 'tracking'):
        orcas_scenarios__tracking.toggle_analysis()
    return jsonify({'status': 'success'})         
 
@web_server.route('/release_f1', methods=['POST'])
def release_f1_route():
    return jsonify({'status': 'success'})  
 
@web_server.route('/')
def index():
    scenario_button_img_file = "Scenario_interacting.bmp"
    if (orcas_attributes.status['fire_mode'] == "SAFE"):
        fire_mode_button_img_file = "FireMode_Safe.bmp"
    else:
        fire_mode_button_img_file = "FireMode_Semi.bmp"
    if (orcas_attributes.status['searchlight_en'] == 1):
        searchlight_button_img_file = "Searchlight_enable.bmp"
    else:
        searchlight_button_img_file = "Searchlight_disable.bmp"
    if (orcas_radar.status['en'] == 1):
        radar_button_img_file = "Radar_enable.bmp"
    else:
        radar_button_img_file = "Radar_disable.bmp"        
    return render_template('index.html', 
                           scenario_button_img = scenario_button_img_file,
                           fire_mode_button_img = fire_mode_button_img_file,
                           searchlight_button_img = searchlight_button_img_file,
                           radar_button_img = radar_button_img_file)

if (orcas_serial.init() != 0):
    os._exit(0)
if (orcas_attributes.init() != 0):
    os._exit(0)
orcas_serial_comm_thread = threading.Thread(target=orcas_serial.communicate, daemon=True)
orcas_serial_comm_thread.start()    

orcas_camera.init()

if __name__ == '__main__':
    web_server.run(host='0.0.0.0', port=5000)
