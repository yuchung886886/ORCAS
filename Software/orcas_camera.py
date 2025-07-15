from picamera2 import Picamera2
import orcas_attributes

picam2 = Picamera2()
mode = picam2.sensor_modes[3]

def init():       
    config = picam2.create_still_configuration(main={'format': 'XRGB8888', 'size': (640, 480)}, 
                                               sensor={'output_size': mode['size'], 'bit_depth': mode['bit_depth']})                                             
    picam2.configure(config)
    picam2.start()
    
def switch_cam_zoom_level(level):    
    if (level != orcas_attributes.status['camera_zoom_level']):
        picam2.stop()        
        if (level == 1):
            config = picam2.create_still_configuration(main={'format': 'XRGB8888', 'size': (640, 480)}, 
                                                       sensor={'output_size': mode['size'], 'bit_depth': mode['bit_depth']})
            orcas_attributes.status['aim_icon_size'] = 60
        elif (level == 2):
            config = picam2.create_still_configuration(main={'format': 'XRGB8888', 'size': (1640, 1232)}, 
                                                       sensor={'output_size': mode['size'], 'bit_depth': mode['bit_depth']})           
            orcas_attributes.status['aim_icon_size'] = 90
        orcas_attributes.status['camera_zoom_level'] = level
        picam2.configure(config)
        picam2.start()      
        
def capture():
    return picam2.capture_array()