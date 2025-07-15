import cv2
import numpy as np
import math
import orcas_serial
import orcas_attributes

status = {'en': 0,
          'sectors_angle': 0,
          'sectors_count': 0,
          'ranging_offset': 3,
          'sectors_range_buf_size': 2,
          'sectors_range_buf': [],
          'sectors_range_diff': [],
          'proximity_warning_threshold': -30,
          'proximity_warning_timeout': []}

radar_chart = cv2.imread('static/RadarBackground_600.bmp')

def init():
    ret = 0

    if (ret == 0):
        ret = orcas_serial.set_radar_en(status['en'])    
    if (ret == 0):
        ret = orcas_serial.set_radar_config(status['ranging_offset'])
    for i in range(0, status['sectors_range_buf_size'], 1):
        if (ret == 0):
            ret, sectors_range = orcas_serial.get_radar_ranging()
            if (ret == 0):
                status['sectors_range_buf'].append(sectors_range)
    if (ret == 0):
        status['sectors_count'] = len(sectors_range)
        status['sectors_angle'] = 360 / status['sectors_count']
        for i in range (0, status['sectors_count'], 1):
            status['sectors_range_diff'].append(0)
            status['proximity_warning_timeout'].append(0) 
    return ret

def clear_range_buffer():
    for i in range(0, status['sectors_range_buf_size'], 1):
        for j in range(0, status['sectors_count'], 1):
            status['sectors_range_buf'][i][j] = 0
    for i in range(0, status['sectors_count'], 1):
        status['sectors_range_diff'][i] = 0
        status['proximity_warning_timeout'][i] = 0

def update_radar_chart():
    global radar_chart
    
    latest_buf_index = status['sectors_range_buf_size'] - 1
    max_distance = max(status['sectors_range_buf'][latest_buf_index])
    if (max_distance > 600):
        radar_chart = cv2.imread('static/RadarBackground_900.bmp')
        max_distance = 900
    elif (max_distance > 300):
        radar_chart = cv2.imread('static/RadarBackground_600.bmp')
        max_distance = 600
    else:
        radar_chart = cv2.imread('static/RadarBackground_300.bmp')      
        max_distance = 300
    
    radar_sectors_count = int(360 // status['sectors_angle'])
    for i in range(0, radar_sectors_count, 1):
        status['sectors_range_diff'][i] = (status['sectors_range_buf'][latest_buf_index][i % radar_sectors_count] -
                                           status['sectors_range_buf'][latest_buf_index - 1][i % radar_sectors_count])
    for i in range(0, radar_sectors_count, 1):
        if (status['sectors_range_diff'][i % radar_sectors_count] >= status['proximity_warning_threshold']):
            status['sectors_range_diff'][i % radar_sectors_count] = 0
        elif ((status['sectors_range_diff'][(i - 1) % radar_sectors_count] >= status['proximity_warning_threshold']) and 
              (status['sectors_range_diff'][(i + 1) % radar_sectors_count] >= status['proximity_warning_threshold'])):
            status['sectors_range_diff'][i % radar_sectors_count] = 0
        else:
            status['proximity_warning_timeout'][i % radar_sectors_count] = 10        
    
    pan_angle = orcas_attributes.status['aeg_pan_angle']
    if (pan_angle < 0):
        pan_angle += 360
    pan_sector_index = int(((pan_angle + status['sectors_angle'] // 2) % 360) // status['sectors_angle'])    
    
    center = (radar_chart.shape[1] // 2, radar_chart.shape[0] // 2)
    line_start = [0, 0]
    line_end = [0, 0]
    for i in range(0, radar_sectors_count + 1, 1):
        radius = (radar_chart.shape[1] // 2) * (status['sectors_range_buf'][latest_buf_index][(pan_sector_index + i) % radar_sectors_count] / max_distance) 
        line_start[0] = line_end[0]
        line_start[1] = line_end[1]
        line_end[0] = center[0] + int(radius * math.sin(math.radians(status['sectors_angle'] * (i % radar_sectors_count))))
        line_end[1] = center[1] - int(radius * math.cos(math.radians(status['sectors_angle'] * (i % radar_sectors_count))))
        if (i > 0):
            if (status['proximity_warning_timeout'][(pan_sector_index + i) % radar_sectors_count] > 0):
                status['proximity_warning_timeout'][(pan_sector_index + i) % radar_sectors_count] -= 1
                color = (0, 0, 255) # red
            else:
                color = (0, 255, 0) # green
            cv2.line(radar_chart, line_start, line_end, color, 2) 
            
def get_proximity_warning_pan_offset():       
    # Pan to the proximity warning sector when touch the radar chart
    latest_proximity_warning_sector_index = -1
    latest_proximity_warning_timeout = 0
    for i in range(0, status['sectors_count'], 1):
        if (status['proximity_warning_timeout'][i] > latest_proximity_warning_timeout):
            latest_proximity_warning_sector_index = i
            latest_proximity_warning_timeout = status['proximity_warning_timeout'][i]
    if (latest_proximity_warning_sector_index != -1):
        pan_angle = orcas_attributes.status['aeg_pan_angle']
        if (pan_angle < 0):
            pan_angle += 360
        pan_offset = latest_proximity_warning_sector_index * status['sectors_angle'] - pan_angle
        if (pan_offset > 180):
            pan_offset -= 360
        elif (pan_offset < -180):
            pan_offset += 360
    else:
        pan_offset = 0
    return latest_proximity_warning_sector_index, pan_offset

def append_radar_chart(frame):
    global radar_chart

    # Crop radar_chart to a circle
    center = (radar_chart.shape[1] // 2, radar_chart.shape[0] // 2)
    radius = min(radar_chart.shape[1] // 2, radar_chart.shape[0] // 2)
    mask = np.zeros_like(radar_chart, dtype=np.uint8)
    cv2.circle(mask, center, radius, (255, 255, 255), -1)
    cropped_circle = cv2.bitwise_and(radar_chart, mask)
    cropped_circle_gray = cv2.cvtColor(cropped_circle, cv2.COLOR_BGR2GRAY)
    _, circle_mask = cv2.threshold(cropped_circle_gray, 1, 255, cv2.THRESH_BINARY)
    cropped_circle = cv2.bitwise_and(cropped_circle, cropped_circle, mask=circle_mask)
    
    # Past the radar_chart to the frame
    y_offset = frame.shape[0] - cropped_circle.shape[0]
    x_offset = frame.shape[1] - cropped_circle.shape[1]
    y1, y2 = y_offset, y_offset + cropped_circle.shape[0]
    x1, x2 = x_offset, x_offset + cropped_circle.shape[1]
    for i in range(3):
        frame[y1:y2, x1:x2, i] = np.where(circle_mask > 0, cropped_circle[:, :, i], frame[y1:y2, x1:x2, i])                  