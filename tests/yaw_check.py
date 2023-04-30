from pymavlink import mavutil
import time
import math
# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('127.0.0.1:14550')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

mode = 'GUIDED'
speed = 0
angle = 1.57
# Get mode ID
mode_id = the_connection.mode_mapping()[mode]
# Set new mode
print(the_connection)
the_connection.mav.set_mode_send(
    the_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

system = the_connection.recv_match(type='ATTITUDE', blocking=True)
initial = math.degrees(system.yaw)
final = initial + math.degrees(angle)
current = initial
        # the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
        #                 the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED , int(0b100111100111), 0, 0, 0, (speed), 0, 0, 0, 0, 0, angle, 0))
        
while True:
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED , int(0b100111100111), 0, 0, 0, (speed), 0, 0, 0, 0, 0, (angle/6), 0))

    system = the_connection.recv_match(type='ATTITUDE', blocking=True)
    current = math.degrees(system.yaw)
            
    if final > 180:
        if current > 0:
            change = current - initial
        else: 
            neg_change = 180 + current
            change += neg_change
    elif final < -180:
        if current < 0:
            change = initial - current
        else:
            neg_change = 180 - current
            change += neg_change
    else:
        change = abs(current - initial)
                
    #if change >= 90:
    if change >= math.degrees(angle):    
        #the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
        #    the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED , int(0b100111100111), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        break
            
    print('initial head', initial)
    print('current head', current)
    print('change head', change)