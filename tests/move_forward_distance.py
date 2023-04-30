from pymavlink import mavutil
from ..src.vehicle import Vehicle
from ..src.proximity.ultrasonic import Ultrasonic

Vehicle.back_edge = Ultrasonic(7,8)

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('127.0.0.1:14550')

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

mode = 'GUIDED'
mode_H = 'HOLD'
mode_id_H = the_connection.mode_mapping()[mode_H]

speed = 0.2
dist = 0.3
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

the_connection.mav.command_long_encode(
		0, 0,
		mavutil.mavlink.MAV_CMD_DO_SET_REVERSE,
		0,
		1,
		0,
		0,
		0,
		0,0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

system = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)

initial = system.x
current = initial
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), dist, 0, 0, speed, 0, 0, 0, 0, 0, 0, 0))

while True:
    change = abs(current - initial)
    if change >= dist:
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                         the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000111), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) 
        break
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), dist, 0, 0, speed, 0, 0, 0, 0, 0, 0, 0))
    print('moving forward')

    
    if Vehicle.front_edge.check_drive_ok() == False:
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        print('Stopped moving')
        
        #the_connection.mav.set_mode_send(
        #    the_connection.target_system,
        #    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        #    mode_id_H)
        
        break
        
    system = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    current = system.x
