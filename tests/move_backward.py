from pymavlink import mavutil
from ..src.vehicle import Vehicle
from ..src.proximity.ultrasonic import Ultrasonic

Vehicle.back_edge = Ultrasonic(7,8) #Change

the_connection = mavutil.mavlink_connection('127.0.0.1:14550')

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

mode = 'GUIDED'

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

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                         the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b11011100111), 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0))

while 1:
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                         the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b11011100111), 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0))
    if Vehicle.back_edge.check_drive_ok() == False:
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                the_connection.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        print('Edge detected')
        break
    msg = the_connection.recv_match(
        type='LOCAL_POSITION_NED', blocking=True)
    print(msg)
