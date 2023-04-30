## Vehicle Class and methods

from pymavlink import mavutil
from ..proximity.ultrasonic import Ultrasonic
import math

class Vehicle:
    def __init__(self, connection):
        vehicle = mavutil.mavlink_connection(connection)
        vehicle.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (vehicle.target_system, vehicle.target_component))

        _ = vehicle.messages.keys() #All parameters that can be fetched

        self.vehicle = vehicle
        self.working_status = False
        self.front_edge = Ultrasonic(21,20)
        self.back_edge = Ultrasonic(7,8)
        self.vehicle_status = "Free"
        self.update_vehicle()

    def change_vehicle_mode(self, mode):
        print("Changing vehicle mode to", mode)
        # Get mode ID
        mode_id = self.vehicle.mode_mapping()[mode]
        # Set new mode

        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

    def setup_arm(self):
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

        self.vehicle.mav.command_long_encode(0, 0,mavutil.mavlink.MAV_CMD_DO_SET_REVERSE,0,1,0,0,0,0,0,0)

        self.vehicle.mav.command_long_encode(0, 0,mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,0,30,0,0,0,0,0, 0)

    def move_forward(self, speed):
        self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000111), 0, 0, 0, speed, 0, 0, 0, 0, 0, 0, 0))
        if self.front_edge.check_drive_ok() == False:
                self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                return

    def move_backward(self, speed):
        self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b11011100111), 0, 0, 0, -(speed), 0, 0, 0, 0, 0, 0, 0))
        if self.back_edge.check_drive_ok() == False:
                self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                return
    
    def move_forward_dist(self, speed, dist):
        system = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)

        initial = system.x
        current = initial
        self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), dist, 0, 0, speed, 0, 0, 0, 0, 0, 0, 0))
        
        while True:
            change = abs(current - initial)
            if change >= dist:
                self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                return
            self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000111), dist, 0, 0, speed, 0, 0, 0, 0, 0, 0, 0))
            
            if self.front_edge.check_drive_ok() == False:
                self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                return
                
            system = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            current = system.x
            
    def move_backward_dist(self, speed, dist):
        system = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)

        initial = system.x
        current = initial
        self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b11011100111), 0, 0, 0, -(speed), 0, 0, 0, 0, 0, 0, 0))
        
        while True:
            change = abs(current - initial)
            if change >= dist:
                self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                return
            self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b11011100111), 0, 0, 0, -(speed), 0, 0, 0, 0, 0, 0, 0))

            if self.back_edge.check_drive_ok() == False:
                self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, int(0b110111000110), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
                return

            system = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            current = system.x
        
    def current_yaw(self):
        system = self.vehicle.recv_match(type='ATTITUDE', blocking=True)
        angle = math.degrees(system.yaw)
        return angle

    def change_yaw(self, angle, speed=0):
        #print(angle) # Correct angle by adding abs difference in 180 degrees
        system = self.vehicle.recv_match(type='ATTITUDE', blocking=True)
        initial = math.degrees(system.yaw)
        final = initial + math.degrees(angle)
        current = initial
        change = 0
        # self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
        #                 self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED , int(0b100111100111), 0, 0, 0, (speed), 0, 0, 0, 0, 0, angle, 0))
        
        while True:
            self.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.vehicle.target_system,
                        self.vehicle.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED , int(0b100111100111), 0, 0, 0, (speed), 0, 0, 0, 0, 0, (angle), 0))
            
            system = self.vehicle.recv_match(type='ATTITUDE', blocking=True)
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
            
            if change >= (angle):
                break
            
            print('initial head', initial)
            print('current head', current)
            print('change head', change)
   
    def update_vehicle(self):
        pos = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        system = self.vehicle.recv_match(type='SYS_STATUS', blocking=True)
        self.battery = system.battery_remaining
        self.alt = pos.relative_alt * 10e-3
        self.lat = pos.lat * 10e-8
        self.lon = pos.lon * 10e-8
            
if __name__== "__main__":
    pass