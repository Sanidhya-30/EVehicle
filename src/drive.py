## Main Code on how to run the vehicle

from .utils import keyboard_shutdown
from time import sleep
import math
from .vehicle import Vehicle

def driver(vehicle : Vehicle):
    print('check Vehicle status')
    Vehicle.working_status = True
    Vehicle.setup_arm()
    Vehicle.change_vehicle_mode('GUIDED')
    sleep(1)
    
    try:
        print('Driving')

    # DRIVE LOGIC
    
    except KeyboardInterrupt:
        keyboard_shutdown()

if __name__ == '__main__':
    pass