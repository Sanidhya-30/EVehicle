## init Vehicle Class and launch Drive

from .vehicle import Vehicle
from .drive import drive

def main_start(serial=None, connection=None):
    if serial != None:
        print(serial)
        vehicle = Vehicle(connection=connection)
        drive(vehicle=vehicle)

if __name__ == '__main__':
    pass
else:
    main_start()