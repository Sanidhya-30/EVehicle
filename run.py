## Main Run File to init all

import argparse
import src.settings as settings
import src.start as start
import src.utils as utils

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='127.0.0.1:14550')
    args = parser.parse_args()
    print ('Connecting to vehicle: ')
    start.main_start(connection=args.connect)


if __name__ == '__main__':
    main()