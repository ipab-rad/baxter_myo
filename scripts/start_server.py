#!/usr/bin/python

from baxter_myo.server import SocketListener

def main():
    s = SocketListener('', 50007, 'myo_data_low', 'myo_data_high')
    s.loop()

if __name__ == "__main__":
    main()
