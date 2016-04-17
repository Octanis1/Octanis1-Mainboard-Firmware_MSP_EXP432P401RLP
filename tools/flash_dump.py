#! /usr/bin/env pyhton3
import serial
import serial_datagram
import argparse
import time
import struct

import progressbar
import sys

def parse_commandline_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port',
                        help='Serial port.',
                        metavar='DEVICE', required=True)
    parser.add_argument('-b', '--baudrate',
                        help='Serial baudrate.',
                        metavar='BAUDRATE',
                        type=int,
                        default=38400)
    parser.add_argument('-o', '--outfile',
                        help='Output file.',
                        metavar='FILE',
                        default='flash.bin')
    parser.add_argument('-s', '--size',
                        help='Output file.',
                        metavar='SIZE',
                        type=int,
                        default=0x1000000) # 16M
    return parser.parse_args()

def get_datagram(dev):
    while True:
        try:
            return serial_datagram.read(dev)
        except (serial_datagram.CRCMismatchError, serial_datagram.FrameError):
            print("CRC error")
            # sys.exit(1)

def main():
    args = parse_commandline_args()
    dev = serial.Serial(args.port, baudrate=args.baudrate, timeout=5)

    print("send flash dump command")
    start = 0
    end = start + args.size
    cmd = 'flash_dump'+' {:x} {:x}'.format(start, end)+'\r'
    cmd = bytes(cmd.encode('ascii'))
    dev.write(cmd)
    # dev.flush()
    time.sleep(0.1)
    dev.reset_input_buffer()
    # while True:
    #     c = dev.read()
    #     if c == b'' or c == b'\n':
    #         break

    print("start downloading flash...")
    pbar = progressbar.ProgressBar(maxval=args.size).start()

    flash = bytes()
    while True:
        d = get_datagram(dev)
        if d is None:
            break
        addr = struct.unpack('<I', d[:4])[0]
        if addr != len(flash):
            print("missing data")
            # sys.exit(1)
            break

        d = d[4:]
        flash+=d

        if addr + len(d) >= args.size:
            break
        else:
            pbar.update(addr)


    if len(flash) > args.size:
        flash = flash[0:args.size]

    if len(flash) == args.size:
        pbar.finish()

    print("write {} bytes to file: {}".format(len(flash), args.outfile))
    with open(args.outfile, 'wb') as f:
        f.write(flash)

    dev.close()


if __name__ == '__main__':
    main()
