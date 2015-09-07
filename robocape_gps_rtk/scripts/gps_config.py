#!/usr/bin/env python

from optparse import OptionParser
import pyUblox.ublox
import sys

def main():
    parser = OptionParser("gps_config.py [options]")
    parser.add_option("--port", help="serial port", default='/dev/ttyACM0')
    parser.add_option("--baudrate", type='int',
                      help="serial baud rate", default=115200)

    (opts, args) = parser.parse_args()

    dev = pyUblox.ublox.UBlox(opts.port, baudrate=opts.baudrate, timeout=2)
    dev.set_binary()
    dev.configure_poll_port()
    dev.configure_poll(pyUblox.ublox.CLASS_CFG, pyUblox.ublox.MSG_CFG_USB)
    dev.configure_poll(pyUblox.ublox.CLASS_MON, pyUblox.ublox.MSG_MON_HW)
    dev.configure_port(port=pyUblox.ublox.PORT_SERIAL1, inMask=1, outMask=0)
    dev.configure_port(port=pyUblox.ublox.PORT_USB, inMask=1, outMask=1)
    dev.configure_port(port=pyUblox.ublox.PORT_SERIAL2, inMask=1, outMask=0)
    dev.configure_poll_port()
    dev.configure_poll_port(pyUblox.ublox.PORT_SERIAL1)
    dev.configure_poll_port(pyUblox.ublox.PORT_SERIAL2)
    dev.configure_poll_port(pyUblox.ublox.PORT_USB)
    dev.configure_solution_rate(rate_ms=1000)

    dev.configure_message_rate(pyUblox.ublox.CLASS_NAV, pyUblox.ublox.MSG_NAV_POSLLH, 1)
    dev.configure_message_rate(pyUblox.ublox.CLASS_NAV, pyUblox.ublox.MSG_NAV_POSECEF, 1)
    dev.configure_message_rate(pyUblox.ublox.CLASS_RXM, pyUblox.ublox.MSG_RXM_RAW, 1)
    dev.configure_message_rate(pyUblox.ublox.CLASS_RXM, pyUblox.ublox.MSG_RXM_SFRB, 1)

if __name__ == '__main__':
    main()
