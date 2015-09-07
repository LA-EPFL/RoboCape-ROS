from optparse import OptionParser
import pyUblox.ublox
import sys

def main():
    parser = OptionParser("gps_config.py [options]")
    parser.add_option("--port", help="serial port", default='/dev/ttyACM0')
    parser.add_option("--baudrate", type='int',
                      help="serial baud rate", default=115200)

    (opts, args) = parser.parse_args()

    dev = ublox.UBlox(opts.port, baudrate=opts.baudrate, timeout=2)
    dev.set_binary()
    dev.configure_poll_port()
    dev.configure_poll(ublox.CLASS_CFG, ublox.MSG_CFG_USB)
    dev.configure_poll(ublox.CLASS_MON, ublox.MSG_MON_HW)
    dev.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
    dev.configure_port(port=ublox.PORT_USB, inMask=1, outMask=1)
    dev.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
    dev.configure_poll_port()
    dev.configure_poll_port(ublox.PORT_SERIAL1)
    dev.configure_poll_port(ublox.PORT_SERIAL2)
    dev.configure_poll_port(ublox.PORT_USB)
    dev.configure_solution_rate(rate_ms=1000)

    dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSLLH, 1)
    dev.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSECEF, 1)
    dev.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_RAW, 1)
    dev.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SFRB, 1)

if __name__ == '__main__':
    main()
