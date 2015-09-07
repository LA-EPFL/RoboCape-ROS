#!/usr/bin/env python

from optparse import OptionParser
import pyUblox.ublox
import sys

def main():
    parser = OptionParser("pyUblox.ublox_capture_raw.py [options]")
    parser.add_option("--port", help="serial port", default='/dev/ttyACM0')
    parser.add_option("--baudrate", type='int',
                      help="serial baud rate", default=115200)

    (opts, args) = parser.parse_args()

    # which SV IDs we have seen
    svid_seen = {}
    svid_ephemeris = {}

    def handle_rxm_raw(msg):
        '''handle a RXM_RAW message'''
        global svid_seen, svid_ephemeris

        for i in range(msg.numSV):
            sv = msg.recs[i].sv
            tnow = time.time()
            if not sv in svid_seen or tnow > svid_seen[sv]+30:
                if sv in svid_ephemeris and svid_ephemeris[sv].timereceived+1800 < tnow:
                    continue
                dev.configure_poll(pyUblox.ublox.CLASS_AID, pyUblox.ublox.MSG_AID_EPH, struct.pack('<B', sv))
                svid_seen[sv] = tnow

    while True:
        msg = dev.receive_message()

        if msg is None:
            if opts.reopen:
                dev.close()
                dev = pyUblox.ublox.pyUblox.ublox(opts.port, baudrate=opts.baudrate, timeout=2)
                dev.set_logfile(opts.log, append=True)
                continue
            break

        # Show message received
        try:
            print(str(msg))
            sys.stdout.flush()
        except pyUblox.ublox.pyUblox.ubloxError as e:
            print(e)

        if msg.name() == 'RXM_RAW':
            msg.unpack()
            handle_rxm_raw(msg)
        if msg.name() == 'AID_EPH':
            try:
                msg.unpack()
                svid_ephemeris[msg.svid] = ephemeris.EphemerisData(msg)
            except pyUblox.ublox.pyUblox.ubloxError as e:
            print(e)

if __name__ == '__main__':
    main()
