#!/usr/bin/env python

import pyUblox.ublox

dev = pyUblox.ublox.UBlox('/dev/ttyO4', baudrate=9600, timeout=2)
dev.configure_solution_rate(rate_ms=100); # Uptade rate to F=1/100ms = 10Hz
dev.set_preferred_dynamic_model(4);       # Set to automotive
