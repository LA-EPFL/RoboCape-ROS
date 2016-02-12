#!/usr/bin/env python

import rospy
import pyUblox.ublox

def conf():
   dev = pyUblox.ublox.UBlox('/dev/ttyO4', baudrate=203400, timeout=2)
   dev.configure_solution_rate(rate_ms=200); # Uptade rate to F=1/200ms = 5Hz
   dev.set_preferred_dynamic_model(8);       # Set to airborne with < 4g acceleration (almost no filtering)
   dev.close();

if __name__ == '__main__':
   try:
      conf();
   except rospy.ROSInterruptionException:
      pass
