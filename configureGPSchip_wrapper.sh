#! /bin/bash

stty -F /dev/ttyS0 -echo

# observed failure rate in I2C bus ack comms is a conservatively HIGH guess of 50%
# .5^8 == .004, an acceptable rate to falsely report "failure to configure" for a GPS chip that was just unlucky on I2C bus crosstalk
for ((i=0; i<8; i++))
do
   ./configureGPSchip -b 115200 -t 1 -r 250 -m 4
   rc=$?

   if [ $rc -eq 0 ]; then
      break
   fi
done

if [ ! $rc -eq 0 ]; then
   exit 1
fi

stty -F /dev/ttyS0 115200

exit 0
