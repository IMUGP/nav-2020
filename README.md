# nav-2020
### D Evangelista and L Marino, 2019
Navigation GPS and IMU node for AY2020

## Push Test 1
For Push Test 1 (PT1) and later this node is to generate and send PGN (??navigation messages). 

## Notes
This node should provide the PGN for the compass heading, latitude, longitude, heading, time, and IMU orientation roll pitch yaw (pitch roll yaw? which rotation sequence are we using). 

The BNO055 and GPS libraries need to be added. The libraries were pulled from the mbed website, looking for their repositories on Github so I can add the libraries like Dr. Evangelista instead of added the entire header and cpp files.

## Interface control
This node must send heartbeat 126993 and ISO address claim 060928 plus
Position, Rapid update: 129025
             1: Latitude
             2: Longitude
Direction Data: 130577
              1: Data Mode
              2: COG ref
              3: reserve bits
              4: SID
              5: COG
              6: SOG
              7: heading
              8: Speed through water
              9: set
             10: drift

We have the GPS connected to (p9,p10). The BNO is connected to (p28, p27).  The RPI node is communicating through USB RX and TX.

Attitude 127257

Note for simplicity during early tests we are first having this node send 127250 vessel heading... 

## Applicability
USNA Sailbot AY20 Hull 14 mod 3 and later.

## Contributors
D Evangelista and L Marino

