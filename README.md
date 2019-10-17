# nav-2020
### Dennis Evangelista, 2019
Navigation GPS and IMU node for AY2020

## Push Test 1
For Push Test 1 (PT1) and later this node is to generate and send PGN (??navigation messages). 

## Notes
This node should provide the PGN for the compass heading, latitude, longitude, heading, time, and IMU orientation roll pitch yaw (pitch roll yaw? which rotation sequence are we using). 

## Interface control
This node must send heartbeat and ISO address claim plus
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


## Applicability
USNA Sailbot AY20 Hull 14 mod 3 and later.
