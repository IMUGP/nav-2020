/** @file nav-2020/main.cpp
    @brief Navigational sensors for AY20 Sailbot Hull 14 mod 3
    D Evangelista and L Marino, 2019
*/

#include "mbed.h"
#include "rtos.h"
#include "SailbotIMU.h"
#include "SailbotGPS.h"
#include "nmea2k.h"
//#include "pgn/iso/Pgn60928.h" // ISO address claim
#include "pgn/Pgn126993.h" // heartbeat
#include "pgn/Pgn129025.h" // position, rapid update
#include "pgn/Pgn127250.h" // vessel heading
//#include "pgn/Pgn130577.h" // direction data REQUIRES FASTPACKET
#include "hull14mod3.h"
#define NAV_VERSION "14.3.0 PT1"

Serial pc(USBTX,USBRX);
SailbotIMU imu(p28,p27); // adafruit absolute orientation BNO055
SailbotGPS gps(p9,p10); // adafruit absolute GPS

nmea2k::CANLayer n2k(p30,p29); // used for sending nmea2k messages
DigitalOut txled(LED1);
DigitalOut rxled(LED2);
unsigned char node_addr = HULL14MOD3_NAV_ADDR; 

Thread heartbeat_thread;
Thread gps_thread;
Thread imu_thread;

void heartbeat_process(void);
void gps_process(void);
void imu_process(void);

int main(void)
{
    nmea2k::Frame f;
    nmea2k::PduHeader h;

    pc.printf("0x%02x:main: Nav node version %s\r\n",node_addr,NAV_VERSION);
    pc.printf("0x%02x:main: nmea2k version %s\r\n",node_addr,NMEA2K_VERSION);

    // TODO
    // assert ISO address
    // wait

    // start necessary processes
    heartbeat_thread.start(&heartbeat_process);

    // start GPS and IMU
    //gps_thread.start(&gps_process); 
    imu_thread.start(&imu_process);
    
    pc.printf("0x%02x:main: listening for any pgn\r\n",node_addr);
    while (1) {
        ThisThread::sleep_for(1000);
    } // while(1)
} // int main(void)






void heartbeat_process(void)
{
    nmea2k::Frame m;     // holds nmea2k data frame before sending
    nmea2k::PduHeader h; // ISO11783-3 header information
    nmea2k::Pgn126993 d(6000,0);   // for PGN data fields
    unsigned int heartbeat_interval=60;
    unsigned char c=0;           // heartbeat sends a heartbeat counter

    pc.printf("0x%02x:heartbeat_thread: starting heartbeat_process\r\n",
              node_addr);
    while (1) {
        h = nmea2k::PduHeader(d.p,d.pgn,node_addr,NMEA2K_BROADCAST); // form header
        d = nmea2k::Pgn126993(heartbeat_interval*100,c++); // form PGN fields
        m = nmea2k::Frame(h.id(),d.data(),d.dlen); // assemble message
        if (n2k.write(m)) { // send it!
            txled = 1;
            pc.printf("0x%02x:heartbeat_thread: sent %s, %0.0f s, count %d\r\n",
                      node_addr,
                      d.name,
                      (float) d.update_rate()/100.0,
                      d.heartbeat_sequence_counter());
            ThisThread::sleep_for(5);
            txled = 0;
        } else
            pc.printf("0x%02x:heartbeat_thread: failed sending %s\r\n",
                      node_addr,
                      d.name);
        ThisThread::sleep_for(heartbeat_interval*1000);
    } // while(1)
} // void heartbeat_thread(void)





// GPS process periodically reads GPS and sends PGN 129025 Position
void gps_process(void){

  float lat=0.0;
  float lon=0.0; 
  nmea2k::Frame m;     // holds nmea2k data frame before sending
  nmea2k::PduHeader h; // ISO11783-3 header information
  nmea2k::Pgn129025 d(0,0);   // for PGN data fields
  unsigned int interval=10; //time interval

  pc.printf("0x%02x:gps_thread: starting gps_process\r\n", node_addr);
  // Note GPS constructor should start the gps with following
  // myGPS.begin(9600);
  // myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // or whatever
  // myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // or whatever
  // myGPS.sendCommand(PGCMD_ANTENNA); 
  
  while(1) {
  //TODO get GPS position LATER
    lat = 0.0; //38.9784; // myGPS.latitude;
    lon = 0.0; //-76.4922; // myGPS.longitude; 
    
  // send it
    h = nmea2k::PduHeader(d.p,d.pgn,node_addr,NMEA2K_BROADCAST);
    d = nmea2k::Pgn129025(round(lat*PGN_129025_RES_LATITUDE), // latitude
			  round(lon*PGN_129025_RES_LONGITUDE) // longitude
			  );
    m = nmea2k::Frame(h.id(),d.data(),d.dlen);
    if (n2k.write(m)) {
      txled = 1;
      pc.printf("0x%02x:gps_thread: sent %s\r\n",
		node_addr,
		d.name);
      ThisThread::sleep_for(5);
      txled = 0;
    } else
      pc.printf("0x%02x:gps_thread: failed sending %s\r\n",
		node_addr,
		d.name);
    
    ThisThread::sleep_for(interval*1000);
  } // while(1)
}






// IMU process periodically reads IMU and sends 130577 Direction Data
// or, for now, 127250 Vessel Heading...
// Note BNO055 yaw 0.0 corresponds to magnetic north, so reference for
// this sensor is PGN_127250_REF_MAGNETIC
void imu_process(void){

  float yaw = 0.0;
  float deviation = 0.0; // garbage value
  float variation = 0.0; // garbage value

  nmea2k::Frame m;     // holds nmea2k data frame before sending
  nmea2k::PduHeader h; // ISO11783-3 header information
  nmea2k::Pgn127250 d(0,0,0,0,0);   // for PGN data fields
  unsigned int interval=10; //time interval

  pc.printf("0x%02x:imu_thread: starting imu_process\r\n", node_addr);

    //NOTE these are taken care of in IMU constructor
    //imu.reset();//resets imu
    //imu.setmode(OPERATION_MODE_NDOF);
    //imu.get_calib();
  
  while(1) {
    imu.get_angles();
    yaw = imu.get_yaw(); //get yaw and store it

    h = nmea2k::PduHeader(d.p,d.pgn,node_addr,NMEA2K_BROADCAST);
    d = nmea2k::Pgn127250(0, // sid
			  round(yaw*PGN_127250_ANGLE_RES), // yaw
			  round(deviation*PGN_127250_ANGLE_RES), // deviation
			  round(variation*PGN_127250_ANGLE_RES), // variation
			  PGN_127250_REF_MAGNETIC // reference
			  );
    m = nmea2k::Frame(h.id(),d.data(),d.dlen);
    if (n2k.write(m)) {
      txled = 1;
      pc.printf("0x%02x:imu_thread: sent %s\r\n",
		node_addr,
		d.name);
      ThisThread::sleep_for(5);
      txled = 0;
    } else
      pc.printf("0x%02x:imu_thread: failed sending %s\r\n",
		node_addr,
		d.name);
    
    ThisThread::sleep_for(interval*1000);
  } // while(1)
}


