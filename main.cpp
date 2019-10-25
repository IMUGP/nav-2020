/** @file nav-2020/main.cpp
    @brief Navigational sensors for AY20 Sailbot Hull 14 mod 3
    D Evangelista, 2019 
*/

#include "mbed.h"
#include "rtos.h"
//#include "nmea2k.h"
//#include "pgn/Pgn060928.h" // ISO address claim
//#include "pgn/Pgn126993.h" // heartbeat
//#include "pgn/Pgn129025.h" // position (rapid update?)
//#include "pgn/Pgn130577.h" // direction data 
#define __VERSION__ "14.3.0"

Serial pc(USBTX,USBRX);

// We have the GPS connected to (p9,p10). The BNO is connected to (p28, p27).  The RPI node is communicating through USB RX and TX.
// TODO adafruit absolute GPS
// TODO adafruit absolute orientation

nmea2k::CANLayer n2k(p30,p29); // used for sending nmea2k messages
DigitalOut txled(LED1);
DigitalOut rxled(LED2); 
unsigned char node_addr = 0x00; // TODO FIX LATER

Thread heartbeat_thread;
Thread gps_thread;
Thread imu_thread;

void heartbeat_process(void);
void gps_process(void);
void imu_process(void);

int main(void){
  nmea2k::Frame f;
  nmea2k::PduHeader h;

  pc.printf("Nav node version %s\r\n",__VERSION__); 
  //pc.printf("nmea2k version %s\r\n",NMEA2K_VERSION);

  // TODO
  // assert ISO address
  // wait

  // start necessary processes
  heartbeat_thread.start(&heartbeat_process);
  // start GPS and IMU
  
  pc.printf("0x%02x:main: listening for any pgn\r\n",node_addr);
  while (1){
    ThisThread::sleep_for(1000); 
  } // while(1)
} // int main(void)






void heartbeat_process(void){
  nmea2k::Frame m;     // holds nmea2k data frame before sending
  nmea2k::PduHeader h; // ISO11783-3 header information 
  nmea2k::Pgn126993 d(6000,0);   // for PGN data fields
  unsigned int heartbeat_interval=60;
  unsigned char c=0;           // heartbeat sends a heartbeat counter

  pc.printf("0x%02x:heartbeat_thread: starting heartbeat_process\r\n",
	    node_addr); 
  while (1){
    h = nmea2k::PduHeader(d.p,d.pgn,node_addr,NMEA2K_BROADCAST); // form header 
    d = nmea2k::Pgn126993(heartbeat_interval*100,c++); // form PGN fields
    m = nmea2k::Frame(h.id(),d.data(),d.dlen); // assemble message
    if (n2k.write(m)){ // send it!
      txled = 1; 
      pc.printf("0x%02x:heartbeat_thread: sent %s, %0.0f s, count %d\r\n",
		node_addr,
		d.name,
		(float) d.update_rate()/100.0,
		d.heartbeat_sequence_counter());
      ThisThread::sleep_for(5); 
      txled = 0;
    }
    else
      pc.printf("0x%02x:heartbeat_thread: failed sending %s\r\n",
		node_addr,
		d.name); 
    ThisThread::sleep_for(heartbeat_interval*1000); 
  } // while(1)
} // void heartbeat_thread(void)


// GPS process periodically reads GPS and sends PGN 129025 Position

// IMU process periodically reads IMU and sends PGN 130577 Direction Data
