/** @file nav-2020/main.cpp
    @brief Navigational sensors for AY20 Sailbot Hull 14 mod 3
    D Evangelista and L Marino, 2019
*/

#include "mbed.h"
//#include "rtos.h"
#include "BNO055.h"
#include "MBed_Adafruit_GPS.h"
#include "nmea2k.h"
//#include "pgn/iso/Pgn60928.h" // ISO address claim
#include "pgn/Pgn126993.h" // heartbeat
#include "pgn/Pgn129025.h" // position, rapid update
#include "pgn/Pgn127250.h" // vessel heading
//#include "pgn/Pgn130577.h" // direction data REQUIRES FASTPACKET
#include "hull14mod3.h"
#define NAV_VERSION "14.3.0 PT1"

Serial pc(USBTX,USBRX);
BNO055 imu(p28,p27); // adafruit absolute orientation BNO055
Serial * gps_Serial;
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
float info; //a multiuse variable used for sending things over GPS
Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
const int refresh_Time = 1000; //refresh time in ms

float lat = 0.0;
float lon = 0.0;

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
    if(imu.check()) {
        pc.printf("BNO055 connected\r\n");
        imu.reset();
        wait(.5);
        imu.setmode(OPERATION_MODE_CONFIG);
        imu.SetExternalCrystal(1);
        //bno.set_orientation(1);
        imu.setmode(OPERATION_MODE_NDOF);  //Uses magnetometer
        //bno.setmode(OPERATION_MODE_NDOF_FMC_OFF);   //no magnetometer
        imu.set_angle_units(RADIANS);
    } else {
        pc.printf("BNO055 NOT connected\r\n Program Trap.\r\n");
        while(1);
    }

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
    gps_thread.start(&gps_process);
    imu_thread.start(&imu_process);


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
        ThisThread::sleep_for(heartbeat_interval*400);
    } // while(1)
} // void heartbeat_thread(void)





// GPS process periodically reads GPS and sends PGN 129025 Position
void gps_process(void)
{

    gps_Serial = new Serial(p9,p10); //serial object for use w/ GPS
    Adafruit_GPS myGPS(gps_Serial);

    myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
    //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf

    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);

    //pc.printf("Connection established at 115200 baud...\n");

    Thread::wait(100);
    refresh_Timer.start();  //starts the clock on the timer

    nmea2k::Frame m;     // holds nmea2k data frame before sending
    nmea2k::PduHeader h; // ISO11783-3 header information
    nmea2k::Pgn129025 d(0,0);   // for PGN data fields
    unsigned int intervalgps=7; //time interval

    pc.printf("0x%02x:gps_thread: starting gps_process\r\n", node_addr);
    // Note GPS constructor should start the gps with following
    // myGPS.begin(9600);
    // myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // or whatever
    // myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // or whatever
    // myGPS.sendCommand(PGCMD_ANTENNA);

    while(1) {
        pc.printf("gps in while(1)\r\n");
        //TODO get GPS position LATER
        c = myGPS.read();   //queries the GPS
        pc.printf("c is %c \r\n",c);
        lat = 3859.05;
        lon = 7929.14;
        if ( myGPS.newNMEAreceived() ) {
            pc.printf("gps got message\r\n");
            if ( !myGPS.parse(myGPS.lastNMEA()) ) {
                continue;
            }//if ( !myGPS.parse..
            if (refresh_Timer.read_ms() >= refresh_Time) {
                refresh_Timer.reset();

                if (myGPS.fix) {
                    pc.printf("gps got fix\r\n");
                    lat = myGPS.latitude;
                    lon = myGPS.longitude;
                    pc.printf("lat: %f long: %f\r\n",lat,lon);
                    ThisThread::sleep_for(100);
                }//if (myGPS.fix)
            }//if (refresh_Timer..
        }//if ( myGPS.newNMEAreceived() )

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

        ThisThread::sleep_for(intervalgps*400);
    } // while(1)
}






// IMU process periodically reads IMU and sends 130577 Direction Data
// or, for now, 127250 Vessel Heading...
// Note BNO055 yaw 0.0 corresponds to magnetic north, so reference for
// this sensor is PGN_127250_REF_MAGNETIC
void imu_process(void)
{

    float yaw = 0.0;
    float deviation = 0.0; // garbage value
    float variation = 0.0; // garbage value

    nmea2k::Frame m;     // holds nmea2k data frame before sending
    nmea2k::PduHeader h; // ISO11783-3 header information
    nmea2k::Pgn127250 d(0,0,0,0,0);   // for PGN data fields
    unsigned int interval=5; //time interval

    pc.printf("0x%02x:imu_thread: starting imu_process\r\n", node_addr);

    //NOTE these are taken care of in IMU constructor
    //imu.reset();//resets imu
    //imu.setmode(OPERATION_MODE_NDOF);
    //imu.get_calib();

    while(1) {
        imu.get_angles();
        yaw = imu.euler.yaw; //get yaw and store it
        pc.printf("yaw: %f\r\n",yaw);

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

        ThisThread::sleep_for(interval*400);
    } // while(1)
}

