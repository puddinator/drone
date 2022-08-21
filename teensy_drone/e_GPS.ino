#include <EEPROM.h>
#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



const unsigned char UBX_HEADER[] = {0xB5, 0x62};

const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40, // GxRMC off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

  // Disable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, //NAV-PVT off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, //NAV-STATUS off

  // Enable UBX
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // Rate
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

struct NAV_PVT {
  uint8_t cls;
  uint8_t id;
  uint16_t len;
  uint32_t iTOW;          // GPS time of week of the navigation epoch (ms)

  uint16_t year;         // Year (UTC)
  uint8_t month;         // Month, range 1..12 (UTC)
  uint8_t day;           // Day of month, range 1..31 (UTC)
  uint8_t hour;          // Hour of day, range 0..23 (UTC)
  uint8_t minute;        // Minute of hour, range 0..59 (UTC)
  uint8_t second;        // Seconds of minute, range 0..60 (UTC)

  int8_t valid;          // Validity Flags (see graphic below)
  uint32_t tAcc;         // Time accuracy estimate (UTC) (ns)
  int32_t nano;          // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  uint8_t fixType;       // GNSSfix Type, range 0..5
  int8_t flags;          // Fix Status Flags
  int8_t flags2;
  uint8_t numSV;         // Number of satellites used in Nav Solution

  int32_t lon;           // Longitude (deg)
  int32_t lat;           // Latitude (deg)
  int32_t height;        // Height above Ellipsoid (mm)
  int32_t hMSL;          // Height above mean sea level (mm)
  uint32_t hAcc;         // Horizontal Accuracy Estimate (mm)
  uint32_t vAcc;         // Vertical Accuracy Estimate (mm)

  int32_t velN;          // NED north velocity (mm/s)
  int32_t velE;          // NED east velocity (mm/s)
  int32_t velD;          // NED down velocity (mm/s)
  int32_t gSpeed;        // Ground Speed (2-D) (mm/s)
  int32_t headingMotion; // Heading of motion 2-D (deg)
  uint32_t sAcc;         // Speed Accuracy Estimate
  uint32_t headingAcc;   // Heading Accuracy Estimate
  uint16_t pDOP;         // Position dilution of precision

  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint8_t reserved5;
  uint8_t reserved6;

  int32_t headingVehicle;
  int16_t magDec;
  uint16_t magAcc;
};

NAV_PVT pvt;
uint8_t count = 0;


////////////////////////////////////////////////////////////////////////////////////////////////////////





float gps_p_gain = 2.7;                    //Gain setting for the GPS P-controller (default = 2.7).
float gps_d_gain = 6.5;                    //Gain setting for the GPS D-controller (default = 6.5).

uint8_t error;



void gps_setup(void) {

  // Send configuration data to GPS with UBX protocol
  for (uint16_t i = 0; i < sizeof(UBLOX_INIT); i++) {
    Serial1.write(pgm_read_byte(UBLOX_INIT + i));
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
  
  delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.

  //Disable GPGSV messages by using the ublox protocol.
  uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
  Serial1.write(Disable_GPGSV, 11);
  delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
  //Set the refresh rate to 5Hz by using the ublox protocol.
  uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  Serial1.write(Set_to_5Hz, 14);
  delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
  //Set the baud rate to 57.6kbps by using the ublox protocol.
  uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                               0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
                              };
  Serial1.write(Set_to_57kbps, 28);
  delay(200);

  Serial1.begin(57600);
  Serial.begin(57600);
  delay(200);
  
  Serial.print("GPS Setup Complete! ");
  Serial.println();
}


void calculate_checksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

void read_gps(void) {

  if (gps_add_counter >= 0)gps_add_counter --;

  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);

  while (Serial1.available() > 0) {
    char data = Serial1.read();
    
    if (count < 2) {                                      //Track the header
      if (data == UBX_HEADER[count]) count++;
      else count = 0;
    } 
    
    else {                                              //Reading the subsequent information
      if ((count - 2) < payloadSize){
        ((unsigned char*)(&pvt))[count - 2] = data;
      }
      
      count++;
      if (count == (payloadSize + 2)) {
        calculate_checksum(checksum);
      } 
      
      else if (count == (payloadSize + 3)) {
        if (data != checksum[0]) count = 0;
      } 
      
      else if (count == (payloadSize + 4)) {
        count = 0;
        if (data == checksum[1]) {                        //Data package is correct!
      
          lat_gps_actual=pvt.lat / 10.0f, 6;
          lon_gps_actual=pvt.lon / 10.0f, 6;
          
          #ifndef DEBUG_GPS
          
            Serial.print(" Latitude: "); Serial.print(lat_gps_actual);
            Serial.print(" Longitutde: "); Serial.print(lon_gps_actual);
            Serial.println();

          #endif
          
          if (lat_gps_previous == 0 && lon_gps_previous == 0) {                                              //If this is the first time the GPS code is used.
            lat_gps_previous = lat_gps_actual;                                                               //Set the lat_gps_previous variable to the lat_gps_actual variable.
            lon_gps_previous = lon_gps_actual;                                                               //Set the lon_gps_previous variable to the lon_gps_actual variable.
          }
          
          lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;                              //Divide the difference between the new and previous latitude by ten.
          lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;                              //Divide the difference between the new and previous longitude by ten.
          
         
          l_lat_gps = lat_gps_previous;                                                                      //Set the l_lat_gps variable to the previous latitude value. l_lat_gps is previous reading
          l_lon_gps = lon_gps_previous;                                                                      //Set the l_lon_gps variable to the previous longitude value.
        
          lat_gps_previous = lat_gps_actual;                                                                 //Remember the new latitude value in the lat_gps_previous variable for the next loop.
          lon_gps_previous = lon_gps_actual;                                                                 //Remember the new longitude value in the lat_gps_previous variable for the next loop.
          
          //The GPS is set to a 5Hz refresh rate. Between every 2 GPS measurments, 9 GPS values are simulated.
          gps_add_counter = 5;                                                                               //Set the gps_add_counter variable to 5 as a count down loop timer
          new_gps_data_counter = 9;                                                                          //Set the new_gps_data_counter to 9. This is the number of simulated values between 2 GPS measurements.
          lat_gps_add = 0;                                                                                   //Reset the lat_gps_add variable.
          lon_gps_add = 0;                                                                                   //Reset the lon_gps_add variable.
          new_gps_data_available = 1;                                                                        //Set the new_gps_data_available to indicate that there is new data available.
        }
      } 
      
      else if (count > (payloadSize + 4)) {
        count = 0;
      }      
    }
  }

  
  //After 5 program loops 5 x 4ms = 20ms the gps_add_counter is 0.
  if (gps_add_counter == 0 && new_gps_data_counter > 0) {                                                 //If gps_add_counter is 0 and there are new GPS simulations needed.
    new_gps_data_available = 1;                                                                           //Set the new_gps_data_available to indicate that there is new data available.
    new_gps_data_counter --;                                                                              //Decrement the new_gps_data_counter so there will only be 9 simulations
    gps_add_counter = 5;                                                                                  //Set the gps_add_counter variable to 5 as a count down loop timer

  
    lat_gps_add += lat_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
    if (abs(lat_gps_add) >= 1) {                                                                          //If the absolute value of lat_gps_add is larger then 1.
      l_lat_gps += (int)lat_gps_add;                                                                      //Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part.
      lat_gps_add -= (int)lat_gps_add;                                                                    //Subtract the lat_gps_add value as an integer so the decimal value remains.
    }
  
    lon_gps_add += lon_gps_loop_add;                                                                      //Add the simulated part to a buffer float variable because the l_lon_gps can only hold integers.
    if (abs(lon_gps_add) >= 1) {                                                                          //If the absolute value of lon_gps_add is larger then 1.
      l_lon_gps += (int)lon_gps_add;                                                                      //Increment the lon_gps_add value with the lon_gps_add value as an integer. So no decimal part.
      lon_gps_add -= (int)lon_gps_add;                                                                    //Subtract the lon_gps_add value as an integer so the decimal value remains.
    }

  #ifndef DEBUG_GPS_SIMULATED
  
    Serial.print(" Simulated Latitude: "); Serial.print(l_lat_gps);
    Serial.print(" Simulated Longitude: "); Serial.print(l_lon_gps);
    Serial.println();

  #endif
  
  }
    
  if (new_gps_data_available) {                                                                           //If there is a new set of GPS data available.
    gps_watchdog_timer = millis();                                                                        //Reset the GPS watch dog tmer.
    new_gps_data_available = 0;                                                                           //Reset the new_gps_data_available variable.
            
    if (flight_mode >= 3 && waypoint_set == 0) {                                                          //If the flight mode is 3 (GPS hold) and no waypoints are set.
      waypoint_set = 1;                                                                                   //Indicate that the waypoints are set.
      l_lat_waypoint = l_lat_gps;                                                                         //Remember the current latitude as GPS hold waypoint.
      l_lon_waypoint = l_lon_gps;                                                                         //Remember the current longitude as GPS hold waypoint.
    }
  
    if (flight_mode >= 3 && waypoint_set == 1) {                                                          //If the GPS hold mode and the waypoints are stored.

      //GPS stick move adjustments
      if (flight_mode == 3) {
        if (!latitude_north) {
          l_lat_gps_float_adjust += 0.0015 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //South correction
        }
        else {
          l_lat_gps_float_adjust -= 0.0015 * (((channel_2- 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * 0.017453))); //North correction
        }
  
        if (!longiude_east) {
          l_lon_gps_float_adjust -= (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2- 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //West correction
        }
  
        else {
          l_lon_gps_float_adjust += (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * 0.017453)) + ((channel_2 - 1500) * cos((gps_man_adjust_heading + 90) * 0.017453)))) / cos(((float)l_lat_gps / 1000000.0) * 0.017453); //East correction
        }
      }
  
      if (l_lat_gps_float_adjust > 1) {
        l_lat_waypoint ++;
        l_lat_gps_float_adjust --;
      }
      if (l_lat_gps_float_adjust < -1) {
        l_lat_waypoint --;
        l_lat_gps_float_adjust ++;
      }

      if (l_lon_gps_float_adjust > 1) {
        l_lon_waypoint ++;
        l_lon_gps_float_adjust --;
      }
      if (l_lon_gps_float_adjust < -1) {
        l_lon_waypoint --;
        l_lon_gps_float_adjust ++;
      }

      gps_lon_error = l_lon_waypoint - l_lon_gps;                                                         //Calculate the latitude error between waypoint and actual position.
      gps_lat_error = l_lat_gps - l_lat_waypoint;                                                         //Calculate the longitude error between waypoint and actual position.
  
      gps_lat_total_avarage -=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
      gps_lat_rotating_mem[ gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
      gps_lat_total_avarage +=  gps_lat_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.
  
      gps_lon_total_avarage -=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Subtract the current memory position to make room for the new value.
      gps_lon_rotating_mem[ gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;          //Calculate the new change between the actual pressure and the previous measurement.
      gps_lon_total_avarage +=  gps_lon_rotating_mem[ gps_rotating_mem_location];                         //Add the new value to the long term avarage value.
      gps_rotating_mem_location++;                                                                        //Increase the rotating memory location.
      if ( gps_rotating_mem_location == 35) gps_rotating_mem_location = 0;                                //Start at 0 when the memory location 35 is reached.
 
      gps_lat_error_previous = gps_lat_error;                                                             //Remember the error for the next loop.
      gps_lon_error_previous = gps_lon_error;                                                             //Remember the error for the next loop.
 
      //Calculate the GPS pitch and roll correction as if the nose of the multicopter is facing north.
      //The Proportional part = (float)gps_lat_error * gps_p_gain.
      //The Derivative part = (float)gps_lat_total_avarage * gps_d_gain.
      gps_pitch_adjust_north = (float)gps_lat_error * gps_p_gain + (float)gps_lat_total_avarage * gps_d_gain;
      gps_roll_adjust_north = (float)gps_lon_error * gps_p_gain + (float)gps_lon_total_avarage * gps_d_gain;
  
      if (!latitude_north)gps_pitch_adjust_north *= -1;                                                   //Invert the pitch adjustmet because the quadcopter is flying south of the equator.
      if (!longiude_east)gps_roll_adjust_north *= -1;                                                     //Invert the roll adjustmet because the quadcopter is flying west of the prime meridian.
  
      //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
      gps_roll_adjust = ((float)gps_roll_adjust_north * cos(heading* 0.017453)) + ((float)gps_pitch_adjust_north * cos((heading - 90) * 0.017453));
      gps_pitch_adjust = ((float)gps_pitch_adjust_north * cos(heading * 0.017453)) + ((float)gps_roll_adjust_north * cos((heading+ 90) * 0.017453));
  
      //Limit the maximum correction to 300. This way we still have full controll with the pitch and roll stick on the transmitter.
      if (gps_roll_adjust > 300) gps_roll_adjust = 300;
      if (gps_roll_adjust < -300) gps_roll_adjust = -300;
      if (gps_pitch_adjust > 300) gps_pitch_adjust = 300;
      if (gps_pitch_adjust < -300) gps_pitch_adjust = -300;
    }
  }
  
  if (gps_watchdog_timer + 1000 < millis()) {                                                             //If the watchdog timer is exceeded the GPS signal is missing.
    if (flight_mode >= 3 && start > 0) {                                                                  //If flight mode is set to 3 (GPS hold).
      flight_mode = 2;                                                                                    //Set the flight mode to 2.
      error = 4;                                                                                          //Output an error.
    }
  }
  
  if (flight_mode < 3 && waypoint_set > 0) {                                                              //If the GPS hold mode is disabled and the waypoints are set.
    gps_roll_adjust = 0;                                                                                  //Reset the gps_roll_adjust variable to disable the correction.
    gps_pitch_adjust = 0;                                                                                 //Reset the gps_pitch_adjust variable to disable the correction.
    if (waypoint_set == 1) {                                                                              //If the waypoints are stored
      gps_rotating_mem_location = 0;                                                                      //Set the gps_rotating_mem_location to zero so we can empty the buffer locations.
      waypoint_set = 2;                                                                                   //Set the waypoint_set variable to 2 as an indication that the buffer is not cleared.
    }
    gps_lon_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lon_rotating_mem location.
    gps_lat_rotating_mem[ gps_rotating_mem_location] = 0;                                                 //Reset the current gps_lat_rotating_mem location.
    gps_rotating_mem_location++;                                                                          //Increment the gps_rotating_mem_location variable for the next loop.
    if (gps_rotating_mem_location == 36) {                                                                //If the gps_rotating_mem_location equals 36, all the buffer locations are cleared.
      waypoint_set = 0;                                                                                   //Reset the waypoint_set variable to 0.
      //Reset the variables that are used for the D-controller.
      gps_lat_error_previous = 0;
      gps_lon_error_previous = 0;
      gps_lat_total_avarage = 0;
      gps_lon_total_avarage = 0;
      gps_rotating_mem_location = 0;
      //Reset the waypoints.
      l_lat_waypoint = 0;
      l_lon_waypoint = 0;
    }
  }
}
