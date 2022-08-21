//Version 2.4 with telemetry



//* /////////////////////////////////////////////////////////////////////////////////////////////


//Include all the libraries needed
#include <SPI.h>
#include <Wire.h>
#include <SD.h> 
#include <EEPROM.h>

#include <nRF24L01.h>         //telemetry library
#include <RF24.h>             //telemetry library


//* /////////////////////////////////////////////////////////////////////////////////////////////

//Debugging tools

#define QUADCOPTER
//#define HEXCOPTER

//#define HEADING_HOLD_MODE

#define MAINTAIN_ALTITUDE

#define X_FLIP
#define Y_FLIP
#define Z_FLIP

//#define TUNING_MODE 
#define DEBUG_PID_VALUES

#define DEBUG_GYRO
#define DEBUG_GYRO_AFTER_MOVING_AVERAGE
#define DEBUG_ACCEL
#define DEBUG_ACCEL_AFTER_MOVING_AVERAGE
#define DEBUG_MAG
#define DEBUG_BAROMETER_CALIBRATION_VALUES
#define DEBUG_BAROMETER_SETUP
#define DEBUG_BARO
#define DEBUG_GPS
#define DEBUG_GPS_SIMULATED
#define DEBUG_BATTERY
#define DEBUG_TELEMETRY

#define DEBUG_START
#define DEBUG_LOOP_TIME
#define DEBUG_PRESSURE
#define DEBUG_PITCH_ROLL
#define DEBUG_HEADING
#define DEBUG_ALTITUDE
#define DEBUG_TRANSMITTER
#define DEBUG_PID_SETPOINT
#define DEBUG_PID_OUTPUT
//#define DEBUG_MOTOR


//* /////////////////////////////////////////////////////////////////////////////////////////////


//Defining addresses of each sensor

#define MPU9250_ADDR 0x68
#define AK8963_ADDR 0x0C
//#define MS5611_address 0x76

#define CE_PIN 9        //telemetry pins
#define CSN_PIN 10      //telemetry pins
#define LOOP_TIME 4000


//* /////////////////////////////////////////////////////////////////////////////////////////////

//PID values

float pid_p_gain_roll = 0.75;                    //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.0000;                   //Gain setting for the roll I-controller
float pid_d_gain_roll = 0.000;                    //Gain setting for the roll D-controller
uint16_t pid_max_roll = 350;                    //Maximum output of the PID-controller (+/-)
uint16_t pid_max_i_roll = 100;                  //Eliminate I controller windup

#ifdef QUADCOPTER
float pid_p_gain_pitch = 0.75;       //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.0001;       //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 0.050;       //Gain setting for the pitch D-controller.
uint16_t pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)
uint16_t pid_max_i_pitch = pid_max_i_roll;      //Eliminate I controller windup
#endif

#ifdef HEXCOPTER
float pid_p_gain_pitch = 1;       //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.000;       //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 0.00;       //Gain setting for the pitch D-controller.
uint16_t pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)
uint16_t pid_max_i_pitch = pid_max_i_roll;      //Eliminate I controller windup
#endif

float pid_p_gain_yaw = 0.7;                     //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.0;                     //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                     //Gain setting for the pitch D-controller.
uint16_t pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
float heading_hold_gain = 3.0;


float pid_p_gain_alt = 0.1;
float pid_i_gain_alt = 0.000001;
float pid_d_gain_alt = 0.1;
uint16_t pid_max_alt = 100;   


//* /////////////////////////////////////////////////////////////////////////////////////////////

//Misc. variables
uint8_t start;
float voltage;
uint32_t difference, main_loop_timer;
byte eeprom_data[75];
uint8_t temp_mode = 1, prev_mode = 1, axis_mode = 0;

int flight_mode;

int esc_1, esc_2, esc_3, esc_4, esc_5, esc_6;

//Battery variables
uint16_t battery_voltage;
float battery_sum, battery_mem[32];
uint8_t battery_counter;

//Transmitter variables
uint16_t receiver_input_channel_1 = 0, receiver_input_channel_2 = 0,
         receiver_input_channel_3 = 0, receiver_input_channel_4 = 0,
         receiver_input_channel_5 = 0, receiver_input_channel_6 = 0;

//IMU variables
float gyro_cal[4], accel_cal[6], mag_cal[6];
float acc_cal_roll, acc_cal_pitch;

int16_t acc_x_raw, acc_y_raw, acc_z_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
int16_t temperature;

int16_t acc_x_mem[16], acc_y_mem[16], acc_z_mem[16];
int16_t gyro_x_mem[8], gyro_y_mem[8], gyro_z_mem[8];
int32_t acc_x_sum, acc_y_sum, acc_z_sum, gyro_x_sum, gyro_y_sum, gyro_z_sum;
uint8_t gyro_loop_counter = 0, acc_loop_counter = 0;

int32_t acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;

int8_t roll, pitch;
float angle_roll, angle_pitch, angle_yaw;
float angle_roll_acc, angle_pitch_acc;
float roll_level_adjust, pitch_level_adjust;

//Compass variables
float mag_sensitivity[3];
int16_t mag_x_raw, mag_y_raw, mag_z_raw;
float mag_x, mag_y, mag_z;
float compass_x, compass_y, compass_heading;

float mag_x_offset, mag_y_offset, mag_z_offset;
float mag_x_scale, mag_y_scale, mag_z_scale;

float prev_heading, heading;
bool heading_hold = false;

//PID variables
float /*pid_roll_error_temp, pid_pitch_error_temp,*/ pid_yaw_error_temp, pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

//Variables for altitude
float pid_i_mem_alt, pid_alt_setpoint, pid_output_alt, pid_last_alt_d_error;

double drone_altitude=0;


//GPS variables

int pid_roll_setpoint_base, pid_pitch_setpoint_base;


uint32_t error_timer, flight_mode_timer;

uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
uint8_t waypoint_set, latitude_north=1, longiude_east=1;
uint16_t message_counter;
int16_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
uint8_t new_line_found=0, new_gps_data_available, new_gps_data_counter;
uint8_t gps_rotating_mem_location, return_to_home_step;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
uint32_t gps_watchdog_timer;

int32_t gps_lat, gps_lon;
float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;
float return_to_home_lat_factor, return_to_home_lon_factor, return_to_home_move_factor;
uint8_t home_point_recorded;
int32_t lat_gps_home, lon_gps_home;
uint32_t channel_1_base, channel_2_base, channel_1, channel_2;


//Telemetery variables (Remember to uncomment)

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";


uint8_t telemetry_loop_counter = 0;
uint8_t telemetry_data;
uint32_t telemetry_temp;



typedef union {
  float decimal;
  uint8_t bytes[4];
} converter;

converter number;


//* /////////////////////////////////////////////////////////////////////////////////////////////



void setup() {
  Wire.begin();
  digitalWrite(13, LOW);
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial1.begin(9600);

  for (start = 0; start <= 74; start++)
    eeprom_data[start] = EEPROM.read(start);
  while (eeprom_data[72] != 'J' || eeprom_data[73] != 'M' || eeprom_data[74] != 'B')
    delay(10);

// reading EEPROM data recorded in hardware

  for (int i = 0; i < 6; i++) {
    number.bytes[0] = eeprom_data[i * 4 + 24];
    number.bytes[1] = eeprom_data[i * 4 + 25];
    number.bytes[2] = eeprom_data[i * 4 + 26];
    number.bytes[3] = eeprom_data[i * 4 + 27];
    accel_cal[i] = number.decimal;
  }

  for (int i = 0; i < 6; i++) {
    number.bytes[0] = eeprom_data[i * 4 + 48];
    number.bytes[1] = eeprom_data[i * 4 + 49];
    number.bytes[2] = eeprom_data[i * 4 + 50];
    number.bytes[3] = eeprom_data[i * 4 + 51];
    mag_cal[i] = number.decimal;
  }

  attachInterrupt(digitalPinToInterrupt(14), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(15), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(16), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(22), receiver_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(23), receiver_change, CHANGE);

  PORTD_PCR2 = (1 << 8); //configuring pin 7 as GPIO
  PORTD_PCR3 = (1 << 8); //configuring pin 8 as GPIO
  PORTD_PCR4 = (1 << 8); //configuring pin 6 as GPIO
  PORTD_PCR7 = (1 << 8); //configuring pin 5 as GPIO
  PORTD_PCR5 = (1 << 8); //configuring pin 20 as GPIO
  PORTD_PCR6 = (1 << 8); //configuring pin 21 as GPIO
  //General Purpose Input Output (GPIO)
  GPIOD_PDDR |= 252; //0000 0000 0000 0000 0000 0000 1111 1100 --> Setting pins 5,6,7,8,20,21 as outputs

  PORTC_PCR5 = (1 << 8);  //configuring LED pin as GPIO
  GPIOC_PDDR = (1 << 5);  //configuring LED pin as an output
  //GPIOC_PSOR = (1 << 5);  //setting LED pin high

  Serial.println("Welcome to flight controller setup!");
  Serial.println("Turn on your transmitter and place throttle at lowest position!");

/*Start of comment*/ // Comment the below code to test teensy without the reciever
  
//  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400)  {
//    receiver_input_channel_3 = convert_receiver_channel(3); //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
//    receiver_input_channel_4 = convert_receiver_channel(4); //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
//    start++;                                                //While waiting increment start whith every loop.
//
//    pulse_esc();
//    if (start == 125) {
//      digitalWrite(13, !digitalRead(13));                   //Change the led status.
//      start = 0;                                            //Start again at 0.
//    }
//  }

/*End of comment*/
  start = 0;
  Serial.println("Transmitter detected!");

  delay(500);

  digitalWrite(13, HIGH);
  gps_setup();
  Serial.println("GPS Setup Complete");
  Serial.println("----------------------------------------");
  
  setup_barometer();
  Serial.println("Barometer Setup Complete");
  Serial.println("----------------------------------------");
  
  setup_sensor();
  Serial.println("Sensor Setup Complete");
  Serial.println("----------------------------------------");
  
  calibrate_sensors();
  Serial.println("Sensor Calibration Complete");
  Serial.println("----------------------------------------");
  
  initialise_telemetry();
  Serial.println("Telemetry Setup Complete");
  Serial.println("----------------------------------------");
  
  //setup_sd();

  Serial.println("Magnetometer SCALES: ");
  for (int i = 0; i < 3; i++) Serial.print((String) mag_cal[i * 2] + " ");
  Serial.println();

  Serial.println("Magnetometer OFFSETS: ");
  for (int i = 0; i < 3; i++) Serial.print((String) mag_cal[i * 2 + 1] + " ");
  Serial.println("\n");
  Serial.println("----------------------------------------");
  
  Serial.print("Connect your battery in: ");
  for (int i = 5; i > 0; i--) {
    Serial.print((String) i + " ");
    delay(900);
    pulse_esc();
  }
  Serial.println();
  Serial.println("----------------------------------------");
  
  //detecting battery
  int sensor_value = analogRead(A14);
  float analog_voltage = sensor_value * (3.3 / 1024);
  battery_voltage = 100 * ((analog_voltage + 0.6) * 5.05625 + 0.4);
  Serial.println("Battery left: " + (String) battery_voltage);
  Serial.println("Setup DONE!");
  angle_roll=0.0;
  angle_pitch=0.0;
  digitalWrite(13, LOW);
}


//* /////////////////////////////////////////////////////////////////////////////////////////////


void loop() {
  convert_transmitter_values();

  check_start_stop();

  calculate_pitch_roll();

  calculate_heading();

  read_gps();
  
  channel_1 = receiver_input_channel_1;
  channel_2 = receiver_input_channel_2;
  channel_1_base = channel_1;                                                      //Normally channel_1 is the pid_roll_setpoint input.
  channel_2_base = channel_2;                                                      //Normally channel_2 is the pid_pitch_setpoint input.
  gps_man_adjust_heading = heading;                                              //

  if (flight_mode >= 3 && waypoint_set == 1) {
    pid_roll_setpoint_base = 1500 + gps_roll_adjust;
    pid_pitch_setpoint_base = 1500 + gps_pitch_adjust;
  }
  else {
    pid_roll_setpoint_base = channel_1_base;
    pid_pitch_setpoint_base = channel_2_base;
  }

  //Because we added the GPS adjust values we need to make sure that the control limits are not exceded.
  if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
  if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
  if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
  if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;

  read_barometer();

  set_pid_offsets();

  calculate_pid();

  calculate_esc_output();

  set_escs();

  calculate_battery();

//  test_telemetry();
  send_telemetry_data();

#ifdef TUNING_MODE
  tune_PID_gains();
#endif

#ifdef MAINTAIN_ALTITUDE
  maintain_altitude();
#endif

  maintain_loop_time();
}


//* /////////////////////////////////////////////////////////////////////////////////////////////


void check_start_stop() {
#ifndef DEBUG_START
//if NOT Defined, will execute loop
  Serial.println(start);
#endif
  //For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050 && receiver_input_channel_1 > 1950 && receiver_input_channel_2 < 1050)  {
    if (start == 0) {
      start = 1;
    }
    else if (start == 2) { //Stop motors --> check if already started
      Serial.println("STOP MOTORS");
      start = 3;
    }
  }

  //When yaw stick is back in the center position start the motors (step 2).
  if (receiver_input_channel_4 > 1450 && receiver_input_channel_1 < 1550 && receiver_input_channel_2 > 1450)  {
    if (start == 1)  {
      Serial.println("START MOTORS");
      start = 2;                     //start motors

      angle_pitch = angle_pitch_acc; //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
      angle_roll = angle_roll_acc;   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
      prev_heading = heading;

      //Reset the PID controllers
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
    }
    else if (start == 3)    {
      start = 0; //Stop motors
    }
  }
}


//* /////////////////////////////////////////////////////////////////////////////////////////////


void calculate_battery() {
  float diodeForward = 0.4;
  float calculation_error = 0.4;
  float potDivider = 5.05625; // 1 / (0.8/(0.8+3.245))

  int sensor_value = analogRead(A14);
  float analog_voltage = sensor_value * (3.3 / 1024);

//  battery_sum += analog_voltage;
//  battery_sum -= battery_mem[battery_counter];
//  battery_mem[battery_counter] = analog_voltage;
//  float voltage = battery_sum / 32.0;
//
//  if (battery_counter == 31) battery_counter = 0;
//  else battery_counter++;

  voltage = 1.3452 * voltage - 0.2569;
  float battery = 100 * ((voltage * potDivider) + diodeForward + calculation_error);
  battery_voltage = battery_voltage * 0.95 + battery * 0.05;
  battery_voltage = 1550;

#ifndef DEBUG_BATTERY
//if NOT Defined, will execute loop
  Serial.println(battery_voltage);
#endif
}


//* /////////////////////////////////////////////////////////////////////////////////////////////


void pulse_esc() {
  GPIOD_PSOR |= 252;    //0000 0000 0000 0000 0000 0000 1111 1100 --> Setting pins 5,6,7,8,20,21 as HIGH
  delayMicroseconds(1000);
  GPIOD_PCOR |= 252;    //0000 0000 0000 0000 0000 0000 1111 1100 --> Setting pins 5,6,7,8,20,21 as LOW
  delay(2);
}


//* /////////////////////////////////////////////////////////////////////////////////////////////


void maintain_altitude(){
    //receiver_input_channel_6 == 3 way switch
    //When motors are switched on && maintain altitude is switched on

    if (start ==2){
      if (receiver_input_channel_6 > 1700){
        pid_alt_setpoint = 1.0;
        flight_mode = 1;
      }
      if (receiver_input_channel_6 < 1300){
        pid_alt_setpoint = drone_altitude;
        flight_mode=2;
      }
      else {
        pid_alt_setpoint = drone_altitude;
        Serial.println("GPS HOLD ENABLED");
        flight_mode=3;
      }
    }
  
}


//* /////////////////////////////////////////////////////////////////////////////////////////////


void maintain_loop_time () {
  difference = micros() - main_loop_timer;

#ifndef DEBUG_LOOP_TIME
//if NOT Defined, will execute loop
  Serial.println(difference);
#endif

  while (difference < LOOP_TIME) {
    difference = micros() - main_loop_timer;
  }

  main_loop_timer = micros();
}


//* /////////////////////////////////////////////////////////////////////////////////////////////


void tune_PID_gains() {
  //receiver_input_channel_5 == Turning knob
  //receiver_input_channel_6 == 3 way switch
  uint8_t gain_mode = 1;                             //Set mode to default CENTER => I gain
  if (receiver_input_channel_5 > 1700) gain_mode = 2;       //Set mode to RIGHT => D gain
  else if (receiver_input_channel_5 < 1300) gain_mode = 0;  //Set mode to LEFT => P gain

  uint8_t incr_mode = 1;                             //Set mode to center
  if (receiver_input_channel_6 > 1700) incr_mode = 2;       //Set mode to UP => Increase
  else if (receiver_input_channel_6 < 1300) incr_mode = 0;  //Set mode to DOWN => Decrease

  if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950 && receiver_input_channel_1 < 1050 && receiver_input_channel_2 < 1050)  {
    if (axis_mode == 0) axis_mode = 1;
    else if (axis_mode == 2) axis_mode = 3;
    else if (axis_mode == 4) axis_mode = 5;
  }

  //When yaw stick is back in the center position start the motors (step 2).
  if (receiver_input_channel_4 < 1550 && receiver_input_channel_1 > 1450 && receiver_input_channel_2 > 1450)  {
    if (axis_mode == 1) axis_mode = 2;
    else if (axis_mode == 3) axis_mode = 4;
    else if (axis_mode == 5) axis_mode = 0;
  }

  if (incr_mode == 2 && prev_mode == 1) temp_mode = 2;          //From center to UP
  else if (incr_mode == 0 && prev_mode == 1) temp_mode = 0;     //From center to DOWN
  else if (incr_mode == 1) {
    if (prev_mode == 2 && temp_mode == 2) {                     //Increment!
      Serial.println("Increment!");
      if (axis_mode == 0) {
        if (gain_mode == 0) pid_p_gain_roll += 0.1;
        else if (gain_mode == 1) pid_i_gain_roll += 0.001;
        else if (gain_mode == 2) pid_d_gain_roll += 0.1;
        
      } else if (axis_mode == 2) {
        if (gain_mode == 0) pid_p_gain_pitch += 0.1;
        else if (gain_mode == 1) pid_i_gain_pitch += 0.001;
        else if (gain_mode == 2) pid_d_gain_pitch += 0.1;
        
      } else if (axis_mode == 4) {
        if (gain_mode == 0) pid_p_gain_yaw += 0.1;
        else if (gain_mode == 1) pid_i_gain_yaw += 0.001;
        else if (gain_mode == 2) pid_d_gain_yaw += 0.1;
      }
    } else if (prev_mode == 0 && temp_mode == 0) {              //Decrement!
      Serial.println("Decrement!");
      if (axis_mode == 0) {
        if (gain_mode == 0) pid_p_gain_roll -= 0.1;
        else if (gain_mode == 1) pid_i_gain_roll -= 0.001;
        else if (gain_mode == 2) pid_d_gain_roll -= 0.1;
        
      } else if (axis_mode == 2) {
        if (gain_mode == 0) pid_p_gain_pitch -= 0.1;
        else if (gain_mode == 1) pid_i_gain_pitch -= 0.001;
        else if (gain_mode == 2) pid_d_gain_pitch -= 0.1;
        
      } else if (axis_mode == 4) {
        if (gain_mode == 0) pid_p_gain_yaw -= 0.1;
        else if (gain_mode == 1) pid_i_gain_yaw -= 0.001;
        else if (gain_mode == 2) pid_d_gain_yaw -= 0.1;
      }
    }
    temp_mode = 1;
  }

  prev_mode = incr_mode;

#ifndef DEBUG_PID_VALUES
  Serial.print(pid_p_gain_roll);
  Serial.print(", ");
  Serial.print(pid_i_gain_roll, 3);
  Serial.print(", ");
  Serial.print(pid_d_gain_roll);
  Serial.print(", ");
  Serial.print(pid_p_gain_pitch);
  Serial.print(", ");
  Serial.print(pid_i_gain_pitch, 3);
  Serial.print(", ");
  Serial.print(pid_d_gain_pitch);
  Serial.print(", ");
  Serial.print(pid_p_gain_yaw);
  Serial.print(", ");
  Serial.print(pid_i_gain_yaw, 3);
  Serial.print(", ");
  Serial.println(pid_d_gain_yaw);
#endif

}
