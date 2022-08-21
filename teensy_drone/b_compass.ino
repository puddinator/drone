/*
 * 
As defined in first program: Compass variables
float mag_sensitivity[3];
int16_t mag_x_raw, mag_y_raw, mag_z_raw;
float mag_x, mag_y, mag_z;
float compass_x, compass_y, compass_heading;

float mag_x_offset, mag_y_offset, mag_z_offset;
float mag_x_scale, mag_y_scale, mag_z_scale;

float prev_heading, heading;
bool heading_hold = false;

 */

bool mag_first_start = true;
float temp_heading;


//* /////////////////////////////////////////////////////////////////////////////////////////////


void calculate_heading() {
  mag_x = mag_x_raw * mag_sensitivity[0];
  mag_y = mag_y_raw * mag_sensitivity[1];
  mag_z = mag_z_raw * mag_sensitivity[2];

  //  Apply magnetometer calibration scales and offsets to raw output
  mag_x = (mag_x_raw - mag_cal[1]) * mag_cal[0];
  mag_y = (mag_y_raw - mag_cal[3]) * mag_cal[2];
  mag_z = (mag_z_raw - mag_cal[5]) * mag_cal[4];

#ifdef X_FLIP
  mag_x *= -1;
#endif

#ifdef Y_FLIP
  mag_y *= -1;
#endif

#ifdef Z_FLIP
  mag_z *= -1;
#endif

  //Tilt compensation
  float roll_angle = roll * DEG_TO_RAD; //converting from degrees to radians
  float pitch_angle = pitch * DEG_TO_RAD;

  compass_x = mag_y * cos(pitch_angle)
              - mag_z * sin(pitch_angle);

  compass_y = mag_x * cos(roll_angle)
              + mag_y * sin(pitch_angle) * sin(roll_angle)
              + mag_z * cos(pitch_angle) * sin(roll_angle);

//  if (compass_x == 0) {
//    if (compass_y < 0) compass_heading = 90;
//    else compass_heading = 0;
//  } else compass_heading = atan2(compass_y, compass_x) * RAD_TO_DEG;

  compass_heading = atan2(compass_y, compass_x) * RAD_TO_DEG; //radians to degrees
  if (compass_heading < 0) {
    compass_heading = 360 + compass_heading;
    //making all angles between 0 and 360
  }

  if (mag_first_start) {
    angle_yaw = compass_heading;
    // if it is the first time starting the magnetometer, set angle yaw as the current compass_heading, then stop loop
    mag_first_start = false;
  }

  if (angle_yaw < 0) angle_yaw += 360;
  else if (angle_yaw >= 360) angle_yaw -= 360;
  //making all angles between 0 and 360

  if (abs(angle_yaw - compass_heading) > 330) {
    angle_yaw = compass_heading;
  // checking if previously calculated angle yaw is approximately same as the compass heading
  } else {
    angle_yaw = angle_yaw * 0.94 + compass_heading * 0.06;
  }

#ifndef DEBUG_MAG
  Serial.print(mag_x);
  Serial.print(",");
  Serial.print(mag_y);
  Serial.print(",");
  Serial.println(mag_z);
#endif

  heading = angle_yaw;
#ifndef DEBUG_HEADING
  Serial.print((uint16_t) compass_heading);
  Serial.print(", ");
  Serial.println((uint16_t) heading);
#endif
}
