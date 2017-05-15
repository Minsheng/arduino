void setup() {
  // put your setup code here, to run once:

}

long sensors[] = {0,0,0,0,0};
int sensor_avg = 0;
int sensor_sum = 0;
int bot_pos = 0;
int proportional = 0;
int integral = 0;
int derivative = 0;
int set_point = 0;
int last_prop = 0;
int error_val = 0;

int Kd = 0;
int Ki = 0;
int Kp = 0;

int max_speed = 255;
int right_speed = 0;
int left_speed = 0;

void loop() {
  sensor_avg = 0;
  sensor_sum = 0;
  for (int i=0; i<5; i++) {
    sensors[i] = analogRead(i);
    sensor_avg += sensors[i]*i*1000; // calculate weighted mean
    sensor_sum += int(sensors[i]);
  }
}

void pid_calc() {
  bot_pos = int (sensor_avg / sensor_sum);
  proportional = bot_pos - set_point; // replace by our own set_point
  integral = integral + proportional;
  derivative =  proportional - last_prop;
  last_prop = proportional;
  // need to define Kp, Ki, and Kd for error calculation
  error_val = int (proportional*Kp + integral*Ki + derivative*Kd);
}

void calc_turn() {
  if (error_val < -256) {
    error_val = -256;
  }

  if (error_val > 256) {
    error_val = 256;
  }

  // calculate right turn speed
  if (error_val < 0) {
    right_speed = max_speed + error_val;
    left_speed = max_speed;
  } else {
    right_speed = max_speed;
    left_speed = max_speed - error_val;
  }
}



