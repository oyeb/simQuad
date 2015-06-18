#define FRAME_SIZE 100000//us
#define DEAD_THRESH 3000//us
/*
FOR 6 CHANNELS ONLY:
anything more than 2000us should be dead zone.
deadzone can occur during channel data!
~5000us dead zone

FOR OTHER CHANNELS:
<no data>
*/
#define CHANNELS 6

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(57600);
  attachInterrupt(0, ppmDecode, FALLING); //pin2 on MEGA-2560 and UNO
  Serial.println("Hit 'c' to calibrate.");
}
unsigned long start;
uint8_t calibrated = 0, i; //is the transmitter calibrated?
volatile uint8_t ch_num=0; //current channel
volatile unsigned long last_pulse, this_pulse; //this_pulse - last_pulse = (previous)width
volatile unsigned int command[CHANNELS], width;
unsigned int command_limits[8];


void loop() {
  if (millis() > 100 + start){
    start = millis();
      Serial.print(command[0]);
      for (i=1; i<6; i++){
        Serial.print('\t');
        Serial.print(command[i]);
      }
      Serial.print('\n');
  }
  if (Serial.available() > 0){
    Serial.read();
    calibrate();
  }
}

void ppmDecode(){
  this_pulse = micros();
  /*
  state = ~state;
  digitalWrite(13, state);
  */
  width = this_pulse - last_pulse;
  last_pulse = this_pulse;
  if (width > DEAD_THRESH || ch_num == CHANNELS)
    ch_num = 0;
  else{
    command[ch_num] = width;
    ch_num++;
  }
}

void calibrate(){
  /*
  Calibrates only the 4 main channels.
  */
  if (!calibrated){
    Serial.println("Hit anything (but only one key) to proceed.");
    Serial.print("Move CH1 {Aileron} to nil {LEFT}");
    while (Serial.available() == 0);
    Serial.read();
    command_limits[0] = command[0];
    Serial.println(" OK");
    Serial.print("Move CH1 {Aileron} to max {RIGHT}");
    while (Serial.available() == 0);
    Serial.read();
    command_limits[4] = command[0];
    Serial.println(" OK");
    
    Serial.print("Move CH2 {Elevator} to nil {UP}");
    while (Serial.available() == 0);
    Serial.read();
    command_limits[1] = command[1];
    Serial.println(" OK");
    Serial.print("Move CH2 {Elevator} to max {DOWN}");
    while (Serial.available() == 0);
    Serial.read();
    command_limits[5] = command[1];
    Serial.println(" OK");

    Serial.print("Move CH3 {Throttle} to nil {DOWN}");
    while (Serial.available() == 0);
    Serial.read();
    command_limits[2] = command[2];
    Serial.println(" OK");
    Serial.print("Move CH3 {Throttle} to max {UP}");
    while (Serial.available() == 0);
    Serial.read();
    command_limits[6] = command[2];
    Serial.println(" OK");

    Serial.print("Move CH4 {Rudder} to nil {LEFT}");
    while (Serial.available() == 0);
    Serial.read();
    command_limits[3] = command[3];
    Serial.println(" OK");
    Serial.print("Move CH4 {Rudder} to max {RIGHT}");
    while (Serial.available() == 0);
    Serial.read();
    command_limits[7] = command[3];
    Serial.println(" OK");
    //for safety, do this:
    Serial.print("Move throttle back to nil {DOWN})");
    while (command[2] > command_limits[2]);
    Serial.println(" OK");
    calibrated = 1;
  }
  Serial.println("Limits Are:");
  Serial.print(command_limits[0]);
  Serial.print('\t');
  Serial.print(command_limits[1]);
  Serial.print('\t');
  Serial.print(command_limits[2]);
  Serial.print('\t');
  Serial.print(command_limits[3]);
  Serial.print('\n');
  Serial.print(command_limits[4]);
  Serial.print('\t');
  Serial.print(command_limits[5]);
  Serial.print('\t');
  Serial.print(command_limits[6]);
  Serial.print('\t');
  Serial.print(command_limits[7]);
  Serial.print('\n\n');
}

/*
1196  1208  1004  1196
1776  1808  2004  1796

1192  1212  1004  1196
1776  1808  2004  1792

1196  1212  1008  1196
1772  1812  2004  1792
*/