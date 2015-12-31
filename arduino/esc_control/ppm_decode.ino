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
}
unsigned long start;
uint8_t state = 0, i;
volatile uint8_t ch_num=0; //current channel
volatile unsigned long last_pulse, this_pulse; //this_pulse - last_pulse = (previous)width
volatile unsigned int command[CHANNELS], width;

void loop() {
  if (millis() > 100 + start){
    start = millis();
    Serial.print(command[0]);
    for (i=1; i<6; i++){
      Serial.print('\t');
      Serial.print(command[i]);
    }
    Serial.print('\n');
    //Serial.println(millis() - start);
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
