#define LED 13
#define SIGNAL_OUTPIN 0

int sensorPins[] = {3,9,10,2,1,4,6,5};
float arctan7_13;

float mapFloat(float InputValue, float InputLower, float InputUpper, float OutputLower, float OutputUpper){
  float value = (OutputUpper-OutputLower)*(InputValue - InputLower) / (InputUpper - InputLower) + OutputLower;
  return max(min(value, OutputUpper), OutputLower);
}

float ReadSensorBinary(int sensorNum){
  int state = 0;
  if( analogRead(sensorPins[sensorNum]) >1020) state = 1;
  return state;
}

float sensorDistance[] = {-7, -5, -3, -1, 1, 3, 5, 7};
float getThetaWithArctan(){
  float theta = 0;
  float distance = 0;
  int sensorBlackCount = 0;
  for(int i=0; i<8; i++){
    int sensorValue = ReadSensorBinary(i);
    distance += sensorValue*sensorDistance[i];
    sensorBlackCount += sensorValue;
  }
  if(sensorBlackCount>0){
    distance = distance/sensorBlackCount;
    theta = atan(distance/13);
    return theta;
  }else{
    return 0;
  }
}

void initAll(){
  pinMode(LED, OUTPUT);
  pinMode(SIGNAL_OUTPIN, OUTPUT);
  pinMode(sensorPins[0], INPUT);
  pinMode(sensorPins[1], INPUT);
  pinMode(sensorPins[2], INPUT);
  pinMode(sensorPins[3], INPUT);
  pinMode(sensorPins[4], INPUT);
  pinMode(sensorPins[5], INPUT);
  pinMode(sensorPins[6], INPUT);
  pinMode(sensorPins[7], INPUT);
  arctan7_13 = atan(7.0/13.0);
  
}

void setup(){
  initAll();
}

void loop(){
  analogWrite(SIGNAL_OUTPIN, mapFloat(getThetaWithArctan(),-arctan7_13,arctan7_13,0,700));
}