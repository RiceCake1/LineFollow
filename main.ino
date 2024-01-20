#define ML1 2
#define ML2 3
#define MR1 1
#define MR2 4
#define MP1 0
#define MP2 5
#define MPDA 10
#define MPDB 9
#define MPDC 8
#define LED 13

#define RIGHT 0
#define  LEFT 1

#define SENSOR_DISABLED false

float gearRatio;
float wheelDiameter;

int sensorCallibrateMin[8] = {1543, 1058, 1014, 1054, 1525, 1050, 992, 1132};
int sensorCallibrateMax[8]= {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};

int isActive = true;

float currentSpeed[2] = {0,0};

class Tracer{
  public:
    float gearRatio;
    int sensorCallibrateMin[8];
    int sensorCallibrateMax[8];

    void setup(float gearRatio, int sensorCallibrateMin[8], int sensorCallibrateMax[8]);
};

void Tracer::setup(float gearRatio, int sensorCallibrateMin[8], int sensorCallibrateMax[8]){
  this->gearRatio = gearRatio;
  for(int i = 0; i < 8; i++){
    this->sensorCallibrateMin[i] = sensorCallibrateMin[i];
    this->sensorCallibrateMax[i] = sensorCallibrateMax[i];
  }
}


void ChangeBin(int Dec, int *Bin){
if(Dec>7){
  Serial.println("Error: ChangeBin() argument error");
  }else{
    for (int i = 0; i < 3; i++) {
      Bin[i] = Dec % 2;
      Dec = Dec / 2;
    }
  }
}
float mapZeroToOne(int InputLower, int InputUpper, int InputValue){
  float value = (float(InputValue) - float(InputLower)) / (float(InputUpper) - float(InputLower));
  return max(min(value, 1.0), 0.0);
}

//need 3 microseconds
void SelectMultiplexer(int Dec){
  int Bin[3];
  ChangeBin(Dec, Bin);
  digitalWrite(MPDC, Bin[2]);
  digitalWrite(MPDB, Bin[1]);
  digitalWrite(MPDA, Bin[0]);
  delayMicroseconds(3);
}

int ReadSensor(int sensorNum){
  SelectMultiplexer(sensorNum);
  return analogRead(MP1);
}

float ReadSensorNormalized(int sensorNum){
  return mapZeroToOne(sensorCallibrateMin[sensorNum], sensorCallibrateMax[sensorNum], ReadSensor(sensorNum));
}

float ReadSensorBinary(int sensorNum){
  int state = 0;
  if(ReadSensor(sensorNum)==4095) state = 1;
  return state;
}

int ReadEncoder(int encoderNum){
  SelectMultiplexer(encoderNum);
  return digitalRead(MP2);
}


void ControlMotor(float Rspeed, float Lspeed){ //argument range -1 to 1
  if(Rspeed>=0){
    analogWrite(MR1, int(255*Rspeed));
    analogWrite(MR2, 0);
  }else{
    analogWrite(MR2, int(255*(-Rspeed)));
    analogWrite(MR1, 0);
  }

  if(Lspeed>=0){
    analogWrite(ML1, int(255*Lspeed));
    analogWrite(ML2, 0);
  }else{
    analogWrite(ML2, int(255*(-Lspeed)));
    analogWrite(ML1, 0);
  }
}

int prevEncoderState[4] = {0,0,0,0};
int _smoothEncoderSignal(int encoderNum){
  while(true){
    int encoderSignals[3] = {0,0,0};
    for(int i = 0; i < 3; i++){
      encoderSignals[i] = ReadEncoder(encoderNum);
    }
    if(encoderSignals[0]==encoderSignals[1] && encoderSignals[1]==encoderSignals[2]){
      return encoderSignals[0];  
    }
  }
}

bool _isChanged(int motorNum){ //argument range 0 or 1
  int encoder1 = motorNum*2;
  int encoderState = _smoothEncoderSignal(encoder1);
  bool changed = false;
  if(prevEncoderState[encoder1]==0 && encoderState==1){
    changed = true;
  }
  prevEncoderState[encoder1] = encoderState;
  return changed;
}

int prevSignalTime[2] = {0,0};
float _getSpeedWithTime(int motorNum){
  int nowTime = micros();
  int deadLineTime = 50000;
  int deltaTime = nowTime - prevSignalTime[motorNum];
  if(_isChanged(motorNum)){
    int pulsePerRotate = 11;
    float MotorFreq = 1000000/(deltaTime*pulsePerRotate);
    float enshu = wheelDiameter*PI;//cm
    float speed = MotorFreq/gearRatio*enshu;//cm/s
    prevSignalTime[motorNum] = nowTime;
    return speed;
  }else if(deltaTime > deadLineTime){
    return 0;
  }else{
    return currentSpeed[motorNum];
  }
}

void getSpeedWithTime(float *spd){
  spd[RIGHT] = _getSpeedWithTime(RIGHT);
  spd[LEFT] = _getSpeedWithTime(LEFT);
}

// int _getEncoderState(int encoderPairNum){
//   byte encodeState;
//   int encoder[2];
//   encoder[0] = ReadEncoder(encoderPairNum*2);
//   encoder[1] = ReadEncoder(encoderPairNum*2+1);
//   bitWrite(encodeState,0, encoder[0]);
//   bitWrite(encodeState,1, encoder[1]);
//   return int(encodeState);//0 = 00 1=01 2=10 3=11
// }

// int prevEncoderState[2];
// int prevPulseTime[2];
// float _getSpeed(int motorNum){
//   int encoderState = _getEncoderState(motorNum);
//   int nowTime = micros();


//   prevEncoderState[motorNum] = encoderState;
// }

// bool virtualInputSignal = 0;
// bool mimicInputSignal(int MotorNum){
//   if(_isChanged(MotorNum)) virtualInputSignal = !virtualInputSignal;
//   return virtualInputSignal;
// }

// float getSpeedFromVirtualSignal(int MotorNum){
//   int pulseWidth = pulseIn(mimicInputSignal, HIGH, 10000);
//   if(pulseWidth==0) return 0;
//   int pulsePerRotate = 11;
//   float MotorFreq = 1000000/(pulseWidth*2*pulsePerRotate);
//   float enshu = wheelDiameter*PI;//cm
//   float speed = MotorFreq/gearRatio*enshu;//cm/s
//   return speed;
// }

float _getSpeed( int encoderNum ){
  SelectMultiplexer(encoderNum);
  int pulseWidth = pulseIn(MP2, HIGH, 10000);
  if(pulseWidth==0) return 0;
  int pulsePerRotate = 11;
  float MotorFreq = 1000000/(pulseWidth*2*pulsePerRotate);
  float enshu = wheelDiameter*PI;//cm
  float speed = MotorFreq/gearRatio*enshu;//cm/s
  return speed;
}

void getSpeed(float *spd){
  spd[0] = _getSpeed(0);
  spd[1] = _getSpeed(3);
}


int supportCallibrationSensorMin[8] = {10000,10000,10000,10000,10000,10000,10000,10000}; //BIG VALUE
int supportCallibrationSensorMax[8] = {0,0,0,0,0,0,0,0}; //SMALL VALUE 
void supportCallibration(){
  for(int i=0; i<8; i++){
    supportCallibrationSensorMin[i] = min(supportCallibrationSensorMin[i], ReadSensor(i));
    supportCallibrationSensorMax[i] = max(supportCallibrationSensorMax[i], ReadSensor(i));
  }
  Serial.print("Min: ");
  for(int i=0; i<8; i++){
    Serial.print(supportCallibrationSensorMin[i]);
    Serial.print(", ");
    } 
  Serial.println();
  Serial.print("Max: ");
  for(int i=0; i<8; i++){
    Serial.print(supportCallibrationSensorMax[i]);
    Serial.print(", ");
  } 
  Serial.println();
}

void supportCallibrationAverage(){
  int sensorData[8] = {0,0,0,0,0,0,0,0};
  for(int i=0; i<100; i++){
    for (int j = 0; j < 8; j++){
      sensorData[j] += ReadSensor(j);
      //Serial.print(sensorData[j]);
    }
    delay(50);
  }
  int sensorAverage[8] = {0,0,0,0,0,0,0,0};
  for(int i=0; i<8; i++){
    sensorAverage[i] = sensorData[i]/100;
    Serial.print(sensorAverage[i]);
    Serial.print(", ");
  }
  Serial.println();
}

int originalpow2(int x, int y){
  if(y==0) return 1;
  if(y<0) return -pow(x, -y);
  return pow(x, y);
}

int getLinePos(){
  int linePos = 0;
  for(int i=0; i<8; i++){
    int weight = i-3;
    if(weight<=0) weight--;
    linePos += ReadSensorBinary(i)*weight;
  }
  return linePos;
}


float sensorErrorPrev;
float sensorErrorSum;
float getPIDsensorValue(){
  // float skP = 0.18;
  // float skI = 0.05;
  // float skD = 0.2;
  float skP = 0.8;
  float skI = 0.01;
  float skD = 0.0;
  //float skD = 0.0;
  int linePos = getLinePos();
  float error = linePos;
  sensorErrorSum += error;
  float PIDvalue = error*skP + sensorErrorSum*skI + (sensorErrorPrev-error)*skD;
  sensorErrorPrev = error;
  return PIDvalue;
}


float spdPrev[2]; 
float spdErrorPrev[2];
float spdErrorSum[2];
void controlMotorWithPID(float targetSpeed, bool sensorEnabled=true){
  float mkP = 0.007;
  float mkI = 0.000;
  float mkD = 0.000;
  //float mkI = 0.0;
  //float mkD = 0.00;
  
  float spdError[2];

  float sensorPIDValue;
  if(sensorEnabled){
    sensorPIDValue = getPIDsensorValue();
  }else{
    sensorPIDValue = 0;
  }
  
  spdError[RIGHT] = targetSpeed - currentSpeed[RIGHT];
  spdError[LEFT] = targetSpeed - currentSpeed[LEFT];
  spdErrorSum[RIGHT] += spdError[RIGHT];
  spdErrorSum[LEFT] += spdError[LEFT];
  float pwrR = spdError[RIGHT]*mkP + (spdErrorPrev[RIGHT]-spdError[RIGHT])*mkD + spdErrorSum[RIGHT]*mkI - sensorPIDValue;
  float pwrL = spdError[LEFT]*mkP + (spdErrorPrev[LEFT]-spdError[LEFT])*mkD + spdErrorSum[LEFT]*mkI + sensorPIDValue;
  if(pwrR<0) pwrR = 0;
  if(pwrL<0) pwrL = 0;
  spdErrorPrev[RIGHT] = spdError[RIGHT];
  spdErrorPrev[LEFT] = spdError[LEFT];
  ControlMotor(pwrR, pwrL);
}

void controlEachMotorWithPID(float rightTargetSpeed, float leftTargetSpeed){
  float mkP = 0.07;
  float mkI = 0.008;
  float mkD = 0.003;
  
  float spdError[2];
  
  spdError[RIGHT] = rightTargetSpeed - currentSpeed[RIGHT];
  spdError[LEFT] = leftTargetSpeed - currentSpeed[LEFT];
  spdErrorSum[RIGHT] += spdError[RIGHT];
  spdErrorSum[LEFT] += spdError[LEFT];
  float pwrR = spdError[RIGHT]*mkP + (spdErrorPrev[RIGHT]-spdError[RIGHT])*mkD + spdErrorSum[RIGHT]*mkI;
  float pwrL = spdError[LEFT]*mkP + (spdErrorPrev[LEFT]-spdError[LEFT])*mkD + spdErrorSum[LEFT]*mkI;
  if(pwrR<0) pwrR = 0;
  if(pwrL<0) pwrL = 0;
  spdErrorPrev[RIGHT] = spdError[RIGHT];
  spdErrorPrev[LEFT] = spdError[LEFT];
  ControlMotor(pwrR, pwrL);
}

void controlMotorFromSensors(float targetSpeed){
  float sensorPIDValue = getPIDsensorValue()*7;
  float rightSpeed = targetSpeed - sensorPIDValue;
  float leftSpeed = targetSpeed + sensorPIDValue;
  controlEachMotorWithPID(rightSpeed, leftSpeed);
}

void showSensorNormalized(){
  for(int i=0; i<8; i++){
    Serial.print(ReadSensorNormalized(i));
    Serial.print(", ");
  }
  Serial.println();
}

void showSensor(){
  for(int i=0; i<8; i++){
    Serial.print(ReadSensor(i));
    Serial.print(", ");
  }
  Serial.println();
}

void showSensorBinary(){
  for(int i=0; i<8; i++){
    Serial.print(ReadSensorBinary(i));
    Serial.print(", ");
  }
  Serial.println();
}

void showSpeed(){
  Serial.print("speed: ");
  Serial.print(currentSpeed[0]);
  Serial.print(",");
  Serial.println(currentSpeed[1]);
}

void showLinePos(){
  Serial.println(getLinePos());
}

int initialMillis;
int endMillis;
int loopCount = 0;
void showLoopSpeed(){
  if(loopCount==0){
    initialMillis = millis();
  }
  loopCount++;
  
  if(loopCount==5000){
    endMillis = millis();
    Serial.print("Loop Speed (1K iters): ");
    Serial.print((endMillis-initialMillis));
    Serial.println("ms");
    loopCount = 0;
    showSpeed();
  }
}

float calcLinearSpeed(float targetSpeed, float currentSpeed = currentSpeed[0], float maxError=20){
  if(targetSpeed >= currentSpeed){
    return min(currentSpeed + maxError, targetSpeed); 
  }else{
    return max(currentSpeed - maxError, targetSpeed);
  }
}



void initAll(){
  //define hardware
  Tracer Mochi;
  Mochi.setup(18.8, sensorCallibrateMin, sensorCallibrateMax);

  Tracer Kazushi;
  Kazushi.setup(10.0, sensorCallibrateMin, sensorCallibrateMax);
  

  //select which to write
  Tracer machine = Mochi;

  //global variables initialize
  currentSpeed[RIGHT] = 0;
  currentSpeed[LEFT] = 0;
  sensorErrorPrev = 0;
  sensorErrorSum = 0;
  spdErrorPrev[RIGHT] = 0;
  spdErrorPrev[LEFT] = 0;
  spdErrorSum[RIGHT] = 0;
  spdErrorSum[LEFT] = 0;
  initialMillis = 0;
  endMillis = 0;
  loopCount = 0;
  
  //Motors init
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  analogWrite(MR1, 0);
  analogWrite(MR2, 0);
  analogWrite(ML1, 0);
  analogWrite(ML2, 0);

  //multiplexer init
  pinMode(MPDC, OUTPUT);
  pinMode(MPDB, OUTPUT);
  pinMode(MPDA, OUTPUT);
  digitalWrite(MPDC, 0);
  digitalWrite(MPDB, 0);
  digitalWrite(MPDA, 0);

  //multiplexer input pins init
  pinMode(MP1, INPUT);
  pinMode(MP2, INPUT);

  //set analogread resolution
  analogReadResolution(12);
  
  //set the hardware
  gearRatio = machine.gearRatio;
  wheelDiameter = 5.6;

  for(int i = 0; i<8; i++){
    sensorCallibrateMax[i] = machine.sensorCallibrateMax[i];
    sensorCallibrateMin[i] = machine.sensorCallibrateMin[i];
  }

}

void Xbee(){
  if(Serial1.available()){
    char c = Serial1.read();
    switch (c){
      case 's':
      initAll();
      isActive = false;
      break;

      case 'b':
      initAll();
      isActive = true;
      break;

      
      default: break;

    }
  }
}

void setup() {
  Serial.begin(250000);
  Serial1.begin(250000);
  initAll();
  ControlMotor(0.42,0);
}

void loop() {
  Xbee();
  if (isActive){
    //getSpeed(currentSpeed);
    getSpeedWithTime(currentSpeed);
    controlMotorWithPID(calcLinearSpeed(50), SENSOR_DISABLED);
    //supportCallibrationAverage();
    showLoopSpeed();
    //controlMotorFromSensors(50);
    //supportCallibration();
    //showSpeed();
    //showSensor();
    //showSensorBinary();
    //Serial.println(ReadSensor(0));
    //Serial.println(getPIDsensorValue());
    //showSensor();
    //ReadSensor(0);
  }
}