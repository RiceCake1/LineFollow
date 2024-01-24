#define ML1 2
#define ML2 3
#define MR1 1
#define MR2 4
#define SENSOR_PIN 0
#define ENCODER_L 5
#define ENCODER_R 10
#define LED 13

#define RIGHT 0
#define LEFT 1

#define SENSOR_DISABLED false

float gearRatio;
float wheelDiameter;

int ENCODER_PIN[2] = {ENCODER_R, ENCODER_L};

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

float mapFloat(float InputValue, float InputLower, float InputUpper, float OutputLower, float OutputUpper){
  float value = (OutputUpper-OutputLower)*(InputValue - InputLower) / (InputUpper - InputLower) + OutputLower;
  return max(min(value, OutputUpper), OutputLower);
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

int prevEncoderState[2] = {0,0};
int _smoothEncoderSignal(int encoderNum){
  while(true){
    int encoderSignals[3] = {0,0,0};
    for(int i = 0; i < 3; i++){
      encoderSignals[i] = digitalRead(ENCODER_PIN[encoderNum]);
    }
    if(encoderSignals[0]==encoderSignals[1] && encoderSignals[1]==encoderSignals[2]){
      return encoderSignals[0];  
    }
  }
}

bool _isChanged(int encoderNum){ //argument range 0 or 1
  int encoderState = _smoothEncoderSignal(encoderNum);
  bool changed = false;
  if(prevEncoderState[encoderNum]==0 && encoderState==1){
    changed = true;
  }
  prevEncoderState[encoderNum] = encoderState;
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


float calcLinearSpeed(float targetSpeed, float currentSpeed = currentSpeed[0], float maxError=20){
  if(targetSpeed >= currentSpeed){
    return min(currentSpeed + maxError, targetSpeed); 
  }else{
    return max(currentSpeed - maxError, targetSpeed);
  }
}

int originalpow2(int x, int y){
  if(y==0) return 1;
  if(y<0) return -pow(x, -y);
  return pow(x, y);
}

int getLinePos(){
  float linePos = mapFloat(float(analogRead(SENSOR_PIN)),7.0f,673.0f,-10.0f,10.0f);
  return linePos;
}


float sensorErrorPrev;
float sensorErrorSum;
float getPIDsensorValue(int linePos){
  float skP = 7.8;
  float skI = 0.0001;
  float skD = 8.5;
  float error = linePos;
  sensorErrorSum += error;
  float PIDvalue = error*skP + sensorErrorSum*skI + (sensorErrorPrev-error)*skD;
  sensorErrorPrev = error;
  return PIDvalue;
}


float spdPrev[2]; 
float spdErrorPrev[2];
float spdErrorSum[2];
float power[2];
void controlMotorWithPID(float targetSpeed, bool sensorEnabled=true){
  float mkP = 0.04;
  float mkI = 0.0015;
  float mkD = 0.05;
  
  float spdError[2];

  float sensorPIDValue;
  if(sensorEnabled){
    sensorPIDValue = getPIDsensorValue(getLinePos());
  }else{
    sensorPIDValue = 0;
  }
  
  float rightSpeed = targetSpeed - sensorPIDValue;
  float leftSpeed = targetSpeed + sensorPIDValue;
  controlEachMotorWithPID(rightSpeed, leftSpeed);
}

void controlEachMotorWithPID(float rightTargetSpeed, float leftTargetSpeed){
  float mkP = 0.04;
  float mkI = 0.000007;
  float mkD = 0.05;
  
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

float limitSpeedOnCorner(float targetSpeed, float deacceleratedSpeed, int linePositon){
  float spdDelta = (targetSpeed - deacceleratedSpeed)/6;
  float spd = targetSpeed - spdDelta*abs(float(linePositon));
  return spd;
}

void controlMotorFromSensors(float targetSpeed, float deacceleratedSpeed=40){
  int linePos = getLinePos();
  float sensorPIDValue = getPIDsensorValue(linePos);
  float rightSpeed = targetSpeed - sensorPIDValue;
  float leftSpeed = targetSpeed + sensorPIDValue;
  controlEachMotorWithPID(rightSpeed, leftSpeed);
}

void showSpeed(){
  //Serial1.print("Speed: ");
  Serial1.print(currentSpeed[RIGHT]);
  Serial1.print(", ");
  Serial1.print(80);
  Serial1.print(", ");
  Serial1.println(20);
}

void showLinePos(){
  Serial.println(getLinePos());
}

int initialMillis;
int endMillis;
int loopCount = 0;
void showLoopSpeed(int loopNum){
  if(loopCount==0){
    initialMillis = millis();
  }
  loopCount++;
  
  if(loopCount==loopNum){
    endMillis = millis();
    // Serial.print("Loop Speed ( ");
    // Serial.print(loopNum);
    // Serial.print(" iters): ");
    // Serial.print((endMillis-initialMillis));
    // Serial.println("ms");
    loopCount = 0;
    showSpeed();
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

  //encoder init
  pinMode(ENCODER_L, INPUT);
  pinMode(ENCODER_R, INPUT);

  //multiplexer input pins init
  pinMode(SENSOR_PIN, INPUT);
  
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
  Serial.begin(9600);
  Serial1.begin(9600);
  initAll();
  //ControlMotor(0.3,0);
}

void loop() {
  Xbee();
  if (isActive){
    //getLinePos();
    //getSpeed(currentSpeed);
    getSpeedWithTime(currentSpeed);
    controlMotorFromSensors(50); 
    //controlMotorWithPID(calcLinearSpeed(50), SENSOR_DISABLED);
    //supportCallibrationAverage();
    //supportCallibration();
    //showSpeed();
    //showSensor();
    //showSensorBinary();
    //Serial.println(ReadSensor(1));
    //Serial.println(getPIDsensorValue());
    //showSensor();
    //ReadSensor(0);
  }
  showLoopSpeed(1000);
}