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

int sensorCallibrateMin[8] = {1755, 1455, 2091, 2015, 2241, 1875, 2014, 2262};
int sensorCallibrateMax[8]= {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};



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


void SelectMultiplexer(int Dec){
  int Bin[3];
  ChangeBin(Dec, Bin);
  digitalWrite(MPDC, Bin[2]);
  digitalWrite(MPDB, Bin[1]);
  digitalWrite(MPDA, Bin[0]);
}

int ReadSensor(int sensorNum){
  SelectMultiplexer(sensorNum);
  delay(1);
  return analogRead(MP1);
}

float ReadSensorNormalized(int sensorNum){
  return mapZeroToOne(sensorCallibrateMin[sensorNum], sensorCallibrateMax[sensorNum], ReadSensor(sensorNum));
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

int _getEncoderState(int encoderPairNum){
  byte encodeState;
  int encoder[2];
  encoder[0] = ReadEncoder(encoderPairNum*2);
  encoder[1] = ReadEncoder(encoderPairNum*2+1);
  bitWrite(encodeState,0, encoder[0]);
  bitWrite(encodeState,1, encoder[1]);
  return int(encodeState);//0 = 00 1=01 2=10 3=11
}

// int prevEncoderState[2];
// int prevPulseTime[2];
// float _getSpeed(int motorNum){
//   int encoderState = _getEncoderState(motorNum);
//   int nowTime = micros();


//   prevEncoderState[motorNum] = encoderState;
// }

float _getSpeed( int encoderNum ){
  SelectMultiplexer(encoderNum);
  int pulseWidth = pulseIn(MP2, HIGH, 40000);
  if(pulseWidth==0) return 0;
  int pulsePerRotate = 11;
  float MotorFreq = 1000000/(pulseWidth*2*pulsePerRotate);
  float enshu = wheelDiameter*PI;//cm
  float speed = MotorFreq/gearRatio*enshu;//cm/s
  return speed;
}

void getSpeed(float *spd){
  spd[0] = _getSpeed(0);
  spd[1] = _getSpeed(2);
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

int getLinePos(){
  int linePos = 0;
  for(int i=0; i<8; i++){
    int weight = i-3;
    if(weight<=0) weight--;
    linePos += ReadSensorNormalized(i)*weight;
  }
  return linePos;
}


float sensorErrorPrev;
float sensorErrorSum;
float getPIDsensorValue(){
  float skP = 0.2;
  float skI = 0.02;
  float skD = 0.01;
  int linePos = getLinePos();
  float error = linePos;
  sensorErrorSum += error;
  float PIDvalue = error*skP ;//+ (sensorErrorPrev-error)*skD + sensorErrorSum*skI;
  sensorErrorPrev = error;
  Serial.print("PIDSensor: ");
  Serial.println(PIDvalue);
  return PIDvalue;
}


float spdPrev[2]; 
float spdErrorPrev[2];
float spdErrorSum[2];


void controlMotorWithPID(float targetSpeed, bool sensorEnabled=true){
  float mkP = 0.02;
  float mkI = 0.01;
  float mkD = 0.001;
  //float mkI = 0.0;
  //float mkD = 0.00;
  
  float spd[2];
  float spdError[2];
  getSpeed(spd);

  float sensorPIDValue;
  if(sensorEnabled){
    sensorPIDValue = getPIDsensorValue();
  }else{
    sensorPIDValue = 0;
  }
  
  spdError[RIGHT] = targetSpeed - spd[RIGHT];
  spdError[LEFT] = targetSpeed - spd[LEFT];
  spdErrorSum[RIGHT] += spdError[RIGHT];
  spdErrorSum[LEFT] += spdError[LEFT];
  float pwrR = spdError[RIGHT]*mkP + (spdErrorPrev[RIGHT]-spdError[RIGHT])*mkD + spdErrorSum[RIGHT]*mkI - sensorPIDValue;
  float pwrL = spdError[LEFT]*mkP + (spdErrorPrev[LEFT]-spdError[LEFT])*mkD + spdErrorSum[LEFT]*mkI + sensorPIDValue;
  if(pwrR<0) pwrR = 0;
  if(pwrL<0) pwrL = 0;
  spdErrorPrev[RIGHT] = spdError[RIGHT];
  spdErrorPrev[LEFT] = spdError[LEFT];
  ControlMotor(pwrR, pwrL);
  Serial.print(pwrR);
  Serial.print(",");
  Serial.println(pwrL);
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


void showSpeed(){
  float spd[2];
  getSpeed(spd);
  Serial.print("speed: ");
  Serial.print(spd[0]);
  Serial.print(",");
  Serial.println(spd[1]);
}

void showLinePos(){
  Serial.println(getLinePos());
}

int calcLinearSpeed(int targetSpeed){
  float spd[2];
  getSpeed(spd);
  int speed = min(spd[0]+10, targetSpeed);
  return speed;
}



void initAll(){
  //define hardware
  Tracer Mochi;
  Mochi.setup(18.8, sensorCallibrateMin, sensorCallibrateMax);

  Tracer Kazushi;
  Kazushi.setup(10.0, sensorCallibrateMin, sensorCallibrateMax);
  
  //select which to write
  Tracer machine = Kazushi;

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

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  initAll();
}

void loop() {
  controlMotorWithPID(calcLinearSpeed(30),SENSOR_DISABLED);
  showSpeed();
  //getPIDsensorValue();
  //supportCallibration();
  //showSensorNormalized();
  //Serial.println(ReadSensor(0));
  //Serial.println(getPIDsensorValue());

}