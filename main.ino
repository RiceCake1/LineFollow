#define ML1 2
#define ML2 3
#define MR1 1
#define MR2 4
#define MP1 4
#define MP2 5
#define MPDA 10
#define MPDB 9
#define MPDC 8
#define LED 13

float gearRatio;
float wheelDiameter;

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

void SelectMultiplexer(int Dec){
  int Bin[3];
  ChangeBin(Dec, Bin);
  digitalWrite(MPDC, Bin[2]);
  digitalWrite(MPDB, Bin[1]);
  digitalWrite(MPDA, Bin[0]);
}

int ReadSensor(int sensorNum){
  SelectMultiplexer(sensorNum);
  return analogRead(MP1);
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


float _getSpeed( int encoderNum ){
  SelectMultiplexer(encoderNum);
  int pulseWidth = pulseIn(MP2, HIGH, 50000);

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

void initAll(){
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

  gearRatio = 18.8;
  wheelDiameter = 5.6;
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  initAll();
}

void loop() {
  float spd[2];
  getSpeed(spd);
  Serial.print(spd[0]);
  Serial.print(", ");
  Serial.println(spd[1]);
}