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

int LSensor(int sensorNum){
  SelectMultiplexer(sensorNum);
  return MP1;
}

int ReadEncoder(int encoderNum){
  SelectMultiplexer(encoderNum);
  return digitalRead(MP2);
}

void ControlMotor(float R, float L){ //range -1 to 1
  if(R>=0){
    analogWrite(MR1, int(256*R));
    analogWrite(MR2, 0);
  }else{
    analogWrite(MR2, int(256*(-R)));
    analogWrite(MR1, 0);
  }

  if(L>=0){
    analogWrite(ML1, int(256*L));
    analogWrite(ML2, 0);
  }else{
    analogWrite(ML2, int(256*(-L)));
    analogWrite(ML1, 0);
  }
}

int flgR = 0;
int flgL = 0;
unsigned int microSec[] = {0,0,0,0};
int MS[4][2] = {{0,0},{0,0},{0,0},{0,0}};
int spdR;
int spdL;
void updateSpeed(){
  int MSensor_stream[4][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  for(int i = 0; i < 4; i++){
    for(int k = 0; k < 2; k++){
      MSensor_stream[i][k] = ReadEncoder(i);
      delayMicroseconds(3);
    }
    MSensor_stream[i][2] = ReadEncoder(i);
    MS[i][1] = MS[i][0];
    if(MSensor_stream[i][0]==MSensor_stream[i][1] && MSensor_stream[i][1]==MSensor_stream[i][2]){
      MS[i][0] = MSensor_stream[i][0];
    }else{
      MS[i][0] = MS[i][1];
    }
  }

  if(!flgR){
    if(MS[0][0]==1 && MS[0][1]==0){
      flgR = 1;
      microSec[0] = micros();
    }else if(MS[1][0]==1 && MS[1][1]==0){
      flgR = 1;
      microSec[1] = micros();
    }
  }else{
    if(MS[1][0]==1 && MS[1][1]==0 && microSec[1]==0){
      flgR = 0;
      spdR = micros() - microSec[0];
      if(spdR > 50000 | spdR < -50000){spdR = 0;}
      microSec[0] = 0;
      microSec[1] = 0;
    }else if(MS[0][0]==1 && MS[0][1]==0 && microSec[0]==0){
      flgR = 0;
      spdR = microSec[1] - micros();
      if(spdR > 50000 | spdR < -50000){spdR = 0;}
      microSec[0] = 0;
      microSec[1] = 0;
    }
  }
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
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  initAll();
}

void loop() {
  updateSpeed();

  Serial.print(spdR);
  Serial.print(", ");
  Serial.println(spdL);

}