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

int MRS1;
int MRS2;
int MLS1;
int MLS2;

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

int LSensor(int num){
  int Bin[3];
  ChangeBin(num, Bin);
  digitalWrite(MPDC, Bin[2]);
  digitalWrite(MPDB, Bin[1]);
  digitalWrite(MPDA, Bin[0]);
  return analogRead(MP1);
}

int MSensor(int num){
  int Bin[3];
  ChangeBin(num, Bin);
  digitalWrite(MPDC, Bin[2]);
  digitalWrite(MPDB, Bin[1]);
  digitalWrite(MPDA, Bin[0]);
  return MP2;
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
    analogWrite(ML1, int(256L));
    analogWrite(ML2, 0);
  }else{
    analogWrite(ML2, int(256*(-L)));
    analogWrite(ML1, 0);
  }
}

int flg1 = 0;
int flg2 = 0;
unsigned int MRSS[] = {0,0,0,0};
int MS[4][2] = {{0,0},{0,0},{0,0},{0,0}};
int spdR;
int spdL;
void updateSpeed(){
  int MSensor_stream[4][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  for(int i = 0; i < 4; i++){

    for(int k = 0; k < 2; k++){
      MSensor_stream[i][k] = digitalRead(MSensor(i));
      delayMicroseconds(3);
    }
    MSensor_stream[i][2] = digitalRead(MSensor(i));
    MS[i][1] = MS[i][0];
    if(MSensor_stream[i][0]==MSensor_stream[i][1] && MSensor_stream[i][1]==MSensor_stream[i][2]){
      MS[i][0] = MSensor_stream[i][0];
    }else{
      MS[i][0] = MS[i][1];
    }
  }

  if(!flg1){
    if(MS[0][0]==1 && MS[0][1]==0){

    }
  }else{

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

  pinMode(MP1, INPUT);
  pinMode(MP2, INPUT);
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  initAll();
  MRS1 = digitalRead(MSensor(0));
  MRS2 = digitalRead(MSensor(1));
  MLS1 = digitalRead(MSensor(2));
  MLS2 = digitalRead(MSensor(3));
}


void loop() {
  updateSpeed();

  Serial.print(spdR);
  Serial.print(", ");
  Serial.println(spdL);

  //Serial.println(pulseIn(MSensor(0),HIGH,30000));
  //Serial.println(pulseIn(MSensor(2),HIGH,30000));

  // int spd = 9000;
  // float pwr = 0;
  //   int cspd = pulseIn(MSensor(0),HIGH,30000);
  //   int dspd = 0;
  //   if(cspd == 0){
  //     dspd = -30000;
  //   }else{
  //     dspd = spd - cspd;
  //   }
  //   pwr = pwr-dspd/30000.0;
  //   if(pwr<0){pwr = 0;}
  //   if(pwr>1){pwr = 1;}
  //   Serial.println(pwr);
  //   ControlMotor(pwr,0);

}