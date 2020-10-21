#include <Servo.h>

/*********************************
  This code uses polling 
 ********************************/
#include <SPI.h>

unsigned char motorSpeed[7];
unsigned char dat;
byte marker = 0;


union       
  {
  int p1Int;
  unsigned char  p1Char [2];
  } p1Buffer;
union
  {
    int p2Int;
    unsigned char p2Char [2];
  } p2Buffer;

union       
  {
  int UltrasonicInt;
  unsigned char  UltrasonicChar [2];
  } UltrasonicBuffer;

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;
Servo ESC5;
Servo ESC6;

const int trigPin = 9;
const int echoPin = 10;

long duration;
unsigned char distance;

int reading = 0;
long c = 0;
long s = 0;

void setup() {
  Serial.begin(9600);
  pinMode(MISO, OUTPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input  
  
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);
  SPI.attachInterrupt();

  ESC1.attach(2,1000,2000);
  ESC2.attach(3,1000,2000);
  ESC3.attach(4,1000,2000);
  ESC4.attach(5,1000,2000);
  ESC5.attach(6,1000,2000);
  ESC6.attach(7,1000,2000);
  getAltitude();
  delay(10);
}

void loop() {}
   

ISR (SPI_STC_vect){
  switch (marker){
    case 0:
      dat = SPDR;
      if(dat == 'c'){
        marker++;
        SPDR = 'a';
        }
      break;
    case 1:
      motorSpeed[6] = SPDR;
      UltrasonicBuffer.UltrasonicInt = distance;
      SPDR = UltrasonicBuffer.UltrasonicChar[0];
      marker++;   
      break; 
    case 2:
      motorSpeed[0] = SPDR;
      marker++;
      SPDR = UltrasonicBuffer.UltrasonicChar[1];
      break;
    case 3:
      motorSpeed[1] = SPDR;
      marker++;
      break;
    case 4:
      motorSpeed[2] = SPDR;
      marker++;
      break;
    case 5:
      motorSpeed[3] = SPDR;
      marker++;
      break;
    case 6:
      motorSpeed[4] = SPDR;
      marker++;
      break;
    case 7:
      motorSpeed[5] = SPDR;
      marker=0;
      setMotors();
 }
}
void getAltitude(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
}
void setMotors(){
  c++;
  Serial.print("transfer complete, ");
  if(motorSpeed[0] == 1 && motorSpeed[1] == 2 && motorSpeed[1] == 2 && motorSpeed[2] == 3 && motorSpeed[3] == 4 && motorSpeed[4] == 5 && motorSpeed[5] == 6){
    s++;
  }
  else{
    for(int i = 0; i<6; i++){
      Serial.print(motorSpeed[i]);
      Serial.print(", ");
    }
    }
  Serial.print("success rate: ");
  Serial.print(s*100/c);
  Serial.println("%");
    
  ESC1.write(motorSpeed[0]);
  ESC2.write(motorSpeed[1]);
  ESC3.write(motorSpeed[2]);
  ESC4.write(motorSpeed[3]);
  ESC5.write(motorSpeed[4]);
  ESC6.write(motorSpeed[5]);
  getAltitude();
  //delay(10);
  
}
    
    
    
