#include <SoftwareSerial.h>
#include <AFMotor.h>
#include <Servo.h>
#include <SPI.h>
// generate a MCKL signal pin
 const int clock = 9;
 double objective = 28000; // 목표: 20cm에서의 수압
 double error; // 에러 변수 설정
 double errorprevious; // 이전 에러 저장 변수
 Servo servo; // 서보모터 설정
 double Time = 0.17; // 시간 변화량
 double PP,DD; // P제어와 D제어 변수설정
 double Kp = 0.012; // P제어 비례상수
 double Kd = 3.2; // D제어 비례상수
 double u; // 모터 제어 값
 
 
void resetsensor() //this function keeps the sketch a little shorter
{
  SPI.setDataMode(SPI_MODE0);
  SPI.transfer(0x15);
  SPI.transfer(0x55);
  SPI.transfer(0x40);
}
void setup() {
  Serial.begin(9600);
  SPI.begin(); //see SPI library details on arduino.cc for details
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32); //divide 16 MHz to communicate on 500 kHz
  pinMode(clock, OUTPUT);
  servo.attach(9); // 서보모터 실행
  pinMode(10, OUTPUT); //1번 펌프
  pinMode(11, OUTPUT); //2번 펌프
  digitalWrite(10, HIGH); // 1번 펌프 설정
  digitalWrite(11, HIGH); // 2번 펌프 설정
  delay(100);
  analogWrite(10,255); // 잠항을 위한 펌프 동작
  delay(30000); // n초 동안 동작 후 
  analogWrite(10,0); // 펌프 정지
  servo.write(110); // 압축산소 분사를 위한 서보모터 동작
}
void loop()
{
  TCCR1B = (TCCR1B & 0xF8) | 1 ; //generates the MCKL signal
  analogWrite (clock, 128) ;
  resetsensor(); //resets the sensor - caution: afterwards mode = SPI_MODE0!
  //Calibration word 1
  unsigned int result1 = 0;
  unsigned int inbyte1 = 0;
  SPI.transfer(0x1D); //send first byte of command to get calibration word 1
  SPI.transfer(0x50); //send second byte of command to get calibration word 1
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  result1 = SPI.transfer(0x00); //send dummy byte to read first byte of word
  result1 = result1 << 8; //shift returned byte
  inbyte1 = SPI.transfer(0x00); //send dummy byte to read second byte of word
  result1 = result1 | inbyte1; //combine first and second byte of word
  resetsensor(); //resets the sensor
  //Calibration word 2; see comments on calibration word 1
  unsigned int result2 = 0;
  byte inbyte2 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0x60);
  SPI.setDataMode(SPI_MODE1);
  result2 = SPI.transfer(0x00);
  result2 = result2 <<8;
  inbyte2 = SPI.transfer(0x00);
  result2 = result2 | inbyte2;
  resetsensor(); //resets the sensor
  //Calibration word 3; see comments on calibration word 1
  unsigned int result3 = 0;
  byte inbyte3 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0x90);
  SPI.setDataMode(SPI_MODE1);
  result3 = SPI.transfer(0x00);
  result3 = result3 <<8;
  inbyte3 = SPI.transfer(0x00);
  result3 = result3 | inbyte3;
  resetsensor(); //resets the sensor
  //Calibration word 4; see comments on calibration word 1
  unsigned int result4 = 0;
  byte inbyte4 = 0;
  SPI.transfer(0x1D);
  SPI.transfer(0xA0);
  SPI.setDataMode(SPI_MODE1);
  result4 = SPI.transfer(0x00);
  result4 = result4 <<8;
  inbyte4 = SPI.transfer(0x00);
  result4 = result4 | inbyte4;
 
  //now we do some bitshifting to extract the calibration factors
  //out of the calibration words;
  long c1 = (result1 >> 1) & 0x7FFF;
  long c2 = ((result3 & 0x003F) << 6) | (result4 & 0x003F);
  long c3 = (result4 >> 6) & 0x03FF;
  long c4 = (result3 >> 6) & 0x03FF;
  long c5 = ((result1 & 0x0001) << 10) | ((result2 >> 6) & 0x03FF);
  long c6 = result2 & 0x003F;
  resetsensor(); //resets the sensor
  //Pressure:
  unsigned int presMSB = 0; //first byte of value
  unsigned int presLSB = 0; //last byte of value
  unsigned int D1 = 0;
  SPI.transfer(0x0F); //send first byte of command to get pressure value
  SPI.transfer(0x40); //send second byte of command to get pressure value
  delay(35); //wait for conversion end
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  presMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
  presMSB = presMSB << 8; //shift first byte
  presLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
  D1 = presMSB | presLSB; //combine first and second byte of value
  resetsensor(); //resets the sensor 
  //Temperature:
  unsigned int tempMSB = 0; //first byte of value
  unsigned int tempLSB = 0; //last byte of value
  unsigned int D2 = 0;
  SPI.transfer(0x0F); //send first byte of command to get temperature value
  SPI.transfer(0x20); //send second byte of command to get temperature value
  delay(35); //wait for conversion end
  SPI.setDataMode(SPI_MODE1); //change mode in order to listen
  tempMSB = SPI.transfer(0x00); //send dummy byte to read first byte of value
  tempMSB = tempMSB << 8; //shift first byte
  tempLSB = SPI.transfer(0x00); //send dummy byte to read second byte of value
  D2 = tempMSB | tempLSB; //combine first and second byte of value
  //calculation of the real values by means of the calibration factors and the maths
  //in the datasheet. const MUST be long
  const long UT1 = (c5 << 3) + 20224;
  const long dT = D2 - UT1;
  const long TEMP = 200 + ((dT * (c6 + 50)) >> 10);
  const long OFF  = (c2 * 4) + (((c4 - 512) * dT) >> 12);
  const long SENS = c1 + ((c3 * dT) >> 10) + 24576;
  const long X = (SENS * (D1 - 7168) >> 14) - OFF;
  long PCOMP = ((X * 10) >> 5) + 2500;
  float TEMPREAL = TEMP/10;
  float PCOMPHG = PCOMP * 750.06 / 10000; // mbar*10 -> mmHg === ((mbar/10)/1000)*750/0
  Serial.print("Real Temperature in C = ");
  Serial.println(TEMPREAL);
  Serial.print("Compensated pressure in mbar = ");
  Serial.println(PCOMP);
  Serial.print("Compensated pressure in mmHg = ");
  Serial.println(PCOMPHG);
  //2-nd order compensation only for T < 20°C or T > 45°C
  
  error = PCOMP-objective;
  Serial.println(error);
  
  PP = Kp*error;
  DD = Kd*(error-errorprevious)/Time; // de/dt
  u = PP + DD; // 모터 PD제어값
  Serial.println(u);
  
  u = constrain(u,-255,255); // 모터 제어 상한값
  if(u>200){
  analogWrite(10,0);
  analogWrite(11,u);
  }
  
  else if(u<-200){
  analogWrite(10,abs(u));
  analogWrite(11,0);
  }
  else if(u>-200 and u<200){
  analogWrite(10,0);
  analogWrite(11,0);   
  }
  errorprevious = error;
  delay(100);
}
