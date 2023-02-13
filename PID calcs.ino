//****************************************//
//* Example Code for Sending             *//
//* Signed Integers over I2C             *//
//* ESP32 (Master) to Arduino (Slave)    *//
//*                                      *//
//* Master Code                          *//
//*                                      *//
//* UoN 2022 - Nat Dacombe               *//
//****************************************//

// read through all of the code and the comments before asking for help
// research 'two's complement' if this is not familiar to you as it is used to represented signed (i.e. positive and negative) values

#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04  // 4 in hexadecimal

// name and define IR sensor pins
#define IR6 25
#define IR5 32
#define IR4 14
#define IR3 33
#define IR2 27
#define IR1 26

int sensor1, sensor2, sensor3, sensor4, sensor5, sensor6;

void setup() {
  Serial.begin(9600);
  Wire.begin();  // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)

  //set IR sensor pins as inputs
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
}

// the minimum and maximum values here are determined by the amount of bits used by the chosen variable type
// for int, this is either 16-bits or 32-bits
// due to two's complement, the minimum value is -2^(N-1), and the maximum is (2^(N-1))-1; where N is the number of bits
int x, y, z;
float weightAve;

void loop() {

  sensor1 = analogRead(IR1);
  sensor2 = analogRead(IR2);
  sensor3 = analogRead(IR3);
  sensor4 = analogRead(IR4);
  sensor5 = analogRead(IR5);
  sensor6 = analogRead(IR6);

  // Serial.print(sensor1);
  // Serial.print(",");
  // Serial.print(sensor2);
  // Serial.print(",");
  // Serial.print(sensor3);
  // Serial.print(",");
  // Serial.print(sensor4);
  // Serial.print(",");
  // Serial.print(sensor5);
  // Serial.print(",");
  // Serial.print(sensor6);

  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  /* depending on the mirocontroller, the int variable is stored as 32-bits or 16-bits
  if you want to increase the value range, first use a suitable variable type and then modify the code below
  for example; if the variable used to store x and y is 32-bits and you want to use signed values between -2^31 and (231)-1
  uncomment the four lines below relating to bits 32-25 and 24-17 for x and y
  for my microcontroller, int is 32-bits hence x and y are AND operated with a 32 bit hexadecimal number - change this if needed

  >> X refers to a shift right operator by X bits
  */

  //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
  //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
 
  //calculate weighted average
  float weightAve = ((sensor1*7.5)+(sensor2*22.5)+(sensor3*38)+(sensor4*53)+(sensor5*68)+(sensor6*83))/(sensor1+sensor2+sensor3+sensor4+sensor5+sensor6);
  // Serial.print("  Weighted Ave = ");
  // Serial.print(weightAve);
  float error1 = 45.5-weightAve;
  //Serial.print("  Error = ");
  //Serial.println(weightAve);

  delay(100);

  sensor1 = analogRead(IR1);
  sensor2 = analogRead(IR2);
  sensor3 = analogRead(IR3);
  sensor4 = analogRead(IR4);
  sensor5 = analogRead(IR5);
  sensor6 = analogRead(IR6);

  Serial.print(sensor1);
  Serial.print(",");
  Serial.print(sensor2);
  Serial.print(",");
  Serial.print(sensor3);
  Serial.print(",");
  Serial.print(sensor4);
  Serial.print(",");
  Serial.print(sensor5);
  Serial.print(",");
  Serial.print(sensor6);


    //calculate weighted average
    float weightAve2 = ((sensor1*7.5)+(sensor2*22.5)+(sensor3*38)+(sensor4*53)+(sensor5*68)+(sensor6*83))/(sensor1+sensor2+sensor3+sensor4+sensor5+sensor6);
    //Serial.print("  Weighted Ave = ");
    //Serial.print(weightAve);
    float error2 = 45.5-weightAve2;
    //Serial.print("  Error = ");
    //Serial.println(error);

    float kp=5;
    float ki=3;
    float kd=3;
    float k=0.5;

    float u=kp*error2+ki*error2+kd*(error2-error1);

    Serial.print("PID = ");
    Serial.println(u);

    z=78.5+u;   //servo angle
    x=150+k*u;    //left motor speed
    y=150-k*u;    //right motor speed

    Wire.write((byte)((x & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
    Wire.write((byte)(x & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((y & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
    Wire.write((byte)(y & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((z & 0x0000FF00) >> 8));    // first byte of z, containing bits 16 to 9
    Wire.write((byte)(z & 0x000000FF));           // second byte of z, containing the 8 LSB - bits 8 to 1


    Wire.endTransmission();   // stop transmitting
    delay(100);
}
