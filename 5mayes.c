#include <QTRSensors.h>

#define m1p1 5
#define m1p2 6

#define m2p1 10
#define m2p2 9

#define rs A8
#define ls A12

#define rs2 A14
#define ls2 A11

#define NUM_SENSORS 8

float Kp = 0.06;
#define Kd 0

#define rightMaxSpeed 250
#define leftMaxSpeed 250
int cond = 0;

int inclination = 0;
int leftBaseSpeed = 120;
int rightBaseSpeed = 120;
#define threshhold 250;

QTRSensors qtra;
uint16_t sensorValues[NUM_SENSORS];
uint16_t digitalSensors[NUM_SENSORS];

#define BUFFER_SIZE 10
int rightSensorValues[BUFFER_SIZE];
int leftSensorValues[BUFFER_SIZE];
int farRightSensorValues[BUFFER_SIZE];
int farLeftSensorValues[BUFFER_SIZE];
int rightTotal = 0;
int leftTotal = 0;
int farRightTotal = 0;
int farLeftTotal = 0;
int currentIndex = 0;

bool inSecondHexagon = false;

long MAXIMUM_QTR;
long MINIMUM_QTR;


int minIRValues[4];
int maxIRValues[4];

void calibrate() {
  for (int i = 0; i < 4; i++) {
    minIRValues[i] = 1000;
    maxIRValues[i] = 0;
  }

  // for (int i = 0; i < 100; i++) {  // 100 iterations of calibration
  while (digitalRead(3) == 1) {
    int leftSensor = analogRead(ls);
    int rightSensor = analogRead(rs);
    int leftSensor2 = analogRead(ls2);
    int rightSensor2 = analogRead(rs2);

    // Update minimum values
    if (leftSensor < minIRValues[0]) minIRValues[0] = leftSensor;
    if (rightSensor < minIRValues[1]) minIRValues[1] = rightSensor;
    if (leftSensor2 < minIRValues[2]) minIRValues[2] = leftSensor2;
    if (rightSensor2 < minIRValues[3]) minIRValues[3] = rightSensor2;

    // Update maximum values
    if (leftSensor > maxIRValues[0]) maxIRValues[0] = leftSensor;
    if (rightSensor > maxIRValues[1]) maxIRValues[1] = rightSensor;
    if (leftSensor2 > maxIRValues[2]) maxIRValues[2] = leftSensor2;
    if (rightSensor2 > maxIRValues[3]) maxIRValues[3] = rightSensor2;

    qtra.calibrate();
  
    delay(10);
  }

  long sumMaximum = 0;
  long sumMinimum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sumMaximum = sumMaximum + qtra.calibrationOn.maximum[i];
    sumMinimum = sumMinimum+ qtra.calibrationOn.minimum[i];
  }
  MAXIMUM_QTR=sumMaximum/(NUM_SENSORS);
  MINIMUM_QTR=sumMinimum/(NUM_SENSORS);
}

void setup() {
  pinMode(m1p1, OUTPUT);
  pinMode(m1p2, OUTPUT);
  pinMode(m2p1, OUTPUT);
  pinMode(m2p2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(3, INPUT_PULLUP);
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORS);

  Serial.begin(9600);

  // Serial.println("calibrating");
  calibrate();
  // Serial.println("Done calibratin");

  for (int i = 0; i < BUFFER_SIZE; i++) {
    rightSensorValues[i] = 0;
    leftSensorValues[i] = 0;
  }

  delay(2000);

  mforward(100);
  delay(500);
}

void readQtr() {
  qtra.read(sensorValues); 

  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {

    sensorValues[i] = map((long)sensorValues[i], MINIMUM_QTR, MAXIMUM_QTR, 580, 760);

    sensorValues[i] = (int)(pow((long double)sensorValues[i]/700, 12)*350);
    //   Serial.print(sensorValues[i]);
    // Serial.print("\t");
    digitalSensors[i] = (sensorValues[i] > 700) ? 1 : 0;
    //    Serial.print(digitalSensors[i]);
    // Serial.print("\t");
  }

  // Serial.println();
}

int right;
int left;
int farRight;
int farLeft;
int rightWeight = -8000;
int leftWeight = 8000;

void readIR() {
  left = map(constrain(analogRead(ls), minIRValues[0], maxIRValues[0]), minIRValues[0], maxIRValues[0], 30, 750);
  right = map(constrain(analogRead(rs), minIRValues[1], maxIRValues[1]), minIRValues[1], maxIRValues[1], 30, 750);
  farLeft = map(constrain(analogRead(ls2), minIRValues[2], maxIRValues[2]), minIRValues[2], maxIRValues[2], 30, 750);
  farRight = map(constrain(analogRead(rs2), minIRValues[3], maxIRValues[3]), minIRValues[3], maxIRValues[3], 30, 750);

  // Serial.print(farLeft);
  // Serial.print("\t");
  //  Serial.print(left);
  // Serial.print("\t");
  //  Serial.print(right);
  // Serial.print("\t");
  //  Serial.print(farRight);
  // Serial.println("\t");
}

int entryTime = 0;
int sharpTurnsActivated = 0;
bool firstSharpTurn = 0;

void loop() {
  
  readQtr(); 
  readIR();

  // filterStep();


  if (cond == 0 && isFarLeft()) { // enter hexagon
    cond = 1;
    sharpRight(250);
    while (isFarLeft() || isLeft() || isFarRight()) {
      readIR();
      readQtr();
    }
    entryTime = millis();
  } 
  
  if (cond == 1 && isFarLeft() && (millis() - entryTime) > 1500) { // leave hexagon
    cond = 2;
    sharpRightOneWheel(100);
    while (isLeft()) {
      readIR();
    }
    // sharpRight(250); //250
    
    entryTime = millis();
  }

  if (cond == 2 && (isRight()) && (millis() - entryTime) > 1500) {
    cond = 3;
    sharpRight(250);
    entryTime = millis();
  }

  if (cond == 3 && (isRight() || isLeft() || isFarRight() || isFarLeft()) && (millis() - entryTime) > 500) {
    cond = 4;


    entryTime = millis();
  }

  if (cond == 4 && (isRight() || isFarRight()) && (millis() - entryTime) > 350) {
    cond = 5;
    entryTime = millis();
    mforward(60);
    rightWeight = 0;
    leftWeight = 0;
    while (isRight() || isFarRight() || isLeft() || isFarLeft()) {
      readIR();
      readQtr();
    }
    delay(400);
 
    sharpRight(60);

    while (!digitalSensors[3] || !digitalSensors[4]) {
      readQtr();
    }

    rightBaseSpeed = 40;
    leftBaseSpeed = 40;
    mforward(100);
    
    while (!isRight() && !isLeft()) {
      readIR();
    }

  }

  if (cond == 5) {
    cond = 6;

    mforward(160);

    delay(500);
    
    stopRobot();
    delay(2000);  // 5000
    rightWeight = -50000;
    leftWeight = 8000;
    mforward(100);
    delay(350); // 350
    readQtr();
    entryTime = millis();
  }

  if (cond == 6 && isLeft() && (millis() - entryTime) > 750) {
    cond = 7;
    entryTime = millis();
    sharpLeft(200);
  }

  if (cond == 7 && isLeft() && (millis() - entryTime) > 500) {
    delay(150);
    sharpLeft(200);
    while (!isFarRight()) {readIR();}
    cond = 8;
    rightWeight = -8000;
    leftWeight = 8000;

    // rightBaseSpeed = 140;
    // leftBaseSpeed = 140; 
    rightBaseSpeed = 200;
    leftBaseSpeed = 200;  
  }

  if (cond == 8 && ((isFarRight() || isFarLeft()) && (!digitalSensors[3] || !digitalSensors[4]))) {
    entryTime = millis();
    rightWeight = 0;
    leftWeight = 0;
    rightBaseSpeed = 60;
    leftBaseSpeed = 60;
    while (!digitalSensors[3] && !digitalSensors[4] && ((millis() - entryTime) > 1000 ? (isFarLeft() || isFarRight()) : true)) {// while (!digitalSensors[3] && !digitalSensors[4] && (!isRight() || !isLeft() || !isFarLeft() || isFarRight())) {
      readQtr();
      readIR();

      for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = constrain(1023-sensorValues[i], 0, 1023);
      }

      PID();
      delay(25);
    }
    
    rightBaseSpeed = 120;
    leftBaseSpeed = 120;
    rightWeight = -8000;
    leftWeight = 8000;
    cond = 9;

    if (!isFarLeft()) {
      while (!isFarLeft()) {
        readIR();
        sharpLeft(60);
      }
    } else {
      while(!isFarRight()) {
        readIR();
        sharpRight(60);
      }
    }
   
    
    entryTime = millis();
  }

  if (cond == 9 && (isRight() || isFarRight()) && (millis() - entryTime) > 350) {
    cond = 10;
    entryTime = millis();
    mforward(60);
    rightWeight = 0;
    leftWeight = 0;
    while (isRight() || isFarRight() || isLeft() || isFarLeft()) {
      readIR();
      readQtr();
    }
    delay(400);

    sharpRight(60);

    while (!digitalSensors[3] || !digitalSensors[4]) {
      readQtr();
    }
  }

  if (cond == 10 && isFarLeft()) {
    cond = 11;
    sharpLeft(40);

    while (isLeft() || isFarLeft()) {
      readIR();
    }

    mforward(40);

    while (!isWhite() || isRight() || isLeft() || isFarLeft() || isFarRight()) {
      readIR();
      readQtr();
    }

    sharpRight(40);

    while (!(digitalSensors[3] && digitalSensors[4])) {
      readQtr();
    }

    while (!isWhite() || isRight() || isLeft() || isFarLeft() || isFarRight()) {
      readIR();
      readQtr();
      PID();
    }

    sharpLeft(40);

    while (!(digitalSensors[3] && digitalSensors[4])) {
      readQtr();
    }

    while (!isWhite() || isRight() || isLeft() || isFarLeft() || isFarRight()) {
      readIR();
      readQtr();
      PID();
    }

    sharpLeft(40);

    while (isLeft() || isFarLeft()) {
      readIR();
    }

    sharpRight(40);

    while (!(!isLeft() && !isRight() && !isFarRight() && !isFarLeft() && digitalSensors[5] && digitalSensors[6] && digitalSensors[7] && !digitalSensors[0] && !digitalSensors[1] && !digitalSensors[2] && !digitalSensors[3] && !digitalSensors[4])) {
      readIR();
      readQtr();
    }

    readQtr();

    while (!(isFarLeft() && isLeft())) {
      readIR();
      readQtr();
      PID();
    }


    sharpLeft(40);

    while (!(isFarRight())) {
      readIR();
    }

    mforward(100);
    delay(800);

    stopRobot();
    while (1) {}
  }

  PID();

  
  // if (cond == 6 && (isLeft() || isFarRight() || isFarLeft() || isRight()) && (millis() - entryTime) > 2000) {
  //   cond = 7;
  //   stopRobot();
  //   while(1) {}
  //   sharpRight(250);
  //   // rightWeight = -8000;
  //   // leftWeight = 8000;
  //   entryTime = millis();
  // }

  // if (cond == 8 && (isFarLeft() || isFarRight()) && (millis() - entryTime) > 1000) {
  //   cond = 9;

  //   while (isFarLeft() || isFarRight()) {
  //     readQtr();

  //     for (int i = 0; i < NUM_SENSORS; i++) {
  //       sensorValues[i] = constrain(1023-sensorValues[i], 0, 1023);
  //     }

  //     PID();
  //   }
  // }
  // if (cond == 2 && (isRight() || isLeft()) && (millis() - entryTime) > 1500) { // left/right pad
  //   cond = 3;

  //   stopRobot();
  //   while (1) {
  //     delay(10);
  //   }
  // }


  // if (cond == 2 || cond == 3 && (millis() - entryTime) > 2000) {

  //   if (isRight()) {
  //     sharpRight(250);
  //   } else if (isLeft()) {
  //     sharpLeft(250); 
  //   } else {
  //     PID();
  //   }
  // } else {
  //   PID();
  // }
//   if (isFarRight() && cond == 0) {
//     cond = 1;
//     entryTime = millis();
//     while (isLeft() || isRight()) {
//       mforward(80);
//     }
//     digitalWrite(LED_BUILTIN, HIGH);

//     stopRobot();

//     while(1) {};
//     while (isFarLeft() || isLeft() || isRight() || isFarRight()) {
//       mforward(80);
//     }

//     sharpRight(250);

//      if (isFarRight()) {
//       sharpRight(250);
//     } else if (isFarLeft()) {
//       sharpLeft(250);
//     } else {
//       PID();
//     }

//     // while (1) {
//     //   Serial.print(isFarLeft());
//     //   Serial.print("\t");
//     //    Serial.print(isLeft());
//     //   Serial.print("\t");
//     //    Serial.print(isRight());
//     //   Serial.print("\t");
//     //    Serial.print(isFarRight());
//     //   Serial.println("\t");
//     //   readIR();
//     //   readQtr();
//     // }   

//     rightBaseSpeed = 80;
//     leftBaseSpeed = 80;

//   } else if (cond == 1 && isFarRight() && (millis() - entryTime) > 1500) {
//     // sharpTurnsActivated = millis();

//     sharpRight(140);
//     delay(300);
//     rightBaseSpeed = 140;
//     leftBaseSpeed = 140;
//     cond = 2;
//   // } else if (cond == 2) {
//   //   if (isFrontWhite()) {
//   //     digitalWrite(LED_BUILTIN, HIGH);
//   //   }
//   } else {
//     if (cond == 0 && !firstSharpTurn) {
//       if (isLeft()) {
//         firstSharpTurn = 1;
//         while (!isRight()) {
//           readIR();
//           sharpLeft(80);
//         }
//         readQtr();
//         PID();
//       } else {
//         PID();
//       }
//     } else {
//       PID();
//     }
// }
  // if (isRight()) {
  //   sharpRight(250);
  // } else if (isLeft()) {
  //   sharpLeft(250);
  // } else {
  //   PID();
  // }

  delay(10);
}


int calculateLinePosition(unsigned int *sensorValues, uint8_t sensorCount) {
  float weightedSum = 0;
  float totalValue = 0;
  
  for (int i = 0; i < sensorCount; i++) {
    float weight = (i - 3.5) * 1000;
    weightedSum += sensorValues[i] * weight;
    totalValue += sensorValues[i];
  }

  weightedSum += map(right, minIRValues[1], maxIRValues[1], 0, 1023) * rightWeight;
  totalValue += map(right, minIRValues[1], maxIRValues[1], 0, 1023);
  weightedSum += map(left, minIRValues[0], maxIRValues[0], 0, 1023) * leftWeight;
  totalValue += map(left, minIRValues[0], maxIRValues[0], 0, 1023);

  if (totalValue == 0) {
    return 0;
  }

  int position = weightedSum / totalValue;

  return position;
}

int lastError = 0;
int maxChange = 0;
void PID() {
  float position = calculateLinePosition(sensorValues, NUM_SENSORS) - inclination;
  
  float error = position;
  float diff = error-lastError;
  if (diff > maxChange) {
    maxChange = diff;
    // Serial.println(diff);
  }
  lastError = error;
  int motorSpeed = Kp * error + Kd * diff;
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  rightMotorSpeed = constrain(rightMotorSpeed, 0, rightMaxSpeed);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, leftMaxSpeed);

  // Serial.print(rightMotorSpeed);
  // Serial.print("\t");
  // Serial.println(leftMotorSpeed);
  analogWrite(m1p1, rightMotorSpeed);
  analogWrite(m1p2, 0);
  analogWrite(m2p1, leftMotorSpeed);
  analogWrite(m2p2, 0);
}

bool isRight() {
  return right >= threshhold;
}

bool isLeft() {
  return left >= threshhold;
}

bool eitherRight() {
  return isRight() || isFarRight();
}

bool eitherLeft() {
  return isLeft() || isFarRight();
}

bool leftAndRight() {
  return isLeft() && isRight();
}

bool isFarLeft() {
  return farLeft >= threshhold;
}

bool isFarRight() {
  return farRight >= threshhold;
}

int weights[NUM_SENSORS] = {1, 3, 4, 4, 3, 1};

bool isWhite() {
  int weightedSum = 0;
  int weightTotal = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    weightedSum += sensorValues[i] * weights[i];
    weightTotal += weights[i];
  }

  // Calculate weighted average
  int weightedAverage = weightedSum / weightTotal;

  // Serial.println(weightedAverage);

  // If the weighted average is below the threshold, consider it white
  if (weightedAverage < 250) {
    return true;  // White detected
  } else {
    return false;  // Black detected
  }
}


void sharpRight(int speed) {
  analogWrite(m1p1, 0);
  analogWrite(m1p2, speed);
  analogWrite(m2p1, speed);
  analogWrite(m2p2, 0);
}

void sharpRightOneWheel(int speed) {
  analogWrite(m1p1, 0);
  analogWrite(m1p2, 0);
  analogWrite(m2p1, speed);
  analogWrite(m2p2, 0);
}

void sharpLeft(int speed) {
  analogWrite(m1p1, speed);
  analogWrite(m1p2, 0);
  analogWrite(m2p1, 0);
  analogWrite(m2p2, speed);
}

void stopRobot() {
  analogWrite(m1p1, 0);
  analogWrite(m1p2, 0);
  analogWrite(m2p1, 0);
  analogWrite(m2p2, 0);
}
void mforward(int speed){
  analogWrite(m1p1, speed);
  analogWrite(m1p2, 0);
  analogWrite(m2p1, speed*0.95); // 5ater moteur ysar a9wa mil limin
  analogWrite(m2p2, 0);
}

void filterStep() {
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int rightSensorValue = right;
    rightTotal = rightTotal - rightSensorValues[currentIndex];
    rightSensorValues[currentIndex] = rightSensorValue;
    rightTotal = rightTotal + rightSensorValue;

    int leftSensorValue = left;
    leftTotal = leftTotal - leftSensorValues[currentIndex];
    leftSensorValues[currentIndex] = leftSensorValue;
    leftTotal = leftTotal + leftSensorValue;

    int farRightSensorValue = farRight;
    farRightTotal = farRightTotal - farRightSensorValues[currentIndex];
    farRightSensorValues[currentIndex] = farRightSensorValue;
    farRightTotal = farRightTotal + farRightSensorValue;

    int farLeftSensorValue = farLeft;
    farLeftTotal = farLeftTotal - farLeftSensorValues[currentIndex];
    farLeftSensorValues[currentIndex] = farLeftSensorValue;
    farLeftTotal = farLeftTotal + farLeftSensorValue;
  }

  currentIndex = (currentIndex + 1) % BUFFER_SIZE;
  float leftAverage = (float)leftTotal / BUFFER_SIZE;
  left = leftAverage;
  float rightAverage = (float)rightTotal / BUFFER_SIZE;
  right = rightAverage;
  float farLeftAverage = (float)farLeftTotal / BUFFER_SIZE;
  farLeft = farLeftAverage;
  float farRightAverage = (float)farRightTotal / BUFFER_SIZE;
  farRight = farRightAverage;

}