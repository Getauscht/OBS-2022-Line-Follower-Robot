#include <Arduino.h>
#include <Servo.h>
#include <FalconRobot.h>

#define motorPinDir1 6
#define motorPinDir2 8
#define motorPinEsq1 5
#define motorPinEsq2 7

int motorLetInitialSpeed = 140;
int motorDirInitialSpeed = 127;

#define leftSensorPin A7
#define rightSensorPin A3
#define centralSensorPin A0

#define motorServo 8

FalconRobotDistanceSensor distanceSensor(2, 3); // Configura os pinos para do sensor.

int leftSensorValue = 0;
int rightSensorValue = 0;
int centralSensorValue = 0;

#define LIMITLUMA 700

int distance;

int error;

int P = 0;
int I = 0;
int D = 0;
int PIDValue = 0;
float Kp = 25;
float Kd = 0;
float Ki = 0;
int PreviousError = 0;

int stop = 0;

Servo servo1;

void setup()
{
  pinMode(motorPinDir1, OUTPUT);
  pinMode(motorPinDir2, OUTPUT);
  pinMode(motorPinEsq1, OUTPUT);
  pinMode(motorPinEsq2, OUTPUT);

  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  pinMode(centralSensorPin, INPUT);

  // servo1.attach(motorServo);

  Serial.begin(9600);
}

void SensorValueController()
{

  leftSensorValue = analogRead(leftSensorPin);
  rightSensorValue = analogRead(rightSensorPin);
  centralSensorValue = analogRead(centralSensorPin);

  // 0 = Preto / 1 = Branco
  // First Number = Left Sensor
  // Second Number = Right Sensor
  // Possiveis combinações de leitura
  // ESQUERDA || CENTRO || DIREITA
  // 0 0 0 = - 3 -> Parar
  // 0 0 1 = -2  -> Virar para a esquerda
  // 0 1 1 = -1 -> Virar para a esquerda
  // 1 0 1 = 0 -> Continuar para frente
  // 1 1 0 = 1 -> Virar para a direita
  // 1 0 0 = 2 -> Virar para a direita
  // 1 1 1 = 3 -> Se perdeu, retroceder até corrigir o erro

  // If readValue > LIMITLUMA, then the sensor ir reading black
  // If readValue < LIMITLUMA, then the sensor ir reading white

  if ((leftSensorPin > LIMITLUMA) && (centralSensorValue > LIMITLUMA) && (rightSensorValue > LIMITLUMA))
  { // Parar
    error = -3;
  }
  if ((leftSensorPin > LIMITLUMA) && (centralSensorValue > LIMITLUMA) && (rightSensorValue < LIMITLUMA))
  { // Virar para a esquerda
    error = -2;
  }
  if ((leftSensorPin > LIMITLUMA) && (centralSensorValue < LIMITLUMA) && (rightSensorValue < LIMITLUMA))
  { // Virar para a esquerda
    error = -1;
  }
  if ((leftSensorPin < LIMITLUMA) && (centralSensorValue > LIMITLUMA) && (rightSensorValue > LIMITLUMA))
  { // Continuar para frente
    error = 0;
  }
  if ((leftSensorPin < LIMITLUMA) && (centralSensorValue < LIMITLUMA) && (rightSensorValue > LIMITLUMA))
  { // Virar para a direita
    error = 1;
  }
  if ((leftSensorPin < LIMITLUMA) && (centralSensorValue > LIMITLUMA) && (rightSensorValue < LIMITLUMA))
  { // Virar para a direita
    error = 2;
  }
  if ((leftSensorPin < LIMITLUMA) && (centralSensorValue < LIMITLUMA) && (rightSensorValue < LIMITLUMA))
  { // Se perdeu, retroceder até corrigir o erro
    error = 3;
  }
}

// void PIDController()
// {

//   P = error;
//   I = I + error;
//   D = error - error;

//   PIDValue = ((Kp * P) + (Ki * I) + (Kd * D));

//   PreviousError = error;
// }

// void PIDMotorController()
// {

//   int motorDirSpeed = 127 - motorLetInitialSpeed - PIDValue;
//   int motorEsqSpeed = 132 + motorDirInitialSpeed - PIDValue;

//   analogWrite(motorPinDir1, motorDirSpeed);
//   if (motorDirSpeed <= 0)
//   {
//     digitalWrite(motorPinDir2, LOW);
//   }
//   else
//   {
//     digitalWrite(motorPinDir2, HIGH);
//   }
//   analogWrite(motorPinDir2, motorDirSpeed);
//   if (motorEsqSpeed <= 0)
//   {
//     digitalWrite(motorPinEsq1, LOW);
//   }
//   else
//   {
//     digitalWrite(motorPinEsq1, HIGH);
//   }
// }

// void LineFollow()
// {
//   analogWrite(motorPinDir1, motorDirInitialSpeed);
//   digitalWrite(motorPinDir2, HIGH);
//   delay(1000);
//   analogWrite(motorPinEsq1, motorLetInitialSpeed);
//   digitalWrite(motorPinEsq2, HIGH);
// }

void loop()
{

  leftSensorValue = analogRead(leftSensorPin);
  rightSensorValue = analogRead(rightSensorPin);
  centralSensorValue = analogRead(centralSensorPin);

  distance = distanceSensor.read();

  Serial.print("Distance Sensor: ");
  Serial.println(distance);

  if ((leftSensorValue > LIMITLUMA) && (centralSensorValue > LIMITLUMA) && (rightSensorValue > LIMITLUMA))
  { // Parar

    if (distance < 10)
    {
      if (stop == 0)
      {
        stop = 1;
        analogWrite(motorPinDir1, motorDirInitialSpeed);
        digitalWrite(motorPinDir2, LOW);
        analogWrite(motorPinEsq1, motorLetInitialSpeed);
        digitalWrite(motorPinEsq2, LOW);
        delay(1000);
        motorDirInitialSpeed = motorDirInitialSpeed - 15;
        motorLetInitialSpeed = motorLetInitialSpeed - 15;
      }

      if (stop >= 1)
      {
        delay(50);
        analogWrite(motorPinDir1, 0);
        digitalWrite(motorPinDir2, HIGH);
        analogWrite(motorPinEsq1, 0);
        digitalWrite(motorPinEsq2, HIGH);
        Serial.println("Parar");
      };
    }
  }
  if ((leftSensorValue > LIMITLUMA) && (centralSensorValue > LIMITLUMA) && (rightSensorValue < LIMITLUMA))
  { // Virar para a esquerda

    if (stop == 0 && distance > 10)
    {
      analogWrite(motorPinDir1, 0);
      digitalWrite(motorPinDir2, HIGH);
      analogWrite(motorPinEsq1, motorLetInitialSpeed);
      digitalWrite(motorPinEsq2, HIGH);
      Serial.println("Virar para a esquerda");
    }

    if (stop == 1)
    {
       analogWrite(motorPinDir1, 0);
      digitalWrite(motorPinDir2, HIGH);
      analogWrite(motorPinEsq1, motorLetInitialSpeed);
      digitalWrite(motorPinEsq2, HIGH);
      Serial.println("Virar para a esquerda");
    }
  }
  if ((leftSensorValue > LIMITLUMA) && (centralSensorValue < LIMITLUMA) && (rightSensorValue < LIMITLUMA))
  { // Virar para a esquerda
    if (stop == 0 && distance > 10)
    {
      analogWrite(motorPinDir1, 0);
      digitalWrite(motorPinDir2, HIGH);
      analogWrite(motorPinEsq1, motorLetInitialSpeed);
      digitalWrite(motorPinEsq2, HIGH);
      Serial.println("Virar para a esquerda");
    }

    if (stop == 1)
    {
      analogWrite(motorPinDir1, 0);
      digitalWrite(motorPinDir2, HIGH);
      analogWrite(motorPinEsq1, motorLetInitialSpeed);
      digitalWrite(motorPinEsq2, HIGH);
      Serial.println("Virar para a esquerda");
    }
  }
  if ((leftSensorValue < LIMITLUMA) && (centralSensorValue > LIMITLUMA) && (rightSensorValue < LIMITLUMA))
  { // Continuar para frente
    analogWrite(motorPinDir1, motorDirInitialSpeed);
    digitalWrite(motorPinDir2, HIGH);
    analogWrite(motorPinEsq1, motorLetInitialSpeed);
    digitalWrite(motorPinEsq2, HIGH);
    Serial.println("Continuar para frente");
  }
  if ((leftSensorValue < LIMITLUMA) && (centralSensorValue < LIMITLUMA) && (rightSensorValue > LIMITLUMA))
  { // Virar para a direita
    if (stop == 0 && distance > 10)
    {
      analogWrite(motorPinDir1, motorDirInitialSpeed);
      digitalWrite(motorPinDir2, HIGH);
      analogWrite(motorPinEsq1, 0);
      digitalWrite(motorPinEsq2, HIGH);
      Serial.println("Virar para a direita");
    }

    if (stop == 1)
    {
      analogWrite(motorPinDir1, motorDirInitialSpeed);
      digitalWrite(motorPinDir2, HIGH);
      analogWrite(motorPinEsq1, 0);
      digitalWrite(motorPinEsq2, HIGH);
      Serial.println("Virar para a direita");
    }
  }
  if ((leftSensorValue < LIMITLUMA) && (centralSensorValue > LIMITLUMA) && (rightSensorValue > LIMITLUMA))
  { // Virar para a direita
    if (stop == 0 && distance > 10)
    {
      analogWrite(motorPinDir1, motorDirInitialSpeed);
      digitalWrite(motorPinDir2, HIGH);
      analogWrite(motorPinEsq1, 0);
      digitalWrite(motorPinEsq2, HIGH);
      Serial.println("Virar para a direita");
    }

    if (stop >= 10)
    {
      analogWrite(motorPinDir1, motorDirInitialSpeed);
      digitalWrite(motorPinDir2, HIGH);
      analogWrite(motorPinEsq1, 0);
      digitalWrite(motorPinEsq2, HIGH);
      Serial.println("Virar para a direita");
    }
  }
  if ((leftSensorValue < LIMITLUMA) && (centralSensorValue < LIMITLUMA) && (rightSensorValue < LIMITLUMA))
  { // Se perdeu, retroceder até corrigir o erro
    analogWrite(motorPinDir1, motorDirInitialSpeed);
    digitalWrite(motorPinDir2, LOW);
    analogWrite(motorPinEsq1, motorLetInitialSpeed);
    digitalWrite(motorPinEsq2, LOW);
    Serial.println("Se perdeu, retroceder até corrigir o erro");
    delay(200);
  }

  delay(50);
}