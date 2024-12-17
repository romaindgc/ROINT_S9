//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Servo.h"
#include <I2Cdev.h>



//GYROSCOPE : Définition variable pour la lecture angle y
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;
double angleMesure;
double DeadZoneGyro = 3; //on laisse une marge de 3 degré pour que le servo bouge
const unsigned long periodGyro = 10;
//ASSERVISSEMENT : Définition des variables
Servo servo; // création de l'objet "servo"
const int servoPin = 5; //définition du pin du servo
double angleConsigne = 90;  
double previousAngle = 90; //notre zero est 90 sur le gyro
double prevError = 0.0;
double I = 0;
const double Kp = 0.03;
const double Ki = 0.1;
const double Kd = 0.1;
double DeadZoneAsserv = 5;
const unsigned long periodServo = 10;
//CAMERA: Mode manuel définition des variables
const unsigned long periodCamera = 1000; 

//Définition des variables pour le bluetooth
//const int RX = 1;
//const int TX = 0;
//SoftwareSerial bluetoothSerial(RX, TX); // RX, TX

//Définition des périodes et variables de temps


void setup() {
  //GYROSCOPE : Setup
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  //ASSERVISSEMENT : Setup
  servo.attach(servoPin); // attache le servo au pin spécifié
  servo.write(90); //définir la position initiale du servo

  Serial.begin(9600);

  // define pin modes for tx, rx pins:
  //pinMode(RX, INPUT);
  //pinMode(TX, OUTPUT);
  //bluetoothSerial.begin(38400);
}

void loop() {
  lire_angle();
  //Serial.println(angleMesure);
  //Serial.println(angleConsigne - abs(angleConsigne-angleMesure));
  //servo.write(angleConsigne - abs(angleConsigne-angleMesure));
  //servo.write(90);
  asservissement_servo(angleMesure);
  //int value = Serial.read();
  //camera_manual(300); 
}

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Mode manuel camera
//-----------------------------------------------------------------------------------------------------------------
void camera_manual(int value) {
  unsigned long currentMillisCamera = millis(); 
  static unsigned long previousMillisCamera = 0;
  double angleCameraConsigne = value;
  double previousAngle = 90; 
  angleCameraConsigne = map(angleCameraConsigne, 0, 1024, 0, 180); 
  Serial.println(angleCameraConsigne);  
  if(currentMillisCamera - previousMillisCamera >= periodCamera) {
    previousMillisCamera = currentMillisCamera; 
    servo.write(180-angleCameraConsigne);
  }
}

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Asservissement servomoteur
//-----------------------------------------------------------------------------------------------------------------
void asservissement_servo(double angleY){
  unsigned long currentMillisAsserv = millis(); 
  static unsigned long previousMillisAsserv = 0;
  if(currentMillisAsserv - previousMillisAsserv >= periodServo) {
    previousMillisAsserv = currentMillisAsserv; 
    Serial.print("AngleY=");
    Serial.println(angleY);
    Serial.print("Angleconsigne=");
    Serial.println(angleConsigne);
    double error = angleConsigne - angleY;
    Serial.print("Error = ");
    Serial.println(error);
    if(abs(error) > DeadZoneAsserv) { 
      // Calcul des termes PID
      double P = Kp * error;
      I += Ki * error;
      double D = Kd * (error - prevError)/periodServo;
      I = constrain(I, -90, 90); //il ne faut pas que I augmente trop -> "windup"
      // Calcul de la commande du servomoteur
      Serial.print("I = ");
      Serial.println(I);
      double servoCommand = P + I + D;
      Serial.print("servoCommand = ");
      Serial.println(servoCommand);
      servoCommand = map(servoCommand, -90, 90, 0, 180);
      double servoCommandConsigne = servoCommand;
      servoCommandConsigne = constrain(servoCommand, 0, 180);
      Serial.print("servoCommandConsigne = ");
      Serial.println(servoCommandConsigne);
      servo.write(180-servoCommandConsigne);
    
      // Mise à jour de l'erreur précédente
      prevError = error;
     }
  }
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
//                                    Fonctions : Capter la position avec le gyroscope
//-----------------------------------------------------------------------------------------------------------------
void lire_angle() {
  static unsigned long previousMillisGyro = 0;
  unsigned long currentMillisGyro = millis(); 
  if(currentMillisGyro - previousMillisGyro >= periodGyro) {
    previousMillisGyro = currentMillisGyro; 
    if(abs(angleConsigne - lire_angle_gyro()) > DeadZoneGyro) {  
      angleMesure = lire_angle_gyro();
      previousAngle = angleMesure;
    }
  }
  else {
    angleMesure = previousAngle;
  }  
}

double lire_angle_gyro() { 
    //lecture des angles
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
  
    //lecture des angles
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);
  
    //conversion des angles en degrés
    x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
     
    //renvoi de la valeur de l'angle y
    //Serial.println("AngleY= ");
    //Serial.println("-----------------------------------------");
    return y+10;
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
