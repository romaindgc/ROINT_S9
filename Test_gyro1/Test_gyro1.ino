//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Servo.h"
#include <I2Cdev.h>

Servo servo; // création de l'objet "servo"
const int servoPin = 5; //définition du pin du servo


//Définition variable pour la lecture angle y
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;
double Consigne;
unsigned long currentMillis = millis();   // Temps actuel  
double angleMesure;
double angleConsigne = 90;  
double previousAngle = 90; //notre zero est 90 sur le gyro
double DeadZoneGyro = 3; //on laisse une marge de 3 degré pour que le servo bouge
//Variable asserv
double prevError = 0.0;
double I = 0;
const double Kp = 1;
const double Ki = 1;
const double Kd = 0.5;
//Définition des variables pour le bluetooth
//const int RX = 1;
//const int TX = 0;
//SoftwareSerial bluetoothSerial(RX, TX); // RX, TX

//Définition des périodes et variables de temps
const unsigned long periodGyro = 50;
const unsigned long periodServo = 100;

void setup() {
  //Setup gyroscope
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  

  //Définition servo
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
  asservissement_servo(angleMesure);
}

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Asservissement servomoteur
//-----------------------------------------------------------------------------------------------------------------
void asservissement_servo(double angleY){
  unsigned long currentMillisAsserv = millis(); 
  static unsigned long previousMillisAsserv = 0;
  if(currentMillisAsserv - previousMillisAsserv >= periodServo) {
    previousMillisAsserv = currentMillisAsserv; 
         // Calcul de l'erreur
    double error = angleConsigne - angleY;
    Serial.print("Error = ");
    Serial.println(error);
    // Calcul des termes PID
    double P = Kp * error;
    I += Ki * error;
    double D = Kd * (error - prevError)/periodServo;
  
    // Calcul de la commande du servomoteur
    double servoCommand = P + I + D;
    servoCommand = map(servoCommand, -150, 150, 150, 30);
    Serial.print("servoCommand = ");
    Serial.println(servoCommand);
    servo.write(servoCommand);
  
    // Mise à jour de l'erreur précédente
    prevError = error;
  }
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Test du servomoteur
//-----------------------------------------------------------------------------------------------------------------
void testservo() {
   servo.write(0); // demande au servo de se déplacer à cette position
   delay(1000); // attend 1000 ms entre changement de position

   servo.write(15); // demande au servo de se déplacer à cette position
   delay(1000); // attend 1000 ms entre changement de position

   servo.write(30); // demande au servo de se déplacer à cette position
   delay(1000); // attend 1000 ms entre changement de position
}
//-----------------------------------------------------------------------------------------------------------------
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

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Capter la position avec le gyroscope
//-----------------------------------------------------------------------------------------------------------------
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
