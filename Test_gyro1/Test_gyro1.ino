//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Servo.h"
#include <I2Cdev.h>
//un vieux commentaire
Servo servo; // création de l'objet "servo"
const int servoPin = 6; //définition du pin du servo


//definition variable pour la lecture angle y
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;

//définition des variables pour le système d'éclairage
const int PIN_ENABLE_LEDS = 3; //PIN pour la commande de l'éclairage
int CHOIX_MODE = 0000; // 1000 = éclairage --  2000 = defense
const int frequence_clignotement = 50; //En ms

//definition variable pour asservissement et correcteur PID
const double Consigne = 0.0; //consigne à atteindre
double prevError = 0.0;
double I = 0.0;
const double Kp = 2.0;
const double Ki = 0.1;
const double Kd = 1.0;
int mode_camera = 21; //Mode automatique = 21 - Mode manuel = 31 |

//définition des variables pour le bluetooth
const int RX = 1;
const int TX = 0;
SoftwareSerial bluetoothSerial(RX, TX); // RX, TX


void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  servo.attach(servoPin); // attache le servo au pin spécifié
  servo.write(Consigne); //définir la position initiale du servo

  pinMode (PIN_ENABLE_LEDS, OUTPUT); //définition du PIN ENABLE LED comme une sortie
  analogWrite(PIN_ENABLE_LEDS, 0); //on éteind l'éclairage par défaut

  // define pin modes for tx, rx pins:
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  bluetoothSerial.begin(38400);
}

void loop() {
  //readMPU();
  //readTension();
  CHOIX_MODE = (int)bluetoothSerial.read(); //Lecture de la commande bluetooth
  //testservo();
  choixModeEclairage(CHOIX_MODE);
  choixModeCamera(choixModeCamera);
}


//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Asservissement servomoteur
//-----------------------------------------------------------------------------------------------------------------
void asservissement_servo(double angleY){
  
  // Calcul de l'erreur
  double error = Consigne - angleY;

  // Calcul des termes PID
  double P = Kp * error;
  I += Ki * error;
  double D = Kd * (error - prevError);

  // Calcul de la commande du servomoteur
  double servoCommand = P + I + D;

  // Limiter la commande du servomoteur dans la plage acceptable
  servoCommand = constrain(servoCommand, 0, 180);

  // Envoyer la commande au servomoteur
  servo.write(servoCommand);

  // Mise à jour de l'erreur précédente
  prevError = error;

  delay(20);  // Ajustez si nécessaire pour la fréquence de mise à jour
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Test du servomoteur
//-----------------------------------------------------------------------------------------------------------------
void testservo() {
   servo.write(0); // demande au servo de se déplacer à cette position
   delay(1000); // attend 1000 ms entre changement de position

   servo.write(90); // demande au servo de se déplacer à cette position
   delay(1000); // attend 1000 ms entre changement de position

   servo.write(180); // demande au servo de se déplacer à cette position
   delay(1000); // attend 1000 ms entre changement de position
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------



//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Capter la position avec le gyroscope
//-----------------------------------------------------------------------------------------------------------------
double lire_angle_gyro( ) { 
  //a vérifier
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
  Serial.println("AngleY= ");
  Serial.print(y);
  Serial.println("-----------------------------------------");

  return(y);
  delay(800);
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Lire la tension pour la sécu. batterie
//-----------------------------------------------------------------------------------------------------------------
void readTension( ) {
  digitalWrite(4, HIGH); // Définit la broche 4 en état HIGH
  delay(100);
  digitalWrite(4, LOW); // Définit la broche 4 en état LOW
  float sommeTension = 0.0; //définition pour faire la moyenne de la tension
  int echantillon=1000; //A MODIFIER, valeur du nombre d'echantillon qui définie la précision de notre mesure
  for (int i = 0; i < echantillon; i++) { //on récupère toutes les valeurs selon le nombre d'échantillon désiré
    int valeurTension = analogRead(A0);
    float tension = (float)valeurTension * 5.0 / 1023.0; //on mutlplie par 5/1023 pour obtenir la valeur en tension, l'arduino étant codé sur 1024 bits
    sommeTension += tension; 
    
    delay(100/echantillon); // Attendre un peu entre chaque lecture en fonction du nombre d'échantillon (10 secondes)
  }

  // Calculer la moyenne des échantillons
  float moyenneTension = sommeTension / echantillon;

  // Envoi de la moyenne via Bluetooth
  Serial.println("Moyenne tension (");
  Serial.print(echantillon);
  Serial.print(" echantillons): ");
  Serial.print(moyenneTension, 2); // Affiche la moyenne avec 2 décimales
  Serial.println(" V");

  delay(100); // Attendez  0.1 seconde avant de lire à nouveau la valeur
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : CHOIX MODE CAMERA
//-----------------------------------------------------------------------------------------------------------------
  void choixModeCamera(int choix){
    switch (choix) {
    case 21:
        ModeAutoCamera();
        break;

    case 31:
        //TODO : Mode manuel
        break;

    default:
        //Mode auto par défaut
        break;
}

  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Mode automatique camera
//-----------------------------------------------------------------------------------------------------------------
  void ModeAutoCamera(){
    double angle_y=lire_angle_gyro(); //définir l'angle y comme la lecture de l'angle avec le gyro
    asservissement_servo(angle_y);
  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : CHOIX MODE ECLAIRAGE
//-----------------------------------------------------------------------------------------------------------------
  void choixModeEclairage(int choix){
    switch (choix) {
    case 0000:
        eclairageEteint();
        break;

    case 1000:
        eclairageAllume();
        break;

    case 2000:
        eclairageDefense();
        break;

    default:
        eclairageEteint();
        break;
}

  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Système d'éclairage -- MODE ETEINT -- 0000
//-----------------------------------------------------------------------------------------------------------------
  void eclairageEteint(){
    analogWrite(PIN_ENABLE_LEDS,0); //On éteind les LEDs
    Serial.println("LEDs eteintes");
  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Système d'éclairage -- MODE ECLAIRAGE -- 1000
//-----------------------------------------------------------------------------------------------------------------
 void eclairageAllume(){
    analogWrite(PIN_ENABLE_LEDS,1023); //Allumage des LEDs
    Serial.println("LEDs allumees");
 }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Système d'éclairage -- MODE DEFENSE -- 2000
//-----------------------------------------------------------------------------------------------------------------
void eclairageDefense(){
  Serial.println("Mode defense active");
  analogWrite(PIN_ENABLE_LEDS,1023); //Allumage des LEDs
  delay(frequence_clignotement);
  analogWrite(PIN_ENABLE_LEDS,0); //LEDs éteintes
  delay(frequence_clignotement);
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
