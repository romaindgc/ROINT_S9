//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Servo.h"
#include <I2Cdev.h>
#include <string.h>

//Variables pour data process
String data;  // Utilisation de la variable globale 'data' uniquement
String string_prefix;
String valueString;
int values[2] = {0, 0};
int prefix;

//Définition des variables pour le bluetooth
//const int RX = 0;
//const int TX = 1;
//SoftwareSerial Serial(RX, TX); // RX, TX

//Définition des variables pour le système d'éclairage
const int PIN_ENABLE_LEDS = A3; //PIN pour la commande de l'éclairage
const int frequence_clignotement = 26; //En ms
bool defense_state = false;
int ledState = 0; // État actuel de la LED (allumée ou éteinte)

//Definition MOTEURS
// PIN MOTEURS
const int PIN_M1 = 3;    // Port D3 PWM
const int PIN_M2 = 5;    // Port D5 PWM

const int PIN_SENS_M1 = A2; // Port A2  On peut utiliser les ports analogiques comme des digitaux
const int PIN_SENS_M2 = A1; // Port A1

int value_M1;
int value_M2;

//For testing we put the joytsiks on A3 and A4
const int PIN_JOY1 = A3;
const int PIN_JOY2 = A4;

//Initial value of Y for each joysticks
const int Y_joy1 = 532;
const int Y_joy2 = 506;

//Boundaries for the treshold function
const int upperBound_joy1 = Y_joy1 + ceil(1023 * 0.01);
const int lowerBound_joy1 = Y_joy1 - ceil(1023 * 0.01);

const int upperBound_joy2 = Y_joy2 + ceil(1023 * 0.01);
const int lowerBound_joy2 = Y_joy2 - ceil(1023 * 0.01);



//GYROSCOPE : Définition variable pour la lecture angle y
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int minVal = 265;
int maxVal = 402;
double x;
double y;
double z;
double angleMesure;
double DeadZoneGyro = 3; //on laisse une marge de 3 degré pour que le servo bouge
const unsigned long periodGyro = 10;
//ASSERVISSEMENT : Définition des variables
bool asservissement_state = false;
Servo servo; // création de l'objet "servo"
const int servoPin = 6; //définition du pin du servo
double angleConsigne = 90;
double previousAngle = 90; //notre zero est 90 sur le gyro
double prevError = 0.0;
double I = 0;
const double Kp = 0.03;
const double Ki = 0.1;
const double Kd = 0.1;
double DeadZoneAsserv = 5;
const unsigned long periodServo = 100;

//CAMERA: Mode manuel définition des variables
const unsigned long periodCamera = 300;


void setup() {
  //Serial.begin(9600);

  //Définition des pins
  pinMode (PIN_ENABLE_LEDS, OUTPUT); //définition du PIN ENABLE LED comme une sortie
  analogWrite(PIN_ENABLE_LEDS, 0); //on éteint l'éclairage par défaut

  Serial.begin(9600);

  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  pinMode(PIN_SENS_M1, OUTPUT);
  pinMode(PIN_SENS_M2, OUTPUT);

  pinMode(PIN_JOY1, INPUT);
  pinMode(PIN_JOY2, INPUT);

  //GYROSCOPE : Setup
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  miseZero();
  //ASSERVISSEMENT : Setup
  servo.attach(servoPin); // attache le servo au pin spécifié
  servo.write(90); //définir la position initiale du servo
}




void loop() {


  // Lecture et traitement de la data reçue par Bluetooth
  if (Serial.available()) {

    // Lecture des données jusqu'au caractère '!'
    char c = Serial.read();
    if (c != '!') {
      data += c;

    } else {


      data.trim();  // Nettoie la chaîne reçue

      // Affiche la chaîne reçue pour débogage
      //Serial.println("Donnée reçue : " + data);

      // Affiche chaque caractère pour analyser les problèmes
      for (int i = 0; i < data.length(); i++) {
        char c = data[i];
        //Serial.print("Caractère : ");
        //Serial.println(c);
        //Serial.print("Code ASCII : ");
        //Serial.println((int)c);
      }

      //bluetooth//Serial.println("data reçue : ");
      //bluetooth//Serial.println(data);

      // Vérifier que la chaîne contient au moins un préfixe et une valeur
      if (data.length() >= 3) {
        // Diviser la chaîne reçue en fonction du séparateur '.'
        int firstDotIndex = data.indexOf('.'); // Position du premier point
        int secondDotIndex = data.indexOf('.', firstDotIndex + 1); // Position du second point, s'il existe

        // Extraire le préfixe (avant le premier point)
        if (firstDotIndex != -1) {
          string_prefix = data.substring(0, firstDotIndex);
          string_prefix.trim(); // Supprime les espaces en début et fin de chaîne
          prefix = string_prefix.toInt();
        }

        // Si string_prefix est bien un entier
        if (isInteger(string_prefix)) {
          prefix = string_prefix.toInt();
        } else {
          //bluetooth//Serial.println("Erreur : le préfixe n'est pas un entier !");
          prefix = -1;  // Valeur par défaut ou autre action
        }

        // Extraire la première valeur (entre le premier et le second point, ou après le premier point s'il n'y a qu'une valeur)
        if (firstDotIndex != -1 && secondDotIndex == -1) {
          values[0] = data.substring(firstDotIndex + 1).toInt();
          values[1] = 0; // Pas de deuxième valeur
        } else if (firstDotIndex != -1 && secondDotIndex != -1) {
          values[0] = data.substring(firstDotIndex + 1, secondDotIndex).toInt();
          values[1] = data.substring(secondDotIndex + 1).toInt();
        }

        // Afficher les résultats pour débogage
        //bluetooth//Serial.print("Préfixe reçu : ");
        //bluetooth//Serial.println(prefix);
        //bluetooth//Serial.print("Valeur 1 : ");
        //bluetooth//Serial.println(values[0]);
        //bluetooth//Serial.print("Valeur 2 : ");
        //bluetooth//Serial.println(values[1]);

        // Appel de la fonction de traitement des valeurs selon le préfixe
        choixCible(prefix, values);

        // Vérifiez et réinitialisez après traitement
        if (data.length() > 0) {
          data = "";  // Nettoie après chaque traitement
          //Serial.flush();  // Vide le buffer avant de lire de nouvelles données
        }
      }

    }

  }

  //DEFENSE MODE
  //Si le defense mode est actif, on fait clignoter les LEDs à une certaine fréquence
  if (defense_state) {
    //Serial.print("Defense actif");
    eclairageDefense();
  }

  //Mode camera automatique
  //Si le defense mode est actif, on fait clignoter les LEDs à une certaine fréquence
  if (asservissement_state) {
    lire_angle();
    asservissement_servo(angleMesure);
  }
}



//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : CHOIX MODE ECLAIRAGE
//-----------------------------------------------------------------------------------------------------------------
void choixCible(int prefix, int value[]) {

  if (prefix == 11) {


    // Rotation des moteurs
    int value_M1 = value[1]; //* thresholdFunction(value[0], lowerBound_joy1, upperBound_joy1);
    int value_M2 = value[0]; //* thresholdFunction(value[1], lowerBound_joy2, upperBound_joy2);

    int mapped_m1 = map(value_M1, 0, 1023, -80, 80);
    int mapped_m2 = map(value_M2, 0, 1023, -80, 80);
    
    if (mapped_m1 < 0 ) {
      //bluetooth//Serial.println("On recule");
      digitalWrite(PIN_SENS_M1, LOW); //Counter clockwise rotation direction
      mapped_m1 = -mapped_m1;
    } else {
      //bluetooth//Serial.println("On avance");
      digitalWrite(PIN_SENS_M1, HIGH); //Clockwise rotation direction
    }
    
    if (mapped_m2 < 0 ) {
      //bluetooth//Serial.println("On recule");
      digitalWrite(PIN_SENS_M2, LOW); //Counter clockwise rotation direction
      mapped_m2 = -mapped_m2;
    } else {
      //bluetooth//Serial.println("On avance");
      digitalWrite(PIN_SENS_M2, HIGH); //Clockwise rotation direction
    }
    
    analogWrite(PIN_M1, mapped_m1);
    analogWrite(PIN_M2, mapped_m2);
    // Mapping des valeurs d'entrée des joysticks pour les moteurs
    //commandMotors(map(value_M1, 0, 1023, 0, 255), map(value_M1, 0, 1023, 0, 255));
    //    commandMotors(map(value_M1, 0, 1023, 0, 255), map(value_M2, 0, 1023, 0, 255));



  } else if (prefix == 31) {
    //Serial.println("Lampe switch");
    // Changement d'état de la lampe (soit ON soit OFF)
    defense_state = false;
    lampeOnOff(value[0]);
  } else if (prefix == 32) {
    //Serial.println("Defense");
    // Choix d'activer le mode défense
    defense_state = true;
  } else if (prefix == 22) {//22
    //Serial.println("Asservissement");
    analogWrite(PIN_M1,0);
    analogWrite(PIN_M2, 0);
    asservissement_state = false;
    camera_manual(value[0]);
  } else {
    //Serial.println("MaZ");
    miseZero();
  }


}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : MISE A ZERO
//-----------------------------------------------------------------------------------------------------------------
void miseZero() {
  defense_state = false; //Mode defense non actif
  asservissement_state = true;
  lampeOnOff(0); //On éteint la lampe
  setupMotors();

}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Système d'éclairage -- MODE ETEINT -- 0000
//-----------------------------------------------------------------------------------------------------------------
void lampeOnOff(int value) {
  //Serial.println("Valeur led");
  //Serial.println(value);
  analogWrite(PIN_ENABLE_LEDS, 1023*value); //On change l'état des LEDs en fonction de la valeur
  if (value == 0 ) {
    ledState = 0;
  } else {
    ledState = 1;
  }
  //Serial.println("LEDs changent etat");
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Système d'éclairage -- MODE DEFENSE -- 2000
//-----------------------------------------------------------------------------------------------------------------
void eclairageDefense() {
  static unsigned long previousMillis = 0;  // Enregistre le dernier moment où l'état de la LED a changé

  unsigned long currentMillis = millis();  // Temps actuel

  // Vérifie si le temps pour changer l'état de la LED est écoulé
  if (currentMillis - previousMillis >= frequence_clignotement) {
    previousMillis = currentMillis;        // Met à jour le temps précédent
    ledState = 1 - ledState;                // Inverse l'état de la LED

    // Met à jour la luminosité de la LED en fonction de son nouvel état
    if (ledState == 1) {
      analogWrite(PIN_ENABLE_LEDS, 1023);  // Allume la LED
      //Serial.println("LED allumée");
    } else {
      analogWrite(PIN_ENABLE_LEDS, 0);     // Éteint la LED
      //Serial.println("LED éteinte");
    }
  }
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-------------------------------------Method : Configuration motors --------------------------
void setupMotors() {
  // Vitesse de rotation des moteurs nulle
  analogWrite(PIN_M1, 0);
  analogWrite(PIN_M2, 0);

  // Sens de rotation par défaut : vers l'avant
  digitalWrite(PIN_SENS_M1, HIGH);
  digitalWrite(PIN_SENS_M2, HIGH);
}
//--------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Threshold function
//-----------------------------------------------------------------------------------------------------------------
int thresholdFunction(int x, int lowerBound, int upperBound) {
  if (x < lowerBound) {
    return 0;  // If x is greater than the lowerBound return 0
  } else if (x > upperBound) {
    return 0;  // If x is lower than the upperbound return 0
  } else {
    return 1;  // Else return 1
  }
}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : rotation direction
//-----------------------------------------------------------------------------------------------------------------
void rotationDirection(int y, int Y_joy, int PIN_SENS) {
  if (y < Y_joy ) {
    //bluetooth//Serial.println("On recule");
    digitalWrite(PIN_SENS, LOW); //Counter clockwise rotation direction
  } else {
    //bluetooth//Serial.println("On avance");
    digitalWrite(PIN_SENS, HIGH); //Clockwise rotation direction
  }
}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : commande moteur
//-----------------------------------------------------------------------------------------------------------------
void commandMotors(int value_mapped_M1, int value_mapped_M2) {
  // Vitesse de rotation des moteurs nulle
  /*
    bluetooth//Serial.println("Valeur mot1 mapped");
    bluetooth//Serial.println(value_mapped_M1);

    bluetooth//Serial.println("Valeur mot2 mapped");
    bluetooth//Serial.println(value_mapped_M2);

    bluetooth//Serial.println("PINM1 ");
    bluetooth//Serial.println(PIN_M1);

    bluetooth//Serial.println("PINM2 ");
    bluetooth//Serial.println(PIN_M2);*/

  analogWrite(PIN_M1, value_mapped_M1);
  analogWrite(PIN_M2, value_mapped_M2);

  // Sens de rotation par défaut : vers l'avant
  rotationDirection(value_mapped_M1, Y_joy1, PIN_SENS_M1);
  rotationDirection(value_mapped_M2, Y_joy2, PIN_SENS_M2);
}
//-----------------------------------------------------------------------------------------------------------------

bool isInteger(const String &str) {
  if (str.length() == 0) return false; // Vérifie que la chaîne n'est pas vide

  for (size_t i = 0; i < str.length(); i++) {
    if (!isDigit(str[i])) {  // Vérifie si chaque caractère est un chiffre
      return false;
    }
  }
  return true;
}


//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Mode manuel camera
//-----------------------------------------------------------------------------------------------------------------
void camera_manual(int value) {
  unsigned long currentMillisCamera = millis();
  static unsigned long previousMillisCamera = 0;
  double angleCameraConsigne = value;
  angleCameraConsigne = map(angleCameraConsigne, 0, 1024, 0, 180);
  //Serial.println(angleCameraConsigne);
  if (currentMillisCamera - previousMillisCamera >= periodCamera) {
    previousMillisCamera = currentMillisCamera;
    servo.write(180 - angleCameraConsigne);
  }
}

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Asservissement servomoteur
//-----------------------------------------------------------------------------------------------------------------
void asservissement_servo(double angleY) {
  unsigned long currentMillisAsserv = millis();
  static unsigned long previousMillisAsserv = 0;
  if (currentMillisAsserv - previousMillisAsserv >= periodServo) {
    previousMillisAsserv = currentMillisAsserv;
    //Serial.print("AngleY=");
    //Serial.println(angleY);
    //Serial.print("Angleconsigne=");
    //Serial.println(angleConsigne);
    double error = angleConsigne - angleY;
    //Serial.print("Error = ");
    //Serial.println(error);
    if (abs(error) > DeadZoneAsserv) {
      // Calcul des termes PID
      double P = Kp * error;
      I += Ki * error;
      double D = Kd * (error - prevError) / periodServo;
      I = constrain(I, -90, 90); //il ne faut pas que I augmente trop -> "windup"
      // Calcul de la commande du servomoteur
      //Serial.print("I = ");
      //Serial.println(I);
      double servoCommand = P + I + D;
      //Serial.print("servoCommand = ");
      //Serial.println(servoCommand);
      servoCommand = map(servoCommand, -90, 90, 0, 180);
      double servoCommandConsigne = servoCommand;
      servoCommandConsigne = constrain(servoCommand, 0, 180);
      //Serial.print("servoCommandConsigne = ");
      //Serial.println(servoCommandConsigne);
      servo.write(180 - servoCommandConsigne);

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
  if (currentMillisGyro - previousMillisGyro >= periodGyro) {
    previousMillisGyro = currentMillisGyro;
    if (abs(angleConsigne - lire_angle_gyro()) > DeadZoneGyro) {
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
  Wire.requestFrom(MPU_addr, 14, true);

  //lecture des angles
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  //conversion des angles en degrés
  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  //renvoi de la valeur de l'angle y
  ////Serial.println("AngleY= ");
  ////Serial.println("-----------------------------------------");
  return y + 10;
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
