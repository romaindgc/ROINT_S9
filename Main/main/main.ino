//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Servo.h"
#include <I2Cdev.h>
#include <string.h>

//Variables pour data process
String data;
String string_prefix;
String valueString;
int values[2] = {0, 0}; 
int prefix;



//Définition des variables pour le système d'éclairage
const int PIN_ENABLE_LEDS = A3; //PIN pour la commande de l'éclairage
const int frequence_clignotement = 7; //En ms
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
const int Y_joy1 = 500;
const int Y_joy2 = 564;

//Boundaries for the treshold function
const int upperBound_joy1 = Y_joy1 + ceil(1023*0.01);
const int lowerBound_joy1 = Y_joy1 - ceil(1023*0.01);

const int upperBound_joy2 = Y_joy2 + ceil(1023*0.01);
const int lowerBound_joy2 = Y_joy2 - ceil(1023*0.01);




//Définition des variables pour le bluetooth
const int RX = 11;
const int TX = 10;
SoftwareSerial bluetoothSerial(RX, TX); // RX, TX


void setup() {
  Serial.begin(9600);

  //Définition 
  pinMode (PIN_ENABLE_LEDS, OUTPUT); //définition du PIN ENABLE LED comme une sortie
  analogWrite(PIN_ENABLE_LEDS, 0); //on éteind l'éclairage par défaut

 // define pin modes for tx, rx pins:
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  bluetoothSerial.begin(9600);

  // Configuration des PINS des moteurs comme des sorties
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  pinMode(PIN_SENS_M1, OUTPUT);
  pinMode(PIN_SENS_M2, OUTPUT);

  pinMode(PIN_JOY1, INPUT);
  pinMode(PIN_JOY2, INPUT);

  miseZero();
}

void loop() {
  // Lecture et traitement de la data reçue par Bluetooth
  if (bluetoothSerial.available()) {
    Serial.println("Available");

    // Lire la chaîne jusqu'au caractère '!'
    data = bluetoothSerial.readStringUntil('!');

    Serial.println("data reçue : ");
    Serial.println(data);

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
      
      if (isInteger(string_prefix)) {
          prefix = string_prefix.toInt();
      } else {
          Serial.println("Erreur : le préfixe n'est pas un entier !");
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
      Serial.print("Préfixe reçu : ");
      Serial.println(prefix);
      Serial.print("Valeur 1 : ");
      Serial.println(values[0]);
      Serial.print("Valeur 2 : ");
      Serial.println(values[1]);

    // Appeler une fonction de traitement avec le préfixe et le tableau des valeurs
    Serial.println("Choix");
    Serial.println(prefix == 31 ? "true" : "false");
    choixCible(prefix, values);
    }
  }

  //DEFENSE MODE 
  //Si le defense mode est actif, on fait clignoter les LEDs à une certaine fréquence
  if(defense_state){
    Serial.print("Defense actif");
    eclairageDefense();
  }

}
 

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : CHOIX MODE ECLAIRAGE
//-----------------------------------------------------------------------------------------------------------------
void choixCible(int prefix, int value[]){

  if (prefix == 11) { 
    // Rotation des moteurs 
    int value_M1 = value[0] * thresholdFunction(value[0], lowerBound_joy1, upperBound_joy1);
    int value_M2 = value[1] * thresholdFunction(value[1], lowerBound_joy2, upperBound_joy2);

    // Mapping des valeurs d'entrée des joysticks pour les moteurs
    commandMotors(map(value_M1, 0, 1023, 0, 255), map(value_M2, 0, 1023, 0, 255));
  } else if (prefix == 31) {
    Serial.println("Lampe switch");
    // Changement d'état de la lampe (soit ON soit OFF)
    defense_state = false;
    lampeOnOff(value[0]);
  } else if (prefix == 32) {
    Serial.println("Defense");
    // Choix d'activer le mode défense
    defense_state = true;
  } else {
    Serial.println("MaZ");
    miseZero();
  }


  }
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : MISE A ZERO
//-----------------------------------------------------------------------------------------------------------------
  void miseZero(){
    defense_state = false; //Mode defense non actif
    lampeOnOff(0); //On éteint la lampe
    setupMotors();
  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Système d'éclairage -- MODE ETEINT -- 0000
//-----------------------------------------------------------------------------------------------------------------
  void lampeOnOff(int value){
    Serial.println("Valeur led");
    Serial.println(value);
    analogWrite(PIN_ENABLE_LEDS,value); //On change l'état des LEDs en fonction de la valeur
    if(value ==1024 ){
      ledState = 1;
    }else{
      ledState = 0;
    }
    Serial.println("LEDs changent etat");
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
    ledState = 1-ledState;                  // Inverse l'état de la LED

    // Met à jour la luminosité de la LED en fonction de son nouvel état
    if (ledState == 1) {
      analogWrite(PIN_ENABLE_LEDS, 1023);  // Allume la LED
      Serial.println("LED allumée");
    } else {
      analogWrite(PIN_ENABLE_LEDS, 0);     // Éteint la LED
      Serial.println("LED éteinte");
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
  if (x > lowerBound) {
    return 0;  // If x is greater than the lowerBound return 0
  } else if (x < upperBound) {
    return 0;  // If x is lower than the upperbound return 0
  } else {
    return 1;  // Else return 1
  }
}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : rotation direction
//-----------------------------------------------------------------------------------------------------------------
  void rotationDirection(int y, int Y_joy, int PIN_SENS){
  if (y < Y_joy ){
    digitalWrite(PIN_SENS, LOW); //Counter clockwise rotation direction
  }else{
    digitalWrite(PIN_SENS, HIGH); //Clockwise rotation direction
  }
}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : commande moteur
//-----------------------------------------------------------------------------------------------------------------
  void commandMotors(int value_mapped_M1, int value_mapped_M2) {
  // Vitesse de rotation des moteurs nulle
  analogWrite(PIN_M1, value_mapped_M1);
  analogWrite(PIN_M2, value_mapped_M2);

  // Sens de rotation par défaut : vers l'avant
  rotationDirection(value_mapped_M1, Y_joy1, PIN_M1);
  rotationDirection(value_mapped_M2, Y_joy2, PIN_M2);
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

