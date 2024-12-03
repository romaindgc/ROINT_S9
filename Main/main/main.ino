//Libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Servo.h"
#include <I2Cdev.h>

//Variables pour data process
String data;
String string_prefix;
String valueString;
int value;
int prefix;

//Définition des variables pour le système d'éclairage
const int PIN_ENABLE_LEDS = A3; //PIN pour la commande de l'éclairage
const int frequence_clignotement = 7; //En ms
bool defense_state = false;
int ledState = 0; // État actuel de la LED (allumée ou éteinte)


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
}

void loop() {
  // Lecture et traitement de la data reçue par Bluetooth
  //Lecture  et traitement de la data reçue par bluetooth
  //Format XXvalue!
  if (bluetoothSerial.available()) {
    Serial.println("Available");
    
    // Lire la chaîne jusqu'au caractère '!'
    data = bluetoothSerial.readStringUntil('\n');

    Serial.println("data reçue : ");
    Serial.println(data);

    // Vérifier que la chaîne contient au moins 2 lettres suivies d'une valeur
    if (data.length() >= 3) {
      // Extraire les deux caractères en préfix
      string_prefix = data.substring(0, 2);
      prefix = string_prefix.toInt(); //Convertion en int

      // Extraire la valeur entière après les deux lettres
      valueString = data.substring(2); // À partir du troisième caractère
      value = valueString.toInt(); // Convertir en int

      // Afficher les résultats pour débogage
      Serial.print("Prefix reçues : ");
      Serial.println(prefix);
      Serial.print("Valeur reçue : ");
      Serial.println(value);

      //On réalise l'action liée à la data reçues
      choixCible(prefix, value);
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
void choixCible(int prefix, int value){
    switch (prefix) {
    case 31:
        //Changement d'état de la lampe (soit ON soit OFF)
        defense_state = false;
        lampeOnOff(value);
        break;

    case 32:
        //Choix d'activer le mode défense
        defense_state = true;
        break;

    default:
        miseZero();
        break;
}

  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : MISE A ZERO
//-----------------------------------------------------------------------------------------------------------------
  void miseZero(){
    defense_state = false; //Mode defense non actif
    lampeOnOff(0); //On éteint la lampe
  }
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------
//                                    Fonction : Système d'éclairage -- MODE ETEINT -- 0000
//-----------------------------------------------------------------------------------------------------------------
  void lampeOnOff(int value){
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

