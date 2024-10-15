// PIN MOTEURS
const int PIN_M1 = 3;    // Port D3 PWM
const int PIN_M2 = 5;    // Port D5 PWM

const int PIN_SENS_M1 = A2; // Port A2  On peut utiliser les ports analogiques comme des digitaux
const int PIN_SENS_M2 = A1; // Port A1

void setup() {
  // Initialisation de la connexion série à un débit de 9600 bits/s
  Serial.begin(9600);

  // Configuration des PINS comme des sorties
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  pinMode(PIN_SENS_M1, OUTPUT);
  pinMode(PIN_SENS_M2, OUTPUT);

  // Setup initial des moteurs
  setupMotors();
}

void loop() {
  // Pour l'instant, la boucle principale est vide
}

//-------------------------------------Fonction pour configurer les moteurs--------------------------
void setupMotors() {
  // Vitesse de rotation des moteurs nulle
  analogWrite(PIN_M1, 0);
  analogWrite(PIN_M2, 0);

  // Sens de rotation par défaut : vers l'avant
  digitalWrite(PIN_SENS_M1, HIGH);
  digitalWrite(PIN_SENS_M2, HIGH);
}
//--------------------------------------------------------------------------------------------------
