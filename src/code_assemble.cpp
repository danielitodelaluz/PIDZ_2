/*
 * Projet : Stabilisation d'un drone en hauteur (axe Z) avec un contrôleur PID - Sortie V seule
 */

#include <Arduino.h>
#include <Servo.h>

//======================= pin ======================
const uint8_t PIN_US_TRIG = 2;
const uint8_t PIN_US_ECHO = 3;

const uint8_t PIN_IN_MODE = 4;

const uint8_t PIN_OUT_V = 21;

unsigned long pwm_mode = 0;

float Kp_V = 30.0f;
float Ki_V = 0.0f;
float Kd_V = 10.0f;


//======================= variable ======================
//===PID===
const float dt = 0.1f; // 100 ms
const int pwmMin_V = 991;
const int pwmMax_V = 2016;
float lastError = 0.0f;
float integral = 0.0f;
Servo vServo;


//======================= Fonctions utilitaires ===========================

// Mesure de la distance avec le HC-SR04 en cm
float measureDistanceCM() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_US_ECHO, HIGH, 30000UL); // timeout 30 ms

  if (duration == 0) {
    return -1.0f; // Aucune mesure valide
  }

  float distanceCM = (duration * 0.0343f) / 2.0f;
  return distanceCM;
}

// Limiter une valeur entre un min et un max
float constrainFloat(float x, float minVal, float maxVal) {
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}

// Mapper une valeur d'une plage à une autre
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//======================= Fonction CommandeV ==============================

/*
 * Fonction CommandeV :
 *  - Calcule la commande V (PWM) pour le contrôleur de vol basée sur une consigne et une altitude mesurée
 *  - Entrées :
 *      - consigne_altitude : consigne d'altitude (en cm)
 *      - currentAltitude : altitude mesurée (en cm)
 *  - Sortie :
 *      - Commande V (valeur PWM entre pwmMin_V et pwmMax_V)
 */
int CommandeV(float consigne_altitude, float currentAltitude) {
  // Calcul de l'erreur
  float error = consigne_altitude - currentAltitude;

  // Calcul PID
  float P_V = Kp_V * error;
  integral += error * dt;
  float I_V = Ki_V * integral;
  float derivative = (error - lastError) / dt;
  float D_V = Kd_V * derivative;

  float pidOutput = P_V + I_V + D_V;
  lastError = error;

  // Mapper directement la sortie PID vers la plage PWM
  int vCommand = (int)mapFloat(pidOutput, -Kp_V * 150.0f, Kp_V * 150.0f, pwmMin_V, pwmMax_V);
  vCommand = constrain(vCommand, pwmMin_V, pwmMax_V);

  return vCommand;
}



void setup() {
  // Initialisation du moniteur série
  Serial.begin(115200);
  delay(1000);
  Serial.println("Demarrage du systeme...");

  // Configuration des broches du capteur ultrason
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_IN_MODE, INPUT);

  vServo.attach(PIN_OUT_V);

  // Initialisation de la commande V à un niveau neutre
  vServo.writeMicroseconds((pwmMin_V + pwmMax_V) / 2);

  // Test mode auto activé
  pwm_mode = pulseIn(PIN_IN_MODE, HIGH, 25000);
  while (pwm_mode > 1500 ){
    pwm_mode = pulseIn(PIN_IN_MODE, HIGH, 25000);
    delay(200);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}



void loop() {
  static float consigne_altitude = 0.0f;
  static float lastValidAltitude = 0.0f;

    // Augmenter progressivement la consigne selon la hauteur
  if (consigne_altitude < 150.0f) {
    if (consigne_altitude < 50.0f) {
      consigne_altitude += 5.0f * dt; // Augmentation lente
    } 
    else if (consigne_altitude < 100.0f) {
      consigne_altitude += 10.0f * dt; // Augmentation modérée
    } 
    else {
      consigne_altitude += 5.0f * dt; // Augmentation rapide
    }
    consigne_altitude = constrainFloat(consigne_altitude, 0.0f, 150.0f);
  }

  // Lecture de l'altitude
   float currentAltitude = measureDistanceCM()- 11.34f;

  // Calcul de la commande V
  int vCommand = CommandeV(consigne_altitude, currentAltitude);

  // Envoi de la commande V au contrôleur de vol
  vServo.writeMicroseconds(vCommand);

  // Affichage sur le moniteur série
  Serial.print("Consigne : ");
  Serial.print(consigne_altitude, 2);
  Serial.print(" cm | Altitude : ");
  Serial.print(currentAltitude, 2);
  Serial.print(" cm | PWM V : ");
  Serial.println(vCommand);

  // Pause pour respecter l'intervalle d'échantillonnage
  delay(100);
}