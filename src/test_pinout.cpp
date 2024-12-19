#include <Arduino.h>
#include <Servo.h>

//======================= Configuration ======================
const uint8_t pwmPin = 10; // Remplace par la broche de ton choix
const int pwmValue = 1500.0f; // PWM fixe à 2000 microsecondes

Servo pwmServo; // Création de l'objet Servo

//======================= Setup ==============================
void setup() {
  // Initialisation du moniteur série (optionnel)
  Serial.begin(115200);
  Serial.println("Initialisation...");

  // Attache le Servo à la broche choisie
  pwmServo.attach(pwmPin);

  // Envoie une commande PWM fixe
  pwmServo.writeMicroseconds(pwmValue);

  Serial.print("Commande PWM envoyée : ");
  Serial.println(pwmValue);
}

//======================= Loop ===============================
void loop() {
  Serial.println("Initialisation...");
  Serial.print("Commande PWM envoyée : ");
  Serial.println(pwmValue);
}