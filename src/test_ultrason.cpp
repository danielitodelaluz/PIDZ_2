#include <Arduino.h>


// Définition des broches
const int PIN_US_TRIG = 2; // Broche pour le signal Trigger
const int PIN_US_ECHO = 3; // Broche pour le signal Echo

void setup() {
  // Initialisation des broches
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  Serial.begin(9600); // Initialisation de la communication série
}

void loop() {
  float Z_mes_us = measureDistance(); // Appel de la fonction et stockage de la distance

  // Affichage de la distance dans le moniteur série
  Serial.print("Altitude: ");
  Serial.print(Z_mes_us);
  Serial.println(" cm");

  delay(5); // Attendre un moment avant de refaire la mesure
}

// Fonction pour mesurer la distance
float measureDistance() {
  // Envoi d'une impulsion ultrasonique
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  // Lecture du signal Echo
  long duration = pulseIn(PIN_US_ECHO, HIGH);

  // Calcul de la distance (en cm)
  float distance = duration * 0.034 / 2;

  return distance; // Retourne la distance mesurée
}