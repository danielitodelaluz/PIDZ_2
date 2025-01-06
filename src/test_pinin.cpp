#include <Arduino.h>

// Définition des broches utilisées sur la Raspberry Pi Pico
const uint16_t Pin_Mode = 10;
// Variables pour stocker les signaux PWM
unsigned long pwm_mode = 0;

void setup() {
  // Initialisation de la communication série pour debug
  Serial.begin(9600);
  while (!Serial); // Attendre l'ouverture du moniteur série

  // Configurer les broches comme entrées
  pinMode(Pin_Mode, INPUT);
}

void loop() {
  // Lire les valeurs PWM sur le pin
  pwm_mode = pulseIn(Pin_Mode, HIGH, 25000);

  // Vérifier si les valeurs sont valides
Serial.print("Canal 8 PWM: ");
Serial.println(pwm_mode);
}