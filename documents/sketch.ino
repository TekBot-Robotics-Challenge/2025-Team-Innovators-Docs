#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  
Adafruit_MPU6050 mpu;
const float SEUIL = 1.5;

// Variables pour stocker l'ancienne direction et éviter des rafraîchissements inutiles
String lastDirection = "";
float lastX = 0, lastY = 0, lastZ = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  lcd.init();
  lcd.backlight();
  lcd.clear();

  if (!mpu.begin()) {
    lcd.print("Capteur KO !");
    while (1) delay(500);
  }
  String message = "BONJOUR!!!";
  
  // Calcul de la position de départ pour centrer sur 20 colonnes
  int startPos = (20 - message.length()) / 2;  // 20 colonnes pour un 20x4
  
  // Affichage centré sur la première ligne (ligne 0)
  lcd.setCursor(startPos, 1);
  lcd.print(message);
  delay(1000);
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Détection de la direction
  String dirX = (accel.acceleration.x > SEUIL) ? "Droite" : (accel.acceleration.x < -SEUIL) ? "Gauche" : "";
  String dirY = (accel.acceleration.y > SEUIL) ? "Avant" : (accel.acceleration.y < -SEUIL) ? "Arriere" : "";
  String dirZ = (accel.acceleration.z > SEUIL) ? "Haut" : (accel.acceleration.z < -SEUIL) ? "Bas" : "";

  String direction = dirX;
  if (dirY.length()) direction += (direction.length() ? "/" : "") + dirY;
  if (dirZ.length()) direction += (direction.length() ? "/" : "") + dirZ;
  if (!direction.length()) direction = "Stable";

  // Rafraîchir uniquement si la direction a changé
  if (direction != lastDirection) {
    lcd.setCursor(0, 0);
    lcd.print("                "); // Efface la ligne avec des espaces
    lcd.setCursor(0, 0);
    lcd.print(direction);
    lastDirection = direction;
  }

  // Rafraîchir les valeurs uniquement si elles changent significativement
  if (abs(accel.acceleration.x - lastX) > 0.1 || 
      abs(accel.acceleration.y - lastY) > 0.1 || 
      abs(accel.acceleration.z - lastZ) > 0.1) {
    
    lcd.setCursor(0, 1);
    lcd.print("X:");
    lcd.print(accel.acceleration.x, 1);
    lcd.print(" Y:");
    lcd.print(accel.acceleration.y, 1);
    lcd.print(" Z:");
    lcd.print(accel.acceleration.z, 1);
    
    lastX = accel.acceleration.x;
    lastY = accel.acceleration.y;
    lastZ = accel.acceleration.z;
  }

  delay(100); // Petit délai pour éviter une lecture trop rapide
}