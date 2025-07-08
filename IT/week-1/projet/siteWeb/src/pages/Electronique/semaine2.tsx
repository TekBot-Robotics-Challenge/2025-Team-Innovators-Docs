import { useState } from "react";
import { useCallback } from "react";
import { CodeViewer } from "../../../src/components/CodeViewer";
import accelrometre from "./images/accelerometre.jpg";
import DiagramSimulation from "./images/Diagramsimulation.jpg";
import MPU from "./images/MPU.jpg";
import MEMS from "./images/MEMS-Accelerometer-Working.gif";
import gyroscope from "./images/Coriolis-Force.png";

const ArduinoCode = `#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_MPU6050 mpu;

const float SEUIL = 1.5;    // m/s²
String lastDir = "";
float lastX=0, lastY=0, lastZ=0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    lcd.init(); lcd.backlight(); lcd.clear();
    if (!mpu.begin()) {
      lcd.print("Capteur KO !");
      while (1) delay(500);
      }
      lcd.setCursor((20-7)/2,1);
    lcd.print("BONJOUR"); delay(1000);
    calibrateMPU();  // Calibration au démarrage
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  String dir = detectDirection(accel);
  if (dir != lastDir) {
    lcd.setCursor(0,0); lcd.print("                ");
    lcd.setCursor(0,0); lcd.print(dir);
    lastDir = dir;
    }
    displayValues(accel);
    delay(100);
    }
    
    void calibrateMPU() {
      const int N = 200;
      float sumX=0, sumY=0, sumZ=0;
      for(int i=0; i<N; i++) {
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        sumX += a.acceleration.x;
        sumY += a.acceleration.y;
        sumZ += a.acceleration.z - 9.806;
        delay(5);
        }
        float offX=sumX/N, offY=sumY/N, offZ=sumZ/N;
        Serial.printf("Offsets: X=%.2f Y=%.2f Z=%.2f\\n", offX, offY, offZ);
        }`;
const CapteurCode = `void calibrateMPU() {
            const int N = 200;
            float sumX=0, sumY=0, sumZ=0;
            for(int i=0; i<N; i++) {
                sensors_event_t a,g,t;
                mpu.getEvent(&a,&g,&t);
                sumX += a.acceleration.x;
                sumY += a.acceleration.y;
                sumZ += a.acceleration.z - 9.806;
                delay(5);
            }
            float offX=sumX/N, offY=sumY/N, offZ=sumZ/N;
            Serial.printf("Offsets: X=%.2f Y=%.2f Z=%.2f\n", offX, offY, offZ);
        }`;

const FiltreCode = `float angleRoll=0;
const float alpha=0.98;
String detectDirection(const sensors_event_t &a) {
    float rollAcc=atan2(a.acceleration.y,a.acceleration.z)*RAD_TO_DEG;
    float gyroRate=mpu.getGyroX()/131.0;
    angleRoll=alpha*(angleRoll+gyroRate*0.01f)+(1-alpha)*rollAcc;
    return String(angleRoll,1)+"°";
}`

const DmpCode = `#include <MPU6050_6Axis_MotionApps20.h>
MPU6050 mpu;  // Note : change d'API
Quaternion q;

void setup() {
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
}

void loop() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        Serial.printf("Quaternion: w=%.3f x=%.3f y=%.3f z=%.3f\\n", q.w,q.x,q.y,q.z);
    }
}`
// Table des matières
export const TableOfContents = () => {
  const [isOpen, setIsOpen] = useState(false);
  return (
    <div className="bg-gray-50 border border-gray-200 rounded-lg p-4 mb-6">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center justify-between w-full text-left font-semibold text-gray-800 hover:text-blue-600"
      >
        Table des matières
        <span
          className={`transform transition-transform ${
            isOpen ? "rotate-180" : ""
          }`}
        >
          ▼
        </span>
      </button>
      {isOpen && (
        <div className="mt-4 space-y-1">
          <a
            href="#introduction"
            className="block text-blue-600 hover:underline"
          >
            1. Introduction
          </a>
          <a
            href="#principes-de-base"
            className="block text-blue-600 hover:underline"
          >
            2. Principes de base des capteurs
          </a>
          <div className="ml-4 space-y-1">
            <a
              href="#accelerometre"
              className="block text-blue-600 hover:underline"
            >
              2.1. Accéléromètre
            </a>
            <a
              href="#gyroscope"
              className="block text-blue-600 hover:underline"
            >
              2.2. Gyroscope
            </a>
          </div>
          <a
            href="#presentation-mpu6050"
            className="block text-blue-600 hover:underline"
          >
            3. Présentation du module MPU6050
          </a>
          <a
            href="#schema-cablage"
            className="block text-blue-600 hover:underline"
          >
            4. Schéma de câblage
          </a>
          <a
            href="#installation-bibliotheques"
            className="block text-blue-600 hover:underline"
          >
            5. Installation des bibliothèques
          </a>
          <a
            href="#exemple-code"
            className="block text-blue-600 hover:underline"
          >
            6. Exemple de code Arduino
          </a>
          <a href="#usage-dmp" className="block text-blue-600 hover:underline">
            7. Usage du DMP et quaternions
          </a>
          <a
            href="#simulation-wokwi"
            className="block text-blue-600 hover:underline"
          >
            8. Simulation Wokwi
          </a>
          <a href="#conclusion" className="block text-blue-600 hover:underline">
            9. Conclusion et perspectives
          </a>
        </div>
      )}
    </div>
  );
};

export default function Semaine2() {
  return (
    <div className="prose max-w-none">
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-blue-900 mb-2">
          📄 Test Eléctronique | TRC 2025
        </h1>
      </div>

      <h2
        id="objectif-test3"
        className="text-xl font-semibold text-blue-800 mt-8 mb-3"
      >
        🎯 Objectif du Projet
      </h2>
      <p className="box-border border-2 border-dashed p-4 bg-blue-800 text-white border-blue-800">
        Développer une solution de boîte noire embarquée (dans un cube physique), capable de : <br /><br />
        <b>Les objectifs clés incluent :</b><br />
        • Lire la vitesse et la position spatiale<br />
        • Transmettre les données via I2C<br />
        • Afficher les informations sur un écran LCD 16x2 connecté à une station de contrôle
      </p>
    </div>
  )
};  
