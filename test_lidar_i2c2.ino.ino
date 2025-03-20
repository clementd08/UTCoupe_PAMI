#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

// === Bus I2C ===
TwoWire I2C_1 = TwoWire(0);  // Capteur avant (SDA = 22, SCL = 21)
TwoWire I2C_2 = TwoWire(1);  // Capteur arrière (SDA = 26, SCL = 27) <-- adapte selon ton câblage

// === Capteurs ===
SparkFun_VL53L5CX sensorFront;
SparkFun_VL53L5CX sensorRear;

VL53L5CX_ResultsData dataFront;
VL53L5CX_ResultsData dataRear;

bool capteurFrontOK = false;
bool capteurRearOK = false;

int widthFront = 0;
int widthRear = 0;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initialisation des capteurs...");

  // === I2C1 (Capteur avant) ===
  I2C_1.begin(22, 21);  // SDA1 = 22, SCL1 = 21
  I2C_1.setClock(400000);

  if (sensorFront.begin(0x29, I2C_1)) {
    sensorFront.setResolution(8 * 8);
    widthFront = sqrt(sensorFront.getResolution());
    sensorFront.startRanging();
    capteurFrontOK = true;
    Serial.println("✅ Capteur AVANT détecté et prêt.");
  } else {
    Serial.println("❌ Capteur AVANT non détecté.");
  }

  // === I2C2 (Capteur arrière) ===
  I2C_2.begin(26, 27);  // SDA2 = 26, SCL2 = 27
  I2C_2.setClock(400000);

  if (sensorRear.begin(0x29, I2C_2)) {
    sensorRear.setResolution(8 * 8);
    widthRear = sqrt(sensorRear.getResolution());
    sensorRear.startRanging();
    capteurRearOK = true;
    Serial.println("✅ Capteur ARRIÈRE détecté et prêt.");
  } else {
    Serial.println("❌ Capteur ARRIÈRE non détecté.");
  }

  if (!capteurFrontOK && !capteurRearOK) {
    Serial.println("🚫 Aucun capteur fonctionnel. Vérifiez les connexions.");
    while (1); // Stoppe tout si aucun capteur
  }
}

void loop()
{
  // === Lecture capteur avant ===
  if (capteurFrontOK && sensorFront.isDataReady()) {
    if (sensorFront.getRangingData(&dataFront)) {
      Serial.println("\n📸 CAPTEUR AVANT :");
      for (int y = 0; y < widthFront * widthFront; y += widthFront) {
        for (int x = widthFront - 1; x >= 0; x--) {
          Serial.print(dataFront.distance_mm[x + y]);
          Serial.print("\t");
        }
        Serial.println();
      }
    }
  }

  // === Lecture capteur arrière ===
  if (capteurRearOK && sensorRear.isDataReady()) {
    if (sensorRear.getRangingData(&dataRear)) {
      Serial.println("\n📸 CAPTEUR ARRIÈRE :");
      for (int y = 0; y < widthRear * widthRear; y += widthRear) {
        for (int x = widthRear - 1; x >= 0; x--) {
          Serial.print(dataRear.distance_mm[x + y]);
          Serial.print("\t");
        }
        Serial.println();
      }
    }
  }

  delay(10);
}
