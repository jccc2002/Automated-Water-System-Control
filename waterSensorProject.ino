#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // Dirección I2C y pines de conexión

const int sensorPin = 2;
volatile long pulse;

float volume;
float maxVol = 1.0;
float targetVol = maxVol;

const int solenoidPin = 7;       // Válvula

const int buttonStartPin = 3;    // Inicio
const int buttonResetPin = 4;    // Reinicio
const int buttonIncreasePin = 5; // Volumen ++
const int buttonDecreasePin = 6; // Volumen --

bool systemRunning = false;   // Estado del sistema
//bool valveClosed = false;    

unsigned long lastButtonTime = 0; // Almacén de tiempo de la última acción de los botones
const unsigned long buttonDelay = 200; // Retardo entre acciones de los botones 

unsigned long lastVolumeUpdate = 0; // Almacén de tiempo de la última actualización del volumen
const unsigned long volumeUpdateInterval = 500; // Intervalo de actualización del volumen

unsigned long lastPrintTime = 0; // Almacén de tiempo de la última impresión
const unsigned long printInterval = 1000;

void setup() {
  // Sistema
  pinMode(sensorPin, INPUT);
  pinMode(buttonStartPin, INPUT_PULLUP);
  pinMode(buttonResetPin, INPUT_PULLUP);
  pinMode(buttonIncreasePin, INPUT_PULLUP);
  pinMode(buttonDecreasePin, INPUT_PULLUP);
  pinMode(solenoidPin, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(sensorPin), increase, RISING);
  digitalWrite(solenoidPin, LOW);

  // Configuración LCD
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.begin(16, 2);
  lcd.clear();
}

void loop() {
  // Estado de los botones
  bool startButtonState = digitalRead(buttonStartPin);
  bool resetButtonState = digitalRead(buttonResetPin);
  bool increaseButtonState = digitalRead(buttonIncreasePin);
  bool decreaseButtonState = digitalRead(buttonDecreasePin);

  // Ajustar el volumen 
  if (!systemRunning) {
    unsigned long currentTime = millis();

    if (increaseButtonState == LOW && currentTime - lastButtonTime >= buttonDelay && targetVol < 10.0) {
      targetVol += 0.5; // 500 ml ++
      lastButtonTime = currentTime;
    }
    if (decreaseButtonState == LOW && currentTime - lastButtonTime >= buttonDelay && targetVol > 0.5) {
      targetVol -= 0.5; // 500 ml --
      lastButtonTime = currentTime;
    }
    if (currentTime - lastPrintTime >= printInterval) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Volumen objetivo:");
      lcd.setCursor(0, 1);
      lcd.print(targetVol);
      lcd.print(" L");
      lastPrintTime = currentTime;
    }
  }

  // Iniciar si no está en ejecución
  if (startButtonState == LOW && !systemRunning) {
    systemRunning = true;
    digitalWrite(solenoidPin, HIGH); 
    lastVolumeUpdate = millis(); 
    delay(100); 
  }

  // Finalizar = objetivo
  if (volume >= targetVol && systemRunning) {
    digitalWrite(solenoidPin, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Reinicie el sistema");
    delay(1000);
    //systemRunning = false; // Reiniciar el sistema
  }

  // Reiniciar todo aaaa
  if (resetButtonState == LOW) {
    pulse = 0;
    volume = 0.0;
    targetVol = maxVol;
    systemRunning = false;
    //valveClosed = false; // Restablecer el estado de la válvula
    digitalWrite(solenoidPin, LOW); 
    delay(100); 
  }

  // Actualizar volumen
  if (systemRunning && millis() - lastVolumeUpdate >= volumeUpdateInterval) {
    volume = 1.5373 * pulse / 1000;
    lcd.clear(); 
    lcd.setCursor(0, 0); 
    lcd.print("Volumen:");
    lcd.setCursor(0, 1); 
    lcd.print(volume);
    lcd.print(" L");
    lastVolumeUpdate = millis(); 
  }
}

void increase() {
  if (systemRunning) {
    pulse++;
  }
}

