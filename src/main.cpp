#include "Arduino.h"
#include "BluetoothSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <TFT_eSPI.h>

// Definizione della coda per lo scambio di dati tra le task
QueueHandle_t obd_data_queue;

// Parametri Bluetooth
static BluetoothSerial SerialBT;
static uint8_t obdii_addr[6] = {0x00, 0x1D, 0xA5, 0xF3, 0xAF, 0xE9};
static const char* pin = "1234";

// Oggetto per il display
TFT_eSPI tft = TFT_eSPI();

// Funzione per decodificare i dati RPM dalla risposta OBD-II
int decode_rpm(const char* data) {
    // La risposta tipica è: "41 0C 12 34\r"
    // I dati RPM sono nel 3° e 4° byte (12 e 34)
    if (strlen(data) < 10) return 0; // Controlla la lunghezza minima
    
    char hex_str[3];
    int A, B;

    // Converte il 3° byte esadecimale in intero
    strncpy(hex_str, data + 6, 2);
    hex_str[2] = '\0';
    sscanf(hex_str, "%x", &A);

    // Converte il 4° byte esadecimale in intero
    strncpy(hex_str, data + 9, 2);
    hex_str[2] = '\0';
    sscanf(hex_str, "%x", &B);

    // Calcola l'RPM secondo la formula standard (A*256 + B) / 4
    return (A * 256 + B) / 4;
}

// Funzione per decodificare la velocità dalla risposta OBD-II
int decode_speed(const char* data) {
    // La risposta tipica è: "41 0D 12\r"
    if (strlen(data) < 7) return 0;
    
    char hex_str[3];
    int A;
    strncpy(hex_str, data + 6, 2);
    hex_str[2] = '\0';
    sscanf(hex_str, "%x", &A);

    return A; // La velocità è il valore del byte A
}

// --- Task per la comunicazione OBD-II ---
void obd_task(void* parameter) {
    SerialBT.begin("ESP32_OBDII_Client", true);
    SerialBT.setPin(pin);
    Serial.println("Bluetooth client started.");

    const char* commands[] = {"ATZ\r", "010C\r", "010D\r"};
    int commandIndex = 0;

    while (1) {
        if (!SerialBT.connected()) {
            Serial.println("Attempting to connect to OBD-II adapter...");
            if (SerialBT.connect(obdii_addr)) {
                Serial.println("Connected!");
            } else {
                Serial.println("Connection failed. Retrying in 5 seconds...");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }
        
        if (SerialBT.connected()) {
            Serial.printf("Sending command: %s\n", commands[commandIndex]);
            SerialBT.print(commands[commandIndex]);
            
            String response = "";
            unsigned long startWait = millis();
            while (millis() - startWait < 2000) {
                while (SerialBT.available()) {
                    char c = SerialBT.read();
                    response += c;
                }
                if (response.indexOf('>') != -1) {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (response.length() > 0) {
                char* responseData = strdup(response.c_str());
                if (responseData) {
                    xQueueSend(obd_data_queue, &responseData, portMAX_DELAY);
                }
            }
            
            commandIndex = (commandIndex + 1) % (sizeof(commands) / sizeof(commands[0]));
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// --- Task per il display ---
void display_task(void* parameter) {
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextFont(4);
    
    // Titolo della pagina
    tft.drawString("Car Dashboard", 20, 10);

    char* receivedData;
    int rpm = 0;
    int speed = 0;

    while (1) {
        if (xQueueReceive(obd_data_queue, &receivedData, 0)) {
            Serial.print("Received: ");
            Serial.println(receivedData);
            
            if (strstr(receivedData, "010C") != NULL) {
                rpm = decode_rpm(receivedData);
            }
            if (strstr(receivedData, "010D") != NULL) {
                speed = decode_speed(receivedData);
            }
            
            free(receivedData);
        }

        // Aggiorna il display con i dati correnti
        tft.fillRect(0, 50, tft.width(), 100, TFT_BLACK);
        tft.setCursor(20, 50);
        tft.print("RPM: ");
        tft.println(rpm);
        tft.print("Speed: ");
        tft.println(speed);

        vTaskDelay(pdMS_TO_TICKS(100)); // Breve ritardo per non sovraccaricare il loop
    }
}

// --- Funzione Setup di Arduino ---
void setup() {
    Serial.begin(115200);
    
    obd_data_queue = xQueueCreate(10, sizeof(char*));
    if (obd_data_queue == NULL) {
        Serial.println("Failed to create queue. Halting.");
        while(1);
    }

    xTaskCreatePinnedToCore(obd_task, "OBD Task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(display_task, "Display Task", 4096, NULL, 4, NULL, 0);
}

// --- Funzione Loop di Arduino (lasciata vuota) ---
void loop() {
    vTaskDelay(1);
}