#include "Arduino.h"
#include "BluetoothSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <TFT_eSPI.h>

// --- OBD-II e Bluetooth ---
QueueHandle_t obd_data_queue;
static BluetoothSerial SerialBT;
static uint8_t obdii_addr[6] = {0x00, 0x1D, 0xA5, 0xF3, 0xAF, 0xE9};
static const char* pin = "1234";

// --- Display ---
TFT_eSPI tft = TFT_eSPI();
int currentScreen = 1;

// --- Funzioni di decodifica ---
int decode_rpm(const char* data) {
    if (strlen(data) < 10) return 0;
    char hex_str[3]; int A, B;
    strncpy(hex_str, data+6, 2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    strncpy(hex_str, data+9, 2); hex_str[2]='\0'; sscanf(hex_str,"%x",&B);
    return (A*256+B)/4;
}

int decode_speed(const char* data) {
    if (strlen(data) < 7) return 0;
    char hex_str[3]; int A;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    return A;
}

// --- Task OBD-II ---
void obd_task(void* parameter) {
    SerialBT.begin("ESP32_OBDII_Client", true);
    SerialBT.setPin(pin);
    const char* commands[] = {"ATZ\r","010C\r","010D\r"};
    int commandIndex=0;

    while(1) {
        if(!SerialBT.connected()){
            if(SerialBT.connect(obdii_addr)) Serial.println("Connected!");
            else { vTaskDelay(pdMS_TO_TICKS(5000)); continue; }
        }

        if(SerialBT.connected()) {
            SerialBT.print(commands[commandIndex]);
            String response="";
            unsigned long startWait=millis();
            while(millis()-startWait<2000){
                while(SerialBT.available()) response += (char)SerialBT.read();
                if(response.indexOf('>')!=-1) break;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            if(response.length()>0){
                char* responseData = strdup(response.c_str());
                if(responseData) xQueueSend(obd_data_queue, &responseData, portMAX_DELAY);
            }
            commandIndex = (commandIndex+1)%3;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// --- Disegno elementi fissi ---
void drawStaticElements() {
    tft.fillScreen(TFT_BLACK);

    // Stato Bluetooth al centro in alto
    tft.fillRect(tft.width()/2 - 30, 5, 60, 20, TFT_DARKGREY);
}

void drawBluetoothStatus(bool btConnected) {
    int rectW = 60;
    int rectH = 20;
    int rectX = (tft.width() - rectW) / 2;  // centrato in alto
    int rectY = 5;

    uint16_t btColor = btConnected ? TFT_GREEN : TFT_RED;
    tft.fillRect(rectX, rectY, rectW, rectH, btColor);

    tft.setTextColor(TFT_BLACK, btColor);
    tft.setTextFont(2);
    tft.setCursor(rectX + 5, rectY + 2);
    tft.print(btConnected ? "ON" : "OFF");
}

// --- Aggiornamento dinamico ---
void updateScreen1(int speed, int rpm, bool btConnected){
    // Pulisce l'area dinamica della schermata
    tft.fillRect(0, 50, tft.width(), tft.height()-50, TFT_BLACK);

    // Velocità centrale
    tft.setTextFont(8);
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    tft.setCursor(tft.width()/2 -20,60);
    tft.print(speed);

    // Barra RPM
    int barX=20,barY=180,barW=tft.width()-40,barH=30;
    float rpmRatio = (float)rpm/6000;
    if(rpmRatio>1.0) rpmRatio=1.0;
    tft.fillRect(barX,barY,barW,barH,TFT_DARKGREY);
    uint16_t rpmColor = (rpm<3000)?TFT_GREEN:(rpm<4500)?TFT_YELLOW:TFT_RED;
    tft.fillRect(barX,barY,(int)(barW*rpmRatio),barH,rpmColor);

    tft.setTextFont(2);
    int numTicks = 6;          // Numero di tacche (0, 1000, ..., 6000)
    int barLeft = barX;
    int barRight = barX + barW;

    for(int i = 0; i <= numTicks; i++){
        int tickX = barLeft + i * (barRight - barLeft) / numTicks;
        tft.drawLine(tickX, barY, tickX, barY+barH, TFT_WHITE);

        String label = String(i * 1000);
        int w = tft.textWidth(label); // larghezza del testo
        tft.setCursor(tickX - w/2, barY + barH + 2);
        tft.print(label);
    }

    drawBluetoothStatus(btConnected);
}

void updateScreen2(bool btConnected){
    tft.fillRect(0, 50, tft.width(), tft.height()-50, TFT_BLACK);
    tft.setTextFont(4); tft.setTextColor(TFT_WHITE);
    tft.setCursor(20,tft.height()/2-20); tft.print("Schermata 2");
    drawBluetoothStatus(btConnected);
}

void updateScreen3(bool btConnected){
    tft.fillRect(0, 50, tft.width(), tft.height()-50, TFT_BLACK);
    tft.setTextFont(4); tft.setTextColor(TFT_WHITE);
    tft.setCursor(20,tft.height()/2-20); tft.print("Schermata 3");
    drawBluetoothStatus(btConnected);
}

// --- Task display ---
void display_task(void* parameter){
    tft.init(); 
    tft.setRotation(1);
    drawStaticElements();

    char* receivedData;
    int rpm=0, speed=0;

    int lastBootButtonState = digitalRead(0);  // tasto di reboot come input
    pinMode(0, INPUT_PULLUP);

    while(1){
        // Lettura OBD
        if(xQueueReceive(obd_data_queue,&receivedData,0)){
            if(strstr(receivedData,"010C")!=NULL) rpm = decode_rpm(receivedData);
            if(strstr(receivedData,"010D")!=NULL) speed = decode_speed(receivedData);
            free(receivedData);
        }

        // Lettura tasto di reboot per cambiare schermata
        int buttonState = digitalRead(0);
        if(buttonState == LOW && lastBootButtonState == HIGH){  // cambio solo al fronte di pressione
            currentScreen++;
            if(currentScreen > 3) currentScreen = 1;
        }
        lastBootButtonState = buttonState;

        // Aggiornamento schermata
        switch(currentScreen){
            case 1: updateScreen1(speed,rpm,SerialBT.connected()); break;
            case 2: updateScreen2(SerialBT.connected()); break;
            case 3: updateScreen3(SerialBT.connected()); break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- Setup ---
void setup(){
    Serial.begin(115200);
    obd_data_queue = xQueueCreate(10,sizeof(char*));
    if(obd_data_queue==NULL){ while(1); }

    xTaskCreatePinnedToCore(obd_task,"OBD Task",4096,NULL,5,NULL,1);
    xTaskCreatePinnedToCore(display_task,"Display Task",4096,NULL,4,NULL,0);
}

// --- Loop ---
void loop(){ vTaskDelay(1); }
