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
    if (strlen(data) < 10) return -1;
    char hex_str[3]; int A, B;
    strncpy(hex_str, data+6, 2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    strncpy(hex_str, data+9, 2); hex_str[2]='\0'; sscanf(hex_str,"%x",&B);
    return (A*256+B)/4;
}

int decode_speed(const char* data) {
    if (strlen(data) < 7) return -1;
    char hex_str[3]; int A;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    return A;
}

int decode_temp(const char* data) {
    if (strlen(data) < 7) return -1;
    char hex_str[3]; int A;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    return A - 40;
}

int decode_load(const char* data) {
    if (strlen(data) < 7) return -1;
    char hex_str[3]; int A;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    return (A*100)/255;
}

int decode_maf(const char* data) {
    if (strlen(data) < 10) return -1;
    char hex_str[3]; int A,B;
    strncpy(hex_str, data+6, 2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    strncpy(hex_str, data+9, 2); hex_str[2]='\0'; sscanf(hex_str,"%x",&B);
    return (A*256+B)/100; // g/s
}

int decode_tps(const char* data) {
    if (strlen(data) < 7) return -1;
    char hex_str[3]; int A;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    return (A*100)/255;
}

int decode_fuel(const char* data) {
    if (strlen(data) < 7) return -1;
    char hex_str[3]; int A;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    return (A*100)/255;
}

int decode_fuel_pressure(const char* data) { // PID 0A
    if (strlen(data) < 7) return -1;
    char hex_str[3]; int A;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    return A*3; // kPa
}

int decode_intake_pressure(const char* data) { // PID 0B
    if (strlen(data) < 7) return -1;
    char hex_str[3]; int A;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    return A; // kPa
}

int decode_run_time(const char* data) { // PID 1F
    if (strlen(data) < 10) return -1;
    char hex_str[3]; int A,B;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    strncpy(hex_str, data+9,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&B);
    return A*256+B; // secondi
}

int decode_distance_mil(const char* data) { // PID 21
    if (strlen(data) < 10) return -1;
    char hex_str[3]; int A,B;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    strncpy(hex_str, data+9,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&B);
    return A*256+B; // km
}

int decode_egr_error(const char* data) { // PID 2D
    if (strlen(data) < 7) return -1;
    char hex_str[3]; int A;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    return (A*100)/255; // %
}

int decode_ambient_temp(const char* data) { // PID 46
    return decode_temp(data);
}

int decode_oil_temp(const char* data) { // PID 5C
    return decode_temp(data);
}

int decode_ecu_voltage(const char* data) { // PID 42
    if (strlen(data) < 10) return -1;
    char hex_str[3]; int A,B;
    strncpy(hex_str, data+6,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&A);
    strncpy(hex_str, data+9,2); hex_str[2]='\0'; sscanf(hex_str,"%x",&B);
    return (A*256+B)/1000; // Volt
}

struct VehicleData {
    int rpm = -1;
    int speed = -1;            // km/h
    int coolantTemp = -1;      // °C
    int intakeTemp = -1;       // °C
    int engineLoad = -1;       // %
    int maf = -1;              // g/s
    int tps = -1;              // %
    int fuelLevel = -1;        // %
    int fuelPressure = -1;     // kPa
    int intakePressure = -1;   // kPa
    int engineRunTime = -1;    // s
    int distanceMIL = -1;      // km
    int egrError = -1;         // %
    int ambientTemp = -1;      // °C
    int oilTemp = -1;          // °C
    float ecuVoltage = -1;     // V
};

// Funzione di aggiornamento dei valori
void updateVehicleData(const char* pidResponse, int pidIndex, VehicleData &data) {
    switch(pidIndex){
        case 0: data.rpm = decode_rpm(pidResponse); break;
        case 1: data.speed = decode_speed(pidResponse); break;
        case 2: data.coolantTemp = decode_temp(pidResponse); break;
        case 3: data.intakeTemp = decode_temp(pidResponse); break;
        case 4: data.engineLoad = decode_load(pidResponse); break;
        case 5: data.maf = decode_maf(pidResponse); break;
        case 6: data.tps = decode_tps(pidResponse); break;
        case 7: data.fuelLevel = decode_fuel(pidResponse); break;
        case 8: data.fuelPressure = decode_fuel_pressure(pidResponse); break;
        case 9: data.intakePressure = decode_intake_pressure(pidResponse); break;
        case 10: data.engineRunTime = decode_run_time(pidResponse); break;
        case 11: data.distanceMIL = decode_distance_mil(pidResponse); break;
        case 12: data.egrError = decode_egr_error(pidResponse); break;
        case 13: data.ambientTemp = decode_ambient_temp(pidResponse); break;
        case 14: data.oilTemp = decode_oil_temp(pidResponse); break;
        case 15: data.ecuVoltage = decode_ecu_voltage(pidResponse); break;
    }
}

// Task OBD-II
void obd_task(void* parameter) {
    SerialBT.begin("ESP32_OBDII_Client", true);
    SerialBT.setPin(pin);

    const char* commands[] = {
        "010C\r","010D\r","0105\r","010F\r","0104\r","0110\r","0111\r","012F\r",
        "0A\r","0B\r","1F\r","21\r","2D\r","46\r","5C\r","42\r"
    };
    int commandIndex = 0;

    VehicleData vehicleData;  // Struct condivisa con display

    while(1) {
        if(!SerialBT.connected()){
            if(SerialBT.connect(obdii_addr)) Serial.println("Connected!");
            else { vTaskDelay(pdMS_TO_TICKS(5000)); continue; }
        }

        if(SerialBT.connected()) {
            SerialBT.print(commands[commandIndex]);
            String response = "";
            unsigned long startWait = millis();
            while(millis()-startWait<2000){
                while(SerialBT.available()) response += (char)SerialBT.read();
                if(response.indexOf('>')!=-1) break;
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if(response.length()>0){
                Serial.printf("RAW[%s]: %s\n", commands[commandIndex], response.c_str());
                updateVehicleData(response.c_str(), commandIndex, vehicleData);

                // Stampa tutti i valori aggiornati
                Serial.printf("RPM: %d | Speed: %d km/h | Coolant: %d°C | Intake: %d°C | Load: %d%%\n",
                              vehicleData.rpm, vehicleData.speed, vehicleData.coolantTemp,
                              vehicleData.intakeTemp, vehicleData.engineLoad);
                Serial.printf("MAF: %d g/s | TPS: %d%% | Fuel: %d%% | Fuel Pressure: %d kPa | Intake Pressure: %d kPa\n",
                              vehicleData.maf, vehicleData.tps, vehicleData.fuelLevel,
                              vehicleData.fuelPressure, vehicleData.intakePressure);
                Serial.printf("RunTime: %d s | Dist MIL: %d km | EGR: %d%% | Ambient: %d°C | Oil: %d°C | ECU V: %.2f\n\n",
                              vehicleData.engineRunTime, vehicleData.distanceMIL,
                              vehicleData.egrError, vehicleData.ambientTemp,
                              vehicleData.oilTemp, vehicleData.ecuVoltage);
            }

            commandIndex = (commandIndex+1)%16;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// --- Top bar: temperatura al centro, Bluetooth a destra ---
void drawTopBar(bool btConnected, int ambientTemp){
    tft.fillRect(0, 0, tft.width(), 30, TFT_BLACK); // sfondo

    // Bluetooth a destra
    int rectW = 50, rectH = 20;
    int rectX = tft.width() - rectW - 5, rectY = 5;
    uint16_t btColor = btConnected ? TFT_GREEN : TFT_RED;
    tft.fillRect(rectX, rectY, rectW, rectH, btColor);
    tft.setTextColor(TFT_BLACK, btColor);
    tft.setTextFont(2);
    tft.setCursor(rectX + 5, rectY + 2);
    tft.print(btConnected ? "ON" : "OFF");

    // Temperatura al centro
    String tempStr = String(ambientTemp) + "°C";
    int w = tft.textWidth(tempStr);
    int x = (tft.width() - w)/2;
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(x, rectY + 2);
    tft.print(tempStr);
}

// --- Schermata 1: velocità, RPM, fuel ---
void updateScreen1(const VehicleData &vData, bool btConnected){
    tft.fillRect(0, 30, tft.width(), tft.height()-30, TFT_BLACK); // cancella sotto top bar
    drawTopBar(btConnected, vData.ambientTemp);

    // --- Fuel level a sinistra ---
    int fuelBarH = 120, fuelBarW = 15;
    int fuelX = 10, fuelY = 20; // più in alto
    tft.drawRect(fuelX,fuelY,fuelBarW,fuelBarH,TFT_WHITE);

    // divisione in 4 zone
    for(int i=1;i<4;i++){
        int yLine = fuelY + i*fuelBarH/4;
        tft.drawLine(fuelX, yLine, fuelX+fuelBarW, yLine, TFT_WHITE);
    }

    // riempimento fuel
    int fillH = (vData.fuelLevel*fuelBarH)/100;
    tft.fillRect(fuelX, fuelY + fuelBarH - fillH, fuelBarW, fillH, TFT_YELLOW);

    // tacca finale rossa in alto
    tft.drawLine(fuelX, fuelY, fuelX+fuelBarW, fuelY, TFT_RED);

    // F ed E
    tft.setTextFont(2); tft.setTextColor(TFT_WHITE);
    tft.setCursor(fuelX + fuelBarW + 2, fuelY - 2);
    tft.print("F");
    tft.setCursor(fuelX + fuelBarW + 2, fuelY + fuelBarH - 12);
    tft.print("E");

     // --- Semicerchio centrale ---
    int centerX = tft.width()/2;
    int centerY = 160;
    int arcRadius = 100;
    int arcThickness = 4;

    tft.drawArc(centerX, centerY, arcRadius, arcRadius - arcThickness,
                180, 360, TFT_WHITE, TFT_BLACK, true);

    // velocità al centro
    tft.setTextFont(6); // dimensione più adatta
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    int speedW = tft.textWidth(String(vData.speed));
    tft.setCursor(centerX - speedW/2, centerY - 20);
    tft.print(vData.speed);

    // km/h a destra del semicerchio
    tft.setTextFont(2);
    tft.setCursor(centerX + arcRadius + 5, centerY - 10);
    tft.print("km/h");

    // --- Barra RPM sotto la velocità ---
    int barX = 50, barY = centerY + arcRadius + 20, barW = tft.width() - 100, barH = 20;
    float rpmRatio = (float)vData.rpm/6000;
    if(rpmRatio>1.0) rpmRatio=1.0;

    // sfondo barra
    tft.fillRect(barX, barY, barW, barH, TFT_DARKGREY);

    // riempimento barra
    uint16_t rpmColor = (vData.rpm<3000)?TFT_GREEN:(vData.rpm<4500)?TFT_YELLOW:TFT_RED;
    tft.fillRect(barX, barY, (int)(barW*rpmRatio), barH, rpmColor);

    // testo RPM dentro barra
    tft.setTextFont(2);
    tft.setTextColor(TFT_BLACK, rpmColor);
    tft.setCursor(barX + barW/2 - 15, barY + 2);
    tft.print("RPM");
}


// --- Schermata 2: parametri motore ---
void updateScreen2(const VehicleData &vData, bool btConnected){
    tft.fillRect(0,40,tft.width(),tft.height()-40,TFT_BLACK);
    drawTopBar(btConnected,vData.ambientTemp);

    int x=20, y=60, spacingY=30;
    tft.setTextFont(2); tft.setTextColor(TFT_WHITE);

    tft.setCursor(x,y); tft.printf("Coolant: %d°C", vData.coolantTemp); y+=spacingY;
    tft.setCursor(x,y); tft.printf("Intake: %d°C", vData.intakeTemp); y+=spacingY;
    tft.setCursor(x,y); tft.printf("Oil Temp: %d°C", vData.oilTemp); y+=spacingY;
    tft.setCursor(x,y); tft.printf("Engine Load: %d%%", vData.engineLoad); y+=spacingY;
    tft.setCursor(x,y); tft.printf("MAF: %d g/s", vData.maf); y+=spacingY;
    tft.setCursor(x,y); tft.printf("TPS: %d%%", vData.tps);
}

// --- Schermata 3: ECU e pressioni ---
void updateScreen3(const VehicleData &vData, bool btConnected){
    tft.fillRect(0,40,tft.width(),tft.height()-40,TFT_BLACK);
    drawTopBar(btConnected,vData.ambientTemp);

    int x=20, y=60, spacingY=30;
    tft.setTextFont(2); tft.setTextColor(TFT_WHITE);

    tft.setCursor(x,y); tft.printf("Fuel Level: %d%%", vData.fuelLevel); y+=spacingY;
    tft.setCursor(x,y); tft.printf("Fuel Press: %d kPa", vData.fuelPressure); y+=spacingY;
    tft.setCursor(x,y); tft.printf("Intake Press: %d kPa", vData.intakePressure); y+=spacingY;
    tft.setCursor(x,y); tft.printf("ECU Volt: %.2f V", vData.ecuVoltage); y+=spacingY;
    tft.setCursor(x,y); tft.printf("Run Time: %d s", vData.engineRunTime); y+=spacingY;
    tft.setCursor(x,y); tft.printf("Distance MIL: %d km", vData.distanceMIL);
}

// --- Task display aggiornato ---
void display_task(void* parameter){
    tft.init(); 
    tft.setRotation(1);

    VehicleData vehicleData;  // aggiornato dal task OBD
    int lastBootButtonState = digitalRead(0);  
    pinMode(0, INPUT_PULLUP);

    while(1){
        // Lettura tasto per cambiare schermata
        int buttonState = digitalRead(0);
        if(buttonState == LOW && lastBootButtonState == HIGH){
            currentScreen++;
            if(currentScreen>3) currentScreen=1;
        }
        lastBootButtonState = buttonState;

        // Aggiornamento schermata in base a currentScreen
        switch(currentScreen){
            case 1: updateScreen1(vehicleData, SerialBT.connected()); break;
            case 2: updateScreen2(vehicleData, SerialBT.connected()); break;
            case 3: updateScreen3(vehicleData, SerialBT.connected()); break;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
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
