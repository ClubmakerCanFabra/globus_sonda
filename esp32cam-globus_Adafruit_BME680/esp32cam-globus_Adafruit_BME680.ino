/*********
  Inspirat en el projecte de Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card

  IMPORTANT!!!
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to put the board in flashing mode to upload a sketch
*********/
// Als fitxers de l'esp_cam cal renombrar el typedef sensor_t per sensor_tcam

#include "esp_camera.h"        // Camera de l'ESP32-CAM
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include <Wire.h>              // Bus I2C pels sensors
#include <Adafruit_Sensor.h>   // Llibreria general de sensors d'Adafruit
#include <Adafruit_BME680.h>   // Sensor de Temperatura, Humitat, Pressió i Qualitat aire
#include <WiFi.h>              // Wifi pel Network Time Protocol
#include "time.h"              // Real Time Clock

#define BUILD_IN_LED 33     // Build-in Led in ESP32-CAM located in GPIO33
#define STOP_PIN     16     // GPIO16 per donar l'ordre de tancar fitxers i acabar el codi

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Desactivar l'auto-flash al fer una foto
#define FLASH_GPIO_NUM 4
#define FLASH_BRIGHTNESS 0
const int freq = 5000;
const int ledChannel = LEDC_CHANNEL_6;

// Variable de configuració de la càmera
camera_config_t config_cam;

// Pins bus I2C i instància per utilitzar aquests pins escollits
#define I2C_SDA 1    // SDA Connected to GPIO 1
#define I2C_SCL 3    // SCL Connected to GPIO 3
TwoWire I2CSensors = TwoWire(0);

// Definim el sensor BME680 instanciat als nous pins
Adafruit_BME680 bme_sensor(&I2CSensors);

// Credencials Wifi d'accés
const char* ssid = "";
const char* password = "";

// Configuració Servidor NTP (Network Time Protocol)
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = (+1) * 3600;   // GMT+1
const int daylightOffset_sec = 3600;    // Horari estiu (+1h)

void setup() {
    pinMode(BUILD_IN_LED, OUTPUT);      // Setup Build-in red LED
    digitalWrite(BUILD_IN_LED, HIGH);   // Apaga el LED

    pinMode(STOP_PIN, INPUT_PULLUP);    // Setup Entrada per finalitzar el codi

    Serial.begin(115200);
    while (!Serial) delay(10);    // wait for console

    config_cam.ledc_channel = LEDC_CHANNEL_0;
    config_cam.ledc_timer = LEDC_TIMER_0;
    config_cam.pin_d0 = Y2_GPIO_NUM;
    config_cam.pin_d1 = Y3_GPIO_NUM;
    config_cam.pin_d2 = Y4_GPIO_NUM;
    config_cam.pin_d3 = Y5_GPIO_NUM;
    config_cam.pin_d4 = Y6_GPIO_NUM;
    config_cam.pin_d5 = Y7_GPIO_NUM;
    config_cam.pin_d6 = Y8_GPIO_NUM;
    config_cam.pin_d7 = Y9_GPIO_NUM;
    config_cam.pin_xclk = XCLK_GPIO_NUM;
    config_cam.pin_pclk = PCLK_GPIO_NUM;
    config_cam.pin_vsync = VSYNC_GPIO_NUM;
    config_cam.pin_href = HREF_GPIO_NUM;
    config_cam.pin_sscb_sda = SIOD_GPIO_NUM;
    config_cam.pin_sscb_scl = SIOC_GPIO_NUM;
    config_cam.pin_pwdn = PWDN_GPIO_NUM;
    config_cam.pin_reset = RESET_GPIO_NUM;
    config_cam.xclk_freq_hz = 20000000;
    config_cam.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()) {
        config_cam.frame_size = FRAMESIZE_UXGA;   // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
        config_cam.jpeg_quality = 10;
        config_cam.fb_count = 2;
    }
    else {
        config_cam.frame_size = FRAMESIZE_SVGA;
        config_cam.jpeg_quality = 12;
        config_cam.fb_count = 1;
    }

    // Init Camera
    esp_err_t err = esp_camera_init(&config_cam);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        lockError(1);
    }

    // sensor_tcam *s = esp_camera_sensor_get();
    // s->set_brightness(s, -2);
    // s->set_exposure_ctrl(s, 1);
    // s->set_ae_level(s, 2);

    // https://www.reddit.com/r/esp32/comments/nvgt1p/disabling_auto_flash_on_the_esp32_cam/h13ciph/?utm_source=amp&utm_medium=
    // https://github.com/neil-morrison44/drawdate/blob/master/preview_box/src/camera.cpp
    // pinMode(FLASH_GPIO_NUM, OUTPUT);
    ledcSetup(ledChannel, freq, 8);
    ledcAttachPin(FLASH_GPIO_NUM, ledChannel);
    ledcWrite(ledChannel, FLASH_BRIGHTNESS);

    // Start SD Card
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("SD Card Mount Failed");
        lockError(2);
    }
    // Check SD Card
    if (SD_MMC.cardType() == CARD_NONE) {
        Serial.println("No SD Card attached");
        lockError(3);
    }

    // Inicia el bus I2C sobre els pins particulars escollits
    I2CSensors.begin(I2C_SDA, I2C_SCL);

    // Inicialitza el BME680
    if (!bme_sensor.begin()) {     // BME680 address 0x77
        Serial.println("Error amb el sensor BME680");
        lockError(4);
    }

    // Set up oversampling and filter initialization
    bme_sensor.setTemperatureOversampling(BME680_OS_8X);
    bme_sensor.setHumidityOversampling(BME680_OS_2X);
    bme_sensor.setPressureOversampling(BME680_OS_4X);
    bme_sensor.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme_sensor.setGasHeater(320, 150);   // Set gas sensor heater to 320ºC for 150ms

    // Connectem al Wifi per obtenir la data i hora
    Serial.printf("Connectant a la xarxa %s ", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        digitalWrite(BUILD_IN_LED, LOW);
        delay(200);
        digitalWrite(BUILD_IN_LED, HIGH);
        delay(200);
    }
    Serial.println(" CONNECTAT");

    // Actualitzem el nostre RTC intern
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    readLocalTime();

    // Desconnectem el Wifi ja que ja no cal més
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    timelapse();
    Serial.println("Timelapse finalitzat");
}

void timelapse() {
    fs::FS &fs = SD_MMC;
    String sensors_filename = "/dades_ambientals.csv";
    File sensors_file = fs.open(sensors_filename.c_str(), FILE_WRITE);
    if (!sensors_file) {
        Serial.println("Failed to open sensors file in writing mode");
        lockError(5);
    }

    // Capçalera de les columnes de les dades en format full de càlcul
    String sensors_data = "Lectura,Timestamp [ms],Data_Hora,Temp [ºC],Humitat [%],Pressio [hPa],Altura [m],Alt_Calc [m],Alt_Calc2 [m],Gas [Ohm]";
    Serial.println(sensors_data);
    sensors_file.println(sensors_data);

    unsigned long shoot_time = millis() + 1000;    // Retard inicial
    float pressio_inicial = bme_sensor.readPressure() / 100.0;

    int pictureNumber = 0;
    while (digitalRead(STOP_PIN) == HIGH) {     // Captura dades mentre el GPIO16 sigui High
        unsigned long now_time = millis();
        if (now_time > shoot_time) {
            shoot_time += 1000;     // Captura dades cada 1 seg (limitat a la velocitat de la càmera)
            pictureNumber++;

            // Fa i emmagatzema una foto
            digitalWrite(BUILD_IN_LED, LOW);    // Turn on Built-in Led
            take_picture(pictureNumber);
            digitalWrite(BUILD_IN_LED, HIGH);    // Turn off Built-in Led

            // Llegeix dades del sensor BME680
            sensors_data = String(pictureNumber);
            sensors_data += "," + String(now_time);
            sensors_data += "," + readLocalTime();
            sensors_data += "," + String(bme_sensor.readTemperature());
            sensors_data += "," + String(bme_sensor.readHumidity());
            float pressio_actual = bme_sensor.readPressure() / 100.0;
            sensors_data += "," + String(pressio_actual);
            sensors_data += "," + String(bme_sensor.readAltitude(pressio_inicial));
            // Equation taken from BMP180 datasheet
            // http://forums.adafruit.com/viewtopic.php?f=22&t=58064
            float altura = 44330.0 * (1.0 - pow(pressio_actual / pressio_inicial, 0.1903));
            sensors_data += "," + String(altura);
            altura = (pressio_inicial - pressio_actual) / 0.1196;
            sensors_data += "," + String(altura);
            sensors_data += "," + String(bme_sensor.readGas() / 1000.0);
            Serial.println(sensors_data);

            // Emmagatzema les dades ambientals al fitxer
            sensors_file.println(sensors_data);
            Serial.printf("Saved sensors values %s to sensors file\n", String(pictureNumber));
        }
        delay(1);
    }
    sensors_file.close();
}

void take_picture(int num) {
    // Take a picture with the camera
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
    }
    else {
        // Path where new picture will be saved in SD Card
        String path = "/picture" + String(num) + ".jpg";

        fs::FS &fs = SD_MMC;
        File file = fs.open(path.c_str(), FILE_WRITE);
        if (!file) {
            Serial.println("Failed to open picture file in writing mode");
        }
        else {
            file.write(fb->buf, fb->len);   // payload (image), payload length
            Serial.printf("Saved picture file to path: %s\n", path.c_str());
        }
        file.close();
    }
    // Return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
}

String readLocalTime() {
    struct tm timeinfo;
    char DataHora[20];

    if(!getLocalTime(&timeinfo)){
        Serial.println("Failed to obtain time");
        return "";
    }
    //Serial.println(&timeinfo, "%d/%m/%Y %H:%M:%S");
    //Serial.println(&timeinfo);
    strftime(DataHora, sizeof(DataHora), "%d/%m/%Y %H:%M:%S", &timeinfo);
    return String(DataHora);
}

void lockError(int error) {
    while (true) {
        Serial.print("Critical Error ");   Serial.println(error);
        for (int i=0; i<error; i++) {
            digitalWrite(BUILD_IN_LED, LOW);
            delay(200);
            digitalWrite(BUILD_IN_LED, HIGH);
            delay(200);
        }
        delay(3000);
    }
}

void loop() {
    Serial.println("Procés finalitzat");
    while (true) { delay(1); }
}
