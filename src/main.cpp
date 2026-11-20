#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "STM32_CAN.h"
#include <HerkulexServo.h>
#include <AccelStepper.h>


const int PIN_STEP = PA4;
const int PIN_DIR  = PA3;
const int PIN_EN   = PB1;
const int PIN_STBY = PA5;
const int PIN_M0 = PB0; 
const int PIN_N1 = PA7;
const int PIN_N2 = PA6;


HerkulexServoBus herkulex_bus(Serial1);
HerkulexServo my_servo(herkulex_bus, 0xFD);

STM32_CAN Can1(CAN1, DEF); // PA 11 et 12

QueueHandle_t queue_data; // file pour les trames recues
QueueHandle_t q_envoie_pos; // file pour la position 
QueueHandle_t q_tx_msg; // file pour les trames à envoyer
QueueHandle_t q_dist_msg; // file pour la distance 
SemaphoreHandle_t Mutex;
SemaphoreHandle_t Mutex_servo;
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

void envoie_ordre(int position_deg, float vitesse);

 void passerelle_CAN(void *pvParameters);
void tacheCAN_TX(void *pvParameters);
void gestion_servo(void *pvParameters);
void tacheCAN_RX(void *pvParameters);
void gestion_moteur(void *pvParameters);
void setup()
{
    Serial.begin(115200);
    Serial1.setRx(PB_7);
    Serial1.setTx(PB_6);
    Serial1.begin(115200);
    // Serial.println("Serial.begin");
    Mutex = xSemaphoreCreateMutex();
    Mutex_servo=xSemaphoreCreateMutex();
    Can1.begin();
    Can1.setBaudRate(1000000);
    queue_data = xQueueCreate(8, sizeof(CAN_message_t));
    q_tx_msg = xQueueCreate(8, sizeof(CAN_message_t));
    q_envoie_pos = xQueueCreate(1, sizeof(CAN_message_t));
    q_dist_msg=xQueueCreate(8, sizeof(CAN_message_t));
     pinMode(PIN_STBY, OUTPUT);
    pinMode(PIN_EN,   OUTPUT);
    pinMode(PIN_M0,   OUTPUT);
    pinMode(PIN_N1,   OUTPUT);
    pinMode(PIN_N2,   OUTPUT);

    digitalWrite(PIN_STBY, HIGH); 
    digitalWrite(PIN_EN,   HIGH);   
    digitalWrite(PIN_M0, LOW);    
    digitalWrite(PIN_N1, LOW);
    digitalWrite(PIN_N2, LOW);

    stepper.enableOutputs();

xTaskCreate(tacheCAN_RX,    "CAN_RX",    128, NULL, 1, NULL);
 xTaskCreate(passerelle_CAN, "passerelle", 256, NULL, 1, NULL);
 xTaskCreate(tacheCAN_TX,    "CAN_TX",    128, NULL, 1, NULL);
//xTaskCreate(gestion_servo,  "servo",     256, NULL, 1, NULL);
xTaskCreate(gestion_moteur, "moteur",    256, NULL, 1, NULL);

    vTaskStartScheduler(); // lance le scheduler

    // Affiche si y'a un problème de mémoire
    Serial.println("Insufficient RAM");

    while (1)
        ;
}

void loop()
{
}
void tacheCAN_TX(void *pvParameters)
{
    CAN_message_t msg;
    uint8_t erreur;

    while (1)
    {
        if (xQueueReceive(q_tx_msg, &msg, portMAX_DELAY))
        {
            Can1.write(msg);

            if (msg.id == 0x20)
            {
                 erreur = msg.buf[0];

                xSemaphoreTake(Mutex, portMAX_DELAY); 
                if ((erreur & 0x10)==0x10) Serial.println("Alerte blocage");
                if ((erreur & 0x04)==0x04) Serial.println("Alerte surchauffe");
                if ((erreur & 0x01)==0x01) Serial.println("Alerte tension");
                if ((erreur & 0x02)==0x02) Serial.println("Angle max dépassé");
                if ((erreur & 0x08)==0x08) Serial.println("packet invalide");

                xSemaphoreGive(Mutex);
            }
        }
    }
}

    // vTaskDelay(pdMS_TO_TICKS(1000));


void tacheCAN_RX(void *pvParameters)
{
    CAN_message_t CAN_RX_msg; // message CAN pour la reception
    while (1)
    {
        if (Can1.read(CAN_RX_msg))
        {
            xQueueSend(queue_data, &CAN_RX_msg, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void passerelle_CAN(void *pvParameters)
{
    CAN_message_t CAN_RX_recue;
    CAN_message_t position;
   // CAN_message_t distance;
    int16_t angle; 
    uint8_t servo_id;
    int servo_degres;
    int dist_moteur;

    while (1)
    {
        xQueueReceive(queue_data, &CAN_RX_recue, portMAX_DELAY);

        if (CAN_RX_recue.id == 0x44)
        {
        xSemaphoreTake(Mutex_servo, portMAX_DELAY);
        servo_id = my_servo.getID();
        servo_degres = (my_servo.getPosition() - 512) * 0.325;
        xSemaphoreGive(Mutex_servo);
        position.id = 0x100;
        position.len = 3;
        position.buf[0] = (uint8_t)(servo_id); 
        position.buf[1] = (uint8_t)(servo_degres);
        position.buf[2] = (uint8_t)(servo_degres >> 8);
         xSemaphoreTake(Mutex, portMAX_DELAY);
        Serial.printf("BIEN");
        xSemaphoreGive(Mutex);       
        xQueueSend(q_tx_msg,&position,0);
        angle = (int16_t)(position.buf[1] | (position.buf[2] << 8));
        xSemaphoreTake(Mutex, portMAX_DELAY);
        Serial.printf("la position du servo %02X est %d°\n", position.buf[0], angle);
        xSemaphoreGive(Mutex);
        }

        if(CAN_RX_recue.id == 0x10) // ordre pour la distance du pas à pas 
        {
            
            xQueueSend(q_dist_msg,&CAN_RX_recue,0);
            // dist_moteur=CAN_RX_recue.buf[0]|(CAN_RX_recue.buf[1]<<8);
            // xQueueOverwrite(q_dist_msg,&dist_moteur);
        }


        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void gestion_servo(void *pvParameters)
{
    CAN_message_t erreur_servo;
    HerkulexStatusError erreur;
    HerkulexStatusDetail details;   
    int angle_cons=0;
    xSemaphoreTake(Mutex_servo, portMAX_DELAY);
    my_servo.setTorqueOn();
    envoie_ordre(angle_cons, 5);
    xSemaphoreGive(Mutex_servo);
    while (1)
    {
        erreur_servo.id = 0x20;
        erreur_servo.len = 1;
        xSemaphoreTake(Mutex_servo, portMAX_DELAY);
        my_servo.getStatus(erreur, details);
         int16_t angle_arrive= (my_servo.getPosition() - 512) * 0.325;
        uint8_t code_erreur = static_cast<uint8_t>(erreur);
        xSemaphoreGive(Mutex_servo);
        if ((code_erreur != 0x00))
        {
            erreur_servo.buf[0] = code_erreur;
            xQueueSend(q_tx_msg, &erreur_servo, portMAX_DELAY);
        }
        if (abs(angle_cons-angle_arrive)<=1)
         {
            xSemaphoreTake(Mutex_servo, portMAX_DELAY);
            my_servo.setTorqueOff();
            xSemaphoreGive(Mutex_servo);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void envoie_ordre(int position_deg, float vitesse)
{

    uint16_t position_brute;
    float temps_ms;
    uint8_t play_time;
    int pos_brute_depart = my_servo.getPosition();
    int pos_deg_depart = (pos_brute_depart - 512) * 0.325;
    int delta_pos;
    if (position_deg > 160)
        position_deg = 160;
    if (position_deg < -160)
        position_deg = -160;
    position_brute = 512 + ((position_deg) / 0.325);
    delta_pos = abs(position_deg - pos_deg_depart);
    temps_ms = (delta_pos / vitesse) * 1000;
    play_time = temps_ms / 11.2;
    if (play_time > 255)
    {
        play_time = 255;
    }
    herkulex_bus.prepareIndividualMove();
    my_servo.setPosition(position_brute, play_time);
    herkulex_bus.executeMove();
}

void gestion_moteur(void *pvParameters)
{
    CAN_message_t CAN_distance;
    int16_t pas;

    stepper.setMaxSpeed(800);       
    stepper.setAcceleration(400);   
 
   TickType_t xLastWakeTime = xTaskGetTickCount();   

    while (1)
    {
        if (xQueueReceive(q_dist_msg, &CAN_distance, 0) == pdTRUE)
        {
            
            pas = (uint16_t)(CAN_distance.buf[0] | (CAN_distance.buf[1] << 8));
            stepper.move(pas);
        xSemaphoreTake(Mutex, portMAX_DELAY);
        Serial.printf("la distance du moteur est %d\n", pas);
        xSemaphoreGive(Mutex);
           
        }
        stepper.run();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));   // période fixe de 1 ms
    }
}