#include "Arduino.h"
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_freertos_hooks.h"

Servo myservo; // Biblioteca Servo

//-------- Ecrã LCD -------

TFT_eSPI tft = TFT_eSPI();       // Biblioteca TFT_eSPI

static uint8_t conv2d(const char* p); // Forward declaration needed for IDE 1.6.x

uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); // Get H, M, S from compile time

byte omm = 99, oss = 99;
byte xcolon = 0, xsecs = 0;
//------------------------

//------- Estruturas de dados ---------
typedef struct temphumlumraw{
	uint32_t rawtemp = 0;
	uint32_t rawhum =0;
	uint32_t rawlum1 =0;
	uint32_t rawlum2= 0;
}thlr;

typedef struct temphumlum{
	float temp =0.0;
	float hum =0.0;
	float lum =0.0;
}thl;

struct flags{
	bool t29 = false, t25 = false, t20 = false, t26 = false;
	bool hl75400 = false, hl50400 = false;
};
//---------------------------------------

bool check, manual = false; // flags
int SensorLum = 0x29; // Endereço Sensor de luminosidade I2c.
int SensorTemp_Hum = 0x40; // Endereço Sensor de temperatura e humidade I2c.

//-- variáveis globais servo motor
int pos = 0;
bool on = false;

volatile unsigned long ulIdleCycleCount = 0UL; // variável para a função IdleHook

//-- Tasks
void vTask0(void * pvParameters); // BRAIN/LCD
void vTask1(void * pvParameters); // RAW DATA
void vTask2(void * pvParameters); // CALCULOS
void vTask3(void * pvParameters); // ATUADORES
void vTask4(void * pvParameters); // BLE
void vTask5(void * pvParameters); // SERVO

#define Measure_temp 0xF3 // Endereço interno do registo da temperatura.
#define Measure_hum 0xF5 // Endereço interno do registo da humidade.

//-- Definir pinos I/O
#define I2C_SDA 21
#define I2C_SCL 22
#define LED_REGA 14
#define LED_RESISTENCIA 12
#define LED_LUZ 2
#define SERVO 25

//------- Queues / Semaphore----------
QueueHandle_t xQueue;
QueueHandle_t xQueue2;
QueueHandle_t xStringQueue;
QueueHandle_t xQueueFlags;
QueueHandle_t xQueueStates;
SemaphoreHandle_t xBinarySemaphore;

TaskHandle_t xTask2Handle, xTaskAtuadoresHandle, xTaskServoHandle;

//---------BLE---------------------------------------------
#define SERVICE_UUID "36353433-3231-3039-3837-363534333231"
#define CHARACTERISTIC_UUID "36353433-3231-3039-3837-363534336261"
bool deviceConnected;
char dataTxBLE[8];
BLECharacteristic *pCharacteristic;

//Callback principal do BLE ex: ligar/desligar
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.print("Conectado!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.print("Desconectado!");
    }
};
//Callback da escrita e leitura da carateristica
class MyCallbacks: public BLECharacteristicCallbacks {

	//Envia o valor previamente colocado dentro da variavle dataTxBLE
	void onRead(BLECharacteristic *pCharacteristic) {
		pCharacteristic->setValue(dataTxBLE); // Recebe a informação e coloca na variável dataTxBLE
	}

	void onNotify(BLECharacteristic *pCharacteristic) {
		pCharacteristic->setValue(dataTxBLE);
	}

	//Caso sao recebidos dados
    void onWrite(BLECharacteristic *pCharacteristic) {
    	static portBASE_TYPE xHigherPriorityTaskWoken;
       	xHigherPriorityTaskWoken = pdFALSE;

    	std::string rxValue = pCharacteristic->getValue(); //receber o valor como string
    	char dataRx [5];
    	strcpy(dataRx, rxValue.c_str()); //copiar num array de chars

    	xQueueSendToBackFromISR( xStringQueue, &dataRx, (portBASE_TYPE*)&xHigherPriorityTaskWoken ); //enviar no queue BLE

    	/* 'Give' the semaphore to unblock the task. */
    	xSemaphoreGiveFromISR( xBinarySemaphore,(portBASE_TYPE*)&xHigherPriorityTaskWoken ); //entregar o semaphore a task ble

    	if( xHigherPriorityTaskWoken == pdTRUE ){
    		vPortYield();
    	}
    }

};

//setup, corre so uma vez no inicio
void setup(){
	Serial.begin(115200);
	//-------- Ecrã LCD --------

	tft.init(); // inicialização TFT
	tft.setRotation(0);
	tft.fillScreen(TFT_BLACK);
	//--------- info do projeto
	tft.setTextSize(1);
	tft.setTextColor(TFT_YELLOW, TFT_BLACK);

	tft.drawString("Realizado por:", 70, 100,2);
	tft.drawString("Goncalo Lopes", 70, 120,2);
	tft.drawString("Mateo Rodriguez", 70, 140,2);

	delay(1000); // espera 1 segundo para ver

	//-------- escreve e desenha informações fixas
	tft.fillScreen(TFT_BLACK);
	tft.drawString("Projeto SCE", 85, 0,2);
	tft.drawLine(0,20,240,20,TFT_YELLOW);

	tft.fillRect(0, 25, 240, 20, TFT_BLUE);
	tft.drawRect(0, 25, 240, 115, TFT_BLUE);
	tft.setTextColor(TFT_WHITE, TFT_BLUE);
	tft.drawString("Valor de Sensores", 60, 28,2);

	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.drawString("Temperatura: ", 10, 55,2);
	tft.drawString("Humidade: ", 10, 83,2);
	tft.drawString("Luminosidade: ", 10, 110,2);

	tft.drawString("C", 180, 55,2);
	tft.drawString("RH%", 180, 83,2);
	tft.drawString("lux", 180, 110,2);

	tft.fillRect(0, 145, 240, 20, TFT_RED);
	tft.drawRect(0, 145, 240, 155, TFT_RED);
	tft.setTextColor(TFT_WHITE, TFT_RED);
	tft.drawString("Estado dos Atuadores", 55, 148,2);

	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	tft.drawString("Janela: ", 10, 185,2);
	tft.drawString("Resistência: ", 10, 225,2);
	tft.drawString("Rega: ", 10, 265,2);
	tft.drawString("Modo: ", 5, 300,2);
	tft.drawString("BLE: ", 125, 300,2);
	tft.setTextColor(TFT_GREEN, TFT_BLACK);
	tft.drawString("Automatico", 45, 300,2);
	tft.setTextColor(TFT_RED, TFT_BLACK);
	tft.drawString("Disconnected", 155, 300,2);

	tft.drawCircle(180, 193, 10, TFT_WHITE);
	tft.drawCircle(180, 233, 10, TFT_WHITE);
	tft.drawCircle(180, 273, 10, TFT_WHITE);

	//------ Wire Setup (I2C)
	Wire.begin(I2C_SDA, I2C_SCL, 100000);
	Wire.setClock(400000);

	//------- Criar os Queues
	xQueue = xQueueCreate(5, sizeof(thlr)); // Data dos sensores
	xQueue2 = xQueueCreate(5, sizeof(thl)); // Valores Convertidos para Celsius, RH e lux
	xQueueFlags = xQueueCreate(3, sizeof(flags)); // Data dos sensores
	xStringQueue = xQueueCreate(5, sizeof(char[5])); // Transmissão BLE

	//--------- BLE
	BLEDevice::init("LoPy"); // Give it a name // Create the BLE Device
	BLEServer *pServer = BLEDevice::createServer(); // Configura o dispositivo como Servidor BLE
	BLEService *pService = pServer->createService(SERVICE_UUID);  // Cria o serviço
	BLECharacteristic *pCharacteristic = pService->createCharacteristic(
			CHARACTERISTIC_UUID,
			BLECharacteristic::PROPERTY_READ |
			BLECharacteristic::PROPERTY_WRITE |
			BLECharacteristic::PROPERTY_NOTIFY); // Caracteristicas BLE, pode escrever e ler
	pCharacteristic->addDescriptor(new BLE2902());
	pServer->setCallbacks(new MyServerCallbacks());//asignar os callbacks
	pCharacteristic->setCallbacks(new MyCallbacks());//...
	pService->start();// Inicia o serviço BLE
	pServer->getAdvertising()->start();// Inicia a descoberta do ESP32
	Serial.println("À espera que um cliente se conecte...");
	pCharacteristic->setValue("on | off | t | h | l");//mensagem de leitura sem escrita previa

	//-------- IdleTaskHook
	esp_register_freertos_idle_hook(my_vApplicationIdleHook);

	//-------- Semáforo Binario
	vSemaphoreCreateBinary( xBinarySemaphore );

	// --------- I/O
	pinMode(LED_LUZ, OUTPUT);
	pinMode(LED_REGA, OUTPUT);
	pinMode(LED_RESISTENCIA, OUTPUT);
	myservo.attach(SERVO);
	myservo.write(0); // Inicializa com 0 graus

	// ---- Inicializar Queue Flags com tudo a false
	flags ff;
	    portBASE_TYPE xStatus = xQueueSendToBack(xQueueFlags, &ff, 0);
	    if(xStatus != pdPASS){
	        Serial.println(">>>>>>>>>>>>>>Couldnt send to queue flags");
	    }

	// -------- Tarefas
	if(xBinarySemaphore != NULL){
		xTaskCreatePinnedToCore(vTask4, "BLE", 1024, NULL, 7, NULL, 1);
	}
	xTaskCreatePinnedToCore(vTask0, "LCD", 1024, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(vTask1, "RAW DATA", 1024, NULL, 5, NULL, 1);
	xTaskCreatePinnedToCore(vTask2, "CALC DATA", 1024, NULL, 4, &xTask2Handle, 1);
}
//------ LCD
void vTask0(void * pvParameters) {
    //const char * pcTaskName = "--------------------------LCD-------------------------------\r\n";
    thl thlRx; //inicializa estrutura thlRx
    portBASE_TYPE xStatus;
    TickType_t xLastWakeTime;
    for (;;) {
    	//-------- Relógio canto superior esquerdo do ecrã
    	tft.setTextColor(TFT_YELLOW, TFT_BLACK);
		ss++;              // Advance second
		if (ss == 60) {    // Check for roll-over
			ss = 0;          // Reset seconds to zero
			omm = mm;        // Save last minute time for display update
			mm++;            // Advance minute
			if (mm > 59) {   // Check for roll-over
				mm = 0;
				hh++;          // Advance hour
				if (hh > 23) { // Check for 24hr roll-over (could roll-over on 13)
					hh = 0;      // 0 for 24 hour clock, set to 1 for 12 hour clock
				}
			}
		}


		// Update digital time
		int xpos = 0;
		int ypos = 0; // Top left corner ot clock text, about half way down

		if (omm != mm) { // Redraw hours and minutes time every minute
			omm = mm;
			// Draw hours and minutes
			if (hh < 10) xpos += tft.drawChar('0', xpos, ypos, 2); // Add hours leading zero for 24 hr clock
			xpos += tft.drawNumber(hh, xpos, ypos, 2);             // Draw hours
			xcolon = xpos; // Save colon coord for later to flash on/off
			xpos += tft.drawChar(':', xpos, ypos - 8, 2);
			if (mm < 10) xpos += tft.drawChar('0', xpos, ypos, 2); // Add minutes leading zero
			xpos += tft.drawNumber(mm, xpos, ypos, 2);             // Draw minutes
			xsecs = xpos; // Save seconds 'x' position for later display updates
		}
		if (oss != ss) { // Redraw seconds time every second
			oss = ss;
			xpos = xsecs;

			if (ss % 2) { // Flash the colons on/off
				tft.setTextColor(0x39C4, TFT_BLACK);        // Set colour to grey to dim colon
				tft.drawChar(':', xcolon, ypos, 2);     // Hour:minute colon
				xpos += tft.drawChar(':', xpos, ypos, 2); // Seconds colon
				tft.setTextColor(TFT_YELLOW, TFT_BLACK);    // Set colour back to yellow
			}
			else {
				tft.drawChar(':', xcolon, ypos, 2);     // Hour:minute colon
				xpos += tft.drawChar(':', xpos, ypos, 2); // Seconds colon
			}

			//Draw seconds
			if (ss < 10) xpos += tft.drawChar('0', xpos, ypos, 2); // Add leading zero
			tft.drawNumber(ss, xpos, ypos, 2);                     // Draw seconds
		}

    	if(uxQueueMessagesWaiting(xQueue2) > 1){ //Se existirem mais do que 1 valor na queue2 entra aqui
        	check = false; // Valores da Queue2 atualizados
				xStatus = xQueueReceive( xQueue2, &thlRx, 0);	//Recebe e remove valores da Queue2 (Queue dos dados calculados)
				xStatus = xQueuePeek( xQueue2, &thlRx, 0);
				if( xStatus == pdPASS ){
					Serial.print("LCD: Received = ");
					// ----- Converte float para string
					char txString_temp[8], txString_hum[8], txString_lum[8];
					dtostrf(thlRx.temp, 1, 1, txString_temp);
					dtostrf(thlRx.hum, 1, 1, txString_hum);
					dtostrf(thlRx.lum, 1, 0, txString_lum);

					// ----- Imprime valores no LCD
					tft.fillRect(110, 55, 50, 75, TFT_BLACK);
					tft.setTextColor(TFT_WHITE, TFT_BLACK);
					tft.drawString(txString_temp, 110, 55,2);
					tft.drawString(txString_hum, 110, 83,2);
					tft.drawString(txString_lum, 110, 110,2);

					Serial.print(thlRx.temp); Serial.print(thlRx.hum); Serial.println(thlRx.lum);
				}else{
					Serial.print( "LCD: Could not receive from the queue2.\r\n" );
				}

				// -- Atualizar estado dos atuadores LCD
				if(on){
					tft.fillCircle(180, 193, 10, TFT_GREEN);
				}else{
					tft.fillCircle(180, 193, 10, TFT_BLACK);
					tft.drawCircle(180, 193, 10, TFT_WHITE);
				}
				if(digitalRead(LED_RESISTENCIA)==HIGH){
					tft.fillCircle(180, 233, 10, TFT_GREEN);
				}else{
					tft.fillCircle(180, 233, 10, TFT_BLACK);
					tft.drawCircle(180, 233, 10, TFT_WHITE);
				}
				if(digitalRead(LED_REGA)==HIGH){
					tft.fillCircle(180, 273, 10, TFT_GREEN);
				}else{
	        		tft.fillCircle(180, 273, 10, TFT_BLACK);
	        		tft.drawCircle(180, 273, 10, TFT_WHITE);
				}
				// ----- Verifica se há algum dispositivo ligado via BLE
				if (deviceConnected == true){
					tft.setTextColor(TFT_GREEN, TFT_BLACK);
					tft.drawString("Connected    ", 155, 300,2);
				}else{
					manual =false;
					tft.setTextColor(TFT_RED, TFT_BLACK);
					tft.drawString("Disconnected", 155, 300,2);
				}
				// -- Atualizar modo LCD
				if (manual) {
					tft.setTextColor(TFT_ORANGE, TFT_BLACK);
					tft.drawString("Manual    ", 45, 300,2);
				}else{
					tft.setTextColor(TFT_GREEN, TFT_BLACK);
					tft.drawString("Automatico", 45, 300,2);
				}
        }
    	vTaskDelayUntil( & xLastWakeTime, (1000 / portTICK_PERIOD_MS)); // Insere um temporizador para voltar a correr esta tarefa ao fim de 1 segundo
    }
}
//------ Get Sensores
void vTask1(void * pvParameters) {
    TickType_t xLastWakeTime;
    const char * pcTaskName = "-------------------------------RAW DATA------------------------------\r\n";
    thlr thlrTx; // inicializa estrutura
    portBASE_TYPE xStatus;
    unsigned portBASE_TYPE uxPriority;
    xLastWakeTime = xTaskGetTickCount();
    int  data[4];
    for (;;) {
        Serial.print(pcTaskName);

        // ----------------- TEMPERATURA ---------------
		 Wire.beginTransmission(SensorTemp_Hum); // Inicia comunicação com o sensor Si7006-A20
		 Wire.write(Measure_temp); // Indica endereço da temperatura
		 Wire.requestFrom(SensorTemp_Hum,2); // Pede para devolver 2 bytes
		 int i=0;
		 while(Wire.available()) {  //
			 data[i] = Wire.read();
			 i ++;
		 }
		 Wire.endTransmission(); // Acaba transmissão I2C

		 thlrTx.rawtemp = float(((data[0])<<8) + (data[1])); // guarda valor na estrutura
	  // ----------------- HUMIDADE ---------------
		Wire.beginTransmission(SensorTemp_Hum);
		Wire.write(Measure_hum);
		Wire.requestFrom(SensorTemp_Hum,2);

		i=0;
		while(Wire.available()) {  //
		  data[i] = Wire.read();
		  i ++;
		 }
		Wire.endTransmission();

		thlrTx.rawhum = float(((data[0])<<8) + (data[1]));
	  // ----------------- LUMINOSIDADE ---------------
		i=0;
		Wire.beginTransmission(SensorLum);
		Wire.write(0x80); // Coloca ganhos a default
		Wire.write(0x85); // Integration time default
		Wire.endTransmission();
		Wire.beginTransmission(SensorLum);
		Wire.write(0x88); // Endereço do canal 1 do sensor
		Wire.endTransmission();
		Wire.requestFrom(SensorLum,4); // Pede 4 bytes, o sensor envia dados do canal 1 e 2
		while(Wire.available()) {  //
		  data[i] = Wire.read();
		  i ++;
		 }

		thlrTx.rawlum1 = float(((data[1])<<8) + (data[0])); // guarda valor do canal 1 do sensor na estrutura
		thlrTx.rawlum2 = float(((data[3])<<8) + (data[2])); // guarda valor do canal 2 do sensor na estrutura

		xStatus = xQueueSendToBack( xQueue, &thlrTx, 0); // coloca a estrutura na Queue

        if( xStatus != pdPASS ){
        	Serial.print( ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Could not send to the queue1.\r\n" );
        }

        if(uxQueueMessagesWaiting(xQueue2) <= 1){ // Se a queue 2 estiver vazia entra
			uxPriority = uxTaskPriorityGet( NULL); // Recebe prioridade da tarefa
			Serial.print("Raise the Task2 priority to "); Serial.println(uxPriority+1);
			vTaskPrioritySet(xTask2Handle, (uxPriority + 1)); //Aumenta a prioridade da tarefa dos cálculos para esvaziar a queue e colocar na queue2
        }
        vTaskDelayUntil( & xLastWakeTime, (4000 / portTICK_PERIOD_MS)); // Insere um temporizador para voltar a correr esta tarefa ao fim de 4 segundos
    }
}
//------ Cálculos
void vTask2(void * pvParameters) {
    TickType_t xLastWakeTime;
    const char * pcTaskName = "-----------------------------CALCULOS--------------------------\r\n";
    xLastWakeTime = xTaskGetTickCount();
    thlr thlrRx; // inicializa estrutura dos dados não calculados
    thl thlTx;	// inicializa estrutura dos dados calculados
    unsigned portBASE_TYPE uxPriority;
    portBASE_TYPE xStatus;
    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

    for (;;) {
        Serial.print(pcTaskName);
        xStatus = xQueueReceive( xQueue, &thlrRx, 0);	//recebe e elimina valores da queue
        if( xStatus == pdPASS ){
        	Serial.print("T2: Received = ");
        	Serial.print(thlrRx.rawtemp); Serial.print(thlrRx.rawhum); Serial.print(thlrRx.rawlum1); Serial.println(thlrRx.rawlum2);

        	taskENTER_CRITICAL(&myMutex);{  //Entra zona crítica

        	thlTx.temp= ((175.72*(thlrRx.rawtemp))/65536.0)-46.85; //Calcula valor da temperatura e insere na estrutura
        	thlTx.hum = ((125*(thlrRx.rawhum))/65536.0)-6;	//Calcula valor da humidade e insere na estrutura
        	thlTx.lum = ((thlrRx.rawlum1)+(thlrRx.rawlum2))/2;	///Calcula valor da luminosidade e insere na estrutura
        	Serial.print("T2: Will Send = ");
        	Serial.print(thlTx.temp); Serial.print(thlTx.hum); Serial.println(thlTx.lum);
        	xStatus = xQueueSendToBack( xQueue2, &thlTx, 0 ); // Insere estrutura na queue 2

        	}taskEXIT_CRITICAL(&myMutex); // Sai da zona critica

        	if( xStatus == pdPASS ){
        		Serial.println("Sent to queue2");
        	}else{
        		Serial.print( ">>>>>>>>>>>>>>>>>>>>>>>>T2: Could not send to the queue2.\r\n" );
        	}
        }else{
        	Serial.print( "T2: Could not receive from the queue1.\r\n" );
        }
        if(uxQueueMessagesWaiting(xQueue) <= 1){ // Se a queue (valores dos sensores) estiver vazia entra
			uxPriority = uxTaskPriorityGet( NULL); // Valor da prioridade da tarefa 2
			Serial.print("lower T2 priority "); Serial.println(uxPriority-2);
			vTaskPrioritySet( NULL, (uxPriority - 2)); // diminui a prioridade da tarefa 2 para a original, assim a tarefa 1 tem mais prioridade e enche a queue
        }
        vTaskDelayUntil( & xLastWakeTime, (4000 / portTICK_PERIOD_MS)); // Insere um temporizador para voltar a correr esta tarefa ao fim de 4 seundos
    }
}
//------ Atuadores
void vTask3(void * pvParameters) {
    const char * pcTaskName = "----------------------------ATUADORES-----------------------\r\n";
    thl thlRx; // inicializa estrutura dos dados calculados
    portBASE_TYPE xStatus;
    portBASE_TYPE xStatus2;
    flags fTx, fRx;
    for (;;) {
        Serial.print(pcTaskName);
        xStatus = xQueuePeek( xQueue2, &thlRx, 0);	//recebe dados da queue 2 (valores cálculados)
        xStatus2 = xQueueReceive( xQueueFlags, &fRx, 0);	//recebe e elimina valores da queue flags
        if( xStatus == pdPASS ){
        //condicoes para ativar o desativar os diferentes atuadores.
        //sao utilizadas condicoes e variaveis extra para evitar que o sistema
        //continuamente este a ligar ou desligar algum atuador se o mesmo ja se encontra no estado desejado

        	fTx = fRx;//copy structs
        	//------Temperatura---------
        	if(!fRx.t25 && thlRx.temp > 25){	//temp > 25 desliga o aquecimento
        		fTx.t25= true;
        		digitalWrite(LED_RESISTENCIA, LOW);
        	}else if(fRx.t25 && thlRx.temp < 25){
        		fTx.t25 = false;
        	}

        	if(!fRx.t29 && thlRx.temp > 29){	//temp > 29 abre a janela
        		fTx.t29 = true;
        		on= false;
        		xTaskCreatePinnedToCore(vTask5, "SERVO", 1024, NULL, 4, NULL, 1);
        	}else if(fRx.t29 && thlRx.temp < 29){
        		fTx.t29 = false;
        	}

        	if(!fRx.t20 && thlRx.temp < 20){	//temp < 20 liga o aquecimento
        		fTx.t20 = true;
        		digitalWrite(LED_RESISTENCIA, HIGH);
        	}else if(fRx.t20 && thlRx.temp > 20){
        		fTx.t20 = false;
        	}

        	if(!fRx.t26 && thlRx.temp < 26){	//temp < 26 fecha a janela
        		fTx.t26 = true;
        		on= true;
        		xTaskCreatePinnedToCore(vTask5, "SERVO", 1024, NULL, 4, NULL, 1);
        	}else if(fRx.t26 && thlRx.temp > 26){
        		fTx.t26 = false;
        	}

        	//------Humidade e Luminosidade-------
        	if(!fRx.hl50400 && (thlRx.hum < 50 && thlRx.lum < 400)){	//ativa o sistema de rega
        		fTx.hl50400 = true;
        		digitalWrite(LED_REGA, HIGH);
        	}else if(fRx.hl50400 && (thlRx.hum > 50 || thlRx.lum > 400)){
        		fTx.hl50400 = false;
        	}

        	if(!fRx.hl75400 && (thlRx.hum > 75 || thlRx.lum > 400)){	//desativa o sistema de rega
        		fTx.hl75400 = true;
        		digitalWrite(LED_REGA, LOW);
        	}else if(fRx.hl75400 && (thlRx.hum < 75 || thlRx.lum < 400)){
        		fTx.hl75400 = false;
        	}

        	xStatus2 = xQueueSendToBack( xQueueFlags, &fTx, 0 ); // Insere estrutura na queue flags
        	if (xStatus2 != pdPASS){
        		Serial.print( ">>>>>>>>>>>>>>>>>>>>>>>>T2: Could not send to the queue flags.\r\n" );
        	}
        }else{
        	Serial.print( "T3: Could not receive from the queue2.\r\n" );
        }
        vTaskDelete( xTaskAtuadoresHandle ); //Esta tarefa é eliminada até que as variaveis estejam fora dos limites
    }
}
//------ BLE
void vTask4(void * pvParameters) {

    const char * pcTaskName = "--------------------------BLE-------------------------\r\n";
    thl thlRx; //estrutura valores cálculados
    flags fTx, fRx; // estrutura flags
    portBASE_TYPE xStatus, xStatus2,xStatus3;
    char txString_temp[8], txString_hum[8], txString_lum[8]; //strings a ser enviados no ble
    char rxValue[5]; //string a ser recebida
    for (;;) {
        xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
        Serial.print(pcTaskName);
        xStatus = xQueuePeek( xQueue2, &thlRx, 0);	//recebe os dados da queue2
        xStatus2 = xQueueReceive( xStringQueue, &rxValue, 0);	 //recebe os dados da queue de strings

        if( xStatus == pdPASS ){
            Serial.print("T4: read = ");
            Serial.print(thlRx.temp); Serial.print(thlRx.hum); Serial.println(thlRx.lum);
            if( xStatus2 == pdPASS){
            	//transformar os dados de floats a strings
                dtostrf(thlRx.temp, 1, 1, txString_temp);
                dtostrf(thlRx.hum, 1, 1, txString_hum);
                dtostrf(thlRx.lum, 1, 0, txString_lum);

				//Mostrar o valor recebido
				Serial.println("*********");
				Serial.print("Received Value: ");
				for (int i = 0; i < sizeof(rxValue); i++) {
					Serial.print(rxValue[i]);
				}
				Serial.println();
				Serial.println("*********");

				 //SEND DATA -> copia os strings dos dados calculados a variavel dataTxBLE
				if (rxValue[0] == 't') {	//temperatura
					Serial.println("TEMP");
					strcpy(dataTxBLE, txString_temp);
				}
				else if (rxValue[0] == 'h') {	//humidade
					Serial.println("HUM");
					strcpy(dataTxBLE, txString_hum);
				}
				else if (rxValue[0] == 'l') {	//luminosidade
					Serial.println("LUM");
					strcpy(dataTxBLE, txString_lum);
				}
				else if (rxValue[1] == 'n') { 	//on
					Serial.println("Turning ON!");
					digitalWrite(LED_LUZ, HIGH);
				}
				else if (rxValue[1] == 'f') { 	//off
					Serial.println("Turning OFF!");
					digitalWrite(LED_LUZ, LOW);
				}

				//MANUAL CONTROL -> ativa ou desativa o controlo automatico dos atuadores
				if (rxValue[0] == 'm' && !manual) {
					Serial.println("Modo manual ativado");
					manual = true;
				}else if(rxValue[0] == 'm' && manual){
					Serial.println("Modo manual desativado");
					//coloca as flags a seu valor default
					xStatus3 = xQueueReceive( xQueueFlags, &fRx, 0); // Recebe valores da queue flags para limpar de estados anteriores
					if( xStatus3 == pdPASS){
						fRx.t29 = false; fRx.t25 = false; fRx.t20 = false; fRx.t26 = false;
						fRx.hl75400 = false; fRx.hl50400 = false;
						manual = false;
						xStatus3 = xQueueSendToBack( xQueueFlags, &fTx, 0 ); // Insere estrutura atualizada na queue flags
					}else{
						Serial.println( "T4: Could not receive from the string queue FLAGS.\r\n" );
					}
				}

				//Controlo manual foi ativado
				if (manual){
					if (rxValue[0] == 'r' && rxValue[2] == 'n') { //ron -> rega
						digitalWrite(LED_REGA, HIGH);
					}
					else if (rxValue[0] == 'r' && rxValue[2] == 'f') { //roff
						digitalWrite(LED_REGA, LOW);
					}
					else if (rxValue[0] == 't' && rxValue[2] == 'n') { //ton -> servo
						on = false;//permite a abertura da janela
						xTaskCreatePinnedToCore(vTask5, "SERVO", 1024, NULL, 4, NULL, 1);//cria a tarefa do servo com prioridade media
																						//ela pode ser interrompida e comtinua seu funcionamento
					}
					else if (rxValue[0] == 't' && rxValue[2] == 'f') { //toff
						on= true;//permite fechar a janela
						xTaskCreatePinnedToCore(vTask5, "SERVO", 1024, NULL, 4, NULL, 1);//cria a tarefa do servo
					}
					else if (rxValue[0] == 'a' && rxValue[2] == 'n') { //aon -> resistencia
						digitalWrite(LED_RESISTENCIA, HIGH);
					}
					else if (rxValue[0] == 'a' && rxValue[2] == 'f') { //aoff
						digitalWrite(LED_RESISTENCIA, LOW);
					}
				}

        }else{
				Serial.println( "T4: Could not receive from the string queue.\r\n" );
			}
		}else{
			Serial.println("T4 Couldnt receive from q2");
		}
	}
}
//------ Servo
void vTask5(void * pvParameters) {
    const TickType_t xDelay15ms = 15 / portTICK_PERIOD_MS;
    const char * pcTaskName = "----------------------------SERVO-----------------------\r\n";
    for (;;) {
        Serial.print(pcTaskName);
        if(!on){	//de 0 até 180 graus
        	//O servo se move de grau em grau, permitindo que ele seja interrompido
        	do{
            	myservo.write(pos);              // mover o servo à posição especificada
                pos++;							//aumenta os graus
                Serial.print(pos);
                vTaskDelay( xDelay15ms );        // waits 15ms for the servo to reach the position
              }while(pos < 180);
            on = true;
            pos = 180;
            vTaskDelete( xTaskServoHandle ); //a tarefa e eliminada
        }else{	//de 180 ate 0 graus
            do{
                myservo.write(pos);
                pos--;						//diminui os graus
                Serial.print(pos);
                vTaskDelay( xDelay15ms );
             }while(pos > 0);
            on = false;
            pos = 0;
            vTaskDelete( xTaskServoHandle ); //a tarefa e eliminada
        }
        //condições default/maximas/minimas
        if(pos<0){
        	on = false;
        	pos = 0;
        }else if(pos > 180){
        	on = true;
        	pos = 180;
        }
    }
}

//-- Função IdleHook : 	evita processador correr em vazio
// 						a idle task corre com prioridade de 0 e limpa recursos do kernel depois de ter tarefas destruidas
bool my_vApplicationIdleHook( void )
  {
	thl thlRx; //inicializa estrutura thlRx
	flags fRx;
	portBASE_TYPE xStatus, xStatus2;
	xStatus = xQueuePeek( xQueue2, &thlRx, 0);	//Vai buscar valores do Queue2 (Queue dos dados calculados)
	xStatus2 = xQueuePeek( xQueueFlags, &fRx, 0);	//Vai buscar valores do Queue2 (Queue dos dados calculados)
	if( xStatus == pdPASS && xStatus2 == pdPASS){
		// Se os valores dos sensores forem maior ou menores que o desejado entram aqui.
		if (((thlRx.temp > 29  && !fRx.t29) || (thlRx.hum < 50 && thlRx.lum < 400 && !fRx.hl50400) ||
			((thlRx.hum > 75 || thlRx.lum > 400) && !fRx.hl75400) ||	(thlRx.temp < 20  && !fRx.t20) ||
			(thlRx.temp < 26 && !fRx.t26) || (thlRx.temp > 25  && !fRx.t25)) && !manual && !check){
			xTaskCreatePinnedToCore( vTask3, "Task Atuadores", 1024, NULL, 7, &xTaskAtuadoresHandle, 1); // Tarefa dos atuadores é criada
			check = true; //flag que previne a entrada nesta condição sem ter recebido dados novos para comparação
		}
	}else{
		Serial.print( "IDLE: Could not PEEK from the queue2.\r\n" );
	}
	return true;
  }

// The loop function is called in an endless loop
void loop()
{
	vTaskDelete( NULL );
}

// Function to extract numbers from compile time string
// Utilizada para inicialização do relógio no display
static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

