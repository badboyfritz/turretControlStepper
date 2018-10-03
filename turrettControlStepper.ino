// * S * K * Y  |)  W * E * T *
//  -=-=-=-=-=-=-=-=-=-=-=-=-
//  esp32 control firmware
//  october 3, 2018


#include "pulseCounter.h"
#include "Adafruit_AM2320.h"
#include <Stepper.h>
#include <BluetoothSerial.h>

void delayNonStop(float t);


// ********   B L U E T O O T H 
BluetoothSerial SerialBT;
byte stepperCase;

// ********   T E M P / M O I S T   S E N S 0 R 
Adafruit_AM2320 am2320 = Adafruit_AM2320();

// ********   S T E P P E R S  
const int stepsPerRevolution = 200;                         // change this to fit the number of steps per revolution for your motor
int stepCount = 0;
Stepper stepperDome(stepsPerRevolution, 17, 5, 18, 19);
//Stepper stepperValve(stepsPerRevolution, 15, 2, 0, 4);

// ********   P U L S E   C O U N T E R

// global variables
byte pulsePin = 23;											// pulse pin input
int16_t flow0Counter = 0;
int16_t Pulses = 0;
int16_t x;

// timer shit
hw_timer_t * timer = NULL;									// typedef struct hw_timer_s hw_timer_t;--> hw_timer_t * timerBegin(uint8_t timer, uint16_t divider, bool countUp);
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;		// typedef struct  --> * - Uninitialized (invalid)*portMUX_FREE_VAL - Mux is free, can be locked by either CPU CORE_ID_PRO / CORE_ID_APP - Mux is locked to the particular core 	 * Any value other than portMUX_FREE_VAL, CORE_ID_PRO, CORE_ID_APP indicates corruption 	 * If mux is unlocked, count should be zero.* If mux is locked, count is non - zero & represents the number of recursive locks on the mux.

// variables for data proccessing functions
volatile byte state = LOW;
volatile byte state2 = LOW;
volatile byte state_tmr = 0;								// used in onTimer() function's case statement
volatile byte value_ready = 0;								// used in onTimer() function's case statement

// variables for non-stoppping delays
float beforeTime = 0;
float afterTime = 0;


void setup() 
{

	// esp32 general related
	Serial.begin(115200);

	while (!Serial)
	{

		delayNonStop(10);									// wait here until Serial Port Opens
	
	}

	// bluetooth 
	SerialBT.begin("Rain|)Bow");							// RainBow is name for Bluetooth device

	// turret control related
	stepperDome.setSpeed(15);								// set the speed at 10  rpm:
	// MAKE FUNCTION HERE TO ZERO OUT STEPPER MOTOR?? OR FIND ZERO AND RETURN??

	// pulse counter relate
	pinMode(pulsePin, INPUT);								// pin to read pulse frequency
	initPcnt(pulsePin);										// init pulseCounter function
	initPcntTimers();										// init timers need for pulse COunters

	// adafruit sensor
	am2320.begin();

	SerialBT.println("*S*KY*W*E*T");
	SerialBT.println("Rain|)Bow");

	SerialBT.println("Adafruit AM2320 Sensor...");

}


void loop() 
{

	SerialBT.println("Fan Speed Value (a or b)");
	delayNonStop(1000);

	SerialBT.print("Temp: "); 
	SerialBT.println(am2320.readTemperature());

	SerialBT.print("Hum: "); 
	SerialBT.println(am2320.readHumidity());

	if (SerialBT.available())
	{

		// read the incoming byte:
		stepperCase = SerialBT.read();

		switch (stepperCase)
		{

		case 'a':

			stepperDome.step(1);
			SerialBT.print("steps:");
			SerialBT.println(stepCount);
			stepCount++;
			break;

		case 'b':

			stepperDome.step(-1);
			SerialBT.print("steps:");
			SerialBT.println(stepCount);
			stepCount--;
			break;
		}

	}

	readPcntCounter_0();

	if (value_ready == 1)
	{
		SerialBT.print("Pulses: ");
		SerialBT.println(Pulses);

		SerialBT.print("Freq: (Hz) ");
		SerialBT.println((float)Pulses);
	}

	delay(100);

}


void readPcntCounter_0()
{

	// * @brief Get pulse counter value * @param pcnt_unit  Pulse Counter unit number * @param count Pointer to accept counter value  -->  esp_err_t pcnt_get_counter_value(pcnt_unit_t pcnt_unit, int16_t* count);
	if (pcnt_get_counter_value(PCNT_TEST_UNIT, &flow0Counter) == ESP_OK)
	{
		SerialBT.println("pcnt unit 0...");
	}

	delay(10);

	SerialBT.printf("flow0Counts = %d", flow0Counter);

}


// (ISR) |) alarm timer - IRAM_ATTTR ensures this interrupt function will run from RAM and not slower FLASH
void IRAM_ATTR onTimer()
{

	portENTER_CRITICAL_ISR(&timerMux);								 // declare by calling the portENTER_CRITICAL_ISR and portExit_CRITICAL_ISR macros. They also both receive as input the address of the global portMUX_TYPE variable.

	switch (state_tmr)
	{

	case 0:
		pcnt_counter_clear(PCNT_TEST_UNIT);
		pcnt_counter_resume(PCNT_TEST_UNIT);
		value_ready = 0;
		state_tmr = 1;
		break;

	case 1:
		pcnt_counter_pause(PCNT_TEST_UNIT);							 // * @brief Pause PCNT counter of PCNT unit
		pcnt_get_counter_value(PCNT_TEST_UNIT, &flow0Counter);		 // * @brief Get pulse counter value  -->  esp_err_t pcnt_get_counter_value(pcnt_unit_t pcnt_unit, int16_t* count);
		Pulses = flow0Counter;
		flow0Counter = 0;
		value_ready = 1;
		state_tmr = 2;
		break;

	case 2:
		// if you have been thru all states reset state_tmr
		state_tmr = 0;
		break;

	default:
		break;
	}
	portEXIT_CRITICAL_ISR(&timerMux);

}

void initPcntTimers()
{

	// timer declared like this:  -->  hw_timer_t * timer = NULL;	
	timer = timerBegin(0, 80, true);								// hw_timer_t * timerBegin(uint8_t timer, uint16_t divider, bool countUp);  -->  Use 1st timer of 4 (counted from zero). Set 80 divider for prescaler 
	timerAttachInterrupt(timer, &onTimer, true);					// void timerAttachInterrupt(hw_timer_t *timer, void(*fn)(void), bool edge);  -->  Attach onTimer function to our timer.
	timerAlarmWrite(timer, 1000000, true);							// void timerAlarmWrite(hw_timer_t *timer, uint64_t interruptAt, bool autoreload);  -->  Set alarm to call onTimer function every second (value in microseconds).  -->  Repeat the alarm (third parameter)
	timerAlarmEnable(timer);

}

void delayNonStop(float t)
{

	beforeTime = millis();
	afterTime = (beforeTime + t);

	while (beforeTime < afterTime)						//retain status for amount of TIME as defined before switch to next nibble of step sequence array
	{

		beforeTime = millis();
	
	}

}	