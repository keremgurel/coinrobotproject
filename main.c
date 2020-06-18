// final program
//some code taken from and coded with reference to sample code including PrintADCeff2 and Freq_Gen. Makefile framework from Freq_Gen was used.
/*
PIN ASSIGNMENTS
physical pin	pio assignment	thing				input/output
1		GPIO_B23		MOTOR_L1			OUTPUT
2		GPIO_B17		MOTOR_L2			OUTPUT
9		GPIO_B10		MOTOR_R1			OUTPUT
10		GPIO_B11		MOTOR_R2			OUTPUT
12		GPIO_B1			EMAG				OUTPUT
3		GPIO_B13		COIN_DETECT			INPUT
7		GPIO_B3			SERVO_1				OUTPUT
8		GPIO_B2			SERVO_2				OUTPUT
13		GPIO_B9/ADC_2	PERIMETER_DETECT1	INPUT
*/

#include "lpc824.h"
#include "serial.h"

#define SYSTEM_CLK 30000000L
#define DEFAULT_F 15000L

#define COINS_TOTAL 20

#define COIN_DETECT GPIO_B13	//coin detect - 3
#define MOTOR_L1 GPIO_B23	//motor 1 left - 1
#define MOTOR_L2 GPIO_B17	//motor 2 left - 2
#define MOTOR_R1 GPIO_B10	//motor 1 right - 9
#define MOTOR_R2 GPIO_B11	//motor 2 right - 10
#define SERVO_1 GPIO_B3	//servo 1  - 7
#define SERVO_2 GPIO_B2	//servo 2  - 8
#define EMAG GPIO_B1	//electromagnet - 12

#define FAST 90	//speed
#define NORMAL 50	//speed

#define TIME1_CONST 100

//positions in the servo sweep
#define SERVO1_REST 180
#define SERVO2_REST 0
#define SERVO1_A 50
#define SERVO2_A 180
#define SERVO1_B 170
#define SERVO2_B 10
#define SERVO1_C 22
#define SERVO2_C 50

/*global variables and volatile variables*/
//motors:
//left motor:
volatile int moL_1 = 0 , moL_2 = 0;
//right motor:
volatile int moR_1 = 0 , moR_2 = 0;
//servos:
volatile int servo_base = 0 , servo_arm = 0;
//flags. perimeter is set in the interrupt, coin is set in search_coin() function
volatile int perimeter_flag = 0;
//time related stuff:
volatile int  pwm_count = 0 , period = 0, pwm_servo = 0, long_time=0;
//thresholds
volatile int thresh_coin, thresh_perim;
volatile int voltage_coin = 0;

// Configure the pins as outputs
void ConfigPins(void)
{
	//enable switch matrix clock (page 41, 42)
	SYSCON_SYSAHBCLKCTRL |= BIT7;
	
	//turn off stupid extra features (page 87)
	SWM_PINENABLE0 |= BIT4 | BIT5 | BIT11 | BIT12 | BIT23 | BIT6 | BIT7;
	SWM_PINENABLE0 &=~BIT15; // Enable the ADC function on PIO_14 page 88
	
	//kill the clock
	SYSCON_SYSAHBCLKCTRL &= ~BIT7;
	
	GPIO_DIR0 |= BIT23; //motor
	GPIO_DIR0 |= BIT17;	//motor 
	GPIO_DIR0 |= BIT11;	//motor 
	GPIO_DIR0 |= BIT10;	//motor 
	
	GPIO_DIR0 |= BIT2;	//servo
	GPIO_DIR0 |= BIT3;	//servo
	
	GPIO_DIR0 &= ~(BIT13);	//coin detection
	
	GPIO_DIR0 |= BIT1;	//emag
}


/* Start ADC calibration */
void ADC_Calibration(void)
{
	unsigned int saved_ADC_CTRL;

	// Follow the instructions from the user manual (21.3.4 Hardware self-calibration)
	
	//To calibrate the ADC follow these steps:
	
	//1. Save the current contents of the ADC CTRL register if different from default.	
	saved_ADC_CTRL=ADC_CTRL;
	// 2. In a single write to the ADC CTRL register, do the following to start the
	//    calibration:
	//    – Set the calibration mode bit CALMODE.
	//    – Write a divider value to the CLKDIV bit field that divides the system
	//      clock to yield an ADC clock of about 500 kHz.
	//    – Clear the LPWR bit.
	ADC_CTRL = BIT30 | ((300/5)-1); // BIT30=CALMODE, BIT10=LPWRMODE, BIT7:0=CLKDIV
//	ADC_CTRL &= ~BIT10;
	// 3. Poll the CALMODE bit until it is cleared.
	while(ADC_CTRL&BIT30);
	// Before launching a new A/D conversion, restore the contents of the CTRL
	// register or use the default values.
	ADC_CTRL=saved_ADC_CTRL;
}

void InitADC(void)
{
	// Will use Pin 1 (TSSOP-20 package) or PIO_23 for ADC.
	// This corresponds to ADC Channel 3.  Also connect the
	// VREFN pin (pin 17 of TSSOP-20) to GND, and VREFP the
	// pin (pin 17 of TSSOP-20) to VDD (3.3V).
	
	SYSCON_PDRUNCFG &= ~BIT4; // Power up the ADC
	SYSCON_SYSAHBCLKCTRL |= BIT24;// Start the ADC Clocks
	ADC_Calibration();
	ADC_SEQA_CTRL &= ~BIT31; // Ensure SEQA_ENA is disabled before making changes	
	
	ADC_CTRL =1;// Set the ADC Clock divisor
	ADC_SEQA_CTRL |= BIT2 ; // Select Channel 2  page 88
	ADC_SEQA_CTRL |= BIT31 + BIT18; // Set SEQA and Trigger polarity bits
		
	//disable pull up / pull down resistors
	PIO0_13 |= BIT3;
	PIO0_14 |= BIT3;
}


void InitTimer(void)
{
	SCTIMER_CTRL |= BIT2; // halt SCTimer

    // Assign a pin to the timer.
    // Assign GPIO_14 to SCT_OUT0_O
	SWM_PINASSIGN7 &= 0x00ffffff;
	SWM_PINASSIGN7 |= (14 << 24);
	
	SYSCON_SYSAHBCLKCTRL |= BIT8; // Turn on SCTimer 
	SYSCON_PRESETCTRL |=  BIT8; // Clear the reset SCT control
	
	SCTIMER_CONFIG |= BIT0; // Unified 32 bit counter
	SCTIMER_MATCH0 = SYSTEM_CLK/(DEFAULT_F*2L); // Set delay period 
	SCTIMER_MATCHREL0 = SYSTEM_CLK/(DEFAULT_F*2L);
	SCTIMER_EV0_STATE = BIT0;  // Event 0 pushes us into state 0
	// Event 0 configuration:
	// Event occurs on match of MATCH0, new state is 1	
	SCTIMER_EV0_CTRL =  BIT12 + BIT14 + BIT15;
	// State 1 configuration
	SCTIMER_EV1_STATE = BIT1;  // Event 1 pushes us into state 1
	// Event 1 configuration
	// Event occurs on MATCH0, new state is 0
	SCTIMER_EV1_CTRL =  BIT12 + BIT14;
	// OUT0 is set by event 0
	SCTIMER_OUT0_SET = BIT0;
	// OUT1 is cleared by event 1
	SCTIMER_OUT0_CLR = BIT1;
	// Processing events 0 and 1
	SCTIMER_LIMIT_L = BIT0 + BIT1;
	// Remove halt on SCTimer
	SCTIMER_CTRL &= ~BIT2;		
		
	SCTIMER_EVEN = 0x01; //Interrupt on event 0
	NVIC_ISER0|=BIT9; // Enable SCT interrupts in NVIC
}

void Reload_SCTIMER (unsigned long Dly)
{
	SCTIMER_CTRL |= BIT2; // halt SCTimer
	SCTIMER_MATCH0 = Dly; // Set delay period 
	SCTIMER_MATCHREL0 = Dly;
	SCTIMER_COUNT = 0;
	SCTIMER_CTRL &= ~BIT2;	// Remove halt on SCTimer	
}

//ISR
void STC_IRQ_Handler(void)
{
	SCTIMER_EVFLAG = 0x01; // Clear interrupt flag
	pwm_count++;
	pwm_servo++;
	long_time++;
	//motors
	if(pwm_count>=100){
		pwm_count=0;
		period++;
	}
	//delay_time
	if(period>=TIME1_CONST){
		period = 0;
	}
	//search_coin
	if(long_time>=10000000000000){
		long_time = 0;
	}
	//servo
	if (pwm_servo>=600){
		pwm_servo = 0;
	}
	//left motor - 300Hz
	MOTOR_L1 = (pwm_count>moL_1)?0:1;
	MOTOR_L2 = (pwm_count>moL_2)?0:1;
	//right motor - 300Hz
	MOTOR_R1 = (pwm_count>moR_1)?0:1;
	MOTOR_R2 = (pwm_count>moR_2)?0:1;
	//servos - *6 because they must operate and 50Hz
	SERVO_1 = (pwm_servo>servo_base*6)?1:0;
	SERVO_2 = (pwm_servo>servo_arm*6)?1:0;
	//coin voltage
	voltage_coin = COIN_DETECT;
}

//use adc to determine perimeter flag
void detect_perimeter (void){
	ADC_SEQA_CTRL |= BIT26; // Start a conversion:. page 330. bit26 is always start
	//from first pin (pin 20 - PIO14)
	while( ( (ADC_SEQA_GDAT) & BIT31)==0); // Wait for data valid. table 281 page 330
	perimeter_flag = ( ( (ADC_SEQA_GDAT >> 4) & 0xfff)    > thresh_perim    )? 1: 0; //perimeter_flag;

	return;
}

//delays for a probably short amount of time - need to test further
//set time0 to number <100 and it will set a delay for that long in deciseconds(?)
void delay_time(int time0){
	period = 0;
	while (period != time0*TIME1_CONST/100){
		//time wasting loop
	}
	return;
}

void move_forward(int speed){
	//left motor
	moL_1=speed;
	moL_2=0;
	//right motor
	moR_1=speed;
	moR_2=0;
	return;
}
void move_backward(int speed){
	//left motor
	moL_1=0;
	moL_2=speed;
	//right motor
	moR_1=0;
	moR_2=speed;
	return;
}
void stop_moving(void){
	//left motor
    moL_1=0;
    moL_2=0;
	//right motor
    moR_1=0;
    moR_2=0;
    return;
}

void turn_around(void){
	//turn wheels different directions
	//left motor:
	moL_1=FAST;
	moL_2=0;
	//right motor:
	moR_1=0;
	moR_2=FAST;
	delay_time(50);	//time spent turning is slightly randomised
	delay_time(50);
	return;
}

int search_coin(int index){

	int time, i;
	while(voltage_coin==0);	//in case function starts halfway through v=low
	while(voltage_coin > 0);	//wait until v is no longer high
	//we're now at falling edge of signal -> record time
	long_time=0;
	for(i=0;i<17;i++){	//17 was experimentally found to be sufficiently large to get a reliable reading
		while(voltage_coin==0);	//wait until voltage is no longer low
		while(voltage_coin>0);//record time again
	}
	time = long_time;	//record time taken
	if(index != 0){	//compare to threshold
		if(time < thresh_coin)  //if there’s a coin (it might have to be <= not >= need to check if its period or frequency that increases)
			return 1;
		else 	//if no coin
			return 0;
	}
	else{	//establish threshold
		return time;
	}
}

//for servo motors - get pwm out of angle
int get_pwm(int angle){
	int pwm;
	pwm= ((double)angle/18.0 + 2.75);
	if (pwm < 1){
		pwm= 1;
	}
	if (pwm > 9){
		pwm= 9;
	}
	return pwm;
}

//arm will move to pick up coin
void sweep_coin(void){
	int angle;
	stop_moving();
	servo_base = get_pwm(SERVO1_REST);
	//	delay_time(5);
	servo_arm = get_pwm(SERVO2_REST);
	//move servo motor
	servo_base = get_pwm(SERVO1_A);
	delay_time(90);
	servo_arm = get_pwm(SERVO2_A);
	delay_time(90);
	EMAG = 1;
	delay_time(90);

	for (angle=SERVO1_A; angle<SERVO1_B ; angle++){
		servo_base=get_pwm(angle);
		delay_time(1);
	}
	for (angle=SERVO2_A; angle>SERVO2_B ; angle--){
		servo_arm = get_pwm(angle);
		delay_time(1);
	}

	for (angle=SERVO1_B; angle>SERVO1_C; angle--){
		servo_base = get_pwm(angle);
		delay_time(1);
	}
	for (angle=SERVO2_B; angle<SERVO2_C ; angle++){
		servo_arm = get_pwm(angle);
		delay_time(2);
	}
	//in position for drop
	delay_time(90);
	EMAG = 0;
	delay_time(90);
	servo_base = get_pwm(SERVO1_REST);
	servo_arm = get_pwm(SERVO2_REST);
	return;
}

int get_coin(void){
	stop_moving();
	//double check that there is actually a coin
	delay_time(90);
	if ( !search_coin(1) ){
		return 0;
	}
	
	//retreat more for set time period to go from coin detection to coin pickup spot
	move_backward(NORMAL);
	delay_time(90);	//this will need to be changed experimentally
	stop_moving();
	//perform the coin pickup
	sweep_coin();
	//return coin_check;
	return 1;
}

void arm_default(void){
	servo_arm = get_pwm(SERVO2_REST);
	servo_base = get_pwm(SERVO1_REST);
	EMAG = 0;
	return;
}

void main(void)
{
	ConfigPins();	
	InitTimer();
	initUART(115200);
	InitADC();
	enable_interrupts();
	
	int coins = 0;
	stop_moving();
	arm_default();
	delay_time(90);
	
	//calibrate thresholds
	thresh_coin = search_coin(0);
	ADC_SEQA_CTRL |= BIT26; // Start a conversion:. page 330. bit26 is always start
	while( ( (ADC_SEQA_GDAT) & BIT31)==0); // Wait for data valid. table 281 page 330
	thresh_perim += ((ADC_SEQA_GDAT) >> 4) & 0xfff; //try setting it equal to perim1 if things go wrong
	delay_time(90);
	thresh_coin += search_coin(0);
	ADC_SEQA_CTRL |= BIT26; // Start a conversion:. page 330. bit26 is always start
	while( ( (ADC_SEQA_GDAT) & BIT31)==0); // Wait for data valid. table 281 page 330
	thresh_perim += (ADC_SEQA_GDAT >> 4) & 0xfff; //try setting it equal to perim1 if things go wrong
	delay_time(90);
	thresh_coin += search_coin(0);
	thresh_coin /= 3;
	ADC_SEQA_CTRL |= BIT26; // Start a conversion:. page 330. bit26 is always start
	while( ( (ADC_SEQA_GDAT) & BIT31)==0); // Wait for data valid. table 281 page 330
	thresh_perim += (ADC_SEQA_GDAT >> 4) & 0xfff; //try setting it equal to perim1 if things go wrong
	thresh_perim /= 3;
	thresh_coin -=2;
	thresh_perim +=200;
	perimeter_flag = 0;
	
	while(coins < COINS_TOTAL) {
		arm_default();
		move_forward(FAST);
		detect_perimeter();
		if(perimeter_flag){
			turn_around();
			perimeter_flag = 0;
		}
		if(search_coin(1)) {
			coins += get_coin();
			//recalibrate coin detection
			thresh_coin = search_coin(0);
			delay_time(90);
			thresh_coin += search_coin(0);
			delay_time(90);
			thresh_coin += search_coin(0);
			thresh_coin /= 3;
			thresh_coin -=2;
		}
	}
	stop_moving();
	while (1){
		arm_default();
	}
}
