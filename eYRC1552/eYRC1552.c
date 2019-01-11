/*
*
* Team Id: 		eYRC#1552-FF
* Author: 		Naveen Gunasekaran
*			
* Filename: 		eYRC1552
* Theme: 		Fire Fighting Robot
* Functions: 		timer1_init(), servo1_pin_config(), Sharp_GP2D12_estimation(unsigned char), motion_pin_config(), left_encoder_pin_config(), right_encoder_pin_config(),
					port_init(), left_position_encoder_interrupt_init(), right_position_encoder_interrupt_init(), ISR(INT5_vect), ISR(INT4_vect), motion_set(unsigned char),
					forward(), left(), right(), soft_left(), soft_right(), stop(), angle_rotate(unsigned int), linear_distance_mm(unsigned int), forward_mm(unsigned int),
					left_degrees(unsigned int), right_degrees(unsigned int), soft_left_degrees(unsigned int), soft_right_degrees(unsigned int), servo_1(unsigned char), init_devices_1(),
					init_devices_2(), lcd_port_config(), adc_pin_config(), timer5_init(), adc_init(), ADC_Conversion(unsigned char), print_sensor(char,char,unsigned char), velocity(unsigned char, unsigned char),
					buzzer(int), A(), B(), main()		
* Global Variables: angle,LED, ADC_Value, flag, fl,count, sharp9, sharp11, sharp13
					Left_white_line, Center_white_line, Right_white_line, ShaftCountLeft, ShaftCountRight, Degrees
*
*/
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> 
#include "lcd.h"

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
unsigned char ADC_Conversion(unsigned char);

int angle=0,LED=1;//angle to ratate the servo; value of externally interfaced white line sensor
unsigned char ADC_Value; //
unsigned char flag = 0, fl=0,count=0;//flags; count holds number of rooms checked
unsigned int sharp9 = 0,sharp11=0,sharp13=0;//value returned by sharp sensor
unsigned char Left_white_line = 0;//value of left white line sensor
unsigned char Center_white_line = 0;//value of center white line sensor
unsigned char Right_white_line = 0;//value of right white line sensor
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning

/*
*
* Function Name: 	timer1_init
* Input: 		NONE
* Output: 		initialize timer 1
* Logic: 		appropriate values are given to various registers for the initialization
* Example Call:		timer1_init();
*
*/
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}
/*
*
* Function Name: 	servo1_pin_config
* Input: 		NONE
* Output: 		initialize pins for servo 1
* Logic: 		appropriate values are given to various registers for the initialization
* Example Call:		servo1_pin_config();
*
*/
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}
/*
*
* Function Name: 	Sharp_GP2D12_estimation
* Input: 		adc_reading -> the value return after adc conversion
* Output: 		returns the distance measured by the sharp sensor in mm
* Logic: 		The value returned after A to D conversion is used in the formula (10.00*(2799.6*(1.00/(pow(adc_reading,1.1546))))),
				this value is then converted to an integer and then returned.
* Example Call:		sharp9=Sharp_GP2D12_estimation(100);
*
*/
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance; //Distance measured by the sharp sensor in mm
	unsigned int distanceInt; //Integer value of the distance measured in mm
	//converting ADC value to distance in mm
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	//Since the sensors measure only upto 800 mm accurately, if the distance calculated is
	//greater than 800, then it is stored as 800 only
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}
/*
*
* Function Name: 	motion_pin_config
* Input: 		NONE
* Output: 		configure pins for motion of bot
* Logic: 		appropriate values are given to various registers for the initialization
* Example Call:		motion_pin_config();
*
*/
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
*
* Function Name: 	left_encoder_pin_config
* Input: 		NONE
* Output: 		configure INT4 (PORTE 4) pin as input for the left position encoder
* Logic: 		appropriate values are given to various registers for the configuration
* Example Call:		left_encoder_pin_config();
*
*/
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}


/*
*
* Function Name: 	right_encoder_pin_config
* Input: 		NONE
* Output: 		configure INT5 (PORTE 5) pin as input for the right position encoder
* Logic: 		appropriate values are given to various registers for the configuration
* Example Call:		right_encoder_pin_config();
*
*/
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

/*
*
* Function Name: 	port_init
* Input: 		NONE
* Output: 		initialize the ports
* Logic: 		appropriate values are given to various registers for the intialization
* Example Call:		port_init();
*
*/
void port_init()
{
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	servo1_pin_config();
}
/*
*
* Function Name: 	left_position_encoder_interrupt_init
* Input: 		NONE
* Output: 		enable interrupt 4
* Logic: 		appropriate values are given to various registers for the intialization
* Example Call:		left_position_encoder_interrupt_init();
*
*/
void left_position_encoder_interrupt_init (void) 
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}
/*
*
* Function Name: 	right_position_encoder_interrupt_init
* Input: 		NONE
* Output: 		enable interrupt 5
* Logic: 		appropriate values are given to various registers for the intialization
* Example Call:		right_position_encoder_interrupt_init();
*
*/
void right_position_encoder_interrupt_init (void) 
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

/*
*
* Function Name: 	motion_set
* Input: 		Direction -> set the direction the bot will move
* Output: 		setting motor's direction
* Logic: 		appropriate values are given to various registers to set the motion of the bot
* Example Call:		motion_set(0x06);
*
*/
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}
/*
*
* Function Name: 	forward
* Input: 		NONE
* Output: 		move both wheels forward
* Logic: 		call function motion_set();
* Example Call:		forward();
*
*/
void forward (void)
{
	motion_set(0x06);
}

/*
*
* Function Name: 	left
* Input: 		NONE
* Output: 		Left wheel backward, Right wheel forward
* Logic: 		call function motion_set();
* Example Call:		left();
*
*/

void left (void)
{
	motion_set(0x05);
}
/*
*
* Function Name: 	right
* Input: 		NONE
* Output: 		Left wheel forward, Right wheel backward
* Logic: 		call function motion_set();
* Example Call:		right();
*
*/
void right (void)
{
	motion_set(0x0A);
}
/*
*
* Function Name: 	soft_left
* Input: 		NONE
* Output: 		Left wheel stationary, Right wheel forward
* Logic: 		call function motion_set();
* Example Call:		soft_left();
*
*/
void soft_left (void)
{
	motion_set(0x04);
}
/*
*
* Function Name: 	soft_right
* Input: 		NONE
* Output: 		Left wheel forward, Right wheel is stationary
* Logic: 		call function motion_set();
* Example Call:		soft_right();
*
*/
void soft_right (void)
{
	motion_set(0x02);
}

/*
*
* Function Name: 	stop
* Input: 		NONE
* Output: 		both wheels stationary
* Logic: 		call function motion_set();
* Example Call:		stop();
*
*/

void stop (void)
{
	motion_set(0x00);
}


/*
*
* Function Name: 	angle_rotate
* Input: 		Degrees -> angle by which bot should be rotated
* Output: 		bot rotates by angle specified
* Logic: 		the precision encoder is used to count number of pulses and thus rotate the bot by specified angle.
* Example Call:		angle_rotate(90);
*
*/
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

/*
*
* Function Name: 	linear_distance_mm
* Input: 		DistanceInMM -> distance by which bot should be moved forward
* Output: 		bot moves forward by specified distance
* Logic: 		the precision encoder is used to count number of pulses and thus move the bot forward by given distance.
* Example Call:		linear_distance_mm(100);
*
*/

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}
/*
*
* Function Name: 	forward_mm
* Input: 		DistanceInMM -> distance the bot should move in mm
* Output: 		move both wheels forward by distance given
* Logic: 		call function linear_distance_mm(int);
* Example Call:		forward_mm(100);
*
*/
void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

/*
*
* Function Name: 	left_degrees
* Input: 		Degrees -> angle by which the bot should turn
* Output: 		left wheel rotates backwards and right wheel forwards
* Logic: 		call function angle rotate(int);
* Example Call:		left_degrees(90);
*
*/

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}

/*
*
* Function Name: 	right_degrees
* Input: 		Degrees -> angle by which the bot should turn
* Output: 		right wheel rotates backwards and left wheel forwards
* Logic: 		call function angle rotate(int);
* Example Call:		right_degrees(90);
*
*/

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}
/*
*
* Function Name: 	soft_left_degrees
* Input: 		Degrees -> angle by which the bot should turn
* Output: 		right wheel rotates forwards and left wheel stationary
* Logic: 		call function angle rotate(int);
* Example Call:		soft_left_degrees(90);
*
*/

void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}
/*
*
* Function Name: 	soft_right_degrees
* Input: 		Degrees -> angle by which the bot should turn
* Output: 		left wheel rotates forwards and right wheel stationary
* Logic: 		call function angle rotate(int);
* Example Call:		soft_right_degrees(90);
*
*/
void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}
/*
*
* Function Name: 	servo_1
* Input: 		degrees -> angle by which the servo should turn
* Output: 		servo is rotated by given angle
* Logic: 		appropriate values are given to the registers.
* Example Call:		servo_1(90);
*
*/

void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}
/*void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}*/

/*
*
* Function Name: 	init_devices_1
* Input: 		NONE
* Output: 		initialize all the devices
* Logic: 		appropriate values are given to the registers.
* Example Call:		init_devices_1();
*
*/
void init_devices_1()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   // Enables the global interrupt
	
}
/*
*
* Function Name: 	init_devices_2
* Input: 		NONE
* Output: 		initialize all the devices
* Logic: 		appropriate values are given to the registers.
* Example Call:		init_devices_2();
*
*/
void init_devices_2()
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	timer1_init();
	sei();   //Enables the global interrupts
}


/*
*
* Function Name: 	lcd_port_config
* Input: 		NONE
* Output: 		configure LCD port
* Logic: 		appropriate values are given to the registers.
* Example Call:		lcd_port_config();
*
*/
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

/*
*
* Function Name: 	adc_pin_config
* Input: 		NONE
* Output: 		configure ADC pin
* Logic: 		appropriate values are given to the registers.
* Example Call:		adc_pin_config();
*
*/
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

/*
*
* Function Name: 	timer5_init
* Input: 		NONE
* Output: 		initialize timer 5
* Logic: 		appropriate values are given to various registers for the initialization
* Example Call:		timer5_init();
*
*/

void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
/*
*
* Function Name: 	adc_init
* Input: 		NONE
* Output: 		initialize ADC
* Logic: 		appropriate values are given to various registers for the initialization
* Example Call:		adc_init();
*
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/*
*
* Function Name: 	ADC_Conversion
* Input: 		Ch -> Channel for which we require to perform ADC
* Output: 		returns 8 bit value sensed by the sensor interfaced at channel Ch
* Logic: 		the function accepts a channel number and then returns the 8 bit digital value
* Example Call:		linear_distance_mm(100);
*
*/
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

/*
*
* Function Name: 	print_sensor
* Input: 		row -> row of LCD on which to print; column -> column of LCD on which to print; channel -> decide which sensor's value to print
* Output: 		prints value of sensor interfaced at channel at the row and column specified.
* Logic: 		it first gets the ADC value and then calls function lcd_print(), which prints the value. lcd_print() is present in lcd.h
* Example Call:		print_sensor(1,1,3)
*
*/
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/*
*
* Function Name: 	velocity
* Input: 		left_motor -> speed at which left motor rotates; right_motor -> speed at which right motor rotates
* Output: 		the motors rotate with the specified speed
* Logic: 		Appropriate values are written onto registers to rotate the motors
* Example Call:		velocity(100,100);
*
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}
/*
*
* Function Name: 	buzzer
* Input: 		t -> time in milli seconds the buzzer should sound
* Output: 		buzzer will beep for the time specified by t
* Logic: 		

* Example Call:		buzzer(1000);
*
*/
/*void buzzer(const int t)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
	_delay_ms(t);		//delay
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
	_delay_ms(t);
}
*/
/*
*
* Function Name: 	A
* Input: 		NONE
* Output: 		If the door is present at position d2, this function gets executed. It is used to detect fires and if they are present to put them out.
	After detection a fire or if it realizes a fire is not present, the function makes the bot leave the room and return to the center home position.
* Logic: 		Various sensors are used in this function. 
	1. The bot moves forward till the white sensor detects the line.
	2. The white line sensor is used to follow the black line.
	3. The fire led is detected with the help of external white line senor and sharp ir sensor 3.
	4. If led is present the servo is used to drop the magnet.
	5. Once led is detected or no led is present, the bot makes a 180 degree turn.
	6. On leaving the room, the bot follows the wall, using sharp sensor 1, till it detects the black line of the home zone.
	7. A 90 degree left turn is made.
	8. return 
* Example Call:		A();
*
*/

void A (void)
{
	while(Left_white_line<0x10&&Center_white_line<0x10&&Right_white_line<0x10)//move forward till black line is detected
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
	
		forward();
		velocity(150,150);
	}	
	while(1)//motion of bot inside the room till fire is detected or end of room is reached
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		sharp11=ADC_Conversion(11);	//Getting data of Sharp Sensor 3
		sharp11=Sharp_GP2D12_estimation(sharp11);	//Getting data of Sharp Sensor 3
		
		LED=ADC_Conversion(7);	//Getting data of externally interfaced White Line Sensor
		
		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		
		

		if(Center_white_line>0x10)//Center WL is black, move forward
		{
			flag=1;
			forward();
			velocity(150,150);
		}

		if((Left_white_line<0x10) && (flag==0))//Left WL sensor is white and center is not black. move right
		{
			flag=1;
			forward();
			velocity(130,50);
		}

		if((Right_white_line<0x10) && (flag==0))//Right WL sensor is white and center is not black. move left
		{
			flag=1;
			forward();
			velocity(50,130);
		}
		
		if(Center_white_line<0x10 && Left_white_line<0x10 && Right_white_line<0x10)//if all WL sensors are white, turn right. This is to turn the bot at the corners.
		{
			forward();
			
			velocity(130,50);
		}
		
		if (LED<0x10 && sharp11<275)//Condition for detection of fire
		{
		
			stop();
			_delay_ms(750);
			angle+=45;	//increase angle by which the servo should rotate by 45 degrees
			stop();
			//buzzer(1000);//sound buzzer for 1 second
			DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
			PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
			unsigned char port_restore = 0;
			port_restore = PINC;
			port_restore = port_restore | 0x08;
			PORTC = port_restore;
			_delay_ms(1000);		//delay
			//switch off buzzer
			port_restore = PINC;
			port_restore = port_restore & 0xF7;
			PORTC = port_restore;
			_delay_ms(1000);		//delay
				
			servo_1(angle);//rotate servo to drop magnet and put out fire
			_delay_ms(1000);
			stop();
			_delay_ms(1000);
			left_degrees(180);//take 180 degree turn
			_delay_ms(1000);	
			stop();
			_delay_ms(1000);
			break;//get out of the while loop
		}
		if(sharp11<100)//Condition for no fire LED present in the room
		{
			stop();
			_delay_ms(1000);
			left_degrees(180);	//rotate bot by 180 degrees
			_delay_ms(750);
			break;
		}			
			
	
	}
	fl=0;//reset flag
	while(1)
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		sharp11=ADC_Conversion(11);	//Getting data of Sharp Sensor 3
		sharp11=Sharp_GP2D12_estimation(sharp11);	//Getting data of Sharp Sensor 3
		sharp9=ADC_Conversion(9);	//Getting data of Sharp Sensor 1
		sharp9=Sharp_GP2D12_estimation(sharp9);	//Getting data of Sharp Sensor 1

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		if(Center_white_line>0x10)	//Center WL sensor black ,move forward
		{
			flag=1;
			forward();
			velocity(150,150);
		}

		if((Left_white_line<0x10) && (flag==0))	//Left WL sensor is white and center is not black. move right
		{
			flag=1;
			forward();
			velocity(130,50);
		}

		if((Right_white_line<0x10) && (flag==0))	//Right WL sensor is white and center is not black. move left
		{
			flag=1;
			forward();
			velocity(50,130);
		}

		if(Center_white_line<0x10 && Left_white_line<0x10 && Right_white_line<0x10)	//if all WL sensors are white, turn left. This is to turn the bot at the corners.
		{
			forward();
			fl=1;
			velocity(50,130);
		}
		if(sharp9>650)	//condition if the bot detects that the door is present when it is at a corner and about to turn. Sharp sensor 1 will be greater than 650mm
			fl=1;	//flag fl is set
		if(Center_white_line>0x10 && sharp11>650 && fl==1)	//condition that the Center WL sensor is black and Sharp Sensor 3 detects a door
		{
			
			break;	//exit the current loop
		}
	}
	while(1)	//similar to previous loop except that here if all 3 WL sensors detect white, the bot moves forward
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		sharp11=ADC_Conversion(11);	//Getting data of Sharp Sensor 3
		sharp11=Sharp_GP2D12_estimation(sharp11);	//Getting data of Sharp Sensor 3
	

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		if(Center_white_line>0x10)	//Center WL sensor black ,move forward
		{
			flag=1;
			forward();
			velocity(150,150);
		}

		if((Left_white_line<0x10) && (flag==0))	//Left WL sensor is white and center is not black. move right
		{
			flag=1;
			forward();
			velocity(130,50);
		}

		if((Right_white_line<0x10) && (flag==0))	//Right WL sensor is white and center is not black. move left
		{
			flag=1;
			forward();
			velocity(50,130);
		}
		
		if(Center_white_line<0x10 && Left_white_line<0x10 && Right_white_line<0x10 && sharp11>400)	//All WL sensors are white and distance from front to the other room's wall is greater than 400mm
		{
			forward();
			
			velocity(150,150);
		}

		if(Center_white_line<0x10 && Left_white_line<0x10 && Right_white_line<0x10 && sharp11<=400)	//when bot exits the room, distance from front to the wall will be less than 400mm
		{
			break;
		}

	}
	while (sharp11>200)	//move bot forward till it is 200mm from the wall
	{
		
		sharp11=ADC_Conversion(11);
		sharp11=Sharp_GP2D12_estimation(sharp11);
		forward();
		velocity(180,180);
	}
	soft_right_degrees(90);	//turn the bot right by 90 degrees
	while(1)	//Wall follower to reach the home circle
	{
		sharp9=ADC_Conversion(9);
		sharp9=Sharp_GP2D12_estimation(sharp9);
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		if (sharp9>200)	//distance of bot from wall is greater than 200mm, move to the left
		{
			forward();
			velocity(50,130);
		}
		if (sharp9<200)	//distance of bot from wall is lesser than 200mm, move to the right
		{
			forward();
			velocity(130,50);
		}
		if (sharp9==200)	//distance of bot from wall is equal to 200mm, move forward
		{
			forward();
			velocity (255,255);
		}
		if (Center_white_line>0x10 || Left_white_line>0x10 || Right_white_line>0x10)	//once black line of center circle is detected, exit the loop
		{
			break;
		}
		
	}
	
	while(1)	//follow line till the center of the circle where all three WL sensors become black
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		if(Center_white_line>0x10)	//Center WL sensor black ,move forward
		{
			flag=1;
			forward();
			velocity(150,150);
		}

		if((Left_white_line<0x10) && (flag==0))	//Left WL sensor is white and center is not black. move right
		{
			flag=1;
			forward();
			velocity(130,50);
		}

		if((Right_white_line<0x10) && (flag==0))	//Right WL sensor is white and center is not black. move left
		{
			flag=1;
			forward();
			velocity(50,130);
		}


		if(Center_white_line>0x10 && Left_white_line>0x10 && Right_white_line>0x10)	//All 3 WL sensors detect black line
		{
			soft_left_degrees(92);	//rotate bot by 92 degrees
			_delay_ms(1000);
			stop();
			_delay_ms(1000);
			count++;	//increment checked room count
			break;
		}
	}	
}
/*
*
* Function Name: 	B
* Input: 		NONE
* Output: 		If the door is present at position d1, this function gets executed. It is used to detect fires and if they are present and then put them out.
	After detection of a fire or if it realizes a fire is not present, the function makes the bot leave the room and return to the center home position.
* Logic: 		Various sensors are used in this function. 
	1. The bot moves forward till the white sensor detects the line.
	2. The white line sensor is used to follow the black line.
	3. The fire led is detected with the help of external white line senor and sharp ir sensor 3.
	4. If led is present the servo is used to drop the magnet.
	5. Once led is detected or no led is present, the bot makes a 180 degree turn.
	6. On leaving the room, the bot follows the wall,using sharp ir sensor 1, till it detects the black line of the home zone.
	7. A 90 degree left turn is made.
	8. return 
* Example Call:		B();
*
*/
void B (void)
{
	while(Left_white_line<0x10&&Center_white_line<0x10&&Right_white_line<0x10) //move bot forward till black line is detected
	{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		forward();
		velocity(150,150);
	}
	while(1)	//movement of the bot till fire LED is detected or till end of room is reached
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		sharp11=ADC_Conversion(11);				//Getting data of Sharp Sensor 3 
		sharp11=Sharp_GP2D12_estimation(sharp11);	//Converting Sharp Sensor value to mm
		sharp9=ADC_Conversion(9);				//Getting data of Sharp Sensor 1
		sharp9=Sharp_GP2D12_estimation(sharp9);	//Converting Sharp Sensor value to mm
		LED=ADC_Conversion(7);					//Getting data of center WL sensor attached in place of IR sensor 

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		if(Center_white_line>0x10)//If center WL sensor is black, move forward
		{
			flag=1;
			forward();
			velocity(150,150);
		}

		if((Left_white_line<0x10) && (flag==0))//If left WL sensor is white and center WL is not black, turn right 
		{
			flag=1;
			forward();
			velocity(130,50);
		}

		if((Right_white_line<0x10) && (flag==0))//If right WL sensor is white and center WL is not black, turn left
		{
			flag=1;
			forward();
			velocity(50,130);
		}

		if(Center_white_line<0x10 && Left_white_line<0x10 && Right_white_line<0x10)	//If all three WL sensors are white, turn left at the corners
		{
			forward();
			
			velocity(50,130);
		}
		
		if (LED<0x10 && sharp11<275)//Condition for fire LED present
		{
		
			stop();
			angle+=45;	//increase angle the servo must rotate by 45 degrees
			_delay_ms(750);
			//sound buzzer for 1 second
			DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
			PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
			unsigned char port_restore = 0;
			port_restore = PINC;
			port_restore = port_restore | 0x08;
			PORTC = port_restore;
			_delay_ms(1000);		//delay
			//switch off buzzer
			port_restore = PINC;
			port_restore = port_restore & 0xF7;
			PORTC = port_restore;
			_delay_ms(1000);		//delay
			servo_1(angle); //rotate the mechanism and drop the magnet to put out the fire
			_delay_ms(1000);
				
			stop();
			_delay_ms(1000);
			right_degrees(180); //make 180 degree turn
			_delay_ms(1000);
			stop();
			_delay_ms(1000);
			
			break;
		}	
		if(sharp11<100)//condition for no fire LED present. True when it reaches the end of the room
		{
			stop();
			_delay_ms(1000);
			right_degrees(180);//Make 180 degree turn
			_delay_ms(1000);
			stop();
			_delay_ms(1000);
			break;	//exit current loop
		}		
		
		
		
	}
	fl=0;
	while(1)	//movement of bot after LED detection or after reaching end of the room
				//similar to previous loop except that WL all white turns the bot right
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		sharp11=ADC_Conversion(11);				//Getting data of Sharp Sensor 1
		sharp11=Sharp_GP2D12_estimation(sharp11);	//Converting Sharp Sensor value to mm
		sharp13=ADC_Conversion(13);				//Getting data of Sharp Sensor 5
		sharp13=Sharp_GP2D12_estimation(sharp13);	//Converting Sharp Sensor value to mm
		
		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		

		if(Center_white_line>0x10)	//Move forward if Center WL reads black
		{
			flag=1;
			forward();
			velocity(150,150);
		}

		if((Left_white_line<0x10) && (flag==0))	//If Left WL is white and center is not black, turn right
		{
			flag=1;
			forward();
			velocity(130,50);
		}

		if((Right_white_line<0x10) && (flag==0))	//If Right WL is white and center is not black, turn left
		{
			flag=1;
			forward();
			velocity(50,130);
		}
		
		if(Center_white_line<0x10 && Left_white_line<0x10 && Right_white_line<0x10)	//If all three WL sensors are white, turn left at the corners
		{
			forward();
			fl=1;
			velocity(130,50);
		}
		if(sharp13>650)	//Sharp Sensor 5 detects door at corner before it turns, sharp sensor 5 is greater than 650mm
			fl=1;//sets flag fl
		if(Center_white_line>0x10 && sharp11>650 && fl==1)	//condition that the Center WL sensor is black and Sharp Sensor 3 detects a door
		{
			break;	//exit current loop
		}
	}
	
	while(1)	//similar to previous loop except that WL all white moves bot forward
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		sharp11=ADC_Conversion(11);	//Getting data of Sharp Sensor 3
		sharp11=Sharp_GP2D12_estimation(sharp11);	//Getting data of Sharp Sensor 3
		

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		if(Center_white_line>0x10)	//Move forward if Center WL reads black
		{
			flag=1;
			forward();
			velocity(150,150);
		}

		if((Left_white_line<0x10) && (flag==0))	//If Left WL is white and center is not black, turn right
		{
			flag=1;
			forward();
			velocity(130,50);
		}

		if((Right_white_line<0x10) && (flag==0))	//If Right WL is white and center is not black, turn left
		{
			flag=1;
			forward();
			velocity(50,130);
		}
		
		if(Center_white_line<0x10 && Left_white_line<0x10 && Right_white_line<0x10 && sharp11>400) //All WL sensors are white but sharp 3 is greater than 400mm, move forward
		{
			forward();
			
			velocity(150,150);
		}

		if(Center_white_line<0x10 && Left_white_line<0x10 && Right_white_line<0x10 && sharp11<=400) //All WL sensors are white and sharp 3 is lesser than or equal to 400mm
		{
			break;	//exit current function
		}

	}
	while (sharp11>200)	//move forward till bot is 200mm from wall
	{
		sharp11=ADC_Conversion(11);
		sharp11=Sharp_GP2D12_estimation(sharp11);
		
		forward();
		velocity(130,130);
	}
	soft_right_degrees(90);	//turn bot right by 90 degrees
	while(1)	//Wall following till center home circle
	{
		
		sharp9=ADC_Conversion(9);
		sharp9=Sharp_GP2D12_estimation(sharp9);
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		if (sharp9>200)	//bot is more than 200mm from wall, move to the left
		{
			forward();
			velocity(50,130);
		}
		if (sharp9<200)	//bot is less than 200mm from wall, move to the right
		{
			forward();
			velocity(130,50);
		}
		if (sharp9==200)	//bot is equal to 200mm from wall, move forward
		{
			forward();
			velocity (255,255);
		}
		if (Center_white_line>0x10 || Left_white_line>0x10 || Right_white_line>0x10)	//Detection of black line of home circle
		{
			break;	//exit current loop
		}
		
	}
	
	while(1)	//line following till center of home cirlce is reached
	{
		
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
	

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
	
		if(Center_white_line>0x10)	//Move forward if Center WL reads black
		{
			flag=1;
			forward();
			velocity(150,150);
		}

		if((Left_white_line<0x10) && (flag==0))	//If Left WL is white and center is not black, turn right
		{
			flag=1;
			forward();
			velocity(130,50);
		}

		if((Right_white_line<0x10) && (flag==0))	//If Right WL is white and center is not black, turn left
		{
			flag=1;
			forward();
			velocity(50,130);
		}

		if(Center_white_line>0x10 && Left_white_line>0x10 && Right_white_line>0x10)	//Either one of the 3 WL sensors detect black line
		{
			soft_left_degrees(92);	//rotate bot left by 90 degrees
			_delay_ms(1000);
			stop();
			_delay_ms(750);
			count++;	//increment checked room count
			break;
		}	
	}	
}



	
/*
*
* Function Name: 	main
* Input: 		none
* Output: 		initializes the bot and sets it in motion to complete the task.
* Logic: 		it is the first function that gets called and executed. various sensors which are interfaced are used to move the bot, detect fires, buzz the buzzer and put them out.
* Example Call:		main()
*
*/
int main()
{
	
		//initializing the robot
		init_devices_1();
		init_devices_2();
		lcd_set_4bit();
		lcd_init();
		
		servo_1(0);//keep the servo at 0 degrees
		_delay_ms(1000);
		forward_mm(195);//move bot forward by 195mm
		while(count<4)//count represents number of rooms checked, so it is executed while count is less than 4
		{
			
			forward_mm(270); //Moves robot forward 270mm
			stop();
			_delay_ms(750);
			
			
			sharp9=ADC_Conversion(9);
			sharp9=Sharp_GP2D12_estimation(sharp9);
			
		
			//check if door is present
			if(sharp9>200)//door is present
			{
				soft_left_degrees(90); //Rotate (soft turn) by 90 degrees
				stop();
				_delay_ms(750);
				A();//call function A
			}			
						
			else//door not present
			{
				forward_mm(400); //Moves robot forward 400mm
				stop();
				_delay_ms(750);	
				soft_left_degrees(90); //Rotate (soft turn) by 90 degrees
				stop();
				_delay_ms(750);	
				B();//call function b
			}
		}
		stop();
		_delay_ms(1000);
		//sound the buzzer for 6 seconds to indicate completion of the task
		DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
		PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
		unsigned char port_restore = 0;
		port_restore = PINC;
		port_restore = port_restore | 0x08;
		PORTC = port_restore;
		_delay_ms(6000);		//delay
		//switch off buzzer
		port_restore = PINC;
		port_restore = port_restore & 0xF7;
		PORTC = port_restore;
		_delay_ms(6000);		//delay
	}				