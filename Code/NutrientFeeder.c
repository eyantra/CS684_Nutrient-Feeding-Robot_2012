//Nutrient Feeding Bot by Group1 - Nabeel Nasir, Nikhil Limaje, Parmeshwar Reddy, Amol Morey
//Please refer Project Report for a better understanding of the code
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"
#include "ADC.c"
#include "motion.c"
#include "servo.c"


unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned long int ShaftCountC2 = 0;

unsigned int Degrees; //to accept angle in degrees for turning




//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA =  0xCF;
 PORTA = 0x00;

 DDRE = 0x08;
 PORTE = 0x08;

 DDRL = 0x18;   
 PORTL = 0x18; 
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
}





void c1motor_control(unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x30; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xCF; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command

}

void c2motor_control(unsigned char Direction)
{

unsigned char PortARestore = 0;

 Direction &= 0xC0; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0x3F; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}


void c1forward(void)
{
	c1motor_control(0x20);
	
}

void c1backward()
{
	c1motor_control(0x10);	
}

void c1stop()
{
	c1motor_control(0x00);
	
}



void c2forward(void)
{
	c2motor_control(0x40);
	
}

void c2backward()
{
	c2motor_control(0x80);
}

void c2stop()
{
	c2motor_control(0x00);
	
}


void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	uart0_init(); //Initailize UART1 for serial communiaction
	sei();   //Enables the global interrupts
}








 

void buzzer_config()
{
	DDRC = DDRC | 0x08;
	PORTC = PORTC & 0xF7;
}

void buzzer_off()
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}






void turn_on_sharp234_wl()
{
	PORTG = PORTG & 0xFB;
}

void turn_on_sharp15 (void) //turn on Sharp IR range sensors 1,5 
{ 
   PORTH = PORTH & 0xFB; 
}

void c1velocity (unsigned char val)
{
	OCR5CL = (unsigned char)val;
	
} 



int distance_cm_4to30sensor(int ADCposition)
{
	unsigned char val;
	val = ADC_Conversion(ADCposition);
	int distmm = (int) 10.00 * ((1.00/ ((0.001240875 * (float) val) +  0.005)) - 0.42);
	
	if ((distmm/10) >30)
		return 50;
	
	return (distmm/10);
}



int distance_cm_10to80sensor(int ADCposition)
{
	unsigned char val;
	val = ADC_Conversion(ADCposition);
	int distmm = (int) (10.00 * (2799.6 * (1.00 /  (float) (pow( (int) val , 1.1546)) )));  
	
	if ((distmm/10) >80)
		return 99;

	return (distmm/10);
}

void buzzer_on (void) 
{ 
 unsigned char port_restore = 0; 
 port_restore = PINC; 
 port_restore = port_restore | 0x08; 
 PORTC = port_restore; 
} 






void vibrateOn(int delay)
{
	servo_1(180);
	_delay_ms(delay);
	
	servo_1_free();
	_delay_ms(2000);
}

void vibrateOff(int delay)
{
	servo_1_free();
	_delay_ms(delay);
}



//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

void c2_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xBF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x40; //Enable internal pull-up for PORTE 4 pin
}






void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}

void c2_position_encoder_interrupt_init (void) //Interrupt 6 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x20; // INT6 is set to trigger with falling edge
 EIMSK = EIMSK | 0x40; // Enable Interrupt INT6 for right position encoder
 sei();   // Enables the global interrupt 
}


ISR(INT6_vect)
{
 ShaftCountC2++;  //increment C2 shaft position count
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

//Function used for turning robot by specified degrees
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


//Function used for moving robot forward by specified distance
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


//C2 spec dist mm
void c2_linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountC2 = 0;
 while(1)
 {
  if(ShaftCountC2 > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 c2stop(); //Stop c2
}


void c2forward_mm(unsigned int DistanceInMM)
{
 c2forward();
 c2_linear_distance_mm(DistanceInMM);
}

void c2backward_mm(unsigned int DistanceInMM)
{
 c2backward();
 c2_linear_distance_mm(DistanceInMM);
}


void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left_2(); //Turn reverse soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right_2();  //Turn reverse soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

//Function to initialize all the devices
void init_encoders()
{
 cli(); //Clears the global interrupt
 left_encoder_pin_config();
 right_encoder_pin_config();
 c2_encoder_pin_config();

 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 c2_position_encoder_interrupt_init();

 sei();   // Enables the global interrupt 
}

void vibrate(int delay)
{
	servo_3(180);
	_delay_ms(delay);

	servo_3_free();
	_delay_ms(500);
}


//Functions for Communication via Zigbee with Greenhouse server

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x47; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}


// Pass the message to be passed as a string
void send_status(char msg[])
{
	int i;
	UDR0 = 0x28;
	for(i = 0; msg[i]!=0; i++) {
		while ( !( UCSR0A & (1<<UDRE0)) );
		UDR0 = msg[i];
	}
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = 0x29;
}

char fcall[5][5];
int i =0, j = 0;
int botId;
unsigned char data; //to store received data from UDR1

void function_caller()
{
	int val, par;
	val = atoi(fcall[0]);
	switch(val) {
		case 1 : doTask(); break;
		default: UDR0 = 0x26;
	}
}



//FORMAT "botId$funCode$par1$par2$par3#"
SIGNAL(SIG_USART0_RECV)
{
	cli();
	data = UDR0;

	if(data == 0x23) // #
	{
		if(atoi(fcall[0]) == botId) {
			if(j != 0) {
				fcall[i][j] = 0;
				sei();
				function_caller();
				cli();
			}
			UDR0 = data;
		}
		i = 0;
		j = 0;
	}
	else if(data == 0x24) // $
	{
		fcall[i][j] = 0;
		i++;
		j = 0;
	}
	else
	{	
		fcall[i][j] = data;
		j++;
	}
	sei();
}



void doTask()
{
	//Initialize all required counters
	unsigned char wlflag = 0;
	
	int shaftval = 0;
	
	int firstTime = 1;
	int movedForward = 0; 

	int right_flag = 0; 
	int flag = 0;

	int added_degree_count =0;

	
	

	

		
	
	while(1)
	{

		lcd_print(2, 1, distance_cm_10to80sensor(13), 2);
		lcd_print(1, 1, distance_cm_4to30sensor(10), 2);
		
		
	
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		wlflag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2z
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		
		
		//Testing on black line. If center is blackline
		if(Center_white_line>0x28)
		{
			wlflag=1;
			
			//Detection of stem by Sensor1 
			if ( (distance_cm_10to80sensor(13) >= 10 ) && (distance_cm_10to80sensor(13) <= 32) )
			{
				//Detected 2 times (just to be sure) 
				if(right_flag == 1)
				{
					//An indication
					buzzer_on();
					_delay_ms(250);
					buzzer_off();

					
					stop();
					_delay_ms(1000);

					//take a right turn a little less than 90
					right_degrees(85);

					
					stop();
					_delay_ms(3000);

					
					//Take right turns of 5 degrees till sensor2 detects the stem
					while(1)
					{
						//Reached till stem
						if( distance_cm_4to30sensor(10) <= 10)
						{
							//Was the value obtained after moving the slider forward? 
							if(movedForward == 1)
							{
								c2stop();

								//do vibration
								vibrate(5000);
								
								
								//Save the shaft count to get the value of how much the slider moved fwd
								shaftval = ShaftCountC2;
								
								ShaftCountC2 = 0;
								
								//Take Back the slider
								while(1)
								{
									c2backward();
									_delay_ms(30);
			
									c2stop();
									_delay_ms(250);

									//Check if shaft count of C2 reaches initially saved value. Move bwd till then
									if(ShaftCountC2 >= shaftval)
									{
										
										ShaftCountC2 = 0;
										break;
									}
								}
																

				

								c2stop();
								_delay_ms(1000);

								
								movedForward = 0;
								buzzer_on();
								_delay_ms(500);
								buzzer_off();
								break;
							}

						}
						
						//Slider has not reached until stem. Move slider fwd
						else if ( (distance_cm_4to30sensor(10) >= 11  && distance_cm_4to30sensor(10) <=25 ) ) 
						{

							//Just to be sure abt the credibility of the value. (Refer report for description)
							if(flag == 2)
							{
								movedForward = 1;
								if(firstTime == 1)
								{
									if(added_degree_count !=0)
										right_degrees(5);
								
									ShaftCountC2 = 0;
			

									firstTime = 0;
								}
								c2forward();
								_delay_ms(40);

								flag=0;
								c2stop();
								_delay_ms(250);
							}

							else
							{
								flag++;
							}

						}
						//nothing found - Take small 5 degrees till found
						else 
						{
							c2stop();
							buzzer_on();
							_delay_ms(1000);
							buzzer_off();

							added_degree_count += 5;
							right_degrees(5);
						
						}

					}
	
					
					//Job is done. Take back bot to initial position by taking left turn 
					
					left_degrees(75  + added_degree_count);
					added_degree_count = 0;

					stop();
					_delay_ms(3000);
					
					
					right_flag = 0;

					forward();
					_delay_ms(500);

					stop();

				}
				//increment a counter to check credibility of value
				else
				{
					right_flag++;
				}
			}
			else 
			{
				
				forward();
				velocity(150,150);
			}


			
		
		}		


		if((Center_white_line<0x28 && Left_white_line<0x28) && (wlflag==0))
		{
			
			wlflag=1;
			
			forward();
			velocity(100,50);
		}

		if((Center_white_line<0x28 && Right_white_line<0x28) && (wlflag==0))
		{
			wlflag=1;
			
			forward();
			velocity(50,130);
		}

		if(Center_white_line>0x28 && Left_white_line>0x28 && Right_white_line>0x28)
		{
			forward();
			velocity(0,0);
		}		
		

		
	

		
	}		
	

}


int main()
{
	//Do all initial configurations
	init_devices();
	init_encoders();
	lcd_set_4bit();
	lcd_init();
	buzzer_config();
	init_devices_servo();
	turn_on_sharp234_wl();
	turn_on_sharp15();
	
	//Bot ID for use by Greenhouse server
	botId = 1;
	while(1);
}
