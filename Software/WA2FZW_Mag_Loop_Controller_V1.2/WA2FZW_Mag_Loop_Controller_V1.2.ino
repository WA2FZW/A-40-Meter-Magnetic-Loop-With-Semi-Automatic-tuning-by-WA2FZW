/*
 *	WA2FZW Magnetic Loop Antenna Controller - Version 1.1
 *	
 *	V1.1	Fixes a bug in LCD_Print to add a null to the end of the display buffer.
 *
 *	V1.2	Played around with delay between turning on the transmitter and first check of the SWR.
 *			Add slight delay between moving the capacitor and checking the SWR.
 *	
 *
 *	This source file is under General Public License version 3. Please feel free to
 *	distribute it, hack it, or do anything else you like to it. I would ask, however
 *	that is you make any cool improvements, you let me know via any of the websites
 *	on which I have published this.
 * 	
 * 		Controller:		Adrunio Uno	
 * 		Motor Driver:	DFRobot DRI0023 (V2)	
 * 		Motor:			Oukeda 42HS40-1704 (NEMA Size 17, 1.8 degree, 2 phase)
 * 	
 *
 * 	The Oukeda 42HS40-1704 stepper motor that we are using is a 1.8 degree per
 * 	step motor by default (200 steps = 360 degrees).
 * 	
 * 	The DFRobot DRI0023 driver shield has a set of 3 switches that allow smaller
 * 	steps. The three switches basically allow us to set a number from 0 to 7
 * 	to select the step size. We will fix the setting of the	micro-step switch
 * 	setting to 2 (1/4 step), which seems to work well in testing and display that
 * 	on line 2 of the LCD display.
 * 	
 * 	The switch settings produce the following step sizes (Note, MS-1 is the low order
 * 	bit, so looking at the switches on the board, the bits are reversed from what you
 * 	would expect by a quick glance at the board:
 * 			
 *		0	Full step		 200 steps/revolution or 1.80000 degrees/step		
 *		1	Half step		 400 steps/revolution or 0.90000 degrees/step	
 *		2	Quarter step	 800 steps/revolution or 0.45000 degrees/step	
 *		3	Eighth step	 	1600 steps/revolution or 0.22500 degrees/step	
 *		4	16th step	 	3200 steps/revolution or 0.11250 degrees/step	
 *		5	32nd step		6400 steps/revolution or 0.05625 degrees/step	
 *		6	32nd step		6400 steps/revolution or 0.05625 degrees/step	
 *		7	32nd step		6400 steps/revolution or 0.05625 degrees/step
 *		
 *	Having observed how sensitive the tuning capacitor is, 1/4 step seems to be
 *	the best setting for the switches, so we have hard-coded that in here. 
 *	
 *	Summary of Arduino input/output pin usage:
 *	
 *		A0	SWR_Input					A3	
 *		A1	PWR_Input					A4	LCD_SDA (Data)
 *		A2								A5	LCD_SCL (Clock)
 *		
 *		D0								D7	Motor_1_Direction
 *		D1								D8	Motor_1_Enable
 *		D2	Encoder_Pin_B				D9	Encoder_Switch
 *		D3	Encoder_Pin_A				D10	Tune_Button
 *		D4	Motor_2_Direction			D11	Xmit_Key
 *		D5	Motor_2_Step				D12	Motor_2_Enable
 *		D6	Motor_1_Step				D13	
 *
 */

//	#define	Debug					// Enable to enable progress tracking
//	#define SWR_Debug				// Debugging for tuning functions

#include <Rotary.h>					// From https://github.com/brianlow/Rotary
#include <LiquidCrystal_I2C.h>		// From https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
#include <Wire.h> 					// Built-in
#include <String.h>					// Built-in

/*
 *	Define the digital I/O pins on the Arduino that control the movement of the
 *	motor. The "Direction" pins control which way the motor turns; when these are
 *	set to "LOW", the motors will rotate clockwise; when set to "HIGH", the motors
 *	will rotate counter-clockwise. HIGH on the enable pins disable the motor driver
 *	and LOW enables them. Note, we keep them disabled when not actually being used
 *	as the driver hardware makes a lot of RF noise.
 */

#define Motor_1_Direction	 7		// Motor #1 direction control pin
#define Motor_1_Step		 6		// Motor #1 step command pin
#define	Motor_1_Enable		 8		// Motor #1 enable/disable pin

#define	Motor_2_Direction	 4		// Motor #2 direction control pin
#define	Motor_2_Step		 5		// Motor #2 step command pin
#define	Motor_2_Enable		12		// Motor #2 enable/disable pin

#define Motor_Disable	  HIGH		// Turns motors off
#define Motor_Enable	   LOW		// Turns motors on

#define CW   HIGH					// HIGH on the direction pin causes clockwise rotation
#define CCW  LOW					// LOW causes counter-clockwise rotation


/*
 * 	Define some other things related to rotating the motor.
 */
 
float 	Step_Factor;				// Used in degrees to steps math

/*
 *	Define the variables associated with the rotary encoder. There are 2 definitions
 *	for how much to turn the motor depending on whether the shaft on the encoder has
 *	been pressed to activate the onboard switch. When the switch is activated, we will
 *	tune 10 degrees for each movement of the encoder, and 1 step  (or partial step,
 *	depending on the Step_Factor) if the switch is not activated.
 * 
 *	Note that the HALF_STEP definition is currently disabled in the Rotary.h file, thus
 *	each click of the encoder is actually 1 encoder step. If HALF_STEP is defined, each
 *	click is actually 2 steps on the encoder.
 */

#define	Encoder_Pin_A	3			// My encoders work backwards, so we treat Pin_A as DT
#define	Encoder_Pin_B	2			// and Pin_B as CLK
#define Encoder_Switch	9			// Switch is on digital pin 9 of the Arduino

int	Encoder_Big_Step   =  2;		// 2 degrees per encoder click (Switch on)
int Encoder_Small_Step =  1;		// 1 step per click (Switch off)


/*	
 *	Define the pins used for the auto-tune button and the SWR input.
 *	
 * 	Digital pin 11 will be used to read a "Tune" button. It sould be great if we could
 * 	make the antenna tune automatically upon detecting a reading on the SWR pin (which
 * 	would indicate we are transmitting), however when transmitting in SSB mode, the
 * 	signal at the SWR pin will vary widely based on the randomness of the transmitted
 * 	signal. Thus to tune the antenna based on SWR, we're going to have to do it in CW
 * 	mode and tell the controller to perform the tuning process.
 *	
 *	The code is set up to operate a relay connected to digital pin 10 on the Arduino. That
 *	relay can, in turn, be used to key the transmitter by connecting its contacts to the
 *	CW key line in the transmitter. Optionally, we can key the transmitter directly without
 *	the relay, which works for the BitX, but maybe not for other radios. This will be
 *	controlled by a switch on the front panel of the controller.
 *	
 */

#define SWR_Input (A0)					// Analog pin 0 is for the SWR reading
#define PWR_Input (A1)					// Pin A1 for reading forward power
#define	Tune_Button  10					// D-10 is the tune button
#define	Xmit_Key     11					// D-11 works the transmitter keying

#define	Step_1_Degrees  5				// Step 1 moves the motor in 5 degree steps
#define	Step_1_Limit   10				// If the SWR drops to 10, end Step 1

#define	Step_2_Degrees  1				// Step 2 moves in multiples of 1 degree
#define	Step_2_Limit    1				// When the SWR drops below 1, stop Step 2

#define	No_Limit	  -10				// Used in Read_Encoder()

int		Last_SWR = 0;					// Stored previous SWR measurement

bool	Xmit_On = LOW;					// LOW on D-10 keys the transmitter, regardless
bool	Xmit_Off = HIGH;				// Of the position of the Direct/Relay switch

unsigned long Xmit_Time;				// Last time the transmitter was keyed
unsigned long Xmit_Timeout  = 5000;		// 5 second timeout


/*
 * 	Pin and size definitions for the display:
 */

#define LCD_SDA	(A4)				// Data line
#define LCD_SCL (A5)				// Clock line
#define LCD_Height 2				// 2 Lines high
#define LCD_Width 16				// 16 characters wide
#define	LCD_Address 0x3F			// Non standard address (Standard address is 0x27)

/*
 *	These variables are used to redisplay the hello message after something else has
 *	beem displayed. Hello_Timeout is in milliseconds.
 */
 
unsigned long	Hello_Time = 0;			// Last time something other than hello displayed
unsigned long	New_Time   = 0;			// Current time
unsigned long	Hello_Timeout = 5000;	// 5 second refresh delay
bool Hello = false;						// Hello not on the display


/*
 *	These buffers are used to queue stuff to be displayed on the LCD. This is an attempt
 *	to solve the lockup caused by trying to output to the display from within the
 *	Read_Encoder interrupt function.
 */

String	LCD_Buffer_1;					// Line 1 buffer
String	LCD_Buffer_2;					// Line 2 buffer

/*
 * 	Create the rotary encoder object and the display object. Note, the address of the
 * 	display I got is 0x3F, as opposed to the standard 0x27. If necessary, I think that
 * 	can be changed with jumpers on the board.
 */
 
	Rotary Encoder = Rotary ( Encoder_Pin_A, Encoder_Pin_B );
	LiquidCrystal_I2C LCD_Display ( LCD_Address, LCD_Width, LCD_Height ); 


/*
 *	The "setup" function is called by the Adruino boot loader when the controller
 *	is first powered up, or after a reset. Here, we initialize anything that needs
 *	to be set up before we really go to work.
 */

void setup()
{
	Serial.begin ( 9600 );						// Bit rate for the serial monitor
	delay ( 1000 );								// Give monitor a second to stabilize


/*
 *	Setup the transmitter keying line pin, then we make sure the transmitter is off.
 */

	pinMode ( Xmit_Key, OUTPUT );				// Transmitter keying is an output
	digitalWrite ( Xmit_Key, Xmit_Off );		// Kill the transmitter


/*
 *	Initialize the display.
 */

	LCD_Display.begin ();				//Initialize the lcd
	LCD_Display.backlight ();			//Turn on the backlight


/*
 * 	Although we're no longer going to read the micro-step switch setting from
 * 	the serial monitor, for the time being, I'm going to leave it defined and 
 * 	hard-code the Step_Factor.
 */
 
#define MS_Switch 2								// Hard-wired for 1/4 steps

	Step_Factor = 0.4500;						// Quarter step


/*
 *	Remind the operator where the Step_Factor is set.
 */

	LCD_Buffer_1 = " Mag Loop Tuner";			// Display - Line 1
	LCD_Buffer_2 = " SF: ";						// and line 2 buffer
	LCD_Buffer_2 += Step_Factor;
	Check_LCD ();								// Force immediate display

	noInterrupts ();							// Disable interrupts


	#ifdef	Debug
		Serial.println ( "Starting setup" );
	#endif


/*
 *	Set up the I/O pins for the motors. Note, even though we are only using one 
 *	motor, we define the pins for both. As the motor control shield plugs into
 *	the Arduino Uno board, the motor #2 pins are connected, even though not really
 *	used. The only way to change that is with wire cutters!
 */

	pinMode ( Motor_1_Direction, OUTPUT );		// The motor control pins all
	pinMode ( Motor_1_Step, OUTPUT );			// get initialized as outputs.
	pinMode ( Motor_1_Enable, OUTPUT );
	
	pinMode ( Motor_2_Direction, OUTPUT) ;
	pinMode ( Motor_2_Step, OUTPUT );
	pinMode ( Motor_2_Enable, OUTPUT );


/*
 *	We disable both motors until we need them. For whatever reason, simply having
 *	one enabled creates a lot of RF noise in the receiver. Maybe putting the controller
 *	in a box and not running it of the laptop's USB port will fix that.
 */
 
	digitalWrite ( Motor_1_Enable, Motor_Disable );		// Disable Motor #1 until we need it
	digitalWrite ( Motor_2_Enable, Motor_Disable );		// and disable Motor #2


/*
 *	Set up the encoder pins:
 */
 
	pinMode ( Encoder_Pin_A, INPUT_PULLUP ); 			// Encoder has pullups on the board but
	pinMode ( Encoder_Pin_B, INPUT_PULLUP ); 			// we're going to also use the internal ones
	pinMode ( Encoder_Switch, INPUT_PULLUP );			// Definitely needed here


/*
 * 	Set up the tuning inputs.
 */

	pinMode ( SWR_Input, INPUT );				// Reverse power reading
	pinMode ( PWR_Input, INPUT );				// Forward power reading
	pinMode ( Tune_Button, INPUT_PULLUP );		// Need the pullup resistor


/*
 * 	The amount to move the capacitor in manual tuning mode with the encoder switch
 * 	operated is defined above in terms of degrees. We need to convert to the number
 * 	of steps to rotate the motor when the encoder switch is activated to steps. 
 * 	
 * 	The value of Encoder_Small_Step is defined in steps, so no conversion is needed
 * 	for that.
 */

	Encoder_Big_Step   = Convert_Degrees ( Encoder_Big_Step );


/*
 *	Set up the interrupts for the rotary encoder. Some documents online would lead one to
 *	believe that it can be handled without interrupts, but I couldn't make it work!
 */
	attachInterrupt ( digitalPinToInterrupt ( Encoder_Pin_A ), Read_Encoder, CHANGE);
	attachInterrupt ( digitalPinToInterrupt ( Encoder_Pin_B ), Read_Encoder, CHANGE);
	
	interrupts ();							// Enable the encoder


	#ifdef	Debug
		Serial.println ( "Setup complete" );
	#endif

}


/*	
 * 	Once the "setup" function has completed, the Adruino boot loader calls the "loop"
 * 	function, which will run forever or at least until the power is turned off or the
 * 	hardware is reset.
 */

void loop()
{
	Say_Hello ();					// We're on the air!
	SWR_Tune ();					// See if we need SWR tuning
	Check_LCD ();					// Anything to display?
	
	#ifdef	SWR_Debug
		Serial.print ( "PWR = " );
		Serial.print ( analogRead ( PWR_Input ));
		Serial.print ( ", SWR = " );
		Serial.println ( analogRead ( SWR_Input ));
		delay ( 1000 );
	#endif

}


/*	
 * 	Convert_Degrees converts the number of degrees of rotation requested into a number	
 * 	of steps (or partial steps) to move the motor based on the setting of the micro-step
 * 	switches. 
 * 		
 * 	Because the Step_Factor values are small decimal numbers, we will perform the	
 * 	math in floating point, then convert back to integer to send the number of steps	
 * 	to the motor counter.
 * 		
 * 	Note, we add 0.5 to the floating point result before converting back to an integer.	
 * 	This results in the step count being rounded to the nearest whole integer, as the
 * 	fractional part of the answer gets truncated.
 * 	
 * 	For example if Temp evaluated to 200.123, adding 0.5 would make it 200.623 and it
 * 	would be truncated to 200. If the value of Temp evaluated to 200.789, adding 0.5
 * 	would make it 201.289 and it would be truncated to 201.
 */

int Convert_Degrees ( int lclDegrees )
{
	int   iTemp;								// Use local variables
	float fTemp;
	
	fTemp = lclDegrees / Step_Factor;			// How many steps?
	fTemp = fTemp + 0.5;						// Rounding
	iTemp = fTemp;								// Back to integer

	return iTemp;								// And the answer is
}


/*
 *	The SWR_Tune function checks the status of the Tune_Button, and if it is active
 *	(LOW), we will proceed to tune the antenna based on the SWR reading.
 *	
 *	It's now a two step process. In step 1, We coarse tune by rotating the motor back
 *	and forth starting with Step_1_Degrees (S1D), then -2*S1D, then +3*S1D, etc. until
 *	we see the SWR drop below the value of Step_1_Limit. . If the SWR reading drops
 *	below Step_1_Limit, we move onto Step 2. If the transmitter stops transmitting or the
 *	value of Xmit_Timeout is exceeded, we terminate the process and notify the operator.
 *	
 *	In Step 2, we repeat the same process as was done in step 1, however we rotate the 
 *	capacitor back and forth in smaller increments (Step_2_Degrees) to fine tune the
 *	antenna. This continues until we see the SWR drop below the value of Step_2_Limit.
 *	Once this limit is reached, we're done. Again, if the transmitter stops transmitting
 *	or the value of Xmit_Timeout is exceeded, we terminate the process and notify the
 *	operator.
 *	
 *	In both step 1 and 2, if we see the SWR_Input go to zero, we stop, because it just
 *	can't get any better!
 *	
 *	We've added the capability for the tuning function to key the transmitter in CW mode
 *	through a relay (or directly in the case of the BitX) activated by digital pin Xmit_Key.
 *	After we turn the transmitter on, we check to see that we are actually reading
 *	forward power. If not, we inform the operator and do nothing. Similarly, if the
 *	forward power goes away part way through the process, we stop and inform the
 *	operator.
 *	
 *	We must be careful not to keep the transmitter keyed for an excessive amount
 *	of time. 99% of the time, the tuning process takes less than 2 seconds to retune from
 *	one end of the 40 meter band to the other, but, just to be safe, we will include a 5
 *	second timeout in the code. If the timer expires before the antenna is tuned, we will
 *	turn the transmitter off, display "*** Failed ***" on line 2 of the display, and exit
 *	the function. If that happens, usually, waiting a few seconds and pressing the auto-tune
 *	button again will tune the antenna.
 *	
 *	Note:	The following settings of the search parameters have been fine tuned based on
 *			having the motor step set to 1/4 steps (also determined to be the optimum
 *			setting).
 */

void SWR_Tune ()
{
	int Current_Step;					// Rotation increment each time through
	int	Rotation;						// Current number of steps to move

 	if ( digitalRead ( Tune_Button ))			// If button isn't activated
 		return;									// Nothing to do!

	Xmit_Time = millis ();						// Record start time
	digitalWrite ( Xmit_Key, Xmit_On );			// Turn on the transmitter
	delay ( 200 );								// Wait 0.2 second (V1.2 - Changed from 100mS)

	if ( !Is_Transmitting () )					// Not transmitting?
		return;

	Last_SWR = analogRead ( SWR_Input );		// Get the initial reading	

	LCD_Buffer_1 = "** Auto Tune **";			// Show auto-tune in progress
	Check_LCD ();								// Force immediate display


/*
 *	Before we start the tuning process, let's see if the antenna is already tuned, which we
 *	can determine by seeing if the SWR reading is less than Step_2_Limit. If that is the 
 *	case, report the real SWR and return.
 */

	if ( Last_SWR < Step_2_Limit )				// If already in tune no need to do it again!
	{
		digitalWrite ( Xmit_Key, Xmit_Off );	// Turn the transmitter off
		LCD_Buffer_2 = Compute_SWR ();			// Get final real SWR reading
		return;
	}


/*
 *	Step 1: We alternately rotate the motor back an forth in increasing multiples
 *	of Step_1_Degrees to coarse tune the SWR reading. We stop when the SWR drops below
 *	Step_1_Limit. We also stop the process if the transmitter stops transmitting, or the
 *	Xmit_Timeout is exceeded.
 */

	#ifdef	SWR_Debug
		Serial.println ( "\nTuning for best SWR - Step 1\n" );
	#endif

	Current_Step = Convert_Degrees ( Step_1_Degrees );	// Convert to steps
 	Rotation = Current_Step;							// First move in the search
 
 	while ( true )							// Until we decide to stop
 	{
		if ( !Is_Transmitting () )			// Not transmitting?
			return;							// Nope, give up!

		if ( Check_Time () )				// Time limit exceeded?
			return;							// Yes, we're done
			
		LCD_Buffer_2 = Compute_SWR ();		// Get real SWR reading

		#ifdef	SWR_Debug
			Serial.print ( "PWR = " );
			Serial.print ( analogRead ( PWR_Input ));
			Serial.print ( ", SWR = " );
			Serial.println ( analogRead ( SWR_Input ));
		#endif

		if ( Last_SWR == 0 )						// Can't get any better
		{
			digitalWrite ( Xmit_Key, Xmit_Off );	// Turn the transmitter off
			return;
		}

		if ( Last_SWR < Step_1_Limit )				// Reached step 1 limit?
			break;									// Yes, go to step 2

 		Rotate_Motor ( Rotation, Step_1_Limit );	// Spin the motor

		if ( Rotation > 0 )
			Rotation = -( Rotation + Current_Step );	// Increase rotation & reverse direction

		else
			Rotation = -( Rotation - Current_Step );	// Increase rotation & reverse direction

		delay ( 25 );								// Something needs time to settle (V1.2 changed from 10mS)
 		Last_SWR = analogRead ( SWR_Input );		// Get new SWR value

		LCD_Buffer_2 = Compute_SWR ();				// Get the real SWR
		Check_LCD ();								// Force immediate display
	}
 

/*
 *	Begin Step 2. We alternately rotate the motor back an forth in increasing multiples
 *	of Step_2_Degrees to fine tune the SWR reading. We stop when the SWR drops below
 *	Step_2_Limit. We also stop the process if the transmitter stops transmitting, or the
 *	Xmit_Timeout is exceeded.
 */

	Current_Step = Convert_Degrees ( Step_2_Degrees );	// Convert to steps
 	Rotation = Current_Step;							// First move in the search
 

	#ifdef	SWR_Debug
		Serial.println ( "\nTuning for best SWR - Step 2\n" );
	#endif

 	while ( true )						// Until we decide to stop
 	{
		if ( !Is_Transmitting () )		// Not transmitting?
			return;						// Nope, give up!

		if ( Check_Time () )			// Time limit exceeded?
			return;						// Yes, we're done
			
		LCD_Buffer_2 = Compute_SWR ();			// Get real SWR reading

		#ifdef	SWR_Debug
			Serial.print ( "PWR = " );
			Serial.print ( analogRead ( PWR_Input ));
			Serial.print ( ", SWR = " );
			Serial.println ( analogRead ( SWR_Input ));
		#endif

		if ( Last_SWR == 0 )						// Can't get any better
		{
			digitalWrite ( Xmit_Key, Xmit_Off );	// Turn the transmitter off
			return;
		}

		if ( Last_SWR < Step_2_Limit )				// Tuned?
		{
			digitalWrite ( Xmit_Key, Xmit_Off );	// Turn the transmitter off
			LCD_Buffer_2 = Compute_SWR ();			// Get final real SWR reading
			break;
		}

 		 Rotate_Motor ( Rotation, Step_2_Limit );	// Spin the motor

		if ( Rotation > 0 )
			Rotation = -( Rotation + Current_Step );	// Increase rotation & reverse direction

		else
			Rotation = -( Rotation - Current_Step );	// Increase rotation & reverse direction

		delay ( 10 );									// Necessary delay
 		Last_SWR = analogRead ( SWR_Input );			// Get new SWR value

		LCD_Buffer_2 = Compute_SWR ();				// Get the real SWR
		Check_LCD ();								// Force immediate display
	}

	digitalWrite ( Xmit_Key, Xmit_Off );		// Turn the transmitter off


	#ifdef	SWR_Debug
 		Serial.print ( "Antenna is tuned! Final " );
 		Serial.println ( LCD_Buffer_2 );
	#endif
}


/*
 *	Is_Transmitting checks for transmitter power and if we are not transmitting, it
 *	indicates that on the displey and returns "false". If we are transmitting, it
 *	returns "true"
 */
 
bool Is_Transmitting ()
{
	if ( analogRead ( PWR_Input ))			// Still transmitting?
		return true;						// Yes we are!

	LCD_Buffer_1 = "** Auto Tune **";		// Show auto-tune in progress
	LCD_Buffer_2 = "Transmitter Off";		// Inform operator
	digitalWrite ( Xmit_Key, Xmit_Off );	// Turn the transmitter off
	return ( false );						// Indicate not transmitting
}


/*
 *	Check_Time sees if we have exceeded the Xmit_Timeout, and if not, returns "false". If
 *	the limit has been exceeded, we inform the operator and return "true".
 */

bool Check_Time ()
{
	if ( millis () - Xmit_Time > Xmit_Timeout )		// Time limit exceeded?
		{
			digitalWrite ( Xmit_Key, Xmit_Off );	// Yes, kill the transmitter
			LCD_Buffer_2 = "*** Timeout ***";		// Inform operator
			return true;
		}

	return false;									// Limit not exceeded
}

/*
 * 	The Read_Encoder function first looks to see if the switch on the encoder is activated
 * 	which is indicated by a LOW on the switch pin. If the switch is pressed, we will move
 * 	the motor Encoder_Big_Step steps. If the switch is not activated, we're in fine
 * 	adjustment mode and will rotate the motor by Encoder_Small_Step steps.
 * 	
 * 	We don't automatically turn the transmitter on here, but we do test to see if
 * 	it is on or not. If it is on, we will take and report the SWR reading. If the
 * 	transmitter is not on, we will assume the operator just wants to tweak the
 * 	tuning based on receiver noise.
 * 	
 * 	Note that Encoder_Big_Step and Encoder_Small_Step were originally defined in degrees, 
 * 	but were converted from degrees to steps back in the setup function.
 */

void Read_Encoder ()
{
	int	Steps_To_Rotate;
	int Steps;

	#ifdef	Debug
		Serial.println ( " Starting Read_Encoder" );
	#endif
	
	if ( digitalRead ( Encoder_Switch ) == LOW )	// Switch pressed?
		Steps_To_Rotate = Encoder_Big_Step;			// Yes, use big steps

	else
		Steps_To_Rotate = Encoder_Small_Step;		// No, then fine adjustment
	
/*
 *	Read the encoder. The Rotary object actually does the reading of the state of
 *	the encoder pins and determines which way it is moving and how much.
 */

	unsigned char Result = Encoder.process ();
		
	if ( Result == DIR_NONE )				// Encoder didn't move (don't think this
		return;								// Is actually possible)
		
	else if ( Result == DIR_CW )			// Encoder rotated clockwise
		Steps = Steps_To_Rotate;			// So Steps is positive
		
	else if ( Result == DIR_CCW )			// Encoder rotated counter-clockwise
		Steps = -Steps_To_Rotate;			// So Steps is negative

	if (Steps != 0 )						// If the encoder actually moved
		Rotate_Motor ( Steps, No_Limit );	// Spin the motor appropriately

/*
 *	Announce manual tuning
 */

	LCD_Buffer_1 = "* Manual Tune *";			// Note we can't use display in the interrupt function

	#ifdef	SWR_Debug
		Serial.print ( "PWR = " );
		Serial.print ( analogRead ( PWR_Input ));
		Serial.print ( ", SWR = " );
		Serial.println ( analogRead ( SWR_Input ));
	#endif


	if ( !analogRead ( PWR_Input ))			// Not transmitting?
	{
		LCD_Buffer_2 = "  Receive Mode";	// Inform operator
		return;
	}

	LCD_Buffer_2 = Compute_SWR ();			// Get real SWR reading if transmitting

}


/*
 *	The Rotate_Motor function gets invoked from either SWR_Tune or Read_Encoder functions.
 *	How_Much is the number of steps (or partial steps) the motor is to be turned. If How_Much
 *	is positive, we turn the motor clockwise; if How_Much is negative, we rotate the motor
 *	counter-clockwise.
 *	
 *	The Limit parameter will be Step_1_Limit, Step_2_Limit or No_Limit, depending on where
 *	the function was invoked. After each pulse is sent to the motor shield, we check the
 *	SWR_Input to see if it has reached the requested Limit, and if so, we stop the
 *	rotation.
 */
 
void Rotate_Motor ( int How_Much, int Limit )
{
	int	i;										// Loop counter

	digitalWrite ( Motor_1_Enable, Motor_Enable );		// Turn the motor on

	if ( How_Much < 0 )
		{
			digitalWrite ( Motor_1_Direction, CCW );	// and set direction
			How_Much = abs ( How_Much );				// Make it a positive number
		}
	else
		digitalWrite ( Motor_1_Direction, CW );			// Number was already positive
		

	#ifdef	Debug
		Serial.print ( String ( String ( "Rotating motor " ) + String ( How_Much )));

		if ( How_Much < 0 )
			Serial.println ( " CCW" );
		else
			Serial.println ( " CW" );
	#endif


/*	
 * 	Now we move the motor the requested number of steps. The "delayMicroseconds (2)" 	
 * 	between the pulses is necessary to actually get the motor to turn, as is the	
 * 	"delay (1)" after the high pulse.
 * 	 
 * 	I haven't figured out what is really happening on the motor leads as a	 
 * 	result of this code, but it works! Have to look at it with the scope one
 * 	of these days.
 */

	for ( i = 0; i < How_Much; i++ )		
 	{
		digitalWrite ( Motor_1_Step, LOW );				// Pulse LOW
		delayMicroseconds ( 2 );						// Slight pause
		digitalWrite ( Motor_1_Step, HIGH);				// Pulse HIGH
		delay ( 1 );									// Part of sequence
		
		if ( analogRead ( SWR_Input ) < Limit )			// Reached limit?
			break;										// Yes, stop rotation
 	}
 	
	digitalWrite ( Motor_1_Enable, Motor_Disable );		// Turn the motor off
}


/*
 * 	Say_Hello checks to see if the hello message is already on the display, and if
 * 	so, just returns. It also returns if the Hello_Timeout since the time the last other
 * 	type of message was displayed has not expired.
 */

void Say_Hello ()
{
	if ( Hello )								// Already displaying message?
		return;									// Don't need to do it again

	New_Time = millis ();						// Current run time
	if ( New_Time - Hello_Time < Hello_Timeout )	// Time expired?
		return;									// Nope!

	LCD_Print ( 1, String ( "***  WA2FZW  ***" ));		// We're on the air!
	LCD_Print ( 2, String ( " Mag Loop Tuner" ));

	Hello_Time = New_Time;								// Update last time displayed
	Hello = true;										// Message is displayed
}


/*
 *	Check_LCD looks for data in the LCD buffers, and if anything is in either one,
 *	it uses LCD_Print to display them. This is an attempt to fix the lockup when we
 *	try to use the display from the Read_Encoder interrupt routine.
 */

void Check_LCD ()
{
	if ( LCD_Buffer_1.length ())				// Anything in line 1 buffer?
	{
		LCD_Print ( 1, LCD_Buffer_1 );			// Yep!, display it
		LCD_Buffer_1 = "";						// Clear the buffer
		Finish_LCD ();							// Common finish for both buffers
	}

	if ( LCD_Buffer_2.length ())				// Anything in line 2 buffer?
	{
		LCD_Print ( 2, LCD_Buffer_2 );			// Yep!, display it
		LCD_Buffer_2 = "";						// Clear the buffer
		Finish_LCD ();							// Common finish for both buffers
	}
}

void Finish_LCD ()
{
	Hello = false;								// We dont use this to display hello msg
	Hello_Time = millis ();						// Time stamp
}


/*
 *	LCD_Print displays something on the LCD display. Arguments are which line to put it
 *	on, and what to put there. Note, we allow the caller to specify lines 1 or 2, and
 *	convert those to 0 and 1.
 */


void LCD_Print ( int Line, String Info )
{
	int i;									// Loop counter

	if ( Line < 0 || Line > LCD_Height )	// Range check 
		return;

	int	 Line_No = Line - 1;					// Adjust requested line number
 	char Buffer[LCD_Width + 1];					// Need a regular array for print function
	int	 Buffer_Length = Info.length ();		// Length of data

	if ( Buffer_Length > LCD_Width )			// Make sure not too long
		Buffer_Length = LCD_Width;				// If too long, truncate
	
	Info.toCharArray ( Buffer, Buffer_Length+1 );	// Copy data to internal buffer

	if ( Buffer_Length < LCD_Width )					// Need padding?
		for ( i = Buffer_Length; i < LCD_Width; i++ )	// Yep!
			Buffer[i] = ' ';

	Buffer[LCD_Width] = '\0';		 			// Need a null at end of string

	LCD_Display.setCursor ( 0, Line_No );		// 1st character position on Line_No
	LCD_Display.printstr ( Buffer );			// Display it
}


/*
 * Compute_SWR reads the PWR_Input and SWR_Input pins and computes the actual SWR reading
 * which it returns in a String object ready for displaying on the LCD.
 * 
 * The readings are converted to floating point.
 */


String Compute_SWR ()
{
	String	Answer;							// Build the answer here

	float	fSWR;							// Floating version of SWR pin reading
	float	fPWR;							// Floating version of PWR reading
	float	Ratio;							// Used in computation

	fSWR = analogRead ( SWR_Input );		// Read SWR pin
	fPWR = analogRead ( PWR_Input );		// And the forward power input

	#ifdef Debug
		Serial.print ( "fPWR: " );
		Serial.print ( fPWR );
		Serial.print ( ", fSWR: " );
		Serial.println ( fSWR );
	#endif

	Ratio = ( fSWR / fPWR );					// Standard VSWR formula
	fSWR = (( 1 + Ratio ) / ( 1 - Ratio ));		// Part 2
 
	#ifdef Debug
		Serial.print ( "\nSWR: ");
		Serial.println ( fSWR );
	#endif

	fSWR += 0.05;								// Rounding
	Answer = "  SWR: ";							// Construct the answer string
	Answer += fSWR;
	Answer.remove ( Answer.length () -1 );		// Only want one decimal place
	Answer += " : 1";

	return Answer;
}
