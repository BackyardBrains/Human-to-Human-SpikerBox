/*
    ----------------------------------------------------------------------------------------------------
    Backyard Brains 25. May 2020

    Made for HHI SpikerBox V1(0.953)


    Code is made for testing of parameters of stimulation. DO not use for any other purpose.

    ----------------------------------------------------------------------------------------------------
*/
//Length of stimulation period expressed in 100usec
#define LENGTH_OF_AUTOMATIC_STIMULATION 10000
//Length of silent period between two stimulations in 100usec
#define LENGTH_OF_PAUSE_BETWEEN_STIMULATION 30000
//stimulation frequencies
int stimFrequencies[6] = {10,15,25,40,70,100};
#define TENS_OF_MICROSECONDS_PULSE_WIDTH 8 //80uSec (can not be greater than 10)
float percentageOfStimulationInSixSeconds = 50.0;


volatile uint16_t counterForAutomaticStimulation = 0;


#include <avr/sleep.h>//this AVR library contains the methods that controls the sleep modes
float lowPowerVoltage = 8.1;//In volts
float shutDownVoltage = 7.1;//In volts
//if this % of six second period stimulation is over threshold we will disable TENS and
//wait for TENS to be at least 3 without stimullation over threshold and than enable TENS again



#define POWER_STATE_GOOD 0
#define POWER_STATE_LOW 1
#define POWER_STATE_SHUT_DOWN 2
//Clear/reset bit in register "cbi" macro
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
//Set bit in register "sbi" macro
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#define CURRENT_SHIELD_TYPE "HWT:MUSCLESS;"     //type of the board. Used for detection of shield 
//by desktop Spike Recorder application
#define NOISE_FLOOR_FOR_ENVELOPE 530            //must be greater than 512 and less than 1023
#define SENSITIVITY_BUTTON_PIN 3                //pin for button that changes sensitivity
#define MAX_NUMBER_OF_CHANNELS 2                //maximum number of analog channels

int interrupt_Number = 19;//198;                     // Output Compare Registers  value = (16*10^6) / (Fs*8) - 1
// Set to 198 for 10000 Hz max sampling rate
// Used for main timer that defines period of measurements

volatile uint8_t counterOfPeriods= 0;
volatile uint16_t emgSample = 0;
volatile uint16_t baterySample = 0;

#define MESSAGE_BUFFER_SIZE 100                 //size of buffer used to send messages to serial (to PC)
//it needs to be big enough to hold start escape sequence,
//message and end escape sequence
byte messageBuffer[MESSAGE_BUFFER_SIZE];        //buffer used to send messages to serial (to PC)
#define ESCAPE_SEQUENCE_LENGTH 6                //length of escape sequence that is used when sending messages to PC


byte escapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255, 255, 1, 1, 128, 255};       //start escape sequence
byte endOfescapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255, 255, 1, 1, 129, 255};  //end escape sequence

byte messageSending = 0;                        //flag that signals TX handler to send data from "messageBuffer"
byte lengthOfMessasge = 0;                      //length of message in message buffer




int incrementsForLEDThr[] = {28, 53, 81, 108, 135, 161};      //threshold intervals for different sensitivities (sens-30)/6
//for example:for value = 28. EMG envelope needs to increase 28
//AD units to light up one more LED

int8_t lastSensitivitiesIndex = 0;                               //index of sensitivity from sensitivities[] that is currently in use
byte sensitivityButtonPressed = 0;                            //used to ignore glitches from button bounce
uint16_t sensitivityVisualFeedbackCounter = 0;                //timer counter that holds selected sensitivity LED ON when user changes
//sensitivity with button press
#define SENSITIVITY_LED_FEEDBACK 5000                         //length of interval (expressed in semple periods) sensitivity feedback LED will be ON
unsigned long count = 0;

uint16_t sensitivityVisualFeedbackCounterMax = SENSITIVITY_LED_FEEDBACK;      //max value of sensitivityVisualFeedbackCounter. It changes with sample freq.

volatile byte outputBufferReady = 0;                          //variable that signals to main loop that output data frame buffer is ready for sending
byte outputFrameBuffer[MAX_NUMBER_OF_CHANNELS * 2];           //Output frame buffer that contains measured EMG data
//formated according to SpikeRecorder serial protocol

#include <avr/io.h>
#include <avr/interrupt.h>
volatile uint16_t envelopeFirstChanel = 0;                    //value of calculated envelope of first EMG channel
uint16_t movingThresholdSum;                                  //temp calculation variable for thresholds for LEDs and servo
uint16_t incrementForLEDThreshold = 81;                       //increment for LED threshold (how much EMG needs to change to light up one more LED)
//depends on selected sensitivity
byte tempCalcByteMask = 0;                                    //general purpose bitmask for shifting bits

byte readyToDoAuxComputation = 0;                             //flag that enables auxiliary computation only once per sampling timer interrupt

uint16_t tempEnvValue;
byte envelopeDecrementCounter = 0;
#define MAX_ENV_DECREMENT_COUNTER 5

byte emgCrossedTheThreshold = 0;


uint16_t periodOfStimulationExpressedInPeriodsOfSampling = 0;
uint16_t counterForPeriodOfStimulation = 0;

uint16_t shutDownVoltageInADUnits = 0;
uint16_t lowPowerVoltageInADUnits = 0;
uint16_t lowPowerVoltageInADUnitsPlus20 = 0;

byte powerState = POWER_STATE_GOOD;
uint16_t powerInADUnits = 1023;

uint16_t measurementTimerForBatery = 0;

uint16_t stimulationTimeCounter = 0;
uint16_t maxStimulationSamples = 0;
bool stimulationEnabled = true;
//----------------------------------- SETUP FUNCTION --------------------------------------------------------------------------------------------
void setup()
{

  pinMode( 8, OUTPUT);                                  //VU LED 1 PB0
  pinMode( 2, OUTPUT);                                  //VU LED 2 PD2
  pinMode(10, OUTPUT);                                  //VU LED 3 PB2
  pinMode(11, OUTPUT);                                  //VU LED 4 PB3
  pinMode(12, OUTPUT);                                  //VU LED 5 PB4
  pinMode(13, OUTPUT);                                  //VU LED 6 PB5

  pinMode(5, OUTPUT);                                   //TENS ON/OFF control output - PD5
  pinMode(9, OUTPUT);                                   //Pulse generator output     - PB1

  pinMode(SENSITIVITY_BUTTON_PIN, INPUT);               //sensitivity button pin
  pinMode(3, INPUT);                                    //aux. button

  pinMode(7, OUTPUT);                                   //yelow LED
  pinMode(4, OUTPUT);                                   //Red LED output  - low power - PD4
  pinMode(6, OUTPUT);                                   //Green LED output - power ON - PD6

  //setup serial communication
  Serial.begin(230400); //Serial communication baud rate (alt. 115200)
  delay(300); //whait for init of serial
  Serial.println("StartUp!");
  Serial.setTimeout(2);

  digitalWrite(9, LOW); //do not power collector of driving transistor
  digitalWrite(5,HIGH); //turn on the transistor

  //debug pins ---------------------------------------------
   pinMode(16, OUTPUT);//a2
   pinMode(17, OUTPUT);//a3
   pinMode(18, OUTPUT);//a4
  //debug pins---------------------------------------------
  

   //test potentiometer position
  uint16_t potValue = 0;
  int blinkingCounter = 6;
  while(1)
  {
      potValue = analogRead(A1);
      if(potValue <8)
      {
        break; 
      }
      delay(100);
      blinkingCounter--;
      if(blinkingCounter<1)
      {
        blinkingCounter =6;
      }
      if(blinkingCounter>3)
      {
        digitalWrite(4, HIGH); 
        digitalWrite(6, LOW);   
      }
      else
      {
        digitalWrite(4, LOW); 
        digitalWrite(6, HIGH);   
      }
  }
  //Turn OFF leds
  digitalWrite(4, LOW); 
  digitalWrite(6, LOW); 
  //Turn off base current on driving transistor
  digitalWrite(5,LOW);
  digitalWrite(8, HIGH); //first stimulation active (LED)



  lowPowerVoltageInADUnits = (uint16_t)((lowPowerVoltage / 15.0) * 1023);
  lowPowerVoltageInADUnitsPlus20 = lowPowerVoltageInADUnits+20;
  shutDownVoltageInADUnits = (uint16_t)((shutDownVoltage / 15.0) * 1023);

  if (percentageOfStimulationInSixSeconds > 100)
  {
    percentageOfStimulationInSixSeconds = 100.0;
  }
  maxStimulationSamples = (uint16_t)(60000 * (percentageOfStimulationInSixSeconds / 100.0)); //6sec *(max%/100%)


  powerState = POWER_STATE_GOOD;
  //put initial measurement of voltage to highest to avoid going in sleep mode
  baterySample = 1023;
  //turn ON Green LED
  PORTD &= B11101111;
  PORTD |= B01000000;


  //setup serial communication
  Serial.begin(230400); //Serial communication baud rate (alt. 115200)
  delay(300); //whait for init of serial
  Serial.println("StartUp!");
  Serial.setTimeout(2);

  cli();                                                //stop interrupts

  //setup ADC
  cbi(ADMUX, REFS0);                                    //set ADC reference to AVCC
  cbi(ADMUX, ADLAR);                                    //left Adjust the result
  cbi(ADMUX, ADATE);                                    //left Adjust the result
  sbi(ADCSRA, ADEN);                                    //enable ADC
  sbi(ADCSRA, ADIE);                                    //enable ADC Interrupt

  //set ADC clock division to 16
  sbi(ADCSRA, ADPS2);                                   //1
  cbi(ADCSRA, ADPS1);                                   //0
  cbi(ADCSRA, ADPS0);                                   //0


  //set timer1 interrupt
  TCCR1A = 0;                                           //set entire TCCR1A register to 0
  TCCR1B = 0;                                           //same for TCCR1B
  TCNT1  = 0;                                           //initialize counter value to 0;
  OCR1A = interrupt_Number;                             //output compare registers
  TCCR1B |= (1 << WGM12);                               //turn on CTC mode
  TCCR1B |= (1 << CS11);                                //set CS11 bit for 8 prescaler
  TIMSK1 |= (1 << OCIE1A);                              //enable timer compare interrupt



  //set timers and maximum values of timers to new value according to sample rate
  sensitivityVisualFeedbackCounterMax = SENSITIVITY_LED_FEEDBACK;

  //set sensitivity and all variables that depend on sensitivity depending on lastSensitivitiesIndex
  incrementForLEDThreshold = incrementsForLEDThr[lastSensitivitiesIndex];

  periodOfStimulationExpressedInPeriodsOfSampling = 10000 /stimFrequencies[lastSensitivitiesIndex];
  counterForPeriodOfStimulation = periodOfStimulationExpressedInPeriodsOfSampling;

  sei();                                                //enable Global Interrupts
}

//------------------------------------------------ MAIN LOOP ----------------------------------------------------
//   Here we:
//    - initiate sending of data frame by setting UDR0
//    - parse and execute received messages (for number of channels and board type)
//    - calculate EMG envelope
//    - refresh LEDs based on EMG envelope
//    - check for button press (sensitivity and gripper mode)
//---------------------------------------------------------------------------------------------------------------


void loop()
{




  //do the auxiliary computation once per timer interrupt
  if (readyToDoAuxComputation == 1)
  {
    readyToDoAuxComputation = 0;
/**/
    //----------------------- POWER MANAGEMENT --------------------------
    powerInADUnits =  baterySample;// = 1023;

    if (powerState == POWER_STATE_GOOD)
    {
      if (powerInADUnits < lowPowerVoltageInADUnits)
      {
        powerState = POWER_STATE_LOW;
        //red
        PORTD &= B10111111;
        PORTD |= B00010000;
      }
    }
    if (powerState == POWER_STATE_LOW)
    {
      if (powerInADUnits < shutDownVoltageInADUnits)
      {
        powerState = POWER_STATE_SHUT_DOWN;
        //shut down
        sleep_enable();//Enabling sleep mode
        PORTB = B0;
        PORTD = B0;
        cli();
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_cpu();


      }
      else if (powerInADUnits > lowPowerVoltageInADUnitsPlus20)
      {
        powerState = POWER_STATE_GOOD;
        //green

        PORTD &= B11101111;
        PORTD |= B01000000;
      }
    }


    //----------------------- POWER MANAGEMENT --------------------------


   
      //---------------------------------------- STIMULATION  ----------------------------------
      //check if we should activate stimulation



      if(counterForAutomaticStimulation>0)
      {
         
        counterForAutomaticStimulation--;
        if(counterForAutomaticStimulation<LENGTH_OF_AUTOMATIC_STIMULATION)
        {
          stimulationEnabled = true;
          emgCrossedTheThreshold = 1;

          
        }
        else
        {
          stimulationEnabled = false;  
          emgCrossedTheThreshold = 0;
        }
        
      }
      else
      {
       
          counterForAutomaticStimulation = LENGTH_OF_AUTOMATIC_STIMULATION+LENGTH_OF_PAUSE_BETWEEN_STIMULATION;
           
      }
      

    
      //-------------------------------------------------------------------------------------------------

      if (sensitivityVisualFeedbackCounter == 0) //disable update of LEDs when we display selected sensitivity level
    {

      //---------------------------------- SENSITIVITY BUTTON -----------------------------------------------

      //check if button is pressed (HIGH)
      if (PIND & B00001000)//pin D3
      {
        if (sensitivityButtonPressed == 0) //if button was not pressed when we scan last time
        {
          sensitivityButtonPressed = 1;
          //turn OFF all leds
          PORTB &= B11000010;
          PORTD &= B11111011;

          //turn OFF TENS
          PORTD &= B11011111;// PD5
          PORTB &= B11111101;//PB1

          lastSensitivitiesIndex++;
          if (lastSensitivitiesIndex == 6)
          {
            lastSensitivitiesIndex = 0;
          }

          
         periodOfStimulationExpressedInPeriodsOfSampling = 10000 / stimFrequencies[lastSensitivitiesIndex];
         counterForPeriodOfStimulation = periodOfStimulationExpressedInPeriodsOfSampling;



          for (int i=0;i<6;i++)
          {
                if(i<=lastSensitivitiesIndex)
                {
                    //Turn ON only one LED that represent sensitivity level
                    if (i == 1)
                    {
                      PORTD |= B00000100;
                    }
                    else
                    {
                      tempCalcByteMask = 1 << i;
                      PORTB |= tempCalcByteMask;//light up one LED for visual feedback of sensitivity
                    }
                }
          }
          sensitivityVisualFeedbackCounter = sensitivityVisualFeedbackCounterMax;//set timer for visual feedback
        }
      }
      else
      {
        sensitivityButtonPressed = 0;
      }
    }//end of sensitivity visual LED feedback counter
    else
    {
      //decrement sensitivityVisualFeedbackCounter counter. When it hits
      //zero we are finished with signaling selected sensitivity level
      //and LEDs will continue to update and display EMG level
      sensitivityVisualFeedbackCounter--;
    }




   

    }//end of aux. computation




  //----------------------------------SENDING DATA ----------------------------------------------------------
  if (outputBufferReady == 1  ) //if we have new data
  {
   
  
    

    outputBufferReady = 0;//this will be zero until we send whole frame buffer and fill it again
    //since we want to do aux computation (LEDs, relay, servo) only once per sample period
    //and main loop is called multiple times per period (anytime the code is not in interrupt handler)
    //we set this flag here so that aux comp. is done only once after this initialization of frame sending
    readyToDoAuxComputation = 1;

    //Sends first byte of frame. The rest is sent by TX handler.
    Serial.write(outputFrameBuffer, 2);

  }//end of detection of fresh frame data


  //----------------------------------SENDING MESSAGE ----------------------------------------------------------
  if (messageSending == 1)
  {
    messageSending = 0;
    Serial.write(messageBuffer, lengthOfMessasge - 1);
  }
  

}//end of main loop
//---------------------------------------------- END OF MAIN LOOP ---------------------------------------------------------


void serialEvent()
{
  String inString = Serial.readStringUntil('\n');
  cli();//dissable interrupts
  //turn OFF TENS
  PORTD &= B11011111;// PD5
  PORTB &= B11111101;//PB1
  sendMessage(CURRENT_SHIELD_TYPE);//send message with escape sequence
}


//------------------------------------------ SAMPLING TIMER INTERRUPT --------------------------------------------------
ISR(TIMER1_COMPA_vect) {

  counterOfPeriods++;
  if(counterOfPeriods == TENS_OF_MICROSECONDS_PULSE_WIDTH)
  {
      PORTB &= B11111101;//PB1 (D9) high freq stim  
  }
  if(counterOfPeriods==10)
  { 
            counterOfPeriods=0;
            if (measurementTimerForBatery == 0)
            {
              ADMUX =  B01000101;                                          //Start ADC Conversions
            }
            else
            {
              ADMUX =  B01000000;                                           //Start ADC Conversions
            }
            ADCSRA |= B01000000;                                          //do this at the begining since ADC
                                              //signal main loop to send frame
          
            if (emgCrossedTheThreshold && stimulationEnabled)
            {
              counterForPeriodOfStimulation--;

              if (counterForPeriodOfStimulation == 0)
              {
                PORTB |= B00000010;//PB1 (D9) high freq stim
                counterForPeriodOfStimulation = periodOfStimulationExpressedInPeriodsOfSampling;
              }
              else
              {
                PORTD &= B01111111;//PD7 (D7) LED yellow
              }
              PORTD |= B10000000;//PD7 (D7) LED yellow
              PORTD |= B00100000;//PD5 (D5) enable stim
            }
            else
            {
              counterForPeriodOfStimulation = periodOfStimulationExpressedInPeriodsOfSampling;
              PORTD &= B01011111;//PD7 (D7) and PD5 (D5) (LED yellow and enable stim)
              PORTB &= B11111101;//PB1 (D9) high freq stim
            }

            //can work in paralel with other stuff
          
            outputFrameBuffer[0] =  (emgSample >> 7) | 0x80;       //convert data to frame according to protocol
            outputFrameBuffer[1] =  emgSample & 0x7F;             //first bit of every byte is used to flag start of the framr
            outputBufferReady = 1;      
  }
  
 
}



//---------------------------ADC INTERRUPT HANDLER ----------------------------------------------------------------------
//This is called when ADC conversion is complete.
ISR(ADC_vect)
{
  if (measurementTimerForBatery == 0)
  {
    baterySample = ADCL | (ADCH << 8);      // store lower and higher byte of ADC
  }
  else
  {
    emgSample = ADCL | (ADCH << 8);      // store lower and higher byte of ADC
  }
  measurementTimerForBatery++;
}



//------------------------------- SEND MESSAGE WITH ESCAPE SEQUENCE ----------------------------------------------------
//timer for sampling must be dissabled when
//we call this function

void sendMessage(const char * message)
{
  int i;
  lengthOfMessasge = 0;
  //add escape sequence to buffer
  for (i = 0; i < ESCAPE_SEQUENCE_LENGTH; i++)
  {
    messageBuffer[lengthOfMessasge++] = escapeSequence[i];
    if (lengthOfMessasge == MESSAGE_BUFFER_SIZE)
    {
      lengthOfMessasge = 0;
    }
  }

  //add message to buffer
  i = 0;
  while (message[i] != 0)
  {
    messageBuffer[lengthOfMessasge++] = message[i++];
    if (lengthOfMessasge == MESSAGE_BUFFER_SIZE)
    {
      lengthOfMessasge = 0;
    }
  }

  //add end of escape sequence to buffer
  for (i = 0; i < ESCAPE_SEQUENCE_LENGTH; i++)
  {
    messageBuffer[lengthOfMessasge++] = endOfescapeSequence[i];
    if (lengthOfMessasge == MESSAGE_BUFFER_SIZE)
    {
      lengthOfMessasge = 0;
    }
  }
  messageSending = 1;                                                  //set flag that we are sending message and not data frames

  sei();

}
