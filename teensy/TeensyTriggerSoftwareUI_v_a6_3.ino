// Usage: 
//  Teensy will respond to the following commands over serial.
//  End of message (eom) character is CR (ASCII 13). 
//    SET XX (?)    ... Sets the reference voltage to the comparator by setting the PWM output to XX volts; or reads the current filtered PWM level (?).
//    ENAB XX (?)   ... XX=1 Enables comparator function; XX=0 disables comparator (ie. external input signal is ignored); or reads current state with ?
//    GAIN XX (?)   ... Changes the gain between values 1,2,4,8 on pre-amp stage. 
//
//
// CHANGELOG:
// 9 Aug 2018.  First implementation SetThreshArduinover.4.ino by Blake Haist. 
// 9 Aug 2018.  GWT. Clean up commenting, and minor code ordering for constants. 
// 10 Aug 2018. GWT. Interrupt mode set to toggle. 
//              Made 'LED_Output' a const int, and make spelling/case consistent throughout. 
//              Changed 
//                    'outputPin'   to 'PWM_OUT' 
//                    'outputPinL'  to 'MOD_OUT'
//                    'outputPinLI' to 'NMOD_OUT'
//                    'inputPin'    to 'PD_IN'
//                    'LogicPinA0'  to 'GAIN_A0'
//                    'LogicPinA1'  to 'GAIN_A1'
//              Commented out 'Serial.print("d")' from 'Gain 8' response. 
//              Moved 'while (!Serial);' to immediately after Serial.begin. Solve intermittent Teensy upload issue?
//              Added a version const and prints 'Teensy Trigger v'+version to serial after serial port successfully opens. 
//              Moved initialisations of vtb, btv, eom to outside of loop(). Made them const
//              Changed following to make clear that vtb is not 1/btv because PWM is 8-bit and ADC is 10-bit
//                    'vtb'   to 'vtb_PWM'
//                    'btv'   to 'btv_ADC'
// 10 Aug 2018. GWT. Created version a5.0. Renamed sketch from 'SetThreshArduinover.4' to 'TeensyTriggerFirmware_v_a5'
//              Added command parser module
//              Changed instruction set nomenclature
//              Made commands case insensitive
//              Changed label 'a' to 'PWM_val' in Set; clarify meaning. 
//              Changed 'PWM_val' from type float to int since analogWrite(pin, val) expects int val. 
//              Changed 'btv_ADC' to 'vtb_ADC'. Label was not descriptive. 
//              Removed ADCSRB |= (1 << ACME) from end of SET. Didnt seem like ACME was changed during SET. 
// 14 Aug 2018. GWT. 
//              Added 'isEnab' state variable to keep track of enable state (instead of using LED_PIN).
//                ...something went wrong. Left this buggy -- slow reponses. 
// 15 Aug 2018. GWT. Not convinced that double call to sei() or cli() by themselves causes hangups 
// 17 Aug 2018. GWT. Changed A0 and A1 pins to 10 and 9, resp. 
//              Observed that comparator is broken. Threshold updates, but comparator output suggests otherwise. 
// 27 Aug 2018. BH. Added 'Interupt_state' bool to keep track of wether the interupts are on or not.
//              Added check at end of each loop to make sure the 'Interupt_state' bool is correct and interupts are on/off and match the 'isEnab' state.
//              This is to make sure the end state of each loop is correct after temporary changes to the 'Interupt_state' by turning on/off interupts regardless of 'ENab' (what user set).
// 28 Aug 2018. BH. Changed interrupt enable/disables from sei/cli to setting/clearing ACIE bit. 
//              Restored ADCSRB |= (1 << ACME) lines in SET command. Otherwise trigger level set to bandgap ref. 
//              GWT. GAIN XX now outputs "0" to serial if successful
//              Incremented version to 5.1
//29 Aug 2018. BH. replaced resetting initial pin for interupts at beginning of code with setting the isEnab state in code set up. removed now unnesariy Interupt_state variable. 
//             Confired commented out delay after comport opening unnessary. Documented Code Logic in shared folder. Got rid of pin set to turn on interupts after the command cycle.
//30 Aug 2018. BH. Added Frequency measurement. Measuring the time works, but measuring the frequency does not yet. Fixed the delay in the ISR to microsecondsDelay from Delay
//5  Sep 2018. BH. Tested functionality of program, updated type of variables in the freq measurement to allow fractions of seconds, allowed more accurate and precise frequency measurements aat a quicker speed
//6  Sep 2018. BH. Cleaned up, commented code
//11 Sep 2018. BH. Added pulsewidth set function and optional time out on serial read (time out currently commented out)
//19 Sep 2018. BH. Updated error codes
// TO DO:
//    -- search for 'TODO' and '???'
//    - Test functionality under high trigger rates. Does serial hang? No
//    - Do we need setInterrupt anymore? Do we ever need to handle double-enables/disables? Yes
//    - Init some value for PWM output. Observed output floats on startup I think Garwing has done this
//    - Deal with invalid inputs. eg Set 100, or Thrresg Done
//    - timeout for SET XX 
//    - Allow all comands to have query mode. Done
//    - Add *IDN? command
//    - Understand 'COMMAND?' as well as 'COMMAND ?'
//    - Understand why PWM freq doesnt change after setting TCCR1B bits. 
//    - Allow 'pulseWidth' to be software-settable.  yes need to test
//    





#include <avr/interrupt.h>   // include funtions for the interupt from c

// Version
const String VERNUM   = "a6"; // a=alpha (non-release). 
const bool debug      = false;   // Extra prints for debug mode that would otherwise break comms with PC. 

// Software constants
const int   vtb_PWM   = 51;     // conversion from volts to bits for analogWrite(pin, bitlevel). PWM has 8-bit resolution ie. 255 = 5*vtb with vtb=51. 
const float vtb_ADC   = 204.8;  // conversion from bits to volts for user display. ADC has 10-bit resolution, ie. 2^10 = 1024 is mapped to 5V (Vcc). 
const char  eom       = 13;     // End of Message (eom) character is CR
const int   maxCmdLen = 50;     // max number of chars in a command sent from PC. 
const float PWM_set_err = 0.05; // [V] allowable error between requested threshold level and measured level. 
const float tau_RC    = 16E-3;  // [s] RC time constant for PWM LPFs. Teensy board rev 1.1 has R=3.3k, C=4.7n. 
float mStS=1000;            // milliseconds to seconds for frequency calc
const float Set_clock_out =5000;            // clock out time for set function loop


//Runtime variables
volatile bool isEnab=1;                    // keeps track of trigger enabled state. Needed to avoid sending, starts set to 1 to match ui starting state
volatile float Numb_of_Interupts=0;        // set counter for number of interuts 
volatile unsigned long TimeS=0;             //start with TimeS at Zero
volatile float Set_timer = 0;
//intialize Set_timer
// Teensy 2.0 pinouts
const int PWM_OUT     = A9;     // PWM output pin. 
const int MOD_OUT     = A2;     // Comparator output pin to gate chip (labeled 'MOD' on PCB). Normally high.
const int NMOD_OUT    = A3;     // Complement of A2 (labeled 'NMOD'). Unused on circuit; output for convenience. Normally low. 
const int PD_IN       = A5;     // Comparator input pin for PD input (labeled 'PD_IN' on PCB). Requires ADC capable, and connected to comparitor. 
const int GAIN_A0     = 10;      // A0 bit of gain select word for preamp
const int GAIN_A1     = 9;      // A1 bit of gain select word for preamp. A0,A1.. 00=1x, 01=2x, 10=4x, 11=8x
const int LED_OUTPUT  = 11;     // Pre-populated LED on Teensy 2 to be used as state refrence for the enable/disable button on the ui

// Ringdown trigger parameters
volatile int pulsewidth    = 300;            // Gating pulse width in us. Needs to be much longer than expected longest ringdown time
const int debouncedelay = 3*pulsewidth;   // Debounce lockout time. Ignores trigger events for a set time after a successful trigger.




// Interrupt Service Routine.
// Outputs the MOD and NMOD signals when interrupts enabled and trigger condition satified. 
// Nb.  1. Serial port does not work during ISR. 
//      2. Could potentially bit bang this for higher speed! Current delay is few us???
ISR(ANALOG_COMP_vect) { 
  ACSR &= ~(1 << ACIE);   //ACIE=0();                            // Disable interrupts to service this one
  digitalWrite(MOD_OUT, LOW);       // MOD is pulled low
  digitalWrite(NMOD_OUT, HIGH);     // NMOD is pulled high
  digitalWrite(LED_OUTPUT, HIGH);   // blink for visual indication
  delayMicroseconds(pulsewidth);    // wait for length of gating pulse
  digitalWrite(MOD_OUT, HIGH);      // MOD is pulled high
  digitalWrite(NMOD_OUT, LOW);      // NMOD is pulled low
  digitalWrite(LED_OUTPUT, LOW);
  delayMicroseconds(debouncedelay);             // prevent multiple triggers off of one crossing
  Numb_of_Interupts++;
}



    
void setup() {
  //Set Teensy pin directions
  pinMode(PWM_OUT,  OUTPUT);  
  pinMode(MOD_OUT,  OUTPUT);       
  pinMode(NMOD_OUT, OUTPUT);      
  pinMode(PD_IN,    INPUT);          
  pinMode(GAIN_A0,  OUTPUT);
  pinMode(GAIN_A1,  OUTPUT);
  pinMode(LED_OUTPUT, OUTPUT);


  // Default outputs
  digitalWrite(GAIN_A0,  LOW); digitalWrite(GAIN_A1,  LOW);       //inits prescaler to have gain of 1. 
  digitalWrite(MOD_OUT,  HIGH);                                   //normally high
  digitalWrite(NMOD_OUT,  LOW);                                   //normally low


  //ADC multiplexer settings:
  //  Output of ADC multiplexer is routed to inverting pin of analogue comparator when ACME=1, ADEN=0. Fig. 23-1 (p293) of Atmel-7766 datasheet. 
  //  MUX2..0 = 111 selects ADC7 = PF7
  ADMUX |=  (1 << MUX2);    
  ADMUX |=  (1 << MUX1);
  ADMUX |=  (1 << MUX0);

  //Route ADC mutliplexer output to inverting pin of analogue comparator. Fig. 23-1 (p293) and Sec 23.2 (p295) of Atmel-7766 datasheet.
  ADCSRA &= ~(1 << ADEN);   // ADEN=0. 
  ADCSRB |= (1 << ACME);    // ACME=1. 

  //Analog comparator: (see Section 23.1.2 p.294 of Atmel datasheet)
  //  Nb. BUG... ACIE=1 during startup could cause problems if trigger event occurs during this time! 
  ACSR =
    (0 << ACD)  |                   // Analog Comparator Disable: Enabled (0). Nb, When changing ACD, interrupts must be disabled (ie. ACIE=0)
    (0 << ACBG) |                   // Analog Comparator Bandgap Select: AIN0 to noninverting comparator input (0) 
    (0 << ACO)  |                   // Analog Comparator Output: Off (0). Would this be useful for couting trigger frequency???
    (1 << ACI)  |                   // Analog Comparator Interrupt Flag: Clear Pending Interrupt (1)
    (1 << ACIE) |                   // Analog Comparator Interrupt: Enabled (1)
    (0 << ACIC) |                   // Analog Comparator Input Capture: Disabled (0). Would this be useful for couting trigger frequency???
    (0 << ACIS1) | (0 << ACIS0);    // Analog Comparator Interrupt Mode: on Output Toggle (00). 00=Toggle, 10=Falling Edge, 11=Rising Edge. 

  //Set clock bits for Frequency calculator
//    TCCR0 = ((0 << CS02) | (0 << CS01) | (1 << CS00));
//  TCCR1A = 0;
//  TCCR1B = 0;
//  volatile int timer1_counter = 0;    // preload timer 65536-16MHz/256/2Hz?
//
//  TCNT1 = timer1_counter;   // preload timer
//  TCCR1B |= (1 << CS12);    // 256 prescaler
//  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt


  //Set PWM prescaler factor. 
  //  Want highest PWM carrier frequency to enable lowest choice of LFP corner freq. 
  //  Two modes for PWM, resulting in two PWM freqs: 1) fast PWM f_PWM = f_clk/(N*256) [page 100], 2) phase-correct PWM f_PWM = f_clk/(N*510) [page 102]
  //  N is the prescaler factor: 1(001), 8(010), 64(011), 256(100), 1024(110). 
  //  f_clk = 16 MHz on Teensy 2. 
  //  Bit 2:0 of Timer/Counter1 Control Register B (TCCR1B) sets prescaler factor. Table 14-5 (p134). 
  //
  TCCR1B = TCCR1B & 0b11111000 |0x01;     // sets bits2:0 to 001. Measured PWM freq = 3.92 kHz. 

  
  
  //Set up COM port
  Serial.begin(0);  // baud rate not used for USB. 
  while (!Serial);  // would never finish setup if serial port never opened. So do this last. 
  if (debug) {Serial.println("Teensy Trigger ver "+VERNUM);}
}


// Parser to interpret command from PC. 
//    Returns the command and any arguments to be processed in loop()
String  cmdDelim = " ";                           //define the delimiter
char    buf[maxCmdLen];                           //make an array to hold the characters in String Object
struct  cmdArg {                                  //make struct to hold the cmd and arg
  String cmd;
  String arg; 
};

struct cmdArg commandParser (String inStr) {
  //if (debug) {Serial.println(inStr);}               // echo if debug
  inStr.toCharArray(buf, sizeof(buf));              //convert String Object to String
  struct cmdArg ca;                                 
  ca.cmd = strtok(buf," ");                         //extracts the first word
  ca.arg = strtok(NULL," ");                        //extracts second word. NULL if no second word. 
  return ca;
}



// ---------  CORE FUNCTIONS:

// SET
int doSet (struct cmdArg cmdArg) {
  String STR_set_volts = cmdArg.arg;                          //requested set voltage
  
  // argument checks
  if (STR_set_volts == "") {                                  //check if no arg was send
    Serial.println("-20");                                    //report error to PC
    return -20;                                               //force escape from this function
  }
  else if (STR_set_volts == "?") {
    ADCSRA |= (1 << ADEN);                                    //Enable ADC (1); needed to perform analog read
    int PWM_loopback = analogRead(PD_IN);
    ADCSRA &= ~(1 << ADEN);                                   //Disable ADC to allow ADC MUX to route PD_IN to analog comparator
    ADCSRB |= (1 << ACME);                                    //Analog Comparator Multiplexer Enable: Route analog input to comparator (1)
    Serial.println(String(PWM_loopback/vtb_ADC));
    return 0;
  }
  float set_volts = STR_set_volts.toFloat();                  //cast to float

  if (set_volts < 0 or set_volts > 5) {
    Serial.println("-21");                                    //bad level requested
    return -20;
  }
  
  int PWM_val = (int) (set_volts * vtb_PWM);                  //convert into PWM level 0...255
  if (debug) {Serial.println("PWM_val = "+String(PWM_val));}
  analogWrite(PWM_OUT, PWM_val);                              //sends new threshold to LPFs
  
  // Read back set level for confirmation
  int levellow  = (set_volts - PWM_set_err)*vtb_ADC;          //set lower range in bit level
  int levelhigh = (set_volts + PWM_set_err)*vtb_ADC;          //set upper range in bit level
  ADCSRA |= (1 << ADEN);                                      //Enable ADC (1); needed to perform analog read
  float Set_time1=millis();                                   // set start time for set timeout clock
  
  float PWM_loopback = -1;                                    // init to invalid value since 0 would be a bad choice when user requests 0
  while (levellow > PWM_loopback or levelhigh < PWM_loopback or Set_timer>=Set_clock_out) // wait for LPFs to settle. tau_RC = 16 ms in Rev 1.1 of board. TODO: add a timeout
    {
    PWM_loopback = analogRead(PD_IN);                         //read input from rc circuit in bits
    float Set_time2=millis();                                 //check time to see if loop has clocked out
    float Set_timer=Set_time2-Set_time1;                            // how long set loop has been running
    }
  delay(tau_RC*1000);                                         //wait one more tau before taking final reading
  PWM_loopback = analogRead(PD_IN);
  ADCSRA &= ~(1 << ADEN);                                     //Disable ADC to allow ADC MUX to route PD_IN to analog comparator
  ADCSRB |= (1 << ACME);                                      //Analog Comparator Multiplexer Enable: Route analog input to comparator (1)
  Serial.println(String(PWM_loopback/vtb_ADC));               //Return measured threshold voltage
  return 0;
}


//ENABLE
int setInterrupt (int enab) {             
  if (enab == 0 && isEnab == true) {      //want to disable and is currently enabled
    ACSR &= ~(1 << ACIE);                 //ACIE=0;
    isEnab = !isEnab;                     //update 
    }
  if (enab == 1 && isEnab == false) {     //want to eanble and is currently disabled
    ACSR |= (1 << ACIE);                  //ACIE=1;
    isEnab = !isEnab;
    }
  if (debug) { Serial.println("isEnab: " + String(isEnab)); }
  return 0;
}

int doEnab (struct cmdArg cmdArg) {
  String STR_enab_arg = cmdArg.arg;       //get arg
  
  // argument checks
  if (STR_enab_arg == "") {               //check if no arg was send
    Serial.println("-20");                //report error to PC
    return -20;                           //force escape from this function
  }
  else if (STR_enab_arg == "?") {
    Serial.println(isEnab);
    return 0;
  }
  else if ( !(STR_enab_arg == "0" || STR_enab_arg == "1" ) ) {
    Serial.println("-31");                //bad arg
    return -31;
  }
  
  int enab_arg = STR_enab_arg.toInt();    //convert to int
  setInterrupt(enab_arg);                 //??? dont enable interrupts yet because COM port might still need to be used
  Serial.println("0");                    //success
  return 0;
}


// GAIN
int doGain (struct cmdArg cmdArg) {
  String STR_gain_level = cmdArg.arg;  

  // argument checks
  if (STR_gain_level == "") {                                 //check if no arg was send
    Serial.println("-20");                                    //report error to PC
    return -20;                                               //force escape from this function
  }
  else if (STR_gain_level == "?") {                           // query current gain level
    int G_A0 = digitalRead(GAIN_A0);
    int G_A1 = digitalRead(GAIN_A1);
    Serial.println(String(G_A1)+String(G_A0));
    return 0;
  }

  //set gain word
  if (STR_gain_level == "1")        //00
    { digitalWrite(GAIN_A1, LOW);   digitalWrite(GAIN_A0, LOW); }
  else if (STR_gain_level == "2")   //01
    { digitalWrite(GAIN_A1, LOW);  digitalWrite(GAIN_A0, HIGH); }
  else if (STR_gain_level == "4")   //10
    { digitalWrite(GAIN_A1, HIGH);   digitalWrite(GAIN_A0, LOW); } 
  else if (STR_gain_level == "8")   //11
    { digitalWrite(GAIN_A1, HIGH);  digitalWrite(GAIN_A0, HIGH); }         
  else
    { Serial.println("-41"); return -41;}   // invalid gain argument

  Serial.println("0");
  return 0;
}

// Report Frequency of Interupts
int doFreq (struct cmdArg cmdArg) {
  String STR_Freq_level = cmdArg.arg;                                 
  
  // argument checks
  if (STR_Freq_level == "") {                                  //check if no arg was send
    Serial.println("-20");                                    //report error to PC
    return -20;                                               //force escape from this function
  }
    else if (STR_Freq_level.equalsIgnoreCase("S")) {         //start freq measurement
    TimeS=millis();                                          //set start time
    Numb_of_Interupts=0;                                     //reset interupt counts
    Serial.println("0");                                     //let python know start was successful
    return 0;                                               
    }
    
  else if (STR_Freq_level.equalsIgnoreCase("M")) {           //end and record freq measurement
    unsigned long TimeM=millis();                           //set end time
    float TimeDiff = (float)(TimeM-TimeS);                  //calculate difference between start and finish time, allow for fractions of seconds
    float TimeF= TimeDiff/mStS;                             // calculate diff in seconds as float to allow accuracy to parts of a second
    if (TimeF <= 0)                                         //check if time is possible/correct and avoid divide by zero error
      {Serial.println(-51);}                                //let user know of error
    else {

        Serial.println(Numb_of_Interupts/ TimeF);}          //report frequency to user
    
    return 0;
  }
}

// Pulsewidth
int doPuls (struct cmdArg cmdArg) {
  String STR_Puls_Width = cmdArg.arg;                          //requested Pulsewidth
  
  // argument checks
  if (STR_Puls_Width == "") {                                  //check if no arg was send
    Serial.println("-20");                                    //report error to PC
    return -20;                                               //force escape from this function
  }
  else if (STR_Puls_Width == "?") {

    Serial.println(String(pulsewidth));                        // request pulse width value
    return 0;
  }
  int Puls_Width = STR_Puls_Width.toInt();                   // convert from string to int
  pulsewidth=Puls_Width;                                     // set pulsewidth and therefore also change debounce deay (=3x pulse width)
  Serial.print(pulsewidth);
  return 0;
}


// ---------  End of CORE FUNCTIONS:




void loop ()
{ 
  // Idle state: Loops forever doing nothing. Interrupts are enabled.
  
  
  
  if (Serial.available() > 0)                         //check if the comport has a command on it
  {
    ACSR &= ~(1 << ACIE);                             //ACIE=0; interupt state off

    //Serial.setTimeout(5000);                          // set time out for serial read so the program doesn't hang
    String inputStr = Serial.readStringUntil(eom);    //read string until carriage return. PC must send all commands with a CR eom. 
    if (debug) {Serial.println(inputStr);}            //echo for debug
    struct cmdArg thisCmdArg;                         //make a cmdArg struct to hold this cmd and arg
    thisCmdArg = commandParser(inputStr);             //split up the input string

    String thisCmd = thisCmdArg.cmd;
    if (thisCmd.equalsIgnoreCase("SET")) {
      doSet(thisCmdArg);
    }
    else if (thisCmd.equalsIgnoreCase("ENAB")) {
      doEnab(thisCmdArg);
    }
    else if (thisCmd.equalsIgnoreCase("GAIN")) {
      doGain(thisCmdArg);
    }
    else if (thisCmd.equalsIgnoreCase("FREQ")) {
      doFreq(thisCmdArg);
    }
    else if (thisCmd.equalsIgnoreCase("Puls")) {
      doPuls(thisCmdArg);
    }
    else {
      Serial.println("-1");                              //bad command received. Ignore. 
    }
    
    

   }
  
  //set the Interupt_state so it matches isEnab (what the user wants).
  if (isEnab==true){
    ACSR |= (1 << ACIE);                //interupt state on
  }
  if (isEnab == false) {             
    ACSR &= ~(1 << ACIE);               //interupt state off
  }

}


