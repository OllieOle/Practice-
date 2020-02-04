//ON DIALOUT (3*), sometimes a spurious tone is generated after the confirmation tone; line offhook at that point FIX
//commands verified by keyboard, handset, remote phone
//to test OFF-HOOK fcns when disconnected from line (line power is removed): connect rly pin 11 to +5V, disconnect phone from line
//6Nov17 14:15 Cancel_Timer istalled
//2Nov17 10:30 s/w hook toggle installed;
//30Oct17 16:30 all ON-HOOK features working;  test when OFF-HOOK
// Arduino pin3 to decoder pin 14 'DV'; gives decoder interrupt; do not connect interrupt from GPIO63
// Arduino A4 and A5 are GPIO clock and data

#include "Wire.h"

const byte interruptPin_GPIO = 2;  //digital pin 2 on Arduino interrupts for all GPIO except 63;
const byte interruptPin_DECODER = 3; // Arduino interrupt digital pin for HT9170 decoder
const int GPIO63_Addr = 63; // address of GPIO63//<--NOT REQD ???
const int GPIO62_Addr = 62; // address of GPIO62
# define CLOCK_LINE_63   3 // GPIO63 P3 (pin  7) to HT9200 encoder CLK (pin 11)
# define DATA_LINE_63    2 // GPIO63 P2 (pin  6) to HT9200 encoder DATA (pin12)
# define CE_LINE_63      1 // GPIO63 P1 (pin  5) to HT9200 encoder CHIP ENABLE (pin  1)
# define HOOK_LINE_63    0 // GPIO63 P0 (pin  4) to hookdiode1
# define DTMF_OFF        24   // unused command turns encoder off
# define THERMO_LINE_62   4 // GPIO62 P4 (pin9) thermostat control
# define LIGHTS_62        3  //GPIO62 P3 (pin7) lights control
# define ALARM_LINE_62    6 // GPIO62 P6 (pin 6) alarm control

//Assigmnent of GPIO ports:
//               P7           P6      P5      P4      P3      P2     P1   P0
//GPIO63 P7-P0:  DCDR3,       DCDR2,  DCDR1,  DCDR0,  CLOCK,  DATA,  CE,  HOOK
//GPIO62 P7-P0:  HOOK TOGGLE, ALARM,  TBD,    THERMO, LIGHTS, TBD,   TBD, RING

volatile boolean KeyPressed_decoder = false; // true if new DTMF received on HT9170
volatile boolean KeyPressed_sensor = false; // true if new sensor input received on GPIO; not used this sketch

byte state;


char GPIO62StatusBits[9];// contains char rep'n of status bits of GPIO62

char PHONE_NMBR1[] =  " 6138006929"; // phone # to dial
//char PHONE_NMBR1[] =  " 6137331547"; // phone # to dial
char* OFF_ON_Tone[] = {"1", "0"}; // '0' sounds higher than '1'
char* IntroTone[] = {"01"}; // leads all o/p tones to introduce what follows
char* WarningTone[] = {"0"}; // timeout warning
char* OFF_ON_Char[] = {"NO", "YES", ""};
byte digit;
boolean toneOnly = true; // output tone code only, not a telephone number
int duration = 60; // time tone plays
int pause = 80;    // time between tones
int dialToneWait = 3000; // wait 'dialToneWait' ms after going off-hook, before toning number out
int waitForconnect = 5; // wait time after dialing complete
int RingCount = 0; // number of rings detected
const int RingCountTrigger = 2; // num times ring detected before starting timer to go off-hook
const int RingCountDelay = 2000; // ms wait time to go off-hook after 'RingCountTrigger' rings

int Px; // a GPIO port number
int GPIO;
byte GPIOmask;
byte GPIOstate;
byte GPIOmask62  = B00000000;
byte GPIOstate62 = B11111111;
byte GPIOmask63  = B11110000;
byte GPIOstate63 = B11110001;
byte _data; // byte read from GPIO
byte _data62; //data on GPIO62
byte _data63; //data on GPIO63

int TimeCount = 0;
int TimeSinceRestart;
int TimeOfRestart;
int TimeStretch = 0; // time delay before timer starts
int TimeOut = 60; // total time allowed (w/o TimeStretch) before ON-HOOK generated
int TimeWarningTime = 10; // time from warning to going ON-HOOK
boolean TimeWarningGiven = false;
boolean Timer_On = false;
boolean Cancel_Timer = false;
boolean IgnoreBeep  = false;

boolean SecurityOn = true; //<----TEST
boolean passWordValid = false;
boolean EchoCmd = true; // echo (tone out) command and status of command after Action
boolean HookState = 0;

#define numCodes (sizeof(CmdCode)/sizeof(char *)+1) //array size;
char* CmdCode[] = { "2*", "1*", "990*", "77*", "78*",
                    "71*", "72*", "3*", "4*",
                    "5*", "6*", "73*", "74*",
                    "11*", "007*", "008*", "70*",
                    "001*", "002*"
                  }; //  N.B. keep "password" in cell 13
char* CmdCodeName[] = {"ON-HOOK", "OFF-HOOK", "ring count", "Thermo on (LED on)", "Thermo off (LED off)",
                       "lights on (LED on)", "lights off (LED off)", "call out", "turn echo off",
                       "turn echo on", "status of all", "alarm on (LED on)", "alarm off (LED off)",
                       "password", "security ON", "security OFF", "ring detect",
                       "timer ON", "timer OFF"
                      }; // CmdCodeName[k] element posn 'k' serves as case number in 'Action(k)'
boolean CodeValid = false;
String HndsetEntry;
String KbrdEntry;
String ToneCmd = "";
char* ToneToAscii[] = {"x", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
char HiLo[] = "01";


void setup() {

  //setup Ard timer1 at 1Hz
  pinMode(13, OUTPUT); //Ard timer1 output
  cli();//stop interrupts
  //set timer1 interrupt
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);  // Set CS10 and CS12 bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  sei(); //allow interrupts

  Wire.begin();
  attachInterrupt(digitalPinToInterrupt(interruptPin_GPIO), Intrpt_Sensor_In, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptPin_DECODER), Intrpt_DTMF_In, RISING);
  Serial.begin(9600);
  WriteGPIO(62, GPIOstate62 ); //initialize GPIO62
  WriteGPIO(63, GPIOstate63 ); //initialize GPIO63
  Serial.print("\nCancel_Timer      ");
  Serial.println(OFF_ON_Char[Cancel_Timer]);
  Serial.print("Timer_On          ");
  Serial.println(OFF_ON_Char[Timer_On]);
  Serial.print("SecurityOn        ");
  Serial.println(OFF_ON_Char[SecurityOn]);
  if (SecurityOn == true) {
    Serial.println("\nINPUT PASSWORD");
    for (int i = 1; i < 5; i++) {
      Dialer(WarningTone[0], 100, 100, true);
    }
  }
}

//VVVVVVVVVVV   ISR   VVVVVVVVVVVVV
ISR(TIMER1_COMPA_vect) { //timer1 interrupt; increments 'TimeCount' @ 1Hz
  //generates 1Hz pulse train
  TimeCount += 1;
}


void loop() {  //##
  KeyPressed_decoder = false; // reset - no new DTMF input
  KeyPressed_sensor = false; // reset - no new sensor input
  while (Serial.available()) {  // 'available' if data in buffer
    char ch = Serial.read(); //clear serial buffer
  }

  // WAIT FOR COMMAND
  while (!Serial.available() && KeyPressed_decoder == false && KeyPressed_sensor == false) { //sit here; wait for activity
    if (Timer_On == true) { // when timer ON (==> r OFF-HOOK), check time while awaiting new activity
      Timer();
    }
  }

  //NEW ACTIVITY DETECTED (on keybrd, handset, line or sensor)%%%
  //        TimeWarning tone is not detected as new activity (tone ISR was detached/attached during warning tone)
  //        RESET OFF-HOOK TIMER - to ensure phone goes ON-HOOK after period of (line, handset, keyboard) inactivity
  //            Timer is ON when OFF-HOOK.  When OFF-HOOK, restart timer on new activity, TimeWarning tone does not restart;
  //            Go ON-HOOK if no activity within 'TimeOut'

  if (IgnoreBeep == false) { //TEST
    Serial.println("NEW ACTIVITY");
    TimeOfRestart = TimeCount;//external signal rec'd; reset OFF-HOOK timer
    TimeSinceRestart = 0;
    TimeWarningGiven = false;  // timer was reset in 'Timer()' and time warning removed
    if (Timer_On == true) Serial.print("\nTimer reset" );
  }
  else {
    IgnoreBeep = false;
  }

  //KEYBOARD - enter if keyboard entry rec'd; if command is valid, take Action in ChkCode(...)
  if (Serial.available() > 0) {
    KbrdEntry = Serial.readStringUntil('\n');
    ChkCode(KbrdEntry);// check for valid command
  }

  //TONE - enter if handset tone or line tone rec'd - take Action when valid command in ChkCode(...)
  if (KeyPressed_decoder == true) {
    ReadGPIO(63);//DTMF fcns are on GPIO63
    _data63 = (_data >> 4);// decoder O/P rec'd on GPIO63 P4 to P7; shift right '_data' 4 places
    ToneDigitDecode (_data63);//converts tone decoder o/p to string character & adds it to ToneCmd
    if (_data63 == 11) { // check for "*"
      ChkCode(ToneCmd);// check for valid command; takes Action(i) if valid;
      ToneCmd = "";// reset before return to 'WAIT FOR COMMAND' <---NB NB prob not neededed; this ret also in ChkCode
    }
  } // return to top of 'WAIT FOR COMMAND' loop
  KeyPressed_decoder = false;


  //SENSOR - enter after new sensor input rec'd on GPIO62
  if (KeyPressed_sensor == true) { //'true' if interrupt on GPIO62 ==> a port brought LOW
    detachInterrupt(digitalPinToInterrupt(interruptPin_GPIO));

  }
  //RING DETECT %%
  ReadGPIO(62);//ring detect and sensor fcns are on GPIO62
  _data62 = _data;

  if (bitRead(_data62, 0) == 0) {  // GPIO62 P0 == 0 ==> Ring detected
    Action(16); //go off-hook if number of rings > "RingCountTrigger"
  }

  // HOOK TOGGLE
  if (bitRead(_data, 7) == 0) {  // GPIO62 P7 == 0 or 1 ==> Hook Toggle detected
    HookState = 1 - HookState;
    Action(HookState); //invoke opposite hook state// 0==>is ON-HOOK, 1==> is OFF-HOOK
  }

  attachInterrupt(digitalPinToInterrupt(interruptPin_GPIO), Intrpt_Sensor_In, LOW);

}//loop return

//VVVVVVVVVVV   WriteGPIO   VVVVVVVVVVVVV
// writes 8 bits of byte '_data' to the 8 I/O lines of the PCA8574A GPIO 63
byte WriteGPIO(int GPIO, byte _data) {
  Wire.beginTransmission(GPIO);
  Wire.write(_data);
  Wire.endTransmission();
}

//VVVVVVVVVVV   ReadGPIO   VVVVVVVVVVVVV
// reads 8 bits of GPIO
byte ReadGPIO(int GPIO) {
  Wire.requestFrom(GPIO, 1);
  if (Wire.available()) {
    _data = Wire.read();
  }
  return _data;
}

//VVVVVVVVVV   SetOutputPin   VVVVVVVVVVVVVV
byte SetOutputPin(int GPIO, byte GPIOmask, byte GPIOstatex, int Px, byte state) { //set one output pin 'Px' to state 'state'//<----changed

  if (state) {
    bitSet(GPIOstatex, Px);
  }
  else {
    bitClear(GPIOstatex, Px);
  }
  GPIOstatex |= GPIOmask;// bitwise OR
  WriteGPIO(GPIO, GPIOstatex);
  if (GPIO == 62) {
    GPIOstate62 = GPIOstatex;
  }
  else {
    GPIOstate63 = GPIOstatex;
  }
}

//VVVVVVVVVVV   Intrpt_DTMF_In   VVVVVVVVVVVVV
void Intrpt_DTMF_In() {  // ISR called when DTMF decoder 'Data Valid' goes HIGH; Arduino interrupt digital pin 3
  KeyPressed_decoder = true;
}

//VVVVVVVVVVV   Intrpt_Sensor_In   VVVVVVVVVVVVV
void Intrpt_Sensor_In() { // ISR called with change on input port of GPIO62
  KeyPressed_sensor = true;
}

//VVVVVVVVVVV   DTMF_Out   VVVVVVVVVVVVV
// output one telephone number 'digit' from 'ToneNum';
//'duration' how long to output tone; 'pause' time between tones
// toggle in 4bits of 'digit' into ENCODER HT9200 data line (pin 12); LSB first;
// output data bits on GPIO P6 (pin 10)

void DTMF_Out (int GPIO, byte GPIOmask, byte digit, int duration, long pause) {
  // TURN TONE ON
  if (digit == 0) digit = 10; //'0' not an allowed HT9200 code; '10' is codts of digit e for 0
  for (int i = 0; i < 5; i++) { // xfer 1st 5 bits of digit to 'DATA_LINE_63', lsb first
    GPIOstate63 = ReadGPIO(63);
    SetOutputPin(63, GPIOmask63, GPIOstate63, CLOCK_LINE_63, HIGH); // disable encoder; set GPIO clock line P6 (pin 11)-->HT9200 CLK (pin 11) HIGH while setting data
    SetOutputPin(63, GPIOmask63, GPIOstate63, DATA_LINE_63, bitRead(digit, i)); //enter data bit onto GPIO data line P5(pin 10)-->HT9200 DATA (pin12); enter LSB of digit into HT9200 first
    SetOutputPin(63, GPIOmask63, GPIOstate63, CLOCK_LINE_63, LOW); // latch data bit; set GPIO clock line P6 (pin 11)-->HT9200 CLK (pin 11) LOW (play tone after 5 bits entered)
  }
  delay(duration);
  // TURN TONE OFF
  GPIOstate63 = ReadGPIO(63);
  if (pause != 0) { // tone sounds continuously if zero ==> exit function
    for (int i = 0; i < 5; i++) { //routine to RESET REGISTER;set DTMF OFF
      SetOutputPin(63, GPIOmask63, GPIOstate63, CLOCK_LINE_63, HIGH); // set GPIO clock line P6 HIGH while setting data
      SetOutputPin(63, GPIOmask63, GPIOstate63, DATA_LINE_63, bitRead(DTMF_OFF, i)); //P5(pin 10)-->HT9200 DATA (pin12)
      SetOutputPin(63, GPIOmask63, GPIOstate63, CLOCK_LINE_63, LOW); // set GPIO clock line P6 LOW to latch data (and turn off encoder after 5 bits entered)
    }
    delay(pause); // pause between tones
  }
}

//VVVVVVVVVVVV  Dialer  VVVVVVVVVVVV*5
// places DTMF digits online
// initializes DTMF encoder, takes phone off-hook if dialing reqd; dials a number
// cycles thru & dials each digit of ToneNum
// e.g. ToneNum[] = PHONE_NMBR1 = "6137331547"
// go ON-HOOK is handled by Timer()
void Dialer(char *Nmbr, int duration, int pause, boolean toneOnly) { // *Nmbr is an array (i.e. block of contiguous cells of char type such as PHONE_NMBR1)
  TimeStretch = 0;
  GPIO = 63;
  GPIOmask = GPIOmask63;
  if (toneOnly == false) { // 'false' ==> dial a telephone number; 'true' ==> dial a tone code
    Timer_On = true;
    TimeStretch = waitForconnect;
    Serial.println("Go off-hook");
    SetOutputPin(63, GPIOmask63, GPIOstate63, HOOK_LINE_63, LOW); //GO OFFHOOK before dialing (GPIO3 P3 pin 7 set LOW)
    Serial.println("Wait for dial tone");
    delay(dialToneWait); // wait for dial tone
    Serial.print("Dialing number  ");
  }
  byte indx = 0;
  while (Nmbr[indx] != '\0') { //'\0' if end of 'phone[indx]' string reached
    DTMF_Out(63, GPIOmask63, Nmbr[indx] - '0', duration, pause); // translates ASCII to HT9200 code;
    indx++;
  }
  Serial.println();
}

//VVVVVVVVVVVV  Action  VVVVVVVVVVVV
//take action determined by Command Code
void Action (int k) {
  int ii = k;
  //NB following conditionals reqd b.c. using a "case 3:" e.g.causes an 'OFF-HOOK' (????)

  // THERMO
  if (k == 3) {
    SetOutputPin(62, GPIOmask62, GPIOstate62, THERMO_LINE_62, LOW); // for '77*' //THERMO ON
    Serial.println("Thermostat On");
  }
  if (k == 4) {
    SetOutputPin(62, GPIOmask62, GPIOstate62, THERMO_LINE_62, HIGH);  // for '78*' //THERMO OFF
    Serial.println("Thermostat Off");
  }


  // LIGHTS
  if (k == 5) {
    SetOutputPin(62, GPIOmask62, GPIOstate62, LIGHTS_62, LOW); //LIGHTS ON
    Serial.println("Lights On");
  }
  if (k == 6) {
    SetOutputPin(62, GPIOmask62, GPIOstate62, LIGHTS_62, HIGH); //LIGHTS OFF
    Serial.println("Lights Off");
  }

  // ECHO
  if (k == 8 or k == 9) k = 8;     // combine 'echo on' and 'echo off' commands into case 8
  if (EchoCmd == 1) Dialer(CmdCode[ii], 200, 100, true); // echo back tones of command received

  // ALARM
  if (k == 11 or k == 12) k = 11;   // combine 'ALARM on' and 'ALARM off' commands into case 11

  // SECURITY
  if (k == 14 or k == 15) k = 14;

  // TIMER
  if (k == 17 or k == 18) k = 17;

  int pause;
  boolean toneOnly;
  char ToneNum[12];
  switch (k) {

    case 0: //go ON-HOOK    2*
      SetOutputPin(63, GPIOmask63, GPIOstate63, HOOK_LINE_63, HIGH);
      Timer_On = false;
      Serial.println("ON-HOOK");
      //passWordValid = false; // must request password for each OFF-HOOK // TEST
      break;

    case 1: // go OFF-HOOK    1*
      SetOutputPin(63, GPIOmask63, GPIOstate63, HOOK_LINE_63, LOW); //GO OFF-HOOK after "RingCountTrigger" rings plus "RingCountDelay"
      Timer_On = true;
      Serial.println("OFF-HOOK");
      TimeOfRestart  = TimeCount;
      break;


    case 7:  // DIAL A PHONE NUMBER    3*
      duration = 65;
      pause = 85;
      Timer_On = true;   //orig code:--> Timer_On = true;
      toneOnly = false;  // 'true' prevents fcn Dialer(...) from going ON-HOOK after tone out,<---TEST THIS
      delay(1000);
      Serial.print("Dial ");
      Serial.println(PHONE_NMBR1);
      Dialer(PHONE_NMBR1, duration, pause, toneOnly); // cycle through all digits of ToneNum and dial them
      Serial.println("Number dialed");
      //TimeOut = 120; //REWORK to allow more time for callout off-hook;
      TimeOfRestart  = TimeCount + TimeStretch;
      Serial.print("Timeout in  ");
      Serial.println(TimeStretch + TimeOut);
      break;


    case 8: // SET ECHO OFF   4*    5*
      if (ii == 8) {
        EchoCmd = 0; //
        Serial.println("EchoCmd is OFF");
      }
      else {
        EchoCmd = 1;
        Serial.println("EchoCmd is ON");
      }
      delay(500);
      //echo status always tones out, independent of echo state
      //Dialer(IntroTone[0], 100, 100, true); // leads all o/p tones to introduce what follows
      delay(250);
      Dialer(OFF_ON_Tone[EchoCmd], 1250, 100, true); // tone to indicate echo ON;  "OFF_ON_Tone[1]" == "00"
      break;

    case 10: // SEND 'STATUS OF ALL'    6*
      Serial.println("port7 to port0 on GPIO62");
      delay(500);
      Dialer(IntroTone[0], 100, 100, true); // precursor to all o/p tones to introduce what follows
      delay(500);
      ReadGPIO(62);
      //convert integer '_data' to array 'GPIO62StatusBits'
      for (int i = 0; i < 8; i++) { // tone out port7 to port0 on GPIO6
        GPIO62StatusBits[i] = HiLo[bitRead(_data, i)];
        Serial.print(HiLo[abs(1 - bitRead(_data, i))]);
        Serial.print("  ");
      }
      Serial.println();
      Dialer(GPIO62StatusBits, 1250, 100, true);
      //Serial.println();
      break;

    case 11: // ALARM ON/OFF"alarm on"/ "alarm off"    73* / 74*
      if (ii == 11) { //ALARM ON
        SetOutputPin(62, GPIOmask62, GPIOstate62, ALARM_LINE_62, LOW);
      }
      else { //ALARM OFF
        SetOutputPin(62, GPIOmask62, GPIOstate62, ALARM_LINE_62, HIGH);
      }
      Serial.print("Alarm is ");
      Serial.print(OFF_ON_Char[abs(ii - 12)]);
      Serial.println();
      break;

    case 13: // PASSWORD   11*
      passWordValid = true;
      break;

    case 14: // SECURITY ON/OFF  "SecurityOn" ON/OFF  007* / 008*
      if (ii == 14) {
        SecurityOn = true; // TURN SECURITY ON
        passWordValid = false; // will require re-entry of password
        Serial.println("Security ON");
      }
      else { // TURN SECURITY OFF
        SecurityOn = false;
        Serial.println("Security OFF");
      }
      break;

    case 16: // COUNTS RINGS and GO OFFHOOK // GPIO62 P0 to '0' on ring // '1' otherwise
      Serial.print("  RingCount = ");
      RingCount += 1; // number of rings detected
      Serial.println(RingCount);
      delay(RingCountDelay); // delay until after ringing stops
      if (RingCount >= RingCountTrigger) {
        RingCount = 0;
        Serial.println("Incoming Call");
        Serial.println("Go off-hook");
        delay(100);
        
        SetOutputPin(63, GPIOmask63, GPIOstate63, CE_LINE_63, HIGH);//disable encoder
        SetOutputPin(63, GPIOmask63, GPIOstate63, HOOK_LINE_63, LOW); //GO OFFHOOK after "RingCountTrigger" rings plus "RingCountDelay"
        SetOutputPin(63, GPIOmask63, GPIOstate63, CLOCK_LINE_63, LOW);//enable encoder
        SecurityOn = true; // require password access for incoming call
        passWordValid = false;
        Timer_On = true;
        TimeOfRestart  = TimeCount;
      }
      break;

    case 17: // TURN OFF-HOOK TIMER ON OR OFF  001* / 002*

      if (ii == 17) {
        Cancel_Timer = 0; //
        Serial.println("Timer is ON");
        Timer_On = true;
      }
      else {
        Cancel_Timer = 1;
        Serial.println("Timer is OFF");
        Timer_On = false;
      }
      break;

  }//end switch
  if (Cancel_Timer == true) Timer_On = false;//FOR TESTING; to inhibit off-hhok timeout
  Serial.println("\n**Enter command**");
}


//VVVVVVVVVVVV  ChkCode  VVVVVVVVVVVV%%%
//Command Code determines Action(i) to take
//void ChkCode(String C) {
void ChkCode(String C) {
  CodeValid = false;
  if (C.endsWith("*")) {// if not, allow more inputs until * entered
    if (C == CmdCode[13]) {  //check if valid password was entered
      passWordValid = true; // valid password entered
      Serial.println("Password Valid");
      Serial.print("Security is ");
      Serial.println(OFF_ON_Char[SecurityOn]);  // o/p security state
    }
    if (passWordValid == true) { // then check for valid command codes
      int i = 0;
      while (i < numCodes) {
        if (C == CmdCode[i]) {
          CodeValid = true;
          Action(i);//$
        }
        i += 1;
      }
      if (CodeValid == false) {
        Serial.print("code invalid  ");//invalid code entered
        //Serial.println(C);
        for (int i = 1; i < 3; i++) {
          Dialer("1", 1000, 100, true);
        }
      }

      ToneCmd = ""; // reset tone generated Command Code
    }

    else {  //passWordValid is 'false'
      Serial.println("PASSWORD REQUIRED");
      for (int i = 1; i < 5; i++) {
        Dialer(IntroTone[0], 100, 100, true);
      }
    }
  }
}


//VVVVVVVVVVVV  ToneDigitDecode  VVVVVVVVVVVV
//convert tone digit to string of key label
void ToneDigitDecode (byte x) {
  if (x == 10) { // tone "0"
    ToneCmd += "0";
  }
  else if (x == 11) { // tone "*"
    ToneCmd += "*";
  }
  else if (x == 12) { // tone "#"
    ToneCmd += "#";
  }
  else {
    for (int i = 0; i < 10; i++) {
      if (x == i) {
        ToneCmd += ToneToAscii[i];
      }
    }
  }
}

//VVVVVVVVVVVV  Timer  VVVVVVVVVVVV%%%
// ensures telephone line is not left OFF-HOOK;   called while waiting for new activity
// entered from 'WAIT FOR COMMAND' 'while' loop
//check for timeout hook-line reset;
//'Timer_On' is set 'true'/'false' after each 'SetOutputPin' sets 'OFF-HOOK / ON-HOOK'; *3
void Timer() {
  if (Timer_On == true) {
    TimeSinceRestart = TimeCount - TimeOfRestart;
    //GIVE TIMEOUT WARNING
    if (TimeSinceRestart >= TimeOut - TimeWarningTime && TimeWarningGiven == false) { // enter if time is past the warning time setpoint and no timeout warning has been given
      TimeWarningGiven = true; // ensures only one warning given
      for (int i = 1; i < 6; i++) { // give 4 short warning beeps
        Dialer(WarningTone[0], 100, 100, true);
      }
      IgnoreBeep  = true; //TEST
      Serial.print("\n******Timeout in****** ");
      Serial.println(TimeWarningTime);
      Serial.print("\n");
      delay(100);


    }
    //TIME EXPIRED
    if (TimeSinceRestart >= TimeOut) { // warning given, no time reset generated, and timeout reached;  ---> go ON-HOOK
      Serial.println("ON-HOOK\n");
      for (int i = 1; i < 3; i++) { // call terminated
        Dialer("1", 1000, 100, true);
      }
      SetOutputPin(63, GPIOmask63, GPIOstate63, HOOK_LINE_63, HIGH); //timed out ==> GO ONHOOK (GPIO3 P3 pin 7 set HIGH)
      TimeWarningGiven = false; // give only one warning
      Timer_On = false;
      ToneCmd = "";
      Serial.println("\n**Enter command**");
    }
  }
}


