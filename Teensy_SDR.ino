/* simple software define radio using the Softrock transceiver
   the Teensy audio shield is used to capture and generate 16 bit audio
   audio processing is done by the Teensy 3.1
   simple UI runs on a 160x120 color TFT display - AdaFruit or Banggood knockoff which has a different LCD controller
   Copyright (C) 2014, 2015  Rich Heslip rheslip@hotmail.com
   History:
   4/14 initial version by R Heslip VE3MKC
   6/14 Loftur E. Jónasson TF3LJ/VE2LJX - filter improvements, inclusion of Metro, software AGC module, optimized audio processing, UI changes
   1/15 RH - added encoder and SI5351 tuning library by Jason Milldrum <milldrum@gmail.com>
      - added HW AGC option which uses codec AGC module
      - added experimental waterfall display for CW
   3/15 RH - updated code to Teensyduino 1.21 and audio lib 1.02
      - added a lot of #defines to neaten up the code
      - added another summer at output - switches audio routing at runtime, provides a nice way to adjust I/Q balance and do AGC/ALC
      - added CW I/Q oscillators for CW transmit mode
      - added SSB and CW transmit
      - restructured the code to facilitate TX/RX switching
   2016 Gerrit Polder, PA3BYA - adapted to the following hardware:
      - Teensy 3.2 + Audio Shield
      - SoftRock RXTX 6.3 using Si570
      - 9V1AL Motherboard 3.6
      - 100 pulses/rev rotary encoder for VFO
      - cheap (24?) pulses/rev rotary encoder + switch for menu selection
   12/16 GP
      - added menu item for switching bandfilters 9V1AL motherboard
      - added automatic band switching
      - cleanup - removed band and fast tune switch
      - cleanup - removed SI5351
   Todo:
      add all amateur bands

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Teensy_SDR.h"
#include <Metro.h>
#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include <Encoder.h>
#include <Bounce2.h>
#include <Adafruit_GFX.h>   // LCD Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library

#include <SPI.h>
#include "filters.h"
#include "display.h"
#include <SerialFlash.h>

//#define  DEBUG
//#define DEBUG_TIMING  // for timing execution - monitor HW pin



#include "Si570.h"
#define SI570_I2C_ADDRESS 0x55
#define SI570_CALIBRATION 42800000

#define SEL0_PIN 4  // bandfilter selection pins
#define SEL1_PIN 8


extern void agc(void);      // RX agc function
extern void setup_display(void);
extern void intro_display(void);
extern void main_display(void);
extern void show_spectrum(void);  // spectrum display draw
extern void show_Smeter(void); // Smeter display
extern void show_waterfall(void); // waterfall display
extern void show_bandwidth(int filterwidth);  // show filter bandwidth
extern void show_radiomode(String mode, boolean highlight);  // show filter bandwidth
extern void show_band(String bandname, boolean highlight); // show band
extern void show_bandfilter(String bandname, boolean highlight); // show band
extern void show_frequency(long freq, boolean highlight);  // show frequency


// SW configuration defines
// don't use more than one AGC!
//#define SW_AGC   // define for Loftur's SW AGC - this has to be tuned carefully for your particular implementation
// codec hardware AGC works but it looks at the entire input bandwidth
// ie codec HW AGC works on the strongest signal, not necessarily what you are listening to
// it should work well for ALC (mic input) though
#define HW_AGC // define for codec AGC 
//#define CW_WATERFALL // define for experimental CW waterfall - needs faster update rate
#define AUDIO_STATS    // shows audio library CPU utilization etc on serial console

// band selection stuff
struct band {
  long freq;
  String name;
};

#define SOFTROCK_40M

#ifdef SOFTROCK_40M

#define BAND_80M  0   // these can be in any order but indexes need to be sequential for band switching code
#define BAND_60M  1
#define BAND_49M  2
#define BAND_40M  3
#define BAND_31M  4
#define BAND_30M  5

#define FIRST_BAND BAND_80M
#define LAST_BAND  BAND_30M
#define NUM_BANDS  LAST_BAND - FIRST_BAND + 1

struct band bands[NUM_BANDS] = {
  3580000, "80M",
  5000000, "60M",
  6000000, "49M",
  7040000, "40M",
  9500000, "31M",
  10115000, "30M"
};
#define STARTUP_BAND BAND_40M    // 
#endif

//Function options
#define FUNCTION_MODE 0
#define FUNCTION_BAND 1
#define FUNCTION_STEP 2
#define FUNCTION_BANDFILTER 3
#define FIRST_FUNCTION 0
#define LAST_FUNCTION 3


//SPI connections for Adafruit ST7735 1.8" display
const int8_t sclk   = 14;
const int8_t mosi   = 7;
const int8_t cs     = 2;
const int8_t dc     = 3;
const int8_t rst    = 1;

Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, mosi, sclk, rst); // soft SPI

#define  BACKLIGHT  0  // backlight control signal

// UI switch definitions
// encoder for tuning
Encoder tune(16, 17);
#define TUNE_STEP       1    // slow tuning rate 1hz steps for 100 pulses/ref encoder
#define FAST_TUNE_STEP   10000   // fast tuning rate 10khz steps

// encoder for functions
//Encoder encoder_functions(20,21);
#define encoderPinA  20
#define encoderPinB  21
//#define buttonPin 5
// Instantiate three Bounce objects for all pins with digitalRead
Bounce debouncerA = Bounce();
Bounce debouncerB = Bounce();
//Bounce debouncer5 = Bounce();

int lastEncoded = 0;
int encoderValue = 0;
int lastencoderValue = 0;


// Switches between pin and ground for selecting function mode
const int8_t FunctionSW = 12;   // Select function for encoder 2
// SEL0 and SEL1 control lines for the 9V1AL motherboard band switching
const int8_t SEL0 = SEL0_PIN;   // - SEL0 - Teensy pin SEL0_PIN -> 9V1AL P7 pin 1
const int8_t SEL1 = SEL1_PIN;   // - SEL1 - Teensy pin SEL1_PIN -> 9V1AL P7 pin 2

// see define above - pin 4 used for SW execution timing & debugging
#ifdef  DEBUG_TIMING
#define DEBUG_PIN   4
#endif
const int8_t PTTSW = 6;    // also used as SDCS on the audio card - can't use an SD card!
const int8_t PTTout = 5;    // PTT signal to softrock

Bounce  PTT_in = Bounce();   // debouncer

// in receive mode we use an audio IF to avoid the noise, offset and hum below ~ 1khz
#define IF_FREQ 11000       // IF Oscillator frequency
#define CW_FREQ 700        // audio tone frequency used for CW


// clock generator SI570
Si570 *vfo;

#define MASTER_CLK_MULT  4  // softrock requires 4x clock

// various timers
Metro five_sec = Metro(5000); // Set up a 5 second Metro
Metro ms_100 = Metro(100);  // Set up a 100ms Metro
Metro ms_50 = Metro(50);  // Set up a 50ms Metro for polling switches
Metro lcd_upd = Metro(10); // Set up a Metro for LCD updates
Metro CW_sample = Metro(10); // Set up a Metro for LCD updates
Metro Smetertimer = Metro(50); // Smeter timer

#ifdef CW_WATERFALL
Metro waterfall_upd = Metro(25); // Set up a Metro for waterfall updates
#endif

// mode selection stuff
struct modestruct {
  int mode;
  int bandwidth;
  String name;
};

// radio operation mode defines used for filter selections etc
#define  SSB_USB  0
#define  SSB_LSB  1
#define  CW       2
#define  CWR      3
#define NUM_MODES  CWR - SSB_USB + 1

struct modestruct modes[NUM_MODES] = {
  SSB_USB, USB_WIDE, "USB",
  SSB_LSB, LSB_WIDE, "LSB",
  CW, USB_NARROW, "CW",
  CWR, LSB_NARROW, "CWR",
};

// bandfilter selection stuff
#define FIRST_BANDFILTER 0
#define LAST_BANDFILTER 3
#define NUM_BANDFILTERS LAST_BANDFILTER - FIRST_BANDFILTER +1

struct bandfilterstruct {
  int filternum;
  long lowcutoff;
  long highcutoff;
  String name;
};

struct bandfilterstruct bandfilters[NUM_BANDFILTERS] = {
  0,        0,  2500000, "B0",
  1,  2500001,  9000000, "B1",
  2,  9000001, 20000000, "B2",
  3, 20000001, 40000000, "B3",
};

// audio definitions
//  RX & TX audio input definitions
const int inputTX = AUDIO_INPUT_MIC;
const int inputRX = AUDIO_INPUT_LINEIN;

#define  INITIAL_VOLUME 0.5   // 0-1.0 output volume on startup
#define  CW_SIDETONE_VOLUME    0.25    // 0-1.0 level for CW TX mode
#define  SSB_SIDETONE_VOLUME    0.8   // 0-1.0 adjust to hear SSB TX audio thru the headphones
#define  MIC_GAIN      30      // mic gain in db 
#define  RX_LEVEL_I    1.0    // 0-1.0 adjust for RX I/Q balance
#define  RX_LEVEL_Q    1.0    // 0-1.0 adjust for RX I/Q balance
#define  SSB_TX_LEVEL_I    1.0   // 0-1.0 adjust for SSB TX I/Q balance
#define  SSB_TX_LEVEL_Q    0.956  // 0-1.0 adjust for SSB TX I/Q balance
#define  CW_TX_LEVEL_I    1.0   // 0-1.0 adjust for CW TX I/Q balance
#define  CW_TX_LEVEL_Q    0.956 // 0-1.0 adjust for CW TX I/Q balance

// channel assignments for output Audioselectors
//
#define ROUTE_RX 	    0    // used for all recieve modes
#define ROUTE_SSB_TX 	    1    // SSB modes
#define ROUTE_CW_TX 	    2    // CW modes

// arrays used to set output audio routing and levels
// 3 radio modes x 4 channels for each audioselector
float audiolevels_I[3][4] = {
  RX_LEVEL_I, 0, 0, 0,     // RX mode channel levels
  0, SSB_TX_LEVEL_I, 0, 0, // SSB TX mode channel levels
  0, 0, CW_TX_LEVEL_I, 0 //CW  TX mode channel levels
};

float audiolevels_Q[3][4] = {
  RX_LEVEL_Q, 0, 0, 0,     // RX mode channel levels
  0, SSB_TX_LEVEL_Q, 0, 0, // SSB TX mode channel levels
  0, 0, CW_TX_LEVEL_Q, 0 //CW  TX mode channel levels
};

// Create the Audio components.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
//
AudioInputI2S       audioinput;           // Audio Shield: mic or line-in

// FIR filters
AudioFilterFIR      Hilbert45_I;
AudioFilterFIR      Hilbert45_Q;
AudioFilterFIR      FIR_BPF;
AudioFilterFIR      postFIR;

AudioMixer4         Summer;        // Summer (add inputs)
AudioAnalyzeFFT256  myFFT;      // Spectrum Display
AudioSynthWaveform  IF_osc;           // Local Oscillator
AudioSynthWaveform  CW_tone_I;           // Oscillator for CW tone I (sine)
AudioSynthWaveform  CW_tone_Q;           // Oscillator for CW tone Q (cosine)
AudioEffectMultiply    Mixer;         // Mixer (multiply inputs)
AudioAnalyzePeak     Smeter;        // Measure Audio Peak for S meter
AudioMixer4         Audioselector_I;   // Summer used for AGC and audio switch
AudioMixer4         Audioselector_Q;   // Summer used as audio selector
AudioAnalyzePeak    AGCpeak;       // Measure Audio Peak for AGC use
AudioOutputI2S      audioOutput;   // Audio Shield: headphones & line-out

AudioControlSGTL5000 audioShield;  // Create an object to control the audio shield.

//---------------------------------------------------------------------------------------------------------
// Create Audio connections to build a software defined Radio Receiver
//
AudioConnection c1(audioinput, 0, Hilbert45_I, 0);// Audio inputs to +/- 45 degree filters
AudioConnection c2(audioinput, 1, Hilbert45_Q, 0);
AudioConnection c3(Hilbert45_I, 0, Summer, 0);    // Sum the shifted filter outputs to supress the image
AudioConnection c4(Hilbert45_Q, 0, Summer, 1);
//
AudioConnection c10(Summer, 0, myFFT, 0);         // FFT for spectrum display
AudioConnection c11(Summer, 0, FIR_BPF, 0);       // 2.4 kHz USB or LSB filter centred at either 12.5 or 9.5 kHz
//                                                // ( local oscillator zero beat is at 11 kHz, see NCO )
AudioConnection c12(FIR_BPF, 0, Mixer, 0);        // IF from BPF to Mixer
AudioConnection c13(IF_osc, 0, Mixer, 1);            // Local Oscillator to Mixer (11 kHz)
//
AudioConnection c20(Mixer, 0, postFIR, 0);        // 2700Hz Low Pass filter or 200 Hz wide CW filter at 700Hz on audio output
AudioConnection c30(postFIR, 0, Smeter, 0);       // RX signal S-Meter measurement point
//
// RX is mono output , but for TX we need I and Q audio channel output
// two summers (I and Q) on the output used to select different audio paths for different RX and TX modes
//
AudioConnection c31(postFIR, 0, Audioselector_I, ROUTE_RX);   // mono RX audio and AGC Gain loop adjust
AudioConnection c32(postFIR, 0, Audioselector_Q, ROUTE_RX);   // mono RX audio to 2nd channel
AudioConnection c33(Hilbert45_I, 0, Audioselector_I, ROUTE_SSB_TX);   // SSB TX I audio and ALC Gain loop adjust
AudioConnection c34(Hilbert45_Q, 0, Audioselector_Q, ROUTE_SSB_TX);   // SSB TX Q audio and ALC Gain loop adjust
AudioConnection c35(CW_tone_I, 0, Audioselector_I, ROUTE_CW_TX);   // CW TX I audio
AudioConnection c36(CW_tone_Q, 0, Audioselector_Q, ROUTE_CW_TX);   // CW TX Q audio
// note that last mixer input is unused - PSK mode ???
//
AudioConnection c40(Audioselector_I, 0, AGCpeak, 0);          // AGC Gain loop measure
AudioConnection c41(Audioselector_I, 0, audioOutput, 0);      // Output the sum on both channels
AudioConnection c42(Audioselector_Q, 0, audioOutput, 1);
//---------------------------------------------------------------------------------------------------------


void setup()
{
  Serial.begin(9600); // debug console
#ifdef DEBUG
  while (!Serial) ; // wait for connection
  Serial.println("initializing");
#endif
  pinMode(BACKLIGHT, INPUT_PULLUP); // yanks up display BackLight signal
  pinMode(FunctionSW, INPUT_PULLUP);  // USB/LSB switch
  pinMode(PTTSW, INPUT_PULLUP);  // PTT input
  pinMode(PTTout, OUTPUT);  // PTT output to softrock
  digitalWrite(PTTout, 0); // turn off TX mode
  PTT_in.attach(PTTSW);    // PPT switch debouncer
  PTT_in.interval(5);  // 5ms

  // function encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  //pinMode(buttonPin, INPUT_PULLUP);
  debouncerA.attach(encoderPinA);
  debouncerA.interval(5);
  debouncerB.attach(encoderPinB);
  debouncerB.interval(5);
  //debouncer5.attach(buttonPin);
  //debouncer5.interval(5);
  //get starting position
  debouncerA.update();
  debouncerB.update();

  int lastMSB = debouncerA.read();
  int lastLSB = debouncerB.read();
#ifdef DEBUG
  Serial.print("Starting Position AB  " );
  Serial.print(lastMSB);
  Serial.println(lastLSB);
#endif

  //let start be lastEncoded so will index on first click
  lastEncoded = (lastMSB << 1) | lastLSB;

  pinMode(SEL1_PIN, OUTPUT); 	// set SEL1_PIN and SEL1_PIN as ouputs
  pinMode(SEL0_PIN, OUTPUT);

  digitalWrite(SEL0_PIN, HIGH); // selects filter for default 40M band
  digitalWrite(SEL1_PIN, LOW);

#ifdef  DEBUG_TIMING
  pinMode(DEBUG_PIN, OUTPUT);  // for execution time monitoring with a scope
#endif

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(16);

  // Enable the audio shield and set the output volume.
  audioShield.enable();
  audioShield.volume(INITIAL_VOLUME);
  audioShield.unmuteLineout();
#ifdef DEBUG
  Serial.println("audio shield enabled");
#endif

#ifdef HW_AGC
  /* COMMENTS FROM Teensy Audio library:
    Valid values for dap_avc parameters
    maxGain; Maximum gain that can be applied
    0 - 0 dB
    1 - 6.0 dB
    2 - 12 dB
    lbiResponse; Integrator Response
    0 - 0 mS
    1 - 25 mS
    2 - 50 mS
    3 - 100 mS
    hardLimit
    0 - Hard limit disabled. AVC Compressor/Expander enabled.
    1 - Hard limit enabled. The signal is limited to the programmed threshold (signal saturates at the threshold)
    threshold
    floating point in range 0 to -96 dB
    attack
    floating point figure is dB/s rate at which gain is increased
    decay
    floating point figure is dB/s rate at which gain is reduced
  */
  audioShield.autoVolumeControl(2, 1, 0, -30, 3, 20); // see comments above
  audioShield.autoVolumeEnable();

#endif

  // initialize the TFT and show signon message etc
  SPI.setMOSI(7); // set up HW SPI for use with the audio card - alternate pins
  SPI.setSCK(14);
  setup_display();
  intro_display();
  delay(2000);
  main_display();

  // set up initial band and frequency
  show_band(bands[STARTUP_BAND].name, false);
  show_bandfilter(bandfilters[0].name, false);

  vfo = new Si570(SI570_I2C_ADDRESS, SI570_CALIBRATION);

  if (vfo->status == SI570_ERROR) {
    // The Si570 is unreachable. Show an error for 3 seconds and continue.
    Serial.print("I2C Si570 error.");
    delay(10000);
  }
  vfo->setFrequency((unsigned long)bands[STARTUP_BAND].freq * MASTER_CLK_MULT);

  delay(3);

  setup_RX(SSB_LSB);  // set up the audio chain for LSB reception

#ifdef DEBUG
  Serial.println("audio RX path initialized");
#endif

}


void loop()
{
  static uint8_t functionsw_state = 0;
  static int mode = SSB_LSB, stepswitch, function = FUNCTION_MODE;
  static int band = STARTUP_BAND, Bandsw_state = 0;
  static uint8_t bandfilter = 0;
  static long encoder_pos = 0, last_encoder_pos = 999;
  static long function_encoder_pos = 0, function_last_encoder_pos = 999;
  long encoder_change, function_encoder_change;

  // tune radio using encoder switch
  encoder_pos = tune.read();
  if (encoder_pos != last_encoder_pos) {
    encoder_change = encoder_pos - last_encoder_pos;
    last_encoder_pos = encoder_pos;
    // press encoder button for fast tuning
    bands[band].freq += encoder_change * TUNE_STEP; // tune the master vfo - normal steps
    vfo->setFrequency((unsigned long)bands[band].freq * MASTER_CLK_MULT);
    bandfilter = setband(bands[band].freq + IF_FREQ);         // check if we need to change the filter
    show_bandfilter(bandfilters[bandfilter].name, function == FUNCTION_BANDFILTER); // show new band
    show_frequency(bands[band].freq + IF_FREQ, function == FUNCTION_STEP);  // frequency we are listening to
  }


  // function encoder switch handling
  stepswitch = functionencoder();
  if (stepswitch != 0) {
    switch (function) {
      case FUNCTION_MODE:
        mode = mode + stepswitch;
        if (mode > CWR) mode = SSB_USB; // cycle thru radio modes
        if (mode < SSB_USB) mode = CWR;
        setup_RX(mode);  // set up the audio chain for new mode
        break;
      case FUNCTION_BAND:
        band = band - stepswitch;
        if (band > LAST_BAND) band = LAST_BAND; // cycle thru radio bands
        if (band < FIRST_BAND) band = FIRST_BAND; // cycle thru radio bands
        show_band(bands[band].name, function == FUNCTION_BAND); // show new band

        vfo->setFrequency((unsigned long)bands[band].freq * MASTER_CLK_MULT);
        bandfilter = setband(bands[band].freq + IF_FREQ);         // check if we need to change the filter
        show_bandfilter(bandfilters[bandfilter].name, function == FUNCTION_BANDFILTER); // show new band
        show_frequency(bands[band].freq + IF_FREQ, function == FUNCTION_STEP);  // frequency we are listening to
        break;
      case FUNCTION_STEP:
        bands[band].freq -= stepswitch * FAST_TUNE_STEP; // fast tuning steps
        vfo->setFrequency((unsigned long)bands[band].freq * MASTER_CLK_MULT);
        bandfilter = setband(bands[band].freq + IF_FREQ);         // check if we need to change the filter
        show_bandfilter(bandfilters[bandfilter].name, function == FUNCTION_BANDFILTER); // show new band
        show_frequency(bands[band].freq + IF_FREQ, function == FUNCTION_STEP);  // frequency we are listening to
        break;
      case FUNCTION_BANDFILTER:
        bandfilter = bandfilter - stepswitch;
        if (bandfilter > LAST_BANDFILTER) bandfilter = LAST_BANDFILTER; // cycle thru radio bands
        if (bandfilter < FIRST_BANDFILTER) bandfilter = FIRST_BANDFILTER; // cycle thru radio bands
        select_filter_band(bandfilters[bandfilter].filternum);
        show_bandfilter(bandfilters[bandfilter].name, function == FUNCTION_BANDFILTER); // show new band
        break;
    }
  }


  // every 50 ms, adjust the volume and check the switches
  if (ms_50.check() == 1) {
    float vol = analogRead(15);
    vol = sqrt(sqrt(sqrt(vol / 1023.0))); // compensate for log potmeter
    audioShield.volume(vol);

    if (!digitalRead(FunctionSW)) {
      if (functionsw_state == 0) { // switch was pressed - falling edge
        if (++function > LAST_FUNCTION) function = FIRST_FUNCTION; //cycle thru functions
        show_band(bands[band].name, function == FUNCTION_BAND); // show new band
        show_radiomode(modes[mode].name, function == FUNCTION_MODE);
        show_frequency(bands[band].freq + IF_FREQ, function == FUNCTION_STEP);  // frequency we are listening to
        show_bandfilter(bandfilters[bandfilter].name, function == FUNCTION_BANDFILTER);
        functionsw_state = 1; // flag switch is pressed
      }
    }
    else functionsw_state = 0; // flag switch not pressed
  }


  // TX logic
  // looks like the Teensy audio line outs have fixed levels
  // that allows us to set sidetone levels separately which is really nice
  // have to shift freq up by IF_FREQ on TX
  // keyclicks - could ramp waveform in software.

  PTT_in.update();  // check the PTT switch
  if (!PTT_in.read())  // PTT switch is active, go into transmit mode
  {
    tft.setTextColor(RED);
    tft.setCursor(75, 72);
    tft.print("TX");
    setup_TX(mode);  // set up the audio chain for transmit mode
    // in TX mode we don't use an IF so we have to shift the TX frequency up by the IF frequency
    delay(2); // short delay to allow things to settle
    digitalWrite(PTTout, 1); // transmitter on
    while ( !PTT_in.read()) { // wait for PTT release
      PTT_in.update();  // check the PTT switch
      if ((lcd_upd.check() == 1) && myFFT.available()) show_spectrum(); // only works in SSB mode
    }
    digitalWrite(PTTout, 0); // transmitter off
    // restore the master clock to the RX frequency

    setup_RX(mode);  // set up the audio chain for RX mode
    tft.fillRect(75, 72, 11, 10, BLACK);// erase text
  }

#ifdef SW_AGC
  agc();  // Automatic Gain Control function
#endif

  //
  // Draw Spectrum Display
  //
  //  if ((lcd_upd.check() == 1) && myFFT.available()) show_spectrum();
  if ((lcd_upd.check() == 1)) show_spectrum();
  if ((Smetertimer.check() == 1)) show_Smeter();

#ifdef DEBUG_TIMING
  if ((CW_sample.check() == 1)) {
    digitalWrite(DEBUG_PIN, 1); // for timing measurements
    delay(1);
    digitalWrite(DEBUG_PIN, 0); //
  }
#endif

#ifdef CW_WATERFALL
  if ((waterfall_upd.check() == 1) && myFFT.available()) show_waterfall();
#endif

#ifdef AUDIO_STATS
  //
  // DEBUG - Microcontroller Load Check
  //
  // Change this to if(1) to monitor load

  /*
    For PlaySynthMusic this produces:
    Proc = 20 (21),  Mem = 2 (8)
  */
  if (five_sec.check() == 1)
  {
    Serial.print("Proc = ");
    Serial.print(AudioProcessorUsage());
    Serial.print(" (");
    Serial.print(AudioProcessorUsageMax());
    Serial.print("),  Mem = ");
    Serial.print(AudioMemoryUsage());
    Serial.print(" (");
    Serial.print(AudioMemoryUsageMax());
    Serial.println(")");
  }
#endif
}

// function to set up audio routes for the four channels on the two output audio summers
// summers are used to change audio routing at runtime
//
void Audiochannelsetup(int route)
{
  for (int i = 0; i < 4 ; ++i) {
    Audioselector_I.gain(i, audiolevels_I[route][i]); // set gains on audioselector channels
    Audioselector_Q.gain(i, audiolevels_Q[route][i]);
  }
}

// set up radio for RX modes - USB, LSB etc
void setup_RX(int mode)
{
  AudioNoInterrupts();   // Disable Audio while reconfiguring filters

  audioShield.inputSelect(inputRX); // RX mode uses line ins
  Audiochannelsetup(ROUTE_RX);   // switch audio path to RX processing chain

  CW_tone_I.amplitude(0);  // turn off cw oscillators to reduce cpu use
  CW_tone_Q.amplitude(0);

  // set IF oscillator to 11 kHz for RX
  IF_osc.begin(1.0, IF_FREQ, TONE_TYPE_SINE);

  // Initialize the wideband +/-45 degree Hilbert filters
  Hilbert45_I.begin(RX_hilbertm45, HILBERT_COEFFS);
  Hilbert45_Q.begin(RX_hilbert45, HILBERT_COEFFS);

  if ((mode == SSB_LSB) || (mode == CWR))             // LSB modes
    FIR_BPF.begin(firbpf_lsb, BPF_COEFFS);      // 2.4kHz LSB filter
  else FIR_BPF.begin(firbpf_usb, BPF_COEFFS);      // 2.4kHz USB filter

  show_radiomode(modes[mode].name, true);
  show_bandwidth(modes[mode].bandwidth);

  switch (mode)	{
    case CWR:
      postFIR.begin(postfir_700, COEFF_700);    // 700 Hz LSB filter
      break;
    case SSB_LSB:
      postFIR.begin(postfir_lpf, COEFF_LPF);    // 2.4kHz LSB filter
      break;
    case CW:
      postFIR.begin(postfir_700, COEFF_700);    // 700 Hz LSB filter
      break;
    case SSB_USB:
      postFIR.begin(postfir_lpf, COEFF_LPF);    // 2.4kHz LSB filter
      break;
  }
  AudioInterrupts();
}

// set up radio for TX modes - USB, LSB etc
void setup_TX(int mode)
{
  AudioNoInterrupts();   // Disable Audio while reconfiguring filters

  FIR_BPF.end(); // turn off the BPF - IF filters are not used in TX mode
  postFIR.end();     // turn off 2.4kHz post filter

  switch (mode)	{
    case CW:
      CW_tone_I.begin(1.0, CW_FREQ, TONE_TYPE_SINE);
      CW_tone_Q.begin(1.0, CW_FREQ, TONE_TYPE_SINE);
      CW_tone_Q.phase(90);
      Audiochannelsetup(ROUTE_CW_TX);   // switch audio outs to CW I & Q
      audioShield.volume(CW_SIDETONE_VOLUME);   // fixed level for TX
      break;
    case CWR:
      CW_tone_I.begin(1.0, CW_FREQ, TONE_TYPE_SINE);
      CW_tone_Q.begin(1.0, CW_FREQ, TONE_TYPE_SINE);
      CW_tone_I.phase(90);
      Audiochannelsetup(ROUTE_CW_TX);   // switch audio outs to CW I & Q
      audioShield.volume(CW_SIDETONE_VOLUME);   // fixed level for TX
      break;
    case SSB_USB:
      // Initialize the +/-45 degree Hilbert filters
      Hilbert45_I.begin(TX_hilbert45, HILBERT_COEFFS);
      Hilbert45_Q.begin(TX_hilbertm45, HILBERT_COEFFS);
      Audiochannelsetup(ROUTE_SSB_TX);   // switch audio outs to TX Hilbert filters
      audioShield.inputSelect(inputTX); // SSB TX mode uses mic in
      audioShield.micGain(MIC_GAIN);  // have to adjust mic gain after selecting mic in
      audioShield.volume(SSB_SIDETONE_VOLUME);   // fixed level for TX
      break;
    case SSB_LSB:
      // Initialize the +/-45 degree Hilbert filters
      Hilbert45_I.begin(TX_hilbertm45, HILBERT_COEFFS); // swap filters for LSB mode
      Hilbert45_Q.begin(TX_hilbert45, HILBERT_COEFFS);
      Audiochannelsetup(ROUTE_SSB_TX);   // switch audio outs to TX Hilbert filters
      audioShield.inputSelect(inputTX); // SSB TX mode uses mic in
      audioShield.micGain(MIC_GAIN); // have to adjust mic gain after selecting mic in
      audioShield.volume(SSB_SIDETONE_VOLUME);   // fixed level for TX
      break;
  }
  AudioInterrupts();
}


uint8_t setband(unsigned long freq)
{
  for (int i = 0; i < NUM_BANDFILTERS; i++) {
    if ( freq >= bandfilters[i].lowcutoff && freq <=  bandfilters[i].highcutoff)
    {
      uint8_t bandfilter = bandfilters[i].filternum;
      select_filter_band(bandfilter);
      return bandfilter;
    }
  }
}

void select_filter_band(uint8_t band)
{
  digitalWrite(SEL0, bool(band & 1));
  digitalWrite(SEL1, bool(band & 2));
}


int functionencoder(void) {
  // function encoder
  debouncerA.update();
  debouncerB.update();
  int MSB = debouncerA.read();//MSB = most significant bit
  int LSB = debouncerB.read();//LSB = least significant bit
  int returnval = 0;
  int encoded = (MSB << 1) | LSB; //converting the 2 pin values to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  //test against quadrature patterns CW and CCW
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  lastEncoded = encoded; //store this value for next time

  if (encoderValue != lastencoderValue) {
    if ((encoderValue & 1) != 1) {
      if (encoderValue > lastencoderValue)
        returnval = 1;
      else
        returnval = -1;

#ifdef DEBUG
      Serial.print("Index:  ");
      Serial.print(encoderValue);
      Serial.print('\t');
      Serial.print("Return:  ");
      Serial.print(returnval);
      Serial.print('\t');

      Serial.print("Old-New AB Pattern:  ");

      for (int i = 3; i >= 0; i-- )
      {
        Serial.print((sum >> i) & 0X01);//shift and select first bit
      }

      Serial.println();
#endif

    }
    lastencoderValue = encoderValue;
  }

  return returnval;
}



