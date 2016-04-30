
// code for the TFT display

#include "display.h"
#include <Audio.h>
#include <Adafruit_GFX.h>        // LCD Core graphics library
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Adafruit_ST7735.h>

#define pos_x_smeter 5
#define pos_y_smeter 80
#define s_w 10
#define pos_x_freq 30
#define pos_y_freq 104

//#define DEBUG_SMETER
#ifdef DEBUG_SMETER
#include <Metro.h>
extern Metro five_sec;
#endif

float uv, uvold, dbuv, dbm, s;// microvolts, db-microvolts, s-units, volts, dbm, sold = old S-meter value, deltas = abs difference between sold and s
float line_gain = 0; // possibly different for different bands, see software DD4DH

extern Adafruit_ST7735 tft;
extern AudioAnalyzeFFT256  myFFT;      // FFT for Spectrum Display
void show_s_meter_layout(void);
extern AudioAnalyzePeak       Smeter;   // Measure Audio Peak for S meter

void setup_display(void) {
  // initialize the LCD display
  tft.initR(INITR_BLACKTAB);   // initialize a S6D02A1S chip, black tab
  tft.setRotation(1);
  tft.fillScreen(BLACK);
}

void intro_display(void) {
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(WHITE);
  tft.setTextWrap(true);
  tft.setCursor(0,30);
  tft.print("Teensy SDR");
  tft.setFont(&FreeSans9pt7b);
  tft.setCursor(0,50);
  tft.print("by VE3MKC");
  tft.setCursor(0,80);
  tft.print("PA3BYA");
  tft.setCursor(0,100);
  tft.print("version: 1.0");
  tft.setCursor(0,120);
  tft.print("build: ");
  tft.print(__DATE__);  
}

void main_display(void) {
  tft.fillScreen(BLACK);
  tft.setFont(&FreeSans9pt7b);  
  // Show mid screen tune position
  tft.drawFastVLine(80, 0,60,RED);
  show_s_meter_layout();
}

// draw the spectrum display
// this version draws 1/10 of the spectrum per call but we run it 10x the speed
// this allows other stuff to run without blocking for so long

void show_spectrum(void) {
  static int startx=0, endx;
  endx=startx+16;
  int scale=1;
//digitalWrite(DEBUG_PIN,1); // for timing measurements
  
  for (int16_t x=startx; x < endx; x+=2) 
  {
    int bar=abs(myFFT.output[x*8/10])/scale;
    if (bar >60) bar=60;
    if(x!=80)
    {
       tft.drawFastVLine(x, 60-bar,bar, GREEN);
       tft.drawFastVLine(x, 0, 60-bar, BLACK);    
    }
  }
  startx+=16;
  if(startx >=160) startx=0;
//digitalWrite(DEBUG_PIN,0); // 
}

/* old draw routine
// draw the spectrum display

void show_spectrum(void) {

  int scale=1;
  for (int16_t x=0; x < 160; x+=2) 
  {
    int bar=abs(myFFT.output[x*8/10])/scale;
    if (bar >60) bar=60;
    if(x!=80)
    {
       tft.drawFastVLine(x, 60-bar,bar, GREEN);
       tft.drawFastVLine(x, 0, 60-bar, BLACK);    
    }
  }
}
*/

void show_Smeter(void) {
    float s_sample = 0;  // Raw signal strength (max per 1ms)
    // Collect S-meter data
    if (Smeter.available()) s_sample = Smeter.read(); // Highest sample within 1 millisecond
     // Calculate S units. 50uV = S9
      uv = (s_sample-0.005)*10000;  // microvolts, roughly calibrated
      uv = 0.1 * uv + 0.9 * uvold; // low pass filtering for Smeter values 
      dbuv = 20.0*log10(uv);
      s = 1.0 + (14.0 + dbuv)/6.0;

#ifdef DEBUG_SMETER
    if (five_sec.check() == 1)
    {
      Serial.print("s_sample = ");
      Serial.print(s_sample);
      Serial.print(" uv = ");    
      Serial.print(uv);
      Serial.print(" s =");
      Serial.print(s);
      Serial.print(" dbuv = ");    
      Serial.print(dbuv);
      Serial.println("");
    }
#endif  
      if (s <0.0) s=0.0;
      if (s>9.0)
      {
        dbuv = dbuv - 34.0;
        s = 9.0;
      }
      else dbuv = 0;
      uvold = uv;

      tft.drawFastHLine(pos_x_smeter, pos_y_smeter, s*s_w+1, YELLOW);
      tft.drawFastHLine(pos_x_smeter+s*s_w+1, pos_y_smeter, (9*s_w+1)-s*s_w+1, BLACK);

      tft.drawFastHLine(pos_x_smeter, pos_y_smeter+1, s*s_w+1, YELLOW);
      tft.drawFastHLine(pos_x_smeter+s*s_w+1, pos_y_smeter+1, (9*s_w+1)-s*s_w+1, BLACK);
      tft.drawFastHLine(pos_x_smeter, pos_y_smeter+2, s*s_w+1, YELLOW);
      tft.drawFastHLine(pos_x_smeter+s*s_w+1, pos_y_smeter+2, (9*s_w+1)-s*s_w+1, BLACK);

   //   tft.drawFastHLine(pos_x_smeter, pos_y_smeter+3, s*s_w+1, BLUE);
   //   tft.drawFastHLine(pos_x_smeter+s*s_w+1, pos_y_smeter+3, (9*s_w+1)-s*s_w+1, BLACK);

      if(dbuv>30) dbuv=30;
      tft.drawFastHLine(pos_x_smeter+9*s_w+1, pos_y_smeter, (dbuv/5)*s_w+1, RED);
      tft.drawFastHLine(pos_x_smeter+9*s_w+(dbuv/5)*s_w+1, pos_y_smeter, (6*s_w+1)-(dbuv/5)*s_w, BLACK);
      tft.drawFastHLine(pos_x_smeter+9*s_w+1, pos_y_smeter+1, (dbuv/5)*s_w+1, RED);
      tft.drawFastHLine(pos_x_smeter+9*s_w+(dbuv/5)*s_w+1, pos_y_smeter+1, (6*s_w+1)-(dbuv/5)*s_w, BLACK);
      tft.drawFastHLine(pos_x_smeter+9*s_w+1, pos_y_smeter+2, (dbuv/5)*s_w+1, RED);
      tft.drawFastHLine(pos_x_smeter+9*s_w+(dbuv/5)*s_w+1, pos_y_smeter+2, (6*s_w+1)-(dbuv/5)*s_w, BLACK);
}

void show_waterfall(void) {
  // experimental waterfall display for CW -
  // this should probably be on a faster timer since it needs to run as fast as possible to catch CW edges
  //  FFT bins are 22khz/128=171hz wide 
  // cw peak should be around 11.6khz - 
  static uint16_t waterfall[80];  // array for simple waterfall display
  static uint8_t w_index=0,w_avg;
  waterfall[w_index]=0;
  for (uint8_t y=66;y<67;++y)  // sum of bin powers near cursor - usb only
      waterfall[w_index]+=(uint8_t)(abs(myFFT.output[y])); // store bin power readings in circular buffer
  waterfall[w_index]|= (waterfall[w_index]<<5 |waterfall[w_index]<<11); // make it colorful
  int8_t p=w_index;
  for (uint8_t x=158;x>0;x-=2) {
    tft.fillRect(x,65,2,4,waterfall[p]);
    if (--p<0 ) p=79;
  }
  if (++w_index >=80) w_index=0; 
}

// indicate filter bandwidth on spectrum display
void show_bandwidth(int filtermode) { 
  tft.drawFastHLine(0,61,160, BLACK); // erase old indicator
  tft.drawFastHLine(0,62,160, BLACK); // erase old indicator 
  
  switch (filtermode)	{
    case LSB_NARROW:
      tft.drawFastHLine(72,61,6, RED);
      tft.drawFastHLine(72,62,6, RED);
    break;
    case LSB_WIDE:
      tft.drawFastHLine(61,61,20, RED);
      tft.drawFastHLine(61,62,20, RED);
    break;
    case USB_NARROW:
      tft.drawFastHLine(83,61,6, RED);
      tft.drawFastHLine(83,62,6, RED);
    break;
    case USB_WIDE:
      tft.drawFastHLine(80,61,20, RED);
      tft.drawFastHLine(80,62,20, RED);
    break;
  }
} 

void show_s_meter_layout() {
  tft.drawFastHLine (pos_x_smeter, pos_y_smeter-1, 9*s_w, WHITE);
  tft.drawFastHLine (pos_x_smeter, pos_y_smeter+3, 9*s_w, WHITE);
  tft.fillRect(pos_x_smeter, pos_y_smeter-3, 2, 2, WHITE);
  tft.fillRect(pos_x_smeter+8*s_w, pos_y_smeter-3, 2, 2, WHITE);
  tft.fillRect(pos_x_smeter+2*s_w, pos_y_smeter-3, 2, 2, WHITE);
  tft.fillRect(pos_x_smeter+4*s_w, pos_y_smeter-3, 2, 2, WHITE);
  tft.fillRect(pos_x_smeter+6*s_w, pos_y_smeter-3, 2, 2, WHITE);
  tft.fillRect(pos_x_smeter+7*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+3*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+5*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.fillRect(pos_x_smeter+9*s_w, pos_y_smeter-4, 2, 3, WHITE);
  tft.drawFastHLine (pos_x_smeter+9*s_w, pos_y_smeter-1, 3*s_w*2+2, GREEN);
  tft.drawFastHLine (pos_x_smeter+9*s_w, pos_y_smeter+3, 3*s_w*2+2, GREEN);
  tft.fillRect(pos_x_smeter+11*s_w, pos_y_smeter-4, 2, 3, GREEN);
  tft.fillRect(pos_x_smeter+13*s_w, pos_y_smeter-4, 2, 3, GREEN);
  tft.fillRect(pos_x_smeter+15*s_w, pos_y_smeter-4, 2, 3, GREEN);
  tft.drawFastVLine (pos_x_smeter-1, pos_y_smeter-1, 5, WHITE); 
  tft.drawFastVLine (pos_x_smeter+15*s_w+2, pos_y_smeter-1, 5, GREEN);

  tft.setFont();
  tft.setCursor(pos_x_smeter - 4, pos_y_smeter - 13);
  tft.setTextColor(WHITE);
  tft.setTextWrap(true);
  tft.print("S 1");
  tft.setCursor(pos_x_smeter + 28, pos_y_smeter - 13);
  tft.print("3");
  tft.setCursor(pos_x_smeter + 48, pos_y_smeter - 13);
  tft.print("5");
  tft.setCursor(pos_x_smeter + 68, pos_y_smeter - 13);
  tft.print("7");
  tft.setCursor(pos_x_smeter + 88, pos_y_smeter - 13);
  tft.print("9");
  tft.setCursor(pos_x_smeter + 120, pos_y_smeter - 13);
  tft.print("+20dB");
}

// show signal strength
void show_signalstrength(String s) {
  tft.setFont(&FreeSans9pt7b);  
  tft.fillRect(12, 72, 40, 14,BLACK);
  tft.setCursor(0, 85);
  tft.print(s);
}

// show radio mode
void show_radiomode(String mode) { 
  tft.fillRect(106, 108, 54, 14, BLACK); // erase old string
  tft.setFont(&FreeSans9pt7b);  
  tft.setTextColor(WHITE);
  tft.setCursor(106, 125);
  tft.print(mode);
}  

void show_band(String bandname) {  // show band
  tft.fillRect(60, 108, 40, 14, BLACK); // erase old string
  tft.setFont(&FreeSans9pt7b);  
  tft.setTextColor(WHITE);
  tft.setCursor(60, 125);
  tft.print(bandname);
}

// show frequency
void show_frequency(long int freq) { 
    tft.setFont(&FreeSans12pt7b);
    char string[80];   // print format stuff
    sprintf(string,"%d.%03d.%03d",freq/1000000,(freq-freq/1000000*1000000)/1000,
          freq%1000 );
    tft.fillRect(pos_x_freq,pos_y_freq-17,120,18,BLACK);
    tft.setCursor(pos_x_freq, pos_y_freq);
    tft.setTextColor(WHITE);
    tft.print(string); 
}    
