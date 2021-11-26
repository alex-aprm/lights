#include <Adafruit_NeoPixel.h> // подключаем библиотеку

#define PIN  13              // указываем пин для подключения ленты
#define NUMPIXELS 32  
#define NUMF 128
Adafruit_NeoPixel strip (NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


  //byte diodes[NUMPIXELS] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};
  byte frames[NUMPIXELS][NUMF][3];
  int durations[NUMPIXELS][NUMF];
  int gotos[NUMPIXELS][NUMF];
  
  byte frameCnt[NUMPIXELS];
  int periods[NUMPIXELS];
  int pos[NUMPIXELS];


 void kf(byte d, byte r, byte g, byte b, int msec, int gotox = 0) {
  byte i = frameCnt[d];
  frames[d][i][0] = r;
  frames[d][i][1] = g;
  frames[d][i][2] = b;
  durations[d][i] = msec;
  gotos[d][i] = gotox;
  periods[d] = msec;
  frameCnt[d]++;
}
int totalms = 0;
void setup() {
    Serial.begin(115200);
    word d = 100;
    for (byte i = 0; i < 8; i++) {
    kf(i, 0, 0, 0, 0 + i*d),
    kf(i, 255, 0, 0, 500 + i*d),
    kf(i, 255, 0, 255, 900 + i*d),
    kf(i, 0, 255, 0, 1500 + i*d),
    kf(i, 255, 0, 0, 2000 + i*d, 500 + i*d);

    }

   strip.begin();             
   strip.setBrightness(50);  
   totalms = millis();
}

byte getComponent(byte num, byte prev, byte next, float prc) {
  return round(((next - prev)*prc + prev));
}

byte getNextKf(byte i, int p) {
   byte next = 0;
     for (byte j = 1; j < NUMF; j++) {
       next = j;
       if (durations[i][j] >= p)
         break;
     }
     return next;
}

void loop() {
  word elapsed = millis() - totalms;
  totalms = millis();

  
  for( byte i = 0; i < 8; i++) {
     if (periods[i] == 0) 
       continue;
    //  strip.setPixelColor(i, strip.Color(255, 255, 255)); 
     
     pos[i] = pos[i] + elapsed;
    
     if (pos[i] > periods[i]) {
      //check if there is go to
      byte next = getNextKf(i, periods[i]);
      pos[i] = pos[i] - periods[i];
      if (gotos[i][next] != 0) {
        pos[i] = pos[i]+ gotos[i][next];
      }
     }
     int p = pos[i];
  
     if (p < durations[i][0])
       continue;
     byte next = getNextKf(i, p);   
    

     float prc = 0;
     word diff = durations[i][next] - durations[i][next-1];
     if (diff != 0)
      prc = (float)(durations[i][next] - p) / (diff);

     byte r = getComponent(0, frames[i][next][0], frames[i][next - 1][0], prc);
     byte g = getComponent(1, frames[i][next][1], frames[i][next - 1][1], prc);
     byte b = getComponent(2, frames[i][next][2], frames[i][next - 1][2], prc);

//continue;    
     strip.setPixelColor(i, strip.Color(r, g, b)); 
     strip.setPixelColor(i+8, strip.Color(r, g, b)); 
     strip.setPixelColor(i+16, strip.Color(r, g, b)); 
     strip.setPixelColor(i+24, strip.Color(r, g, b)); 
  }
     strip.show();
     //delay(10);
  //j++;
  //if (j == 255) {
  //   delay(1000);
  // strip.clear();   // выключаем все светодиоды
 

}
