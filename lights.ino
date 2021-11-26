#include <Adafruit_NeoPixel.h> // подключаем библиотеку

#define PIN  13              // указываем пин для подключения ленты
#define NUMPIXELS 32  
#define NUMF 128
Adafruit_NeoPixel strip (NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


  byte diodes[NUMPIXELS] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};

  byte voice[NUMPIXELS] = {0,32,192,192,192,192,32,0, 0,64,192,255,255,192,64,0, 0,64,192,255,255,192,64,0, 0,32,192,192,192,128,32,0};
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
    word d = 1000;

    for (byte j = 0; j < 4; j++) {
     for (byte i = 0; i < 8; i++) {
      kf(i+j*8, 0, 0, 0, 0 + i*d),
    kf(i+j*8, 64, 0, 0, 300 + i*d),
    kf(i+j*8, 64, 0, 64, 10000 + i*d),
    kf(i+j*8, 0, 64, 64, 15000 + i*d),
    kf(i+j*8, 0, 64, 0, 20000 + i*d),
    kf(i+j*8, 64, 64, 0, 25000 + i*d),
    kf(i+j*8, 64, 0, 0, 30000 + i*d, 300 + i*d);
      }
    }

   strip.begin();             
   strip.setBrightness(100);  
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
  elapsed = elapsed * 1;

  
  for( byte i = 0; i < 32; i++) {
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

     float v = sin((float)totalms / 100);
     if (v < 0) v = 0;

     r = max((float)r, v*voice[i]); 
     g = max((float)g, v*voice[i]); 
     b = max((float)b, v*voice[i]); 


     strip.setPixelColor(diodes[i], strip.Color(r, g, b)); 
  }
     strip.show();
     //delay(10);
  //j++;
  //if (j == 255) {
  //   delay(1000);
  // strip.clear();   // выключаем все светодиоды
 

}
