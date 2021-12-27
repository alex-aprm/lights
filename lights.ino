#include <Adafruit_NeoPixel.h>
#define PIN  13
#define NUMPIXELS 64
#define NUMF 64
Adafruit_NeoPixel strip (NUMPIXELS, 13, NEO_GRBW + NEO_KHZ800);
#include <Wire.h>

byte diodes[NUMPIXELS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};

byte voice[NUMPIXELS];
byte frames[NUMPIXELS][NUMF][3];
int durations[NUMPIXELS][NUMF];
int gotos[NUMPIXELS][NUMF];

byte frameCnt[NUMPIXELS];
int periods[NUMPIXELS];
int pos[NUMPIXELS];

int sensorThreshold = 1000;
int sensorTiming = 300;
int voiceThreshold = 3800;
ulong leftSensorLock = 0;
ulong rightSensorLock = 0;
ulong voiceLock = 0;
ulong voiceRelease = 0;
int voiceLockRelease = 1600;

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
  word d = 10;

  for (byte i = 0; i < NUMPIXELS; i++) {
    diodes[i] = i;
    if (i > 4 && i < 10)
      voice[i] = 255;
    int del = i * d;
    kf(i, 0, 0, 0, 0 + del);
    kf(i, 250, 0, 0, 500 + del);
    kf(i, 250, 0, 25, 1000 + del);
    kf(i, 0, 250, 25, 1500 + del);
    kf(i, 0, 250, 0, 2000 + del);
    kf(i, 250, 250, 0, 2500 + del);
    kf(i, 250, 0, 0, 3000 + del, 500 + del);

    kf(i, 0, 0, 0, 42000);

    kf(i, 0, 0, 0, 48000);
    kf(i, 0, 250, 0, 50000);
    kf(i, 0, 0, 250, 60000);
    kf(i, 0, 250, 0, 70000, 50000);

    kf(i, 0, 250, 0, 999000);
  }


  delay(100);
  //Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  //Serial2.println("s7-168#");
  //delay(100);
  Wire.begin();
  Serial.begin(115200);
  strip.begin();
  strip.setBrightness(255);
  totalms = millis();
  analogSetAttenuation(ADC_0db);
}

byte getComponent(byte num, byte prev, byte next, float prc) {
  return round(((next - prev) * prc + prev));
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

word soundLevel;
word samplesCnt;

  double v = 0;
  
void loop() {
  word elapsed = millis() - totalms;
  totalms = millis();
  elapsed = elapsed * 1;

  int leftSensorValue = ReadDistance(0x52);
  int rightSensorValue = ReadDistance(0x54);

  if (leftSensorValue < sensorThreshold && leftSensorLock == 0) {
    leftSensorLock = totalms;
    if (rightSensorLock > 0)
      leftSwitch();
  }
  if (leftSensorValue > sensorThreshold && leftSensorLock > 0 && totalms - leftSensorLock > sensorTiming) {
    leftSensorLock = 0;
  }

  if (rightSensorValue < sensorThreshold && rightSensorLock == 0) {
    rightSensorLock = totalms;
    if (leftSensorLock > 0)
      rightSwitch();
  }
  if (rightSensorValue > sensorThreshold && rightSensorLock > 0 && totalms - rightSensorLock > sensorTiming) {
    rightSensorLock = 0;
  }



  word x = 0;
  for (byte i = 0; i < 50; i++)
    x = max(x, (word)analogRead(34));



  soundLevel = x;//max(soundLevel, x);

  if (soundLevel > voiceThreshold) {
    voiceLock = totalms;
    v = max(v, abs(soundLevel - voiceThreshold) / 100.0);
    if (v > 1)
      v = 1;
  } else {
     v = v - 0.03;
    if (v < 0)
      v = 0;
  }
  
  if (voiceLock > 0 && v == 0 && soundLevel < voiceThreshold - 200 && (totalms - voiceLock > voiceLockRelease)) {
    voiceLock = 0;
    voiceRelease = totalms;
  }

  //v = 0;
  strip.clear();
  float rel = 1;
  if (voiceLock > 0)
    rel = 0;
  if (voiceRelease > 0) {
    int d = totalms - voiceRelease;

    if (d > voiceLockRelease)
      voiceRelease = 0;
    rel = map(d, 0, voiceLockRelease, 0.0, 1000) / 1000.0;
  }
    Serial.print(v);
    Serial.print('\t');
    Serial.print(voiceRelease > 0 ? 1 : 0);
    Serial.print('\t');
    Serial.print(voiceLock > 0 ? 1 : 0);
        Serial.println();

  for (byte i = 0; i < NUMPIXELS; i++) {
    if (periods[i] == 0)
      continue;
    //  strip.setPixelColor(i, strip.Color(255, 255, 255));

    pos[i] = pos[i] + elapsed;
    int p = pos[i];
    if (p < durations[i][0])
      continue;


    byte next = getNextKf(i, p);


    //check if there is go to

    if (gotos[i][next - 1] != 0) {
      pos[i] = (pos[i] - durations[i][next - 1] + gotos[i][next - 1]);
    }

    float prc = 0;
    word diff = durations[i][next] - durations[i][next - 1];
    if (diff != 0)
      prc = (float)(durations[i][next] - p) / (diff);

    byte r = getComponent(0, frames[i][next][0], frames[i][next - 1][0], prc);
    byte g = getComponent(1, frames[i][next][1], frames[i][next - 1][1], prc);
    byte b = getComponent(2, frames[i][next][2], frames[i][next - 1][2], prc);



    r = max((double)r * rel, v * voice[i]);
    g = max((double)g * rel, v * voice[i]);
    b = max((double)b * rel, v * voice[i]);


    strip.setPixelColor(diodes[i], strip.Color(r, g, b));

  }
  strip.show();
}


void leftSwitch() {
  for (byte i = 0; i < NUMPIXELS; i++) {
    pos[i] = 0;
  }
}

void rightSwitch() {
  for (byte i = 0; i < NUMPIXELS; i++) {
    pos[i] = 48000;
  }
}


void SensorRead(byte devaddr, unsigned char addr, unsigned char* datbuf, unsigned char cnt)
{
  unsigned short result = 0;
  Wire.beginTransmission(devaddr);
  Wire.write(byte(addr));
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(devaddr, cnt);
  if (cnt <= Wire.available()) {
    *datbuf++ = Wire.read();
    *datbuf++ = Wire.read();
  }
}

int ReadDistance(byte port) {

  unsigned short lenth_val = 0;
  unsigned char i2c_rx_buf[16];

  SensorRead(port, 0x00, i2c_rx_buf, 2);
  lenth_val = i2c_rx_buf[0];
  lenth_val = lenth_val << 8;
  lenth_val |= i2c_rx_buf[1];
  return lenth_val;
}
