#include "arduinoFFT.h"

#define BUZZER 7
#define MIC A1

const int P_BUTTON = 10;  // Play
const int C_BUTTON = 12;  // Clear
const int R_BUTTON = 11;  // Record
const int NUM_PITCHES = 45;

const uint16_t SAMPLES = 128;         
const double SAMPLING_FREQ = 4000.0;
const unsigned long SAMPLING_PERIOD_US = round(1000000.0 / SAMPLING_FREQ);
const double NOISE_THRESHOLD = 5.0;    // Minimum peak magnitude to register a note

double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

int pitches[] = {
  175, 185, 196, 208, 220, 233, 247, 262, 277,
  294, 311, 330, 349, 370, 392, 415, 440,
  466, 494, 523, 554, 587, 622, 659, 698,
  740, 784, 831, 880, 932, 1047, 1109, 1175,
  1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865,
  1976, 2093, 2217, 2349
};

bool P = false; 
bool C = false; 
bool R = false; 
bool cleared = true;
int music[50];

// Returns the INDEX in pitches[] of the frequency directly
int getClosestPitchIndex(double freq) {
  if (freq < 60.0) return -1; // Filter low rumble/silence

  int closestIdx = 0;
  double minDiff = fabs(freq - pitches[0]);

  for (int i = 1; i < NUM_PITCHES; i++) {
    double diff = fabs(freq - pitches[i]);
    if (diff < minDiff) {
      minDiff = diff;
      closestIdx = i;
    }
  }
  return closestIdx;
}

void setup() {
  Serial.begin(115200);
  Serial.println("--- System Ready ---");
  pinMode(BUZZER, OUTPUT);
  noTone(BUZZER);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(P_BUTTON, INPUT);
  pinMode(C_BUTTON, INPUT);
  pinMode(R_BUTTON, INPUT);
  
  for (int i = 0; i < 50; i++) {
    music[i] = 0;
  }
}

void loop() {
  if (!P && !C && !R) {

    // --- CLEAR MEMORY ---
    if (digitalRead(C_BUTTON) == HIGH) {
      C = true;
      if (!cleared) {
        for (int i = 0; i < 50; i++) {
          music[i] = 0; 
        }
      }
      cleared = true;
      Serial.println("Memory Cleared!");
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      C = false;
    }

    // --- PLAYBACK ---
    else if (digitalRead(P_BUTTON) == HIGH) {
      P = true;
      Serial.println("--- Playing Back Recording ---");
      digitalWrite(LED_BUILTIN, HIGH);

      for (int i = 0; i < 50; i++) {
        int currentNote = music[i];
        int nextNote = (i < 49) ? music[i + 1] : 0;

        if (currentNote > 0) {
          tone(BUZZER, currentNote);
          delay(100);

          if (currentNote != nextNote) {
            noTone(BUZZER);
            delay(30);
          }
        } else {
          noTone(BUZZER);
          delay(100);
        }
      }

      noTone(BUZZER);
      digitalWrite(LED_BUILTIN, LOW);
      P = false;
    }

    // --- RECORDING ---
    else if (digitalRead(R_BUTTON) == HIGH) {
      R = true;
      cleared = false;
      Serial.println("--- Starting Recording (5 Seconds) ---");
      
      digitalWrite(LED_BUILTIN, HIGH); delay(80);
      digitalWrite(LED_BUILTIN, LOW);  delay(80);
      digitalWrite(LED_BUILTIN, HIGH); delay(80);
      digitalWrite(LED_BUILTIN, LOW);  delay(80);
      digitalWrite(LED_BUILTIN, HIGH);
      
      int prevPitchIdx = -1; // Keep track of previous slot's pitch index

      for (int slot = 0; slot < 50; slot++) {
        unsigned long chunkStart = millis();

        // 1. Sample audio
        for (uint16_t i = 0; i < SAMPLES; i++) {
          unsigned long sampleTimer = micros();
          vReal[i] = analogRead(MIC);
          vImag[i] = 0.0;

          while ((micros() - sampleTimer) < SAMPLING_PERIOD_US) {
            // Precise 250us timing loop
          }
        }

        // 2. Perform FFT Analysis
        FFT.dcRemoval();
        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
        FFT.compute(FFTDirection::Forward);
        FFT.complexToMagnitude();

        // 3. Find dominant frequency & peak magnitude
        double peakFreq = 0.0;
        double peakMag = 0.0;
        FFT.majorPeak(&peakFreq, &peakMag);

        // 4. Quantize to pitch with Hysteresis (Filter out 1-note fluctuations)
        if (peakMag < NOISE_THRESHOLD) {
          music[slot] = 0;
          prevPitchIdx = -1; // Reset filter state on silence
        } else {
          int detectedIdx = getClosestPitchIndex(peakFreq);

          if (prevPitchIdx != -1 && abs(detectedIdx - prevPitchIdx) <= 1) {
            // Pitch changed by only 1 semitone -> Lock to previous note to prevent flickering
            music[slot] = pitches[prevPitchIdx];
          } else {
            // Valid intentional note change (> 1 semitone step or fresh note)
            music[slot] = pitches[detectedIdx];
            prevPitchIdx = detectedIdx;
          }
        }
        
        // Debug output
        Serial.print("Slot ");
        if (slot < 10) Serial.print("0");
        Serial.print(slot);
        Serial.print(" | Freq: ");
        Serial.print(peakFreq, 1);
        Serial.print(" Hz | Mag: ");
        Serial.print(peakMag, 1);
        Serial.print(" -> Mapped Pitch: ");
        Serial.println(music[slot]);

        // 5. Pad remaining time for 100ms timing
        while ((millis() - chunkStart) < 100) {
          // Wait out remaining time
        }
      }
      
      Serial.println("--- Recording Completed ---");
      R = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}