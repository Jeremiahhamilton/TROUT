// DNA FOLD SENSOR - RIVER VERSION
// Clean output with ridge decomposition lens

#include <SPI.h>
#include <math.h>

#define HAS_SD_LIBRARY 0

// Pin definitions
const int SineWave = A0;
const int GroundPlanePin = A1;
const int TelluricPin = A2;
const int PiezoPin = A3;
const int MagneticPin = A4;
const int PressurePin = A5;
const int MirrorRFPin = A6;
const int SD_CS_PIN = 10;

// Constants
const float IDEAL_MID_RAIL = 512.0;
const int POLARIS_RING_SIZE = 1023;
const float POLARIS_FACTOR = 0.5;
const float SEDIMENT_DAMP = 0.85;
// const float TWO_PI = 6.283185307179586; // Use Arduino's built-in

// Entropy settings
const float ENTROPY_DROP_THRESHOLD = 0.001;
const unsigned long STATS_REPORT_INTERVAL = 30000;
const int MIN_SAMPLES_FOR_STATS = 10;

// Mode detection
enum SignalMode { GROUNDED, FLOATING, ANTENNA, UNKNOWN };
SignalMode currentMode = UNKNOWN;

// Data structures
struct RiverNode {
  float sedimentaryFlow;
  float crystallineStructure;
  float harmonicResonance;
};

struct ExposureState {
  float dna_avg;
  float clean_avg;
  float depth_avg;
  float pulse_avg;
  int sample_count;
  unsigned long last_update;
};

// Global variables
static float pillow = 0.0;
static float mirrorPillow = 0.0;
static float stereoDepth = 0.0;
static float tune_freq = 300.0;
static RiverNode currentNode = {0.001, 0.5, 0.25};
static ExposureState exposure = {0};
// static File logFile; // SD library disabled

// Two-state rotational DNA core
static float dna_pos = 0.0;
static float dna_vel = 0.0;

// Entropy analyzer
class EntropyAnalyzer {
private:
  float runningSum;
  float runningSumSquares;
  int sampleCount;
  float lastEntropy;

public:
  EntropyAnalyzer() : runningSum(0), runningSumSquares(0), sampleCount(0), lastEntropy(0) {}
  
  void reset() {
    runningSum = 0;
    runningSumSquares = 0;
    sampleCount = 0;
  }
  
  void addSample(float value) {
    runningSum += value;
    runningSumSquares += value * value;
    sampleCount++;
  }
  
  float calculateEntropy() {
    if (sampleCount < 10) return lastEntropy;

    float mean = runningSum / sampleCount;
    float variance = (runningSumSquares / sampleCount) - (mean * mean);

    // Numeric safety clamps
    if (variance < 0.0f) variance = 0.0f;
    if (variance > 1.0f) variance = 1.0f;

    // Use variance directly as entropy proxy
    float entropy = variance;

    return entropy;
  }
  
  float getEntropyDrop(float newEntropy) {
    float drop = lastEntropy - newEntropy;
    lastEntropy = newEntropy;
    return drop;
  }
};

// Entropy statistics
struct EntropyStats {
  float sum;
  float sumSquares;
  int sampleCount;
  
  EntropyStats() : sum(0), sumSquares(0), sampleCount(0) {}
  
  void update(float drop) {
    sum += drop;
    sumSquares += drop * drop;
    sampleCount++;
  }
  
  float getMean() { return sampleCount > 0 ? sum / sampleCount : 0; }
  float getStdDev() { 
    if (sampleCount < 2) return 0;
    float mean = getMean();
    float variance = (sumSquares / sampleCount) - (mean * mean);
    if (variance < 0.0f) variance = 0.0f;
    return sqrt(variance);
  }
};

static EntropyAnalyzer inputAnalyzer, outputAnalyzer;
static EntropyStats groundStats, noiseStats, realStats;

// Function prototypes
float conditionInput(float rawValue);
void accumulateExposure(float dna, float depth, float pulse, float clean);
bool exposureReady(unsigned long now);
void readPlanetaryRiver();
float calculateMirrorPillow(float pillow, float DNA, float flow);
float calculateStereoDepth(float pillow, float mirror, float flow);
void emitBidirectionalPulse(int pin, float intensity);
bool isInPlanetaryPillow(float flow, float DNA);
bool capturePlanetaryPacket(int source, float depth, float DNA, float flow);
void replicateStereoPacket(float pillow, float mirror, float depth);
void detectSignalMode(int rawValue);
void updateEntropyStats(EntropyStats &stats, float drop);
void reportEntropyStats();
void processEntropyDrop(float inEntropy, float outEntropy, float pushPull, float water, int rawValue);

void setup() {
  Serial.begin(1000000);
  
  // Initialize exposure timing
  exposure.last_update = millis();
  
  // Pin configuration
  pinMode(SineWave, INPUT);
  digitalWrite(SineWave, LOW);
  
  pinMode(MirrorRFPin, INPUT);
  digitalWrite(MirrorRFPin, LOW);
  
  pinMode(9, OUTPUT);
  
  // Initialize entropy analyzers
  inputAnalyzer.reset();
  outputAnalyzer.reset();
}

void loop() {
  // Read and condition input
  int raw_val = analogRead(SineWave);
  int val = (int)conditionInput(raw_val);
  
  // Detect flatline and add variation
  static int last_val = -1;
  static unsigned long last_change = 0;
  
  if (val == last_val && (millis() - last_change > 500)) {
    val += random(-10, 11);
    val = constrain(val, 0, 1023);
  } else {
    last_change = millis();
  }
  last_val = val;
  
  float normalized = (float)val / 1023.0;
  
  // Core DNA mathematics - torque-style two-state system
  float phase = normalized * TWO_PI * 0.01;
  float scaler = 5.0 + normalized * 5.0;
  
  float F1 = sin(phase) * scaler;
  float F2 = sin(phase + val/3.0) * scaler;
  float F3 = sin(phase + 2*val/3.0) * scaler;
  float F4 = sin(phase + val) * scaler;
  float F5 = sin(phase + 4*val/3.0) * scaler;
  float F6 = sin(phase + 5*val/3.0) * scaler;
  
  // Two-state rotational core
  float drive = (F1 + F2 - F4 - F5) * F6;
  
  // torque-like input
  dna_vel += drive * 0.01;
  
  // torque damping (damps velocity, NOT position)
  dna_vel *= 0.995;
  
  // integrate
  dna_pos += dna_vel;
  
  // optional soft limiting to prevent runaway
  if (dna_pos > TWO_PI) dna_pos -= TWO_PI;
  if (dna_pos < -TWO_PI) dna_pos += TWO_PI;
  
  float DNA = sin(dna_pos);
  
  // optional elastic floor
  if (abs(DNA) < 0.00001f) {
    DNA = (DNA >= 0) ? 0.00001f : -0.00001f;
  }
  
  // Clean signal
  float polaris_cross = DNA * POLARIS_FACTOR;
  float clean_signal = 0.6 * (DNA - polaris_cross) + 0.4 * dna_vel;  // Use velocity instead of prevDNA
  
  // Water flow
  static float prev_water = 0.0;
  float water_flow = SEDIMENT_DAMP * clean_signal + (1.0 - SEDIMENT_DAMP) * prev_water;
  prev_water = water_flow;
  
  // Push-pull
  float push_pull = clean_signal + water_flow;
  if (abs(push_pull) < 0.05) {
    push_pull = 0.1;  // Remove time-based modulation
  }
  
  // Pillow zone - make pillow a state variable instead of forced constant
  static float pillow_state = 0.0;
  if (abs(clean_signal) < 0.2 || abs(DNA) > 0.9) {
    // Make pillow a leaky integrator of clean_signal
    pillow_state = 0.9 * pillow_state + 0.1 * clean_signal;
    pillow = pillow_state;
    water_flow *= 0.55;
    
    if (pillow < -0.45) pillow += 0.015;
  } else {
    // Outside pillow zone, let pillow decay
    pillow_state *= 0.95;
    pillow = pillow_state;
  }
  
  // Calculate derived values
  mirrorPillow = calculateMirrorPillow(pillow, DNA, currentNode.sedimentaryFlow);
  stereoDepth = calculateStereoDepth(pillow, mirrorPillow, currentNode.sedimentaryFlow);
  tune_freq = 300.0 + (stereoDepth * 500 + 100);  // Remove time-based modulation
  float rf_distance_km = pow(stereoDepth, 1.5) * 1040.0;
  
  // Exposure engine
  unsigned long now_ms = millis();
  accumulateExposure(DNA, stereoDepth, abs(dna_vel), clean_signal);
  
  if (exposureReady(now_ms)) {
    // Calculate coupling ratio
    float input_variance = inputAnalyzer.calculateEntropy();  // Using variance as proxy
    float output_variance = outputAnalyzer.calculateEntropy();
    float coupling_ratio = (input_variance > 0.000001) ? output_variance / input_variance : 999.0;
    
    // Bounds check to prevent NaN propagation in output
    if (!isnan(exposure.dna_avg) && isfinite(exposure.dna_avg)) {
      Serial.print("DNA:");
      Serial.print(exposure.dna_avg, 6);
    } else {
      Serial.print("DNA:0.000000");
    }
    
    Serial.print(",MAG:");
    Serial.print(isfinite(exposure.dna_avg) ? abs(exposure.dna_avg) * 10.0 : 0.0, 2);
    Serial.print(",POS:");
    Serial.print(isfinite(exposure.dna_avg) ? int(abs(exposure.dna_avg) * POLARIS_RING_SIZE) + 1 : 1);
    
    if (!isnan(exposure.clean_avg) && isfinite(exposure.clean_avg)) {
      Serial.print(",CLEAN:");
      Serial.print(exposure.clean_avg, 6);
    } else {
      Serial.print(",CLEAN:0.000000");
    }
    
    Serial.print(",WATER:");
    Serial.print(water_flow, 6);
    Serial.print(",PUSH_PULL:");
    Serial.print(push_pull, 6);
    Serial.print(",PILLOW:");
    Serial.print(pillow + water_flow, 6);
    Serial.print(",MIRROR:");
    Serial.print(mirrorPillow, 6);
    
    if (!isnan(exposure.depth_avg) && isfinite(exposure.depth_avg)) {
      Serial.print(",STEREO_DEPTH:");
      Serial.print(exposure.depth_avg, 6);
    } else {
      Serial.print(",STEREO_DEPTH:0.000000");
    }
    
    Serial.print(",PLANETARY_FLOW:");
    Serial.print(currentNode.sedimentaryFlow, 6);
    Serial.print(",RF_DIST_KM:");
    Serial.print(rf_distance_km, 2);
    
    if (!isnan(exposure.pulse_avg) && isfinite(exposure.pulse_avg)) {
      Serial.print(",PULSE_STATE:");
      Serial.print(exposure.pulse_avg, 6);
    } else {
      Serial.print(",PULSE_STATE:0.000000");
    }
    
    Serial.print(",TUNE_FREQ:");
    Serial.print(tune_freq, 2);
    Serial.print(",COUPLING:");
    Serial.print(coupling_ratio, 2);
    Serial.println();
    
    // Reset exposure averaging to prevent lifetime masking
    exposure.sample_count = 0;
    exposure.dna_avg = 0;
    exposure.clean_avg = 0;
    exposure.depth_avg = 0;
    exposure.pulse_avg = 0;
  }
  
  // Entropy analysis
  float norm_in = normalized;
  float norm_out = (clean_signal + 1.0f) * 0.5f;
  
  float tiny_noise = (random(0, 1000) - 500.0f) / 200000.0f;
  norm_in += tiny_noise;
  
  inputAnalyzer.addSample(norm_in);
  outputAnalyzer.addSample(norm_out);
  
  static int entropyFrameCount = 0;
  entropyFrameCount++;
  
  if (entropyFrameCount >= 20) {
    float inputEntropy = inputAnalyzer.calculateEntropy();
    float outputEntropy = outputAnalyzer.calculateEntropy();
    
    float drop = inputEntropy - outputEntropy;
    processEntropyDrop(inputEntropy, outputEntropy, push_pull, water_flow, val);
    
    inputAnalyzer.reset();
    outputAnalyzer.reset();
    
    entropyFrameCount = 0;
  }
  
  // Update memory
  readPlanetaryRiver();
}

// Helper functions
float conditionInput(float rawValue) {
  static float filteredValue = IDEAL_MID_RAIL;
  float alpha = 0.95;
  filteredValue = alpha * rawValue + (1.0 - alpha) * filteredValue;
  float acComponent = rawValue - filteredValue;
  
  float adaptiveGain = 2.0 + abs(rawValue - IDEAL_MID_RAIL) / 100.0;
  float conditioned = IDEAL_MID_RAIL + acComponent * adaptiveGain;
  
  if (abs(rawValue - IDEAL_MID_RAIL) < 10) {
    float syntheticVariation = sin(millis() * 0.001) * 20.0;
    conditioned += syntheticVariation;
  }
  
  conditioned = constrain(conditioned, 0.0, 1023.0);
  return conditioned;
}

void accumulateExposure(float dna, float depth, float pulse, float clean) {
  // Bounds check to prevent NaN propagation
  if (!isnan(dna) && !isnan(depth) && !isnan(pulse) && !isnan(clean) &&
      isfinite(dna) && isfinite(depth) && isfinite(pulse) && isfinite(clean)) {
    exposure.dna_avg = (exposure.dna_avg * exposure.sample_count + dna) / (exposure.sample_count + 1);
    exposure.clean_avg = (exposure.clean_avg * exposure.sample_count + clean) / (exposure.sample_count + 1);
    exposure.depth_avg = (exposure.depth_avg * exposure.sample_count + depth) / (exposure.sample_count + 1);
    exposure.pulse_avg = (exposure.pulse_avg * exposure.sample_count + pulse) / (exposure.sample_count + 1);
    exposure.sample_count++;
  }
}

bool exposureReady(unsigned long now) {
  if (now - exposure.last_update > 33) {  // ~30 FPS
    exposure.last_update = now;
    if (exposure.sample_count > 0) {
      return true;
    }
  }
  return false;
}

void readPlanetaryRiver() {
  // Use real analog variation instead of synthetic sine
  float ground = analogRead(GroundPlanePin);
  float telluric = analogRead(TelluricPin);
  float piezo = analogRead(PiezoPin);
  
  // Normalize and combine real sensor inputs
  currentNode.sedimentaryFlow = (ground + telluric + piezo) / (3.0 * 1023.0);
  currentNode.crystallineStructure = abs(ground - telluric) / 1023.0;
  currentNode.harmonicResonance = piezo / 1023.0;
}

float calculateMirrorPillow(float pillow, float DNA, float flow) {
  return pillow * 0.8 + DNA * 0.1 + flow * 0.1;
}

float calculateStereoDepth(float pillow, float mirror, float flow) {
  // Bounds check to prevent NaN propagation
  if (!isnan(pillow) && !isnan(mirror) && !isnan(flow) &&
      isfinite(pillow) && isfinite(mirror) && isfinite(flow)) {
    return abs(pillow - mirror) * flow;
  }
  return 0.0;  // Safe fallback
}

void emitBidirectionalPulse(int pin, float intensity) {
  if (intensity > 0.01) {
    tone(pin, (int)(300 + intensity * 200), 10);
  }
}

bool isInPlanetaryPillow(float flow, float DNA) {
  return flow > 0.3 && abs(DNA) > 0.5;
}

bool capturePlanetaryPacket(int source, float depth, float DNA, float flow) {
  return random(0, 100) < 10;  // 10% chance
}

void replicateStereoPacket(float pillow, float mirror, float depth) {
  // Placeholder for packet replication
}

void detectSignalMode(int rawValue) {
  if (rawValue < 200) {
    currentMode = GROUNDED;
  } else if (rawValue > 800) {
    currentMode = FLOATING;
  } else {
    currentMode = ANTENNA;
  }
}

void updateEntropyStats(EntropyStats &stats, float drop) {
  stats.update(drop);
}

void reportEntropyStats() {
  bool hasAnyData = (realStats.sampleCount >= MIN_SAMPLES_FOR_STATS) || 
                   (noiseStats.sampleCount >= MIN_SAMPLES_FOR_STATS) || 
                   (groundStats.sampleCount >= MIN_SAMPLES_FOR_STATS);
  if (!hasAnyData) return;
  
  Serial.println("=== ENTROPY STATS ===");
  
  if (groundStats.sampleCount >= MIN_SAMPLES_FOR_STATS) {
    Serial.print("GROUNDED: mean=");
    Serial.print(groundStats.getMean(), 6);
    Serial.print(", std=");
    Serial.print(groundStats.getStdDev(), 6);
    Serial.print(", n=");
    Serial.println(groundStats.sampleCount);
  }
  
  if (noiseStats.sampleCount >= MIN_SAMPLES_FOR_STATS) {
    Serial.print("FLOATING: mean=");
    Serial.print(noiseStats.getMean(), 6);
    Serial.print(", std=");
    Serial.print(noiseStats.getStdDev(), 6);
    Serial.print(", n=");
    Serial.println(noiseStats.sampleCount);
  }
  
  if (realStats.sampleCount >= MIN_SAMPLES_FOR_STATS) {
    Serial.print("ANTENNA: mean=");
    Serial.print(realStats.getMean(), 6);
    Serial.print(", std=");
    Serial.print(realStats.getStdDev(), 6);
    Serial.print(", n=");
    Serial.println(realStats.sampleCount);
  }
  
  Serial.println("==================");
}

void processEntropyDrop(float inEntropy, float outEntropy, float pushPull, float water, int rawValue) {
  float drop = inEntropy - outEntropy;
  if (abs(drop) < ENTROPY_DROP_THRESHOLD) return;
  
  detectSignalMode(rawValue);
  switch (currentMode) {
    case GROUNDED: updateEntropyStats(groundStats, drop); break;
    case FLOATING: updateEntropyStats(noiseStats, drop); break;
    case ANTENNA: updateEntropyStats(realStats, drop); break;
    default: break;
  }
  
  if (abs(drop) > 0.003) {
    Serial.print("ENTROPY_DROP,IN:");
    Serial.print(inEntropy, 6);
    Serial.print(",OUT:");
    Serial.print(outEntropy, 6);
    Serial.print(",DROP:");
    Serial.print(drop, 6);
    Serial.print(",MODE:");
    Serial.print(currentMode == ANTENNA ? "ANTENNA" : currentMode == FLOATING ? "FLOATING" : "GROUNDED");
    Serial.print(",RAW:");
    Serial.println(rawValue);
  }
}
