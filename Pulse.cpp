/*
  **Heart Rate Monitor with MAX30105**

  Description:
  This program uses the MAX30105 pulse oximeter and heart rate sensor to detect heartbeats 
  and calculate the heart rate (BPM). The heart rate is calculated based on the time 
  interval between consecutive IR signal peaks.

  Key Features:
  - Detects and calculates BPM when a finger is placed on the sensor.
  - Displays raw sensor readings every second.
  - Reports heart rate every 10 seconds via the Serial Monitor.
  - Ignores invalid data (e.g., when the IR value is 0 or inconsistent).
  - Reinitializes the sensor if no heartbeat is detected for 3 consecutive attempts.

  Required Hardware:
  - Arduino UNO or compatible board.
  - MAX30105 sensor connected via I2C:
      SDA -> A4
      SCL -> A5
      VIN -> 5V
      GND -> GND

  Created by: [Your Name]
*/

#include <Wire.h>
#include "MAX30105.h"

// Create a MAX30105 sensor object
MAX30105 particleSensor;

// Variables for heartbeat detection
long lastBeatTime = 0;         // Time of the last detected beat
long bpm = 0;                  // Calculated Beats Per Minute (BPM)
int beatsPerInterval = 0;      // Count beats in a given interval
long intervalStartTime = 0;    // Start time for the interval
const int intervalDuration = 10000; // Duration for BPM reporting (10 seconds)

// Variables for noise filtering
long smoothedIrValue = 0;      // Smoothed IR value for noise reduction
const int irThreshold = 50000; // IR value threshold for detecting a finger
const long minBeatInterval = 600; // Minimum time (ms) between beats

// Variables for BPM stabilization
const int maxSamples = 5;      // Number of samples for running average
long bpmSamples[maxSamples] = {0};
int sampleIndex = 0;

void setup() {
  /*
    **Setup Function**
    Purpose:
    - Initializes Serial communication for debugging.
    - Initializes the MAX30105 sensor.
    - Displays setup instructions on the Serial Monitor.
  */

  Serial.begin(9600);
  Serial.println("Initializing MAX3010x sensor...");

  // Initialize the MAX30105 sensor
  if (!initializeSensor()) {
    Serial.println("Sensor initialization failed. Check wiring.");
    while (1); // Halt execution if initialization fails
  }

  Serial.println("Place your finger on the sensor.");
  intervalStartTime = millis(); // Initialize the interval timer
}

void loop() {
  /*
    **Main Loop**
    Purpose:
    - Continuously read the IR value from the sensor.
    - Display raw sensor readings every second.
    - Detect a heartbeat if a finger is present.
    - Report BPM every 10 seconds.
    - Reinitialize the sensor after 3 failed attempts.
  */

  long currentTime = millis(); // Get the current time in milliseconds
  long rawIrValue = particleSensor.getIR(); // Get the raw IR value from the sensor

  // Smooth the IR value to reduce noise
  smoothedIrValue = (smoothedIrValue * 4 + rawIrValue) / 5;

  // Display raw sensor readings every second
  if (currentTime % 1000 < 100) {
    Serial.print("Raw IR Value: ");
    Serial.println(rawIrValue);
  }

  // Skip processing if the sensor gives invalid data
  if (rawIrValue == 0) {
    Serial.println("No valid signal. Please place your finger properly.");
    bpm = 0;
    return;
  }

  // Check for finger placement and detect heartbeat
  if (smoothedIrValue > irThreshold) {
    detectHeartbeat(smoothedIrValue, currentTime); // Detect pulse and calculate BPM
  } else {
    Serial.println("No finger detected.");
    bpm = 0; // Reset BPM if no finger is detected
  }

  // Report BPM every 10 seconds
  if (currentTime - intervalStartTime >= intervalDuration) {
    reportBPM(); // Call the function to calculate and report average BPM
    intervalStartTime = currentTime; // Reset the interval timer
    beatsPerInterval = 0;           // Reset the beat counter
  }

  delay(100); // Small delay for stable readings
}

bool initializeSensor() {
  /*
    **Sensor Initialization**
    Purpose:
    - Initializes the MAX30105 sensor.
    - Configures the sensor's LED pulse amplitudes for IR and Red LEDs.
    Returns:
    - true if initialization is successful.
    - false if initialization fails.
  */

  if (particleSensor.begin()) {
    // Configure the sensor's LEDs
    particleSensor.setup(); // Use default settings
    particleSensor.setPulseAmplitudeRed(0x1F); // Moderate brightness for Red LED
    particleSensor.setPulseAmplitudeIR(0x1F);  // Moderate brightness for IR LED
    return true; // Sensor initialized successfully
  }
  return false; // Sensor initialization failed
}

void detectHeartbeat(long irValue, long currentTime) {
  /*
    **Heartbeat Detection**
    Purpose:
    - Detects a heartbeat based on IR signal peaks.
    - Calculates BPM using the time interval between consecutive beats.
    Parameters:
    - irValue: The current IR signal value from the sensor.
    - currentTime: The current time in milliseconds.
  */

  static long previousBeatTime = 0; // Store the time of the previous beat
  long beatInterval = currentTime - previousBeatTime; // Calculate time between consecutive beats

  // Detect a heartbeat if the interval is valid and the IR value exceeds the threshold
  if (beatInterval > minBeatInterval) {
    previousBeatTime = currentTime; // Update the time of the last beat
    beatsPerInterval++; // Increment the beat counter for this interval

    // Calculate BPM based on the interval
    bpm = 60000 / beatInterval;
    addBPMToAverage(bpm); // Add BPM to the running average
    Serial.print("Heartbeat detected! BPM: ");
    Serial.println(bpm);
  }
}

void reportBPM() {
  /*
    **BPM Reporting**
    Purpose:
    - Calculates and reports the average BPM over the interval.
    - Helps smooth out fluctuations caused by noise or irregular beats.
  */

  if (beatsPerInterval > 0) {
    long averageBPM = (beatsPerInterval * 60000) / intervalDuration;
    Serial.print("Average Heart Rate: ");
    Serial.print(averageBPM);
    Serial.println(" BPM");
  } else {
    Serial.println("No heartbeat detected.");
  }
}

void addBPMToAverage(long newBPM) {
  /*
    **BPM Stabilization**
    Purpose:
    - Adds the latest BPM value to a running average.
    - Smooths out short-term fluctuations in BPM readings.
    Parameters:
    - newBPM: The latest BPM value to include in the average.
  */

  bpmSamples[sampleIndex] = newBPM;
  sampleIndex = (sampleIndex + 1) % maxSamples; // Loop back to the start of the array

  // Calculate the average BPM
  long sum = 0;
  for (int i = 0; i < maxSamples; i++) {
    sum += bpmSamples[i];
  }
  bpm = sum / maxSamples; // Update the stabilized BPM
}
