/*
 * Copyright (c) 2019, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /*
  * This is an early stage prototype; use at your own risk!
  */
#include <Arduino.h>
#include "sensirion-lf.h"

// comment / uncomment this to generate output for humans or the Arduino IDE plotter
// #define GENERATE_PLOTTER_OUTPUT

// delay between measurements
#define MEASURE_DELAY 500
static const float SLF3X_SCALE_FACTOR_FLOW = 500.0;
static const float SLF3X_SCALE_FACTOR_TEMP = 200.0;
static const uint8_t SLF3X_NEW_I2C_ADDRESS =0x39;

SensirionLF SLF3XB(SLF3X_SCALE_FACTOR_FLOW,
                  SLF3X_SCALE_FACTOR_TEMP,
                  SLF3X_NEW_I2C_ADDRESS,A2);
void setup() {
  Serial.begin(115200); // initialize serial communication
  if (SLF3XB.init() != 0) {
    Serial.println("Error during SLF3X init. Stopping application.");
    while (1) { delay(1000); } // loop forever
  }
}

void loop() {
  int ret = SLF3XB.readSample();
  if (ret == 0) {
#ifdef GENERATE_PLOTTER_OUTPUT
    // for plotter, we use micro liter per hour so scale it into a
    // similar range as temperature is in; depending on your application,
    // you may want to change the scale factor
    Serial.print(SLF3X.getFlow() * 1000, 1);
    Serial.print(" ");
    Serial.println(SLF3X.getTemp(), 1);
#else
    Serial.print("Flow: ");
    Serial.print(SLF3XB.getFlow(), 2);
    Serial.print(" ml/min");

    Serial.print(" | Temp: ");
    Serial.print(SLF3XB.getTemp(), 1);
    Serial.print(" deg C");

    if (SLF3XB.isAirInLineDetected()) {
      Serial.print(" [Air in Line Detected]");
    }
    if (SLF3XB.isHighFlowDetected()) {
      Serial.print(" [High Flow Detected]");
    }
    Serial.print("\n");
#endif /* GENERATE_PLOTTER_OUTPUT */
  } else {
    Serial.print("Error in SLF3X.readSample(): ");
    Serial.println(ret);
  }

  delay(MEASURE_DELAY); // delay between reads
}
