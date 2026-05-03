/*
 * Interactive stepper motor controller for Teensy 4.1
 *
 * Each motor runs on its own IntervalTimer (hardware timer) so all three
 * are fully independent and non-blocking.
 *
 * Serial commands (115200 baud):
 *   bnd1 <rpm>    Set BND1 speed; negative RPM = reverse
 *   bnd2 <rpm>    Set BND2 speed
 *   twi  <rpm>    Set TWI speed
 *   stop          Stop all motors immediately
 *   status        Print current RPM for each motor
 *   steps <n>     Set steps/rev (default 200 for 1.8° full-step;
 *                 e.g. 1600 for 1/8 microstepping)
 *   help          Print command reference
 *
 * The step pin is toggled at twice the desired step frequency, so every
 * rising edge the driver sees is one step.
 */

#include <Arduino.h>

// --- Pin constants (do not change) ---
const int BND1DIR = 33;
const int BND1STP = 34;
const int BND2DIR = 35;
const int BND2STP = 36;
const int TWIDIR  = 37;
const int TWISTP  = 38;

// --- Configurable at runtime via "steps <n>" command ---
volatile int stepsPerRev = 200;

// --- Per-motor state (read/written only from main loop except stepsPerRev) ---
float bnd1RPM = 0.0f;
float bnd2RPM = 0.0f;
float twiRPM  = 0.0f;

IntervalTimer bnd1Timer;
IntervalTimer bnd2Timer;
IntervalTimer twiTimer;

// --- ISRs: placed in FASTRUN (RAM) for deterministic timing ---
void FASTRUN bnd1ISR() { digitalToggleFast(BND1STP); }
void FASTRUN bnd2ISR() { digitalToggleFast(BND2STP); }
void FASTRUN twiISR()  { digitalToggleFast(TWISTP);  }

// ---------------------------------------------------------------------------
// Set a motor to the requested RPM.
// rpm == 0 stops the motor.  Negative rpm reverses direction.
// ---------------------------------------------------------------------------
void setMotorRPM(int dirPin, int stpPin,
                 IntervalTimer &timer, void (*isr)(),
                 float &rpmStore, float rpm)
{
    timer.end();                    // always stop the old timer first

    if (fabsf(rpm) < 0.01f) {
        digitalWrite(stpPin, LOW);  // leave step pin idle-low
        rpmStore = 0.0f;
        return;
    }

    digitalWrite(dirPin, rpm > 0.0f ? HIGH : LOW);

    // Toggle interval (µs) = 1 000 000 / (2 × steps_per_sec)
    //                       = 500 000 / steps_per_sec
    float stepsPerSec = fabsf(rpm) * stepsPerRev / 60.0f;
    float intervalUs  = 500000.0f / stepsPerSec;

    timer.begin(isr, intervalUs);
    rpmStore = rpm;
}

// Restart all running motors after a stepsPerRev change
static void reapplyAllRPMs() {
    setMotorRPM(BND1DIR, BND1STP, bnd1Timer, bnd1ISR, bnd1RPM, bnd1RPM);
    setMotorRPM(BND2DIR, BND2STP, bnd2Timer, bnd2ISR, bnd2RPM, bnd2RPM);
    setMotorRPM(TWIDIR,  TWISTP,  twiTimer,  twiISR,  twiRPM,  twiRPM);
}

static void printStatus() {
    Serial.printf("BND1: %+.1f RPM  |  BND2: %+.1f RPM  |  TWI: %+.1f RPM"
                  "  (steps/rev: %d)\r\n",
                  bnd1RPM, bnd2RPM, twiRPM, (int)stepsPerRev);
}

static void printHelp() {
    Serial.println("------- Stepper Motor Controller -------");
    Serial.println("  bnd1 <rpm>   BND1 speed (neg = reverse)");
    Serial.println("  bnd2 <rpm>   BND2 speed");
    Serial.println("  twi  <rpm>   TWI speed");
    Serial.println("  stop         Stop all motors");
    Serial.println("  status       Show current RPM");
    Serial.println("  steps <n>    Steps/rev (default 200)");
    Serial.println("  help         Show this message");
    Serial.println("----------------------------------------");
}

// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);

    pinMode(BND1DIR, OUTPUT); pinMode(BND1STP, OUTPUT);
    pinMode(BND2DIR, OUTPUT); pinMode(BND2STP, OUTPUT);
    pinMode(TWIDIR,  OUTPUT); pinMode(TWISTP,  OUTPUT);

    digitalWrite(BND1STP, LOW);
    digitalWrite(BND2STP, LOW);
    digitalWrite(TWISTP,  LOW);

    // Brief wait so the host terminal is ready before we print
    delay(500);
    Serial.println("\r\nTeensy stepper controller ready.");
    printHelp();
}

static String inputBuf;

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            inputBuf.trim();

            if (inputBuf.length() == 0) {
                inputBuf = "";
                continue;
            }

            // Split into command word and optional argument
            int space = inputBuf.indexOf(' ');
            String cmd = (space >= 0) ? inputBuf.substring(0, space) : inputBuf;
            cmd.toLowerCase();

            if (cmd == "stop") {
                setMotorRPM(BND1DIR, BND1STP, bnd1Timer, bnd1ISR, bnd1RPM, 0);
                setMotorRPM(BND2DIR, BND2STP, bnd2Timer, bnd2ISR, bnd2RPM, 0);
                setMotorRPM(TWIDIR,  TWISTP,  twiTimer,  twiISR,  twiRPM,  0);
                Serial.println("All motors stopped.");

            } else if (cmd == "status") {
                printStatus();

            } else if (cmd == "help") {
                printHelp();

            } else if (space >= 0) {
                float val = inputBuf.substring(space + 1).toFloat();

                if (cmd == "bnd1") {
                    setMotorRPM(BND1DIR, BND1STP, bnd1Timer, bnd1ISR, bnd1RPM, val);
                    Serial.printf("BND1 -> %+.1f RPM\r\n", val);

                } else if (cmd == "bnd2") {
                    setMotorRPM(BND2DIR, BND2STP, bnd2Timer, bnd2ISR, bnd2RPM, val);
                    Serial.printf("BND2 -> %+.1f RPM\r\n", val);

                } else if (cmd == "twi") {
                    setMotorRPM(TWIDIR, TWISTP, twiTimer, twiISR, twiRPM, val);
                    Serial.printf("TWI  -> %+.1f RPM\r\n", val);

                } else if (cmd == "steps") {
                    int n = (int)val;
                    if (n < 1) {
                        Serial.println("Error: steps/rev must be >= 1");
                    } else {
                        stepsPerRev = n;
                        reapplyAllRPMs();
                        Serial.printf("Steps/rev set to %d (running motors updated)\r\n", n);
                    }

                } else {
                    Serial.printf("Unknown command: %s  (type 'help')\r\n", cmd.c_str());
                }

            } else {
                Serial.printf("Unknown command: %s  (type 'help')\r\n", cmd.c_str());
            }

            inputBuf = "";
        } else {
            inputBuf += c;
        }
    }
}
