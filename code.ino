#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.hpp>
#include <AccelStepper.h>

// LCD & IR
#define IR_RECEIVE_PIN 32
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Stepper motor pins (Arduino Uno - Port D)
#define STEP_PIN 4  // PD4
#define DIR_PIN 3   // PD3
#define MS1_PIN 7   // PD7
#define MS2_PIN 6   // PD6

// DC motor (card launcher) pins (Arduino Uno)
#define ENABLE 12   // PB4
#define DIRA 10     // PB2
#define DIRB 11     // PB3

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Stepper and rotation config
const long totalRotationSteps = 26125;  // total steps for full rotation
long stepsPerPlayer = 0;
long currentPosition = 0;

// IR state
unsigned long lastIRTime = 0;
const unsigned long debounceDelay = 200;

int numPlayers = 0;
int cardsToDeal = 0;
int currentTarget = 0;
int currentPlayer = 1;

bool selectingPlayers = true;
bool selectingCards = false;
bool dealing = false;
bool waitingForPlayerSelection = false;

// For detecting reset sequence
bool received100plus = false;
bool received200plus = false;

// Direct register control functions

// Initialize GPIO pins using registers
void initGPIO() {
  // Configure stepper pins (Port D: PD3, PD4, PD6, PD7)
  // Set pins as outputs using Data Direction Register
  DDRD |= (1 << DDD3) | (1 << DDD4) | (1 << DDD6) | (1 << DDD7);
  
  // Configure DC motor pins (Port B: PB2, PB3, PB4)
  // Set pins as outputs
  DDRB |= (1 << DDB2) | (1 << DDB3) | (1 << DDB4);
  
  // Initialize stepper microstep configuration
  // MS1 = HIGH, MS2 = LOW (1/8 microstepping)
  PORTD |= (1 << PORTD7);   // MS1_PIN (7) HIGH
  PORTD &= ~(1 << PORTD6);  // MS2_PIN (6) LOW
  
  // Initialize DC motor pins to LOW
  PORTB &= ~((1 << PORTB2) | (1 << PORTB3)); // DIRA, DIRB LOW
}

// Set stepper direction using direct register access
void setStepperDirection(bool clockwise) {
  if (clockwise) {
    PORTD |= (1 << PORTD3);   // DIR_PIN HIGH
  } else {
    PORTD &= ~(1 << PORTD3);  // DIR_PIN LOW
  }
}

// Generate step pulse using direct register access
void stepPulse() {
  PORTD |= (1 << PORTD4);   // STEP_PIN HIGH
  delayMicroseconds(2);     // Minimum pulse width
  PORTD &= ~(1 << PORTD4);  // STEP_PIN LOW
  delayMicroseconds(2);     // Minimum low time
}

// Control DC motor using PWM and direction pins
void setDCMotor(uint8_t speed, uint8_t direction) {
  // Set PWM speed on ENABLE pin (OC1B - Pin 12/PB4)
  // Configure Timer1 for Fast PWM mode
  TCCR1A |= (1 << COM1B1) | (1 << WGM11);  // Non-inverting mode, Fast PWM
  TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM, prescaler 8
  ICR1 = 255;  // TOP value for 8-bit resolution
  
  OCR1B = speed;  // Set PWM duty cycle
  
  // Set direction using DIRA and DIRB
  switch(direction) {
    case 0: // Stop
      PORTB &= ~((1 << PORTB2) | (1 << PORTB3)); // Both LOW
      break;
    case 1: // Forward
      PORTB |= (1 << PORTB2);   // DIRA HIGH
      PORTB &= ~(1 << PORTB3);  // DIRB LOW
      break;
    case 2: // Reverse
      PORTB &= ~(1 << PORTB2);  // DIRA LOW
      PORTB |= (1 << PORTB3);   // DIRB HIGH
      break;
  }
}

// Stop DC motor completely
void stopDCMotor() {
  OCR1B = 0;  // PWM to 0
  PORTB &= ~((1 << PORTB2) | (1 << PORTB3)); // Both direction pins LOW
}

// Command to number mapping for IR remote
int getDigitFromCommand(uint8_t command) {
  switch (command) {
    case 0x16: return 0;
    case 0x0C: return 1;
    case 0x18: return 2;
    case 0x5E: return 3;
    case 0x08: return 4;
    case 0x1C: return 5;
    case 0x5A: return 6;
    case 0x42: return 7;
    case 0x52: return 8;
    case 0x4A: return 9;
    default: return -1;
  }
}

// Fire card using optimized DC motor control
void fireCard() {
  // Rapid sequence for card launching
  setDCMotor(230, 0);  // Stop with high power
  delay(250);
  
  setDCMotor(230, 1);  // Forward at high speed
  delay(100);
  
  setDCMotor(230, 2);  // Reverse for card release
  delay(120);
  
  stopDCMotor();       // Complete stop
}

// Optimized stepper rotation using direct control
void rotateSteps(long steps) {
  bool direction = (steps >= 0);
  long absSteps = abs(steps);
  
  setStepperDirection(direction);
  
  // Calculate delay for desired speed (microseconds between steps)
  unsigned long stepDelay = 1000; // 1ms = 1000 steps/sec
  
  for (long i = 0; i < absSteps; i++) {
    stepPulse();
    delayMicroseconds(stepDelay);
  }
  
  currentPosition += steps;
}

// Rotate to specific player position with optimization
void rotateToPlayer(int player) {
  long distance = (player - currentPlayer) * stepsPerPlayer;

  Serial.print("Current position: ");
  Serial.println(stepper.currentPosition());

  // Optimize rotation direction (shortest path)
  if (distance < -1 * totalRotationSteps / 2) {
    distance = totalRotationSteps + distance;
  } else if (distance > totalRotationSteps / 2) {
    distance = -1 * totalRotationSteps + distance;
  }

  // Use AccelStepper for smooth movement (hybrid approach)
  stepper.move(distance);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  Serial.print("Rotated ");
  Serial.println(distance);
  Serial.print("Current position: ");
  Serial.println(stepper.currentPosition());
  stepper.setCurrentPosition(0);
  Serial.print("Rotated to player ");
  Serial.println(player);
  currentPlayer = player;
}

// Reset all process variables and hardware state
void resetProcess() {
  numPlayers = 0;
  cardsToDeal = 0;
  currentTarget = 0;
  selectingPlayers = true;
  selectingCards = false;
  dealing = false;
  waitingForPlayerSelection = false;
  currentPosition = 0;
  stepper.setCurrentPosition(0);
  
  // Reset hardware to initial state
  stopDCMotor();
  
  lcd.clear();
  lcd.print("Select players:");
  Serial.println("Process reset");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize LCD display
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select players:");

  // Initialize IR receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  // Initialize GPIO using direct register control
  initGPIO();

  // Configure stepper motor parameters
  float rpm = 500;
  float stepsPerRev = 200 * 8; // stepsPerRevolution * microstepSetting
  float speedStepsPerSec = (stepsPerRev * rpm) / 60.0;

  stepper.setMaxSpeed(speedStepsPerSec);
  stepper.setAcceleration(1000);
  stepper.setCurrentPosition(0);

  Serial.println("Card dealer initialized with register control");
}

void loop() {
  if (IrReceiver.decode()) {
    unsigned long now = millis();
    
    // Debounce IR input to prevent multiple triggers
    if (now - lastIRTime > debounceDelay) {
      uint8_t command = IrReceiver.decodedIRData.command;
      lastIRTime = now;

      Serial.print("Command: 0x");
      Serial.println(command, HEX);

      int digit = getDigitFromCommand(command);
      bool isDigit = (digit != -1);

      // Detect reset sequences: 100+ (0x19) or 200+ (0xD)
      if (command == 0x19) {  // 100+
        received100plus = true;
      } else if (command == 0xD) { // 200+
        received200plus = true;
      } else {
        // If both received in any order, reset the system
        if (received100plus && received200plus) {
          resetProcess();
          received100plus = false;
          received200plus = false;
          IrReceiver.resume();
          return;
        }
      }

      // State machine for card dealing process
      if (selectingPlayers) {
        // Player selection phase
        if (isDigit) {
          numPlayers = digit;
          stepsPerPlayer = totalRotationSteps / numPlayers;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Players: ");
          lcd.print(numPlayers);
          lcd.setCursor(0, 1);
          lcd.print("Press PLAY...");
          Serial.print("numPlayers = ");
          Serial.println(numPlayers);
        } else if (command == 0x43 && numPlayers > 0) {  // PLAY button
          selectingPlayers = false;
          selectingCards = true;
          lcd.clear();
          lcd.print("Cards per player:");
        }
      }
      else if (selectingCards) {
        // Card count selection phase
        if (isDigit) {
          cardsToDeal = digit;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Cards/player: ");
          lcd.print(cardsToDeal);
          lcd.setCursor(0, 1);
          lcd.print("Press PLAY...");
        } else if (command == 0x43 && cardsToDeal >= 0) {  // PLAY button
          selectingCards = false;
          dealing = true;
          lcd.clear();
          lcd.print("Dealing...");
          
          // Reset stepper position before dealing
          currentPosition = 0;
          stepper.setCurrentPosition(0);
          
          // Deal cards to each player in sequence
          for (int c = 0; c < cardsToDeal; c++) {
            for (int i = 1; i <= numPlayers; i++) {
              rotateToPlayer(i);
              delay(150);  // Brief pause for positioning
              fireCard();
            }
          }

          dealing = false;
          waitingForPlayerSelection = true;
          lcd.clear();
          lcd.print("Done!");
          lcd.setCursor(0, 1);
          lcd.print("Select Player");
        }
      }
      else if (waitingForPlayerSelection) {
        // Manual player selection phase
        if (isDigit && digit <= numPlayers) {
          currentTarget = digit;
          rotateToPlayer(currentTarget);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Player ");
          lcd.print(currentTarget);
          lcd.setCursor(0, 1);
          lcd.print("Press PLAY...");
        } else if (command == 0x43 && currentTarget > 0) {  // PLAY button
          fireCard();  // Fire additional card to selected player
        }
      }
    }
    IrReceiver.resume();
  }
}
