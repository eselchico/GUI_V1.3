#include <TMCStepper.h>

//pins for the board
#define DIRECTION 3
#define STEP 4
#define EN 5
#define LS1 21//far LS
#define LS2 20//close LS
#define ES 19//estop pin

// --- TMC2209 UART ---
//#define SERIAL_PORT     Serial1       // Use Serial1 (pins 18/19 on Mega, or SoftwareSerial on Uno)
SoftwareSerial SERIAL_PORT(8, 9); //Rx, Tx for the driver
#define DRIVER_ADDRESS  0b00          // MS1 & MS2 tied to GND
#define R_SENSE         0.10f

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

//AccelStepper motor1(1, STEP, DIRECTION);
int noOfSteps = 1600;
int microSecondsDelay = 50;
bool dir;

//array for communication
word received = 0;
uint8_t hi = 0;
uint8_t low = 0;
bool bit0 = 0; //should I turn this into an array instead?
bool bit1 = 0;
bool bit2 = 0;
bool bit3 = 0;
bool receivedBits[8];

const int maxLength = 3;
volatile byte safety = 0;
byte pos = 0;
long int stepper[2] = {0, 0}; //current value, max value, with a min of 0
int steps_per_mm = 0;
int delta = 0;

void setup() {

  Serial.begin(115200);
  SERIAL_PORT.begin(31250);          // UART to TMC2209

  // put your setup code here, to run once:
  pinMode(DIRECTION, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(LS1, INPUT_PULLUP);
  pinMode(LS2, INPUT_PULLUP);
  pinMode(ES, INPUT_PULLUP);


  digitalWrite(EN, LOW);

  // --- TMC2209 Init via UART ---
  driver.begin();
  delay(100);
  driver.toff(5);
  driver.blank_time(24);
  driver.rms_current(800);            // Set to your motor's rated current (mA)
  driver.microsteps(2);               // 1/2 microstepping
  driver.en_spreadCycle(false);       // StealthChop (quiet mode)
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  driver.semin(0);                    // Disable CoolStep

  // Confirm UART comms
  uint16_t ms = driver.microsteps();
  Serial.print("Microsteps: 1/");
  Serial.println(ms);
  Serial.print("Driver version: ");
  Serial.println(driver.version());

  //IO interrupts
  attachInterrupt(digitalPinToInterrupt(ES), updateIO, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LS1), updateIO, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LS2), updateIO, CHANGE);

  //start up values
  safety |= (digitalRead(ES));
  safety |= (digitalRead(LS1) << 1);
  safety |= (digitalRead(LS2) << 2);


}

void loop() {

  if (safety & 1) {
    bit0 = 0;
    bit1 = 0;
    bit2 = 0;
    bit3 = 0;
  }

  // --- Receive 3 bits from Python ---
  if (Serial.available() >= 3) {
    if (Serial.read() == 0xFF) {
      hi = Serial.read();
      low = Serial.read();
      received = (hi << 8) | low;
      bit0 = (received >> 8) & 1;
      bit1 = (received >> 9) & 1;
      bit2 = (received >> 10) & 1;
      bit3 = (received >> 11) & 1;
    }
  }

  pos = steps_per_mm != 0 ? stepper[0] / steps_per_mm : 0;
  Serial.write(0xFF);
  Serial.write(safety);
  Serial.write(pos);

  //when ready include && ~sent & 1 for estop
  while ((bit0) && !(safety & 1)) {
    //reading inputs from computer, if button is held move until the stop is sent
    if (Serial.available() >= 3) {
      while (Serial.available() > 4) Serial.read(); // trying to constantly empty out the buffer until there is just the number and the \r
      if (Serial.read() == 0xFF) {
        hi = Serial.read();
        low = Serial.read();
        received = (hi << 8) | low;

        //for some reason there is always a carriage return
        //received = 10, so just ignore that
        if (low != 10) {
          bit0 = (received >> 8) & 1;
          bit1 = (received >> 9) & 1;
          bit2 = (received >> 10) & 1;
          bit3 = (received >> 11) & 1;
        }
      }
    }
    if (bit0 == 0)
      break;
    if (bit1 == 0) //&& sent >> 1 & 1, LS1
      dir = true;
    else if (bit1 == 1)//&& sent >> 2 & 1, LS2
      dir = false;

    if ((dir && (safety >> 1 & 1)) || (!dir && (safety >> 2 & 1)))
      dirMove(dir);
    else
      bit0 = 0;

    //python reads one byte at a time
    //maybe this just wants to write and otherwise it gets mad? FUCK THIS SHIT
    pos = steps_per_mm != 0 ? stepper[0] / steps_per_mm : 0;
    Serial.write(0xFF);
    Serial.write(safety);
    Serial.write(pos);//don't ask why this does what it does but it fucking makes it work
  }

  if (bit2 && !(safety & 1)) {
    homing(stepper);
  }

  //moving to a specific point
  if (bit3 && !(safety & 1) && (safety >> 3 & 1)) {
    delta = low - pos;
    dir = true;
    if (delta > 0) dir = false;
    long int distance_to_move = abs(long(delta) * 700);
    moveSteps(distance_to_move, dir, stepper);
  }
}

//when having to move a certain amount
//steps should be found before
void moveSteps(long int steps, bool stepper_dir, long int* current) {
  for (long int i = 0; i < steps; i++) {
    if (safety & 1)
      break;

    dirMove(stepper_dir);
    if (stepper_dir) {
      if (!(safety >> 1 & 1) || current[0] < 0) break;
      current[0]--;
    }
    else {
      if (!(safety >> 2 & 1) || current[0] > current[1]) break;
      current[0]++;
    }

    uint8_t location = current[0] / steps_per_mm;
    Serial.write(0xFF);
    Serial.write(safety);
    Serial.write(abs(location));
  }
}

//should I adjust the distance value in here?
void dirMove(bool stepper_dir, bool stepper_homed, int* current) {

  if ((dir && !(safety >> 1 & 1)) || (!dir && !(safety >> 2 & 1)) || (safety & 1))
    return;
  if (current[0] > 0 && current[0] <= current[1]) return;

  if (stepper_dir) {
    digitalWrite(DIRECTION, HIGH);
    if (stepper_homed) current[0]--;
  }
  else {
    digitalWrite(DIRECTION, LOW);
    if (stepper_homed) current[0]++; //make sure it doesn't go below 0?
  }

  digitalWrite(STEP, HIGH);
  delayMicroseconds(microSecondsDelay);
  digitalWrite(STEP, LOW);
  delayMicroseconds(microSecondsDelay);
}

//this one won't modify the distance and is just for homing
void dirMove(bool stepper_dir) {

  if ((stepper_dir && !(safety >> 1 & 1)) || (!stepper_dir && !(safety >> 2 & 1)) || (safety & 1))
    return;

  if (stepper_dir) {
    digitalWrite(DIRECTION, HIGH);
  }
  else {
    digitalWrite(DIRECTION, LOW);
  }

  digitalWrite(STEP, HIGH);
  delayMicroseconds(microSecondsDelay);
  digitalWrite(STEP, LOW);
  delayMicroseconds(microSecondsDelay);
}



//make a homing function u slagbag pls, want put values into an arraay for max and min?
//will always move to the right first, and set that as the starting value, move to the other side and make that the max keeping track of each step taken
void homing(long int* values) {
  bool stepper_dir = true;
  values[0] = 0;
  values[1] = 0;

  while (safety >> 1 & 1 && stepper_dir && !(safety & 1)) {
    dirMove(stepper_dir);
    Serial.write(0xFF);
    Serial.write(safety);
    Serial.write(0);
  }
  stepper_dir = !stepper_dir;
  delay(10);

  //why does this move so slow?
  while (safety >> 2 & 1 && !stepper_dir && !(safety & 1)) {
    values[1]++;
    dirMove(stepper_dir);
    Serial.write(0xFF);
    Serial.write(safety);
    Serial.write(0);
  }

  //passes distance to the array
  steps_per_mm = values[1] / 160;
  values[0] = values[1];//current
  safety |= 1 << 3; //tells python that it has been homed
  Serial.write(0xFF);
  Serial.write(safety);
  Serial.write(160);
  delay(10);
  moveSteps(values[1] / 2, true, values);

}

void updateIO() {
  bool switches[maxLength];
  switches[0] = digitalRead(ES); //bit0
  switches[1] = digitalRead(LS1); //bit1
  switches[2] = digitalRead(LS2); //bit2

  for (int i = 0; i < 3; i++) {
    if (switches[i] == 1)
      safety = safety | (1 << i);
    else if (switches[i] == 0)
      safety = safety & ~(1 << i);
  }
}

//for recieving input across serial
//the array will take up to -1 elements for some reason idk why but thats the only way it works
//will be passing arrays and updating that

//void recieving(char* text) {
//  static char c = 0;
//  static int index = 0;
//  while (Serial.available() > 0) {
//    //Serial.println(Serial.available());
//
//    c = Serial.read();
//
//    if (index < maxMessage - 1) {
//      text[index++] = c;
//      text[index] = 0;
//    }
//  }
//
//  Serial.println(text);
//  index = 0;
//  c = 0;
//
//}
