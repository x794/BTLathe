// common:
  HardwareSerial SerialX(1);
  HardwareSerial SerialY(2);
  #include <iostream>
  #include <sstream>
  #include <iomanip>
  #define RESOLUTION 5000
  const uint32_t band = 38400;
  bool flagAbort;
  bool flagDebug = false;

// shaft and machine parametres
  int32_t radiusBorder = 262;
  int32_t radiusShaft = 252;
  int32_t radiusSlot = 120;
  int32_t radiusTail = 140;
  int32_t slice = 2;    // layer thickness
  int32_t layers = 5;       // layers per approach

  int32_t widthSlot = 44;
  int32_t widthGap = 5;
  int32_t slots = 92;

  int32_t borderLeft = 400;
  int32_t borderRight = 5050;
  int32_t center = 2500;  // point of xStart
  int32_t leftSlotCenter; // calculated in constructor of Cutter

// axes:
  const uint8_t pinTxY = 18;
  const uint8_t pinRxY = 5;
  const uint8_t pinTxX = 17;
  const uint8_t pinRxX = 16;
  const int32_t dirX = -1;    // rise on counterclock wise tunrinig
  const int32_t dirY = 1;     // rise on clock wise turning
  const uint32_t powerX = 4 * 32; // 4 (pulses per 1 tenth) * 32 (microstep)
  const uint32_t powerY = 4 * 32;
  const uint32_t releaseX = 100;  // version of servo57c
  const uint32_t releaseY = 102;

// BT:
  #include "BluetoothSerial.h"
  #define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
  const char *pin = "4166"; // Change this to more secure PIN.
  String device_name = "BTLathe";
  BluetoothSerial SerialBT;

// common structs, functions, classes:
struct commandStruct {
  uint8_t driverIndex;
  uint8_t function;
  uint8_t dataByte;
  int32_t data;
};

class BT {
  public:
    BT() {
    }
    std::string pull() {
      std::string result;
      char c;
      while (SerialBT.available()) {
        c = SerialBT.read();
        if (c != '\n' && c != '\r') // CR = 0x0D = 13 = \r;  LF = 0x0A = 10 = \n
          result.push_back(c);
        delay(2);
      }
      return result;
    }
    void commit(std::string tx_) {
      tx.append(tx_);
    }
    void push(std::string tx_) {
      tx.append(tx_);
      push();
    }
    void push() {
      for (int i = 0; i < tx.size(); i++) {
        SerialBT.write(tx[i]);
        delay(2);
      }
      tx.erase();
    }
  private:
    std::string tx;
};
BT bt{};

bool inRange(int32_t item, int32_t min, int32_t max) {
  if (flagDebug) {
    bt.push("\n" + std::to_string(min) + "<" + std::to_string(item) + "<" + std::to_string(max));
    delay(500);
  }
  return (min <= item && item <= max);
}

class Driver57 {
  public:
    Driver57(uint8_t axisIndex) {
      if (axisIndex == 1) {
        SerialX.begin(band, SERIAL_8N1, pinRxX, pinTxX);
        SerialServo = &SerialX;
      }
      if (axisIndex == 2) {
        SerialY.begin(band, SERIAL_8N1, pinRxY, pinTxY);
        SerialServo = &SerialY;
      }
    }
    bool pull(struct commandStruct &answer) {
      bool result = false;
      uint8_t bytecode[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      if (readBytecode(bytecode)) {
        answer.driverIndex = bytecode[1];
        answer.function = bytecode[2];
        int64_t buf64;
        switch (answer.function) {          
          case 0x30:
            answer.dataByte = 0;
            buf64 = (((bytecode[5] * 0x100) + bytecode[6]) * 0x40 + bytecode[7]) * 0x100 + bytecode[8];
            buf64 = (buf64 < 0x20000000) ? buf64 : buf64 - 0x40000000;
            // set one loop RESOLUTION
            buf64 *= RESOLUTION;
            buf64 /= 0x4000;
            answer.data = (int32_t) buf64;
            break;
          case 0x33:
            answer.dataByte = 0;
            answer.data = (((bytecode[3] * 0x100) + bytecode[4]) * 0x100 + bytecode[5]) * 0x100 + bytecode[6];
            break;
          case 0x39:
            answer.dataByte = 0;
            answer.data = bytecode[3] * 0x100 + bytecode[4];
            answer.data = (answer.data < 0x8000) ? answer.data : answer.data - 0x10000;
            // set one loop RESOLUTION
            answer.data *= RESOLUTION;
            answer.data /= 0x10000;
            break;
          default:
            answer.dataByte = (uint8_t) bytecode[3];
            answer.data = 0;
        }
        result = true;
      }
      if (flagDebug) {
        bt.push("\n      get       " + std::to_string(answer.driverIndex) + 
                                 " " + std::to_string(answer.function) + 
                                 " " + std::to_string(answer.dataByte) + 
                                 " " + std::to_string(answer.data) +
                "\n      bCode");
        for (int i = 0; i < 10; i++) bt.push(" " + std::to_string(bytecode[i]));
      }
      return result;
    }
    void push(struct commandStruct command) {
      uint8_t bytecode[10];
      uint8_t bytecodeSize = 0;
      bytecode[0] = 0xFA;
      bytecode[1] = command.driverIndex;
      bytecode[2] = command.function;
      
      switch (bytecode[2]) {
        case 0x30: case 0x33: case 0x39: case 0x3A: case 0x3B: case 0x3D: case 0x3E:
          bytecode[3] = getCRC(bytecode, 3);
          bytecodeSize = 4;
          break;
        case 0x80: case 0x82: case 0x83: case 0x84: case 0x85: case 0x86: case 0x88: case 0x89:
        case 0x8A: case 0x8B: case 0x3F: case 0xF3: case 0xF6: case 0xF7: case 0xFF:
          bytecode[3] = command.dataByte;
          bytecode[4] = getCRC(bytecode, 4);
          bytecodeSize = 5;
          break;
        case 0xFD:
          bytecode[3] = command.dataByte;
          bytecode[4] = (command.data % 0x100000000) / 0x1000000;
          bytecode[5] = (command.data % 0x1000000) / 0x10000;
          bytecode[6] = (command.data % 0x10000) / 0x100;
          bytecode[7] = (command.data % 0x100) / 0x1;          
          bytecode[8] = getCRC(bytecode, 8);
          bytecodeSize = 9;
          break;
      }
      SerialServo -> write(bytecode, bytecodeSize);
      if (flagDebug) {
        bt.push("\n      send      " + std::to_string(command.driverIndex) + 
                                 " " + std::to_string(command.function) + 
                                 " " + std::to_string(command.dataByte) + 
                                 " " + std::to_string(command.data) +
                "\n      bCode");
        for (int i = 0; i < bytecodeSize; i++) bt.push(" " + std::to_string(bytecode[i]));
      }
    }
    void push(uint8_t* bytecode) {
      bytecode[bytecode[0]] = getCRC(&bytecode[1], bytecode[0] - 1); // pass array size of bytecode[0] started from adress &bytecode[1]
      SerialServo -> write(&bytecode[1], bytecode[0]);
      if (flagDebug) {
        bt.push("\n      send      ");
        Serial.print("        send      ");
        for (int i = 1; i <= bytecode[0]; i++) {
          bt.push(" " + std::to_string(bytecode[i]));
          Serial.print(" " + (String) (int) bytecode[i]);
        }
        Serial.println();
      }
    }

  private:
    HardwareSerial *SerialServo;
    bool readBytecode(uint8_t bytecode[10]) { // return true if successfull read, check, put bytecode
      if (flagDebug) bt.push("\n      _read");
      uint8_t bytecodeSize = readNextByteRecursive(bytecode, 0, 9);
      return (bytecodeSize > 0);
    }
    uint8_t readNextByteRecursive(uint8_t bytecode[10], int i, int indexCRC) {  // get target bytecode array, index of current uint8_t to read, size of array to read
      uint32_t waiter = 0;
      while ((waiter++ < 1000) && !SerialServo -> available())  // wait next byte
        delayMicroseconds(100);
      if (SerialServo -> available()) {
        uint8_t incoming = SerialServo -> read();
        if (flagDebug) bt.push(" " + std::to_string(incoming));
        bytecode[i] = incoming;
        if (i == 0 && incoming == 0xFB)     // continue the reading only if flag 0xFB catched
          return readNextByteRecursive(bytecode, i + 1, indexCRC);
        else if (i == 1 && (incoming == 0x01 || incoming == 0x02)) // continue the reading only if motors number < 3
          return readNextByteRecursive(bytecode, i + 1, indexCRC);
        else if (i == 2) {                  // continue the reading only if correct command detected
          if (incoming == 0x30)
            return readNextByteRecursive(bytecode, i + 1, 9); // 3+6, send indexCRC = 9
          else if (incoming == 0x33)
            return readNextByteRecursive(bytecode, i + 1, 7); // 3+4, send indexCRC = 7
          else if (incoming == 0x39)
            return readNextByteRecursive(bytecode, i + 1, 5); // 3+2, send indexCRC = 5
          else if (incoming >= 0x3A || incoming == 0x3B || incoming == 0x3D || incoming == 0x3E ||
                   incoming == 0x80 || incoming == 0x82 || incoming == 0x83 || incoming == 0x84 ||
                   incoming == 0x85 || incoming == 0x86 || incoming == 0x88 || incoming == 0x89 ||
                   incoming == 0x8A || incoming == 0x8B || incoming == 0x3F || incoming == 0xF3 ||
                   incoming == 0xF6 || incoming == 0xF7 || incoming == 0xFF || incoming == 0xFD )
            return readNextByteRecursive(bytecode, i + 1, 4); // 3+1, send indexCRC = 4
        }
        else if (i > 2 && i < indexCRC)          // continue the reading only if the expected data is receiving
          return readNextByteRecursive(bytecode, i + 1, indexCRC);
        else if (i == indexCRC && checkCRC(bytecode, indexCRC)) // it is the last recursive itteration - it checks the CRC
            return i + 1;
      }
      if (flagDebug) bt.push(" noise detected");
      return 0;                   // if correct command was not detected - return bytecodeSize = 0
    }
    uint8_t getCRC(uint8_t bytecode[10], int indexCRC) {
      uint8_t crc = 0;
      for (int i = 0; i < indexCRC; i++)
        crc += bytecode[i];
      return crc;
    }
    bool checkCRC(uint8_t bytecode[10], int indexCRC) {
      uint8_t crc = 0;
      for (int i = 0; i < indexCRC; i++)
        crc += bytecode[i];
      bool result = (crc == bytecode[indexCRC]);
      return (result);
    }
    void reportOutput(String direction, uint8_t* bytecode, uint8_t bytecodeSize) {
      Serial.print(direction);
      for (int i = 0; i < 10; i++) {
        Serial.print((bytecodeSize == i) ? "." : " ");
        Serial.print(bytecode[i], HEX);
      }
      Serial.println();
    }
};

class Axis {
  public:
    Axis(Driver57& driver_, uint8_t axisIndex_, int32_t axisDir_, int32_t power_, uint32_t release_, int32_t min_, int32_t max_, int32_t current_) 
             : driver{driver_}, axisIndex{axisIndex_}, axisName(axisIndex_ == 1 ? "x" : "y"), axisDir{axisDir_}, power{power_}, release{release_}, min{min_}, max{max_}, current{current_} {
      baseLoopsPerMinute = (release == 100) ? 150 : 30;
    }
    int32_t getServoCurrent() {
      struct commandStruct answer;
      driver.push(commandStruct {axisIndex, 0x30, 0, 0});
      while (!(driver.pull(answer) && answer.driverIndex == axisIndex && answer.function == 0x30));
      return servoShift + (axisDir * answer.data);
    }
    int32_t getServoError() { // error == 4369  ~  1mm on 5mm step shaft. Return error in range (-10922, 10923)
      struct commandStruct answer;
      driver.push(commandStruct {axisIndex, 0x39, 0, 0});
      while (!(driver.pull(answer) && answer.driverIndex == axisIndex && answer.function == 0x39));
      return answer.data;
    }
    void setServoShift() {    // rarely used precision function
      bt.push("\n" + axisName + " calibrating...");
      waitOutServoErrors(5);
      int32_t servoCurrent = getServoCurrent();
      servoShift = current * 100 - servoCurrent;
      bt.push("\n    current       " + std::to_string(current));
      bt.push("\n    current*dimen " + std::to_string(current * 100));
      bt.push("\n    servoCurrent  " + std::to_string(servoCurrent));
      bt.push("\n    servoShift    " + std::to_string(servoShift));
      bt.push("\n" + axisName + " calibrated successful\n");
    }
    void waitGoConfirm(uint8_t order) {
      if (flagDebug) bt.push("\n  waitGoConfirm() started " + axisName);
      struct commandStruct answer;
      while (!(driver.pull(answer) && answer.driverIndex == axisIndex    // try to read driver. In success case driver put answer and return true
                                   && answer.function    == 0xFD
                                   && answer.dataByte    == order))
        if (flagDebug) {
          bt.push(".");
          delay(500);
        }
      if (flagDebug) bt.push("finished");
    }
    void waitOutServoRollUp(int32_t target, int32_t tolerance) {
      if (flagDebug) bt.push("\n  waitOutServoRollUp() started " + axisName);
      target *= 100;
      uint32_t start = millis();
      while (!inRange(getServoCurrent(), target - tolerance, target + tolerance)) {
        if (flagDebug) delay(500);
        bt.push(">");
        delay(100);
      }
      if (flagDebug) bt.push(" finished");
    }
    void waitOutServoErrors(int32_t tolerance) {
      if (flagDebug) bt.push("\n  waitOutServoErrors() started " + axisName);
      while (!inRange(getServoError(), -tolerance, tolerance)) {
        if (flagDebug) delay(500);
        bt.push("!");
        delay(100);
      }
      if (flagDebug) bt.push(" finished");
    }
    void go(int32_t speed, int32_t distance, bool lockCurrent) {
      bt.push(axisName + ' ');
      // reject trivial requests
        if (distance == 0) {
          bt.push("reject trivial go\n");
          return;
        }
      // adjust parameters
        int pulses = abs(distance * power);
        int direction = (distance > 0) ? 1 : -1;
      // report BT about go
        std::stringstream stream;
        stream << current << " -> " << current + distance << " s" << speed << " ";
        bt.push(stream.str());
      // request go function depend of driver version
        if (release == 100) goRequest_v100(direction, speed, pulses);
        else                goRequest_v101(direction, speed, pulses);
      // wait go() confirm
        waitGoConfirm(1);
        waitGoConfirm(2);
      // wait the support roll up to the goal
        waitOutServoRollUp(current + distance, 25); // second parameter - specified in microns
      // turn current or servoShift
        if (lockCurrent)
          servoShift -= (distance * 100);
        else
          current += distance;
        bt.push(" ok\n");
    }    
    int32_t getCurrent() {
      return current;
    }
    int32_t getMin() {
      return min;
    }
    int32_t getMax() {
      return max;
    }
  private:
    Driver57 &driver;
    uint8_t axisIndex;   // used to mark rs485 commands
    std::string axisName = "defaultAxisName";
    int32_t axisDir;
    int32_t power;        // quantity of pulses per one tenth (to shaft and motor where 5mm shift per 200 pulses power = 4 * microstep)
    uint32_t release;
    int32_t min;
    int32_t max;
    int32_t current;     // current == servo * power
    int32_t servoShift = 0; // firstival calc of servoShift founds to this 0
    int32_t baseLoopsPerMinute;
    void goRequest_v100(int direction, int speed, int pulses) {
      // request go() to driver
        if (flagDebug) {
          bt.push("\n goRequest_v100 started");
          bt.push("\n direction = " + std::to_string(direction));
          bt.push("\n speed     = " + std::to_string(speed));
          bt.push("\n pulses    = " + std::to_string(pulses));
        }
        speed = (speed + 4) / 5;
        speed = (speed < 1) ? 1 : speed;
        uint8_t bytecode[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        bytecode[0] = 0x09; // 0x0B == 11   // element 0 point the size of sended bytecode (without the bytecode[0])
        bytecode[1] = 0xFA; // marks bytecode like sending
        bytecode[2] = axisIndex;
        bytecode[3] = 0xFD; // command to turn the motor
        bytecode[4] = (direction * axisDir == 1) ? 0b10000000 : 0b00000000; // B4b7 - direction
        if (flagDebug) bt.push("\n bc4 = " + std::to_string(bytecode[4]));
        bytecode[4]+= speed % 0x80;  // B4b6-b0 - speed value
        if (flagDebug) bt.push("\n bc4 = " + std::to_string(bytecode[4]));
        bytecode[5] = (pulses % 0x100000000) / 0x1000000;
        bytecode[6] = (pulses % 0x1000000) / 0x10000;
        bytecode[7] = (pulses % 0x10000) / 0x100;
        bytecode[8] = (pulses % 0x100) / 0x1;          
        driver.push(bytecode);
        if (flagDebug) bt.push("\n goRequest_v100 finished ");
    }
    void goRequest_v101(int direction, int speed, int pulses) {
      // request go() to driver
        if (flagDebug) {
          bt.push("\n goRequest_v101 started");
          bt.push("\n direction = " + std::to_string(direction));
          bt.push("\n speed     = " + std::to_string(speed));
          bt.push("\n pulses    = " + std::to_string(pulses));
        }
        speed = (speed < 1) ? 1 : speed;
        uint8_t bytecode[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        bytecode[0]  = 0x0B; // 0x0B == 11   // element 0 point the size of sended bytecode (without the bytecode[0])
        bytecode[1]  = 0xFA; // marks bytecode like sending
        bytecode[2]  = axisIndex;
        bytecode[3]  = 0xFD; // command to turn the motor
        bytecode[4]  = (direction * axisDir == 1) ? 0b10000000 : 0b00000000; // B4b7 - direction
        if (flagDebug) bt.push("\n bc4 = " + std::to_string(bytecode[4]));
        bytecode[4] += speed / 0x100;  // B4b3-b0 - high section of speed value
        if (flagDebug) bt.push("\n bc4 = " + std::to_string(bytecode[4]));
        bytecode[5]  = speed % 0x100;  // B4b7-b0 - low  section of speed value
        bytecode[6]  = 0x20; // B6 - acceleration; range from 0x00 (no acceleration) to 0x20
        bytecode[7]  = (pulses % 0x100000000) / 0x1000000;
        bytecode[8]  = (pulses % 0x1000000) / 0x10000;
        bytecode[9]  = (pulses % 0x10000) / 0x100;
        bytecode[10] = (pulses % 0x100) / 0x1;          
        driver.push(bytecode);
        if (flagDebug) bt.push("\n goRequest_v101 finished ");
    }
};

class Cutter {
  public:
    Cutter (Axis &x_, Axis &y_) : x{x_}, y{y_} {
      leftSlotCenter = center - widthSlot * (slots / 2);
      leftSlotCenter += (slots % 2 == 0) ? (widthSlot / 2) : 0;
    }
    void checkBT() {
      if (!SerialBT.available())  return;
      char const* digits = "0123456789";
      char const* directions = "xylrio";
      char const* lockedDirections = "XYLRIO";
      // pull BT
        std::string rx = bt.pull();
        Serial.println("checkBT got <" + (String) rx.data() + ">");
      // check onechar commands:
        if (rx.length() == 1) {
          if (rx[0] == 'a') { takeAlarm();      return; }
          if (rx[0] == 'p') { takePause();      return; }
          if (rx[0] == 'A') { flagAbort = true; bt.push("\nProgram aborted");   return; }
        }
      // check threechar commands:
        if (rx.length() == 3) {
          if (rx[0] == 'h' && rx.find_first_of(digits) == 1 && rx.find_last_of(digits) == 2) {
            int32_t initialSlot = std::atoi(rx.substr(1, 2).c_str());
            halfShaft(initialSlot);
            return;
          }
        }
      // check multichar commands:
        else {
          size_t speedPosition = rx.find_first_of(digits);
          size_t dirPosition = rx.find_first_of(directions);
          // check lockCurrent:
            size_t lockedDirPosition = rx.find_first_of(lockedDirections);
            bool lockCurrent = false;
            if (lockedDirPosition != std::string::npos) {
              lockCurrent = true;
              dirPosition = lockedDirPosition;
            }
          size_t targetPosition = rx.find_first_of(digits, dirPosition);
          // check correct input:
            if (speedPosition   != std::string::npos &&
                dirPosition     != std::string::npos &&
                targetPosition  != std::string::npos &&
                speedPosition < dirPosition &&
                dirPosition   < targetPosition) {
              // calc other parameters:
                int32_t speed = std::atoi(rx.substr(speedPosition, dirPosition - speedPosition).c_str());
                uint8_t direction = rx[dirPosition];
                int32_t target = std::atoi(rx.substr(targetPosition, rx.length() - targetPosition).c_str());
              // call go() - turn motor and report to BT
                go(direction, speed, target, lockCurrent);
                return;
            }
        }
        bt.push((std::string) " Command unrecognized\n");
    }
  private:
    Axis &x;
    Axis &y;

    void go(char dir, int32_t speed, int32_t target) {  // call unlocked go(). Intended not for manual commands!!!
      if (flagAbort) return;
      checkBT();
      go(dir, speed, target, false);
    }
    void go(char dir, int32_t speed, int32_t target, bool lockCurrent) { // 230404 edjast target, edjast if target is out of border, call Axis.go(...)
      // break in case of zero distance:
        if (target == 0)  return;
      // set new pointer, min and max limits to selected axis; or break in case of the erroneous direction:
        Axis* axis;
        Axis* axis2;
        if (dir == 'x' || dir == 'l' || dir == 'r' || dir == 'X' || dir == 'L' || dir == 'R')  {  axis = &x;  axis2 = &y;  }
        if (dir == 'y' || dir == 'i' || dir == 'o' || dir == 'Y' || dir == 'I' || dir == 'O')  {  axis = &y;  axis2 = &x;  }
      // calculate target in case relative coordinate
        int32_t distance = target;
        if (dir == 'r' || dir == 'o' || dir == 'R' || dir == 'O')  target = axis -> getCurrent() + distance;
        if (dir == 'l' || dir == 'i' || dir == 'L' || dir == 'I')  target = axis -> getCurrent() - distance;
      // adjust target if out of borders:
        target = min((axis -> getMax()), target);
        target = max((axis -> getMin()), target);
      // call adjusted axis.go()
        distance = target - axis -> getCurrent();
        axis -> go(speed, distance, lockCurrent);
      // wait finishing
        axis -> waitOutServoErrors(25);
        axis2-> waitOutServoErrors(25);
        // bt.push(getCurrentStr());
        // bt.push(getServoCurrentStr());
        // bt.push("\n");
    }
    void halfShaft(int32_t initSlot) {  // h - slice right shaft half (slice and make corner). Cut with 4mm cutter
      int32_t zeroSlotX = x.getCurrent();
      int32_t shiftSlot = 45;
      int32_t slotsNumber = 45;
      int32_t slotBorder = 255;
      int32_t slotRadius = 120;
      int32_t cornerLeft = (slotsNumber + 1) * shiftSlot + 5;
      int32_t cornerRight = 5050;
      int32_t cornerRadius = 150;
      int32_t layer = 5;

      // slice shaft
      go('y', 200, slotBorder);
      for (int i = initSlot; i <= slotsNumber; i++) {
        go('x', 200, (i * shiftSlot) + zeroSlotX);
        go('y',   8, slotRadius + 5);
        go('y',   2, slotRadius);
        delay(2000);
        go('y', 200, slotBorder);
      }
      
      // make corner
      int32_t currentY = 252;
      go('x', 400, cornerRight);
      for (int i = 0; i < 10; i++) {
        go('y',  5, currentY - 5);
        go('x', 20, cornerLeft);
        go('y',  5, currentY - 10);
        go('x', 20, cornerRight);
      }
      go('y', 200, slotBorder);

      // finalyse corner
        go('x', 200, cornerLeft - 2);
        go('y',   5, cornerRadius);   delay(1000);
        go('x',  10, cornerRight);
        go('y', 200, slotBorder);

      // go to start next half shaft
        go('x', 400, zeroSlotX - 5);
        go('x', 200, zeroSlotX);
    }
    void takeAlarm() {
      // remember current position:
        bt.commit(getCurrentStr());   // don't push to BT in this line for the immediate cutter removal 
        int32_t memoryY = y.getCurrent();
      // unroll and wait next BT command:
        go('y', 400, radiusBorder);
        bt.push("\nALARM !!! on Y = " + memoryY);
        waitAbortOrReleaseInput();
        if (flagAbort = false)
          go('y', 10, memoryY);
    }
    void takePause() {
      bt.push("\nPAUSE");
      waitAbortOrReleaseInput();
    }
    void waitAbortOrReleaseInput() {
      bt.push("\nCurrent: " + getCurrentStr());
      bt.push("\nPush 'A' to abort or '0' to continue");
      while (!SerialBT.available())  delay(100);
      checkBT();
      if (flagAbort = false)
        bt.push("\nContinue program executing");
      else
        bt.push("\nProgram aborted!");
      bt.push("\nCurrent: " + getCurrentStr());
    }
    std::string getCurrentStr() {
      std::stringstream stream;
      // stream << "\n  100 x current " << std::right << std::setfill('0') << std::setw(6) << x.getCurrent() * 100 << "_"
      //                              << std::setfill('0') << std::setw(6) << y.getCurrent() * 100;
      return stream.str();
    }
    std::string getServoCurrentStr() {
      std::stringstream stream;
      // stream << "\n  servo current " << std::right << std::setfill('0') << std::setw(6) << x.getServoCurrent() << "_"
      //                              << std::setfill('0') << std::setw(6) << y.getServoCurrent();
      return stream.str();
    }
};

// objects:
  Driver57 driverX{1};
  Driver57 driverY{2};
  Axis x{driverX, 0x01, dirX, powerX, releaseX, borderLeft, borderRight, center};
  Axis y{driverY, 0x02, dirY, powerY, releaseY, radiusSlot, radiusBorder, radiusBorder};
  Cutter cutter{x, y};

void setup() {
  // BT init:
    Serial.begin(115200);
    SerialBT.begin(device_name);
    Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
    #ifdef USE_PIN
      SerialBT.setPin(pin);
      Serial.println("Using PIN");
    #endif

  delay(5000);
  x.setServoShift();
  y.setServoShift();
  bt.push("\nBTLathe ready to execute commands\n");
}

void loop() {
  delay(100);
  flagAbort = false;
  cutter.checkBT();
}