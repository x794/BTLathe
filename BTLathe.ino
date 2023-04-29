// common:
  HardwareSerial SerialX(1);
  HardwareSerial SerialY(2);
  #include <iostream>
  #include <sstream>
  #include <iomanip>
  #define RESOLUTION 5000
  const uint32_t band = 38400;
  bool flagAbort;

// shaft and machine parametres
  int32_t radiusBorder = 260;
  int32_t radiusShaft = 252;
  int32_t radiusSlot = 120;
  int32_t radiusTail = 140;
  int32_t slice = 5;

  int32_t widthSlot = 44;
  int32_t widthGap = 5;
  int32_t slots = 92;

  int32_t borderLeft = 400;
  int32_t borderRight = 5000;
  int32_t center = 2500;  // point of xStart
  int32_t leftSlot; // calculated in constructor of Cutter

// axes:
  const uint8_t pinTxY = 18;
  const uint8_t pinRxY = 5;
  const uint8_t pinTxX = 17;
  const uint8_t pinRxX = 16;
  const int32_t dirX = -1;    // rise on counterclock wise tunrinig
  const int32_t dirY = 1;     // rise on clock wise turning
  const uint32_t powerX = 4 * 16; // 4 (pulses per 1 tenth) * 16 (microstep)
  const uint32_t powerY = 4 * 64;
  const uint8_t speed[10] = {0x01, 0x01, 0x02, 0x03, 0x04, 0x08, 0x10, 0x20, 0x40, 0x7F};  // ... + 0x80 -> backward

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
  if (!(min <= item && item <= max))
    bt.push(".");
    // bt.push("\n    " + std::to_string(min) + "<" + std::to_string(item) + "<" + std::to_string(max));
  return (min <= item && item <= max);
}

class Driver57 {
  public:
    Driver57(uint8_t axisNumber_) {
      if (axisNumber_ == 1) {
        SerialX.begin(band, SERIAL_8N1, pinRxX, pinTxX);
        SerialServo = &SerialX;
      }
      if (axisNumber_ == 2) {
        SerialY.begin(band, SERIAL_8N1, pinRxY, pinTxY);
        SerialServo = &SerialY;
      }
    }
    bool pull(struct commandStruct &answer) {
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
        return true;
      }
      return false;
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
    }
  private:
    HardwareSerial *SerialServo;
    bool readBytecode(uint8_t bytecode[10]) { // return true if successfull read, check, put bytecode
      uint8_t bytecodeSize = readNextByteRecursive(bytecode, 0, 9);
      return (bytecodeSize > 0);
    }
    uint8_t readNextByteRecursive(uint8_t bytecode[10], int i, int indexCRC) {  // get target bytecode array, index of current uint8_t to read, size of array to read
      uint32_t waiter = 0;
      while ((waiter++ < 10) && !SerialServo -> available())  // wait next byte
        delayMicroseconds(100);
      if (SerialServo -> available()) {
        uint8_t incoming = SerialServo -> read();
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
    Axis(Driver57& driver_, uint8_t axisNumber_, int32_t dir_, int32_t power_, int32_t min_, int32_t max_, int32_t current_) 
             : driver{driver_}, axisNumber{axisNumber_}, axisName(axisNumber_ == 1 ? "x" : "y"), dir{dir_}, power{power_}, min{min_}, max{max_}, current{current_} {
    }
    int32_t getServoCurrent() {
      struct commandStruct answer;
      driver.push(commandStruct {axisNumber, 0x30, 0, 0});
      while (!(driver.pull(answer) && answer.driverIndex == axisNumber && answer.function == 0x30));
      return servoShift + (dir * answer.data);
    }
    int32_t getServoError() { // error == 4369  ~  1mm on 5mm step shaft. Return error in range (-10922, 10923)
      struct commandStruct answer;
      driver.push(commandStruct {axisNumber, 0x39, 0, 0});
      while (!(driver.pull(answer) && answer.driverIndex == axisNumber && answer.function == 0x39));
      return answer.data;
    }
    void setServoShift() {    // rarely used precision function
      bt.push("\n" + axisName + " calibrating...");
      waitOutServoErrors(0);
      int32_t servoCurrent = getServoCurrent();
      servoShift = current * 100 - servoCurrent;
      bt.push("\n    current       " + std::to_string(current));
      bt.push("\n    current*dimen " + std::to_string(current * 100));
      bt.push("\n    servoCurrent  " + std::to_string(servoCurrent));
      bt.push("\n    servoShift    " + std::to_string(servoShift));
      bt.push("\n" + axisName + " calibrated successful\n");
    }
    void waitGoConfirm(uint8_t order) {
      uint32_t start = millis();
      // bt.push((std::string) "\n  get waitGo(" + std::to_string(order) + ") take ");
      struct commandStruct answer;
      while (!(driver.pull(answer) && answer.driverIndex == axisNumber    // try to read driver. In success case driver put answer and return true
                                   && answer.function    == 0xFD
                                   && answer.dataByte    == order));
      bt.push(std::to_string(millis() - start));
    }
    void waitOutServoRollUp(int32_t target, int32_t tolerance) {
      target *= 100;
      uint32_t start = millis();
      bt.push("\n  wait " + axisName + " rollup...");
      while (!inRange(getServoCurrent(), target - tolerance, target + tolerance));
      bt.push(" take " + std::to_string(millis() - start));
    }
    void waitOutServoErrors(int32_t tolerance) {
      uint32_t start = millis();
      // bt.push("\n  wait " + axisName + " errors...");
      while (!inRange(getServoError(), -tolerance, tolerance));
      bt.push(" take " + std::to_string(millis() - start));
    }
    void go(uint8_t speed, int32_t distance, bool lockCurrent) {
      uint32_t start = millis();
      bt.push("\n" + axisName + ' ');
      // reject trivial requests
        if (distance == 0 || speed == 0) {
          bt.push("reject trivial go");
          return;
        }
      // adjust parameters
        int32_t pulses = abs(distance * power);
        speed += 0x80 * ((distance * dir) > 0);
      // report BT about go
        std::stringstream stream;
        stream << current << "->" << current + distance << "s" << (int32_t) speed;
        bt.push(stream.str());
      // request go() to driver
        // bt.push("\n  send go() take ");
        driver.push(commandStruct {axisNumber, 0xFD, speed, pulses});
        // bt.push(std::to_string(millis() - start));
      // wait go() confirm
        waitGoConfirm(1);
        waitGoConfirm(2);
      // wait support roll up to the goal
        waitOutServoRollUp(current + distance, 100);
      // turn current or servoShift
        if (lockCurrent)
          servoShift -= (distance * 100);
        else
          current += distance;
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
    uint8_t axisNumber;   // used to mark rs485 commands
    std::string axisName = "defaultAxisName";
    int32_t dir;
    int32_t power;        // quantity of pulses per one tenth (to shaft and motor where 5mm shift per 200 pulses power = 4 * microstep)
    int32_t min;
    int32_t max;
    int32_t current;     // current == servo * power
    int32_t servoShift = 0;
};

class Cutter {
  public:
    Cutter (Axis &x_, Axis &y_) : x{x_}, y{y_} {
      leftSlot = center - widthSlot * (slots / 2);
      leftSlot += (slots % 2 == 0) ? (widthSlot / 2) : 0;
    }
    void checkBT() {
      if (!SerialBT.available())
        return;
      const String strNumbers = "1234567890";
      const String strDirections = "xylrio";
      // pull BT
      std::string rx = bt.pull();
      Serial.println("checkBT got <" + (String) rx.data() + ">");
      // check lockCurrent:
      bool lockCurrent = (rx[0] == 'l');
      if (rx[0] == 'l')  rx[0] = '4';
      // check onechar commands:
      if (rx.length() == 1) {
        if (rx[0] == 'a')  takeAlarm();
        if (rx[0] == 'p')  takePause();
        if (rx[0] == 's')  shaveShaft();
        if (rx[0] == 'c')  sliceShaft();
        if (rx[0] == 't')  cutTail();
        if (rx[0] == 'f')  finaliseTale();
        if (rx[0] == 'A') {
          flagAbort = true;
          bt.push("\nProgram aborted");
        }
      }
      // check multichar commands:
      else if ((rx.length() > 2) && (rx.length() < 7)
                                 && (strNumbers.indexOf(rx[0]) > -1)
                                 && (strDirections.indexOf(rx[1]) > -1)) {
        // calc other parameters:
        uint8_t speedIndex = std::atoi(rx.substr(0,1).c_str());
        uint8_t direction = rx[1];
        int32_t target = std::atoi(rx.substr(2,4).c_str());
        // call go() - turn motor and report to BT
        go(direction, speed[speedIndex], target, lockCurrent);
      }
      else
        bt.push((std::string) "\nCommand not detected\n");
    }
  private:
    Axis &x;
    Axis &y;

    void go(char dir, uint8_t speed, int32_t target) {  // call unlocked go(). Intended not for manual commands!!!
      if (flagAbort) return;
      checkBT();
      go(dir, speed, target, false);
    }
    void go(char dir, uint8_t speed, int32_t target, bool lockCurrent) { // 230404 edjast target, edjast if target is out of border, call Axis.go(...)
      // break in case of zero distance:
        if ((target == 0) || (speed == 0))  return;
      // set new pointer, min and max limits to selected axis; or break in case of the erroneous direction:
        Axis* axis;
        Axis* axis2;
        if (dir == 'x' || dir == 'l' || dir == 'r')  {  axis = &x;  axis2 = &y;  }
        if (dir == 'y' || dir == 'i' || dir == 'o')  {  axis = &y;  axis2 = &x;  }
      // calculate target in case relative coordinate
        int32_t distance = target;
        if (dir == 'r' || dir == 'o')  target = axis -> getCurrent() + distance;
        if (dir == 'l' || dir == 'i')  target = axis -> getCurrent() - distance;
      // adjust target if out of borders:
        target = min((axis -> getMax()), target);
        target = max((axis -> getMin()), target);
      // call adjusted axis.go()
        distance = target - axis -> getCurrent();
        axis -> go(speed, distance, lockCurrent);
      // wait finishing
        axis -> waitOutServoErrors(10);
        axis2-> waitOutServoErrors(10);
        // bt.push(getCurrentStr());
        // bt.push(getServoCurrentStr());
        // bt.push("\n");
      // report new current
        std::stringstream stream;
        stream << std::right << std::setfill('0') << std::setw(4) << x.getServoCurrent() << "_"
                                    << std::setfill('0') << std::setw(3) << y.getServoCurrent() << "\n";
        bt.push(stream.str());
    }
    void shaveShaft() { // s 230319 reset radiusShaft to current Y value and shave shaft to this
      uint32_t start = millis();
      bt.push("\nshaveShaft start " + getCurrentStr());
      radiusShaft = y.getCurrent();
      go('y', speed[6], radiusShaft + 5);
      go('x', speed[7], borderLeft);
      go('y', speed[6], radiusShaft + 1);
      go('x', speed[5], borderRight);
      go('y', speed[2], radiusShaft);
      go('x', speed[4], borderLeft);
      go('y', speed[6], radiusBorder);
      go('x', speed[7], leftSlot);
      go('y', speed[4], radiusShaft);
      bt.push("\nshaveShaft take " + std::to_string(millis() - start));
    }
    void sliceShaft() { // c 230325 cuts all slots
      uint32_t start = millis();
      bt.push("\nsliceShaft start " + getCurrentStr());
      int32_t xGap = widthGap;
      int32_t yCurrent = y.getCurrent();
      int32_t yTarget;
      int32_t yTarget2;
      while (yCurrent > radiusSlot) {
        yTarget = max(yCurrent - slice, radiusSlot);
        yTarget2 = max(yCurrent - slice * 2, radiusSlot);
        xGap = -xGap; // reverse moves into the slots depth (may be unusefull)
        int32_t xTarget = leftSlot;
        for (int i = 0; i < slots; i++) { // cut slot
          xTarget = leftSlot + (i * widthSlot);
          go('y', speed[8], radiusBorder);
          go('x', speed[6], xTarget);
          go('y', speed[8], yCurrent);
          go('y', speed[2], yTarget);
          go('x', speed[2], xTarget + xGap);
          go('x', speed[2], xTarget - xGap);
          go('x', speed[5], xTarget); 

          go('y', speed[2], yTarget2);
          go('x', speed[2], xTarget - xGap);
          go('x', speed[2], xTarget + xGap);
          go('x', speed[5], xTarget);

          go('x', speed[5], xTarget);
          go('y', speed[8], radiusBorder);
        }
        yCurrent = yTarget2;
      }
      // come close to the left side of tail
        go('x', speed[6], leftSlot + (slots * widthSlot));
        go('y', speed[4], radiusShaft);
      bt.push("\nsliceShaft take " + std::to_string(millis() - start));
    }
    void cutTail() {    // t 230319
      uint32_t start = millis();
      bt.push("\ncutTale start " + getCurrentStr());
      int32_t left = x.getCurrent();
      int32_t right = borderRight;
      int32_t bottom = radiusTail;
      int32_t current = radiusShaft;
      int32_t target = current - slice;
      go('x', speed[8], right);
      while (target > bottom + 1) {
        go('x', speed[7], right);
        go('y', speed[2], target);
        go('x', speed[3], left + 1);
        target -= slice;
      }
      // come close to precise cut of tail
        go('x', speed[7], right);
        go('y', speed[2], bottom + 1);
        go('x', speed[3], left + 1);
      // precise cut of fail
        finaliseTale();
      bt.push("\ncutTale take " + std::to_string(millis() - start));
    }
    void finaliseTale() {// p 230319 - make corner (1 tenth left & 1 tenth deep from current)
      uint32_t start = millis();
      bt.push("\nfinaliseTale start " + getCurrentStr());
      int32_t left = x.getCurrent() - 1;
      int32_t right = borderRight;
      int32_t bottom = y.getCurrent() - 1;
      // finish walk
        go('y', speed[6], radiusShaft);
        go('x', speed[2], left);
        go('y', speed[2], bottom);
        go('x', speed[2], right);
      // back to new corner
        go('y', speed[6], radiusShaft);
        go('x', speed[6], left);
      bt.push("\nfinaliseTale take " + std::to_string(millis() - start));
    }
    void takeAlarm() {
      // remember current position:
        bt.commit(getCurrentStr());
        int32_t alarmY = y.getCurrent();
      // unroll and wait next BT command:
        go('y', speed[6], radiusBorder);
        bt.push(" ALARM !!!");
        while (!SerialBT.available())  delay(100);
        checkBT();
      // rollback to remembered Y and continue:
        go('y', speed[2], alarmY);      
    }
    void takePause() {
      bt.commit(getCurrentStr());
      bt.push(" PAUSE");
      while (!SerialBT.available())  delay(100);
      checkBT();
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
  Axis x{driverX, 0x01, dirX, powerX, borderLeft, borderRight, center};
  Axis y{driverY, 0x02, dirY, powerY, radiusSlot, radiusBorder, radiusBorder};
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