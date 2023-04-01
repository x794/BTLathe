// common:
  HardwareSerial SerialX(1);
  HardwareSerial SerialY(2);
  #include <iostream>
  #include <sstream>
  #include <iomanip>
  const uint32_t band = 38400;
  int itt = 0;

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
  
  #define FREE 0
  #define GOUP 1
  #define BUSY 2
  #define SEND 3


// axes:
  const uint8_t pinTxY = 4;
  const uint8_t pinRxY = 5;
  const uint8_t pinTxX = 6;
  const uint8_t pinRxX = 7;
  const uint32_t powerX = 4 * 16; // 4 (pulses per 1 tenth) * 16 (microstep)
  const uint32_t powerY = 4 * 64;
  const uint8_t speed[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x08, 0x10, 0x20, 0x40, 0x7F};  // ... + 0x80 -> backward

// BLE:
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>

  #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
  #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
  #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

  BLEServer *pServer = NULL;
  BLECharacteristic * pTxCharacteristic;
  bool deviceConnected = false;
  bool oldDeviceConnected = false;

// lathe structs and classes:
struct commandStruct {
  uint8_t driverIndex;
  uint8_t function;
  uint8_t dataByte;
  int32_t data;
};

// BLE classes:
  class BLE {
    public:
      BLE() {
      }
      void setRx(std::string &rxValue_) {
        rx = rxValue_;
      }
      bool available() {
        return (!rx.empty());
      }
      std::string pull() {
        std::string result = rx;
        rx.clear();
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
        if (deviceConnected && (tx.length() > 0)) {
          tx.push_back('\n');
          pTxCharacteristic->setValue(tx);
          pTxCharacteristic->notify();
          tx.clear();
    		delay(10); // bluetooth stack will go into congestion, if too many packets are sent
	      }
        // disconnecting
        if (!deviceConnected && oldDeviceConnected) {
            delay(500); // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising(); // restart advertising
            Serial.println("start advertising");
            oldDeviceConnected = deviceConnected;
        }
        // connecting
        if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
            oldDeviceConnected = deviceConnected;
        }
      }
    private:
      std::string tx;
      std::string rx;
  };
  BLE ble{};  
  class MyServerCallbacks: public BLEServerCallbacks {
      void onConnect(BLEServer* pServer) {
        deviceConnected = true;
      };

      void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
      }
  };
  class MyCallbacks: public BLECharacteristicCallbacks {
      void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        ble.setRx(rxValue);
      }
  };

class Driver57 {
  public:
    Driver57(HardwareSerial &SerialServo_) : SerialServo{SerialServo_} {
    }
    bool pull(struct commandStruct &answer) {
      uint8_t bytecode[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, };
      if (readBytecode(bytecode)) {
        answer.driverIndex = bytecode[1];
        answer.function = bytecode[2];
        switch (answer.function) {          
          case 0x30:
            answer.dataByte = 0;
            answer.data = getInt32FromFFFF4FFF(&bytecode[5]);
            break;
          case 0x33:
            answer.dataByte = 0;
            answer.data = getInt32FromBytecode32(&bytecode[3]);
            break;
          case 0x39:
            answer.data = getInt32FromBytecode16(&bytecode[3]);
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
      SerialServo.write(bytecode, bytecodeSize);
    }
  private:
    HardwareSerial & SerialServo;

    bool readBytecode(uint8_t bytecode[10]) { // return true if successfull read, check, put bytecode
      uint8_t bytecodeSize = readNextByteRecursive(bytecode, 0, 9);
      return (bytecodeSize > 0);
    }
    uint8_t readNextByteRecursive(uint8_t bytecode[10], int i, int indexCRC) {  // get target bytecode array, index of current uint8_t to read, size of array to read
      uint32_t waiter = 0;
      while ((waiter++ < 10) && !SerialServo.available())  // wait next byte
        delayMicroseconds(100);
      if (SerialServo.available()) {
        uint8_t incoming = SerialServo.read();
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
    int getInt32FromFFFF4FFF(uint8_t arr[4]) {
      int32_t result = getInt32FromBytecode16(&arr[0]) * 0x4000
                     + getInt32FromBytecode16(&arr[2]);
      if (result >= 0x8000 * 0x4000)
        result -= (0x10000 * 0x4000);
      return result;
    }
    int getInt32FromBytecode32(uint8_t arr[4]){
      uint8_t reverse[4] = {arr[3], arr[2], arr[1], arr[0]};
      int32_t result = (int32_t) *reverse;
      return result;
    }
    int getInt32FromBytecode16(uint8_t arr[2]){
      uint8_t reverse[2] = {arr[1], arr[0]};
      int16_t preResult = (int16_t) *reverse;
      int32_t result = (int32_t) preResult;
      return result;
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
    Axis(Driver57& driver_, uint8_t axisNumber_, int32_t power_, int32_t min_, int32_t max_) 
             : driver{driver_}, axisNumber{axisNumber_}, power{power_}, min{min_}, max{max_} {
    }
    void go(uint8_t speed, int32_t distance, bool lockCurrent) {
      if (distance == 0)
        return;
      int32_t pulses = distance * power;
      if (distance < 0) {
        pulses = -pulses;
        speed += 0x80;
      }
      struct commandStruct command = {axisNumber, 0xFD, speed, pulses};
      driver.push(command);
      setCurrent(lockCurrent ? getCurrent() : getCurrent() + distance);
      setStatus(SEND);
    }
    void setCurrent(int32_t current_) {
      current = current_;
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
    uint8_t getStatus() {
      return status;
    }
    void setStatus(uint8_t status_) {
      status = status_;
    }
    uint8_t checkAxis() {
      struct commandStruct answer = {0, 0, 0, 0};
      if (driver.pull(answer) && answer.driverIndex == axisNumber) {    // try to read driver. In success case driver put answer and return true
        if (answer.function == 0xFD && answer.dataByte == 0x01)  setStatus(BUSY);  // 0xFD - turn motor, 0x01 - started
        if (answer.function == 0xFD && answer.dataByte == 0x02)  setStatus(GOUP);  // 0xFD - turn motor, 0x02 - finished
        if (answer.function == 0x39 && getStatus() == GOUP
             && (answer.data < 0x0010 || answer.data > 0xFFF0))  setStatus(FREE);  // 0x39 - get error, data - quantity of microsteps to the goal
      }
      return getStatus();
    }
    void checkError() {
      status = GOUP;
      driver.push(commandStruct {axisNumber, 0x39, 0, 0});
    }
  private:
    Driver57 &driver;
    uint8_t axisNumber;   // used to mark rs485 commands
    int32_t power;        // quantity of pulses per one tenth (to shaft and motor where 5mm shift per 200 pulses power = 4 * microstep)
    int32_t current = 12345;
    uint8_t status = GOUP;
    int32_t min;
    int32_t max;
};

class Cutter {
  public:
    Cutter (Axis &x_, Axis &y_, BLE &ble_) : x{x_}, y{y_}, ble{ble_} {
      x.setCurrent(center);
      y.setCurrent(radiusBorder);
      leftSlot = center - widthSlot * (slots / 2);
      leftSlot += (slots % 2 == 0) ? (widthSlot / 2) : 0;
    }
    void checkBLE() {
      if (!ble.available())  return;
        
      const String strNumbers = "1234567890";
      const String strDirections = "xylrio";
      // pull BLE
      std::string rx = ble.pull();
      // check lockCurrent:
      bool lockCurrent = (rx[0] == 'l');
      if (rx[0] == 'l')  rx[0] = '4';
      // check onechar commands:
      if (rx.length() == 1) {
        if (rx[0] == 'a')  makeAlarm();
        if (rx[0] == 'p')  makePause();
      }
      if (rx.length() == 1 && getStatus() == FREE) {
        if (rx[0] == 's')  shaveShaft();
        if (rx[0] == 'c')  sliceShaft();
        if (rx[0] == 't')  cutTail();
        if (rx[0] == 'f')  finaliseTale();
      }
      // check multichar commands:
      else if ((rx.length() > 2) && (rx.length() < 7)
                                 && (strNumbers.indexOf(rx[0]) > -1)
                                 && (strDirections.indexOf(rx[1]) > -1)) {
        // calc other parameters:
        uint8_t speedIndex = std::atoi(rx.substr(0,1).c_str());
        uint8_t direction = rx[1];
        int32_t target = std::atoi(rx.substr(2,4).c_str());
        // call cutter.go():
        delay(2);
        go(direction, speed[speedIndex], target, lockCurrent);
      }
      else {
        std::string message = " CMD not detected";
        ble.push(message);
      }
      // send current coordinates to BLE
      ble.push(getCurrentStr());
    }
    void checkServo() {
      if (getStatus() == FREE)  return;
      // checkAxis() - read driver data, update driver status
      uint8_t statusX = x.checkAxis();
      uint8_t statusY = y.checkAxis();
      
      // if one motor is GOUP and another is FREE or both are GOUP -> switch both statuses to GOUP and send requests to check error
      if ((statusX == GOUP && statusY == GOUP) ||
          (statusX == GOUP && statusY == FREE) || 
          (statusX == FREE && statusY == GOUP)) {
        x.checkError();
        y.checkError();
      }
    }
    uint8_t getStatus() {
      return max(x.getStatus(), y.getStatus());
    }
  private:
    Axis &x;
    Axis &y;
    BLE &ble;

    void go(char dir, uint8_t speed, int32_t target) {  // call unlocked go(). Intended not for manual commands!!!
      while (getStatus() != FREE) {
        checkServo();
        checkBLE();
        delay(100);
      }
      go(dir, speed, target, false);
      ble.push(" current: " + getCurrentStr());
    }
    void go(char dir, uint8_t speed, int32_t target, bool lockCurrent) { // 230321 edjast target, edjast if target is out of border, call Axis.go(...)
      delay(100); // marker
      // break in case of zero distance:
        if ((target == 0) || (speed == 0))  return;
      // set new pointer, min and max limits to selected axis; or break in case of the erroneous direction:
        Axis* axis;
        if (dir == 'x' || dir == 'l' || dir == 'r')  axis = &x;
        if (dir == 'y' || dir == 'i' || dir == 'o')  axis = &y;
      // eject if driver is not FREE
        if (axis -> getStatus() != FREE) {
          ble.push(" Rejected. Driver not FREE");
          return;
        }          
      // calculate target in case relative coordinate
        int32_t distance = target;
        if (dir == 'r' || dir == 'o')  target = axis -> getCurrent() + distance;
        if (dir == 'l' || dir == 'i')  target = axis -> getCurrent() - distance;
      // adjust target if out of border:
        target = min((axis -> getMax()), target);
        target = max((axis -> getMin()), target);
      // call adjasted axis.go()
        distance = target - axis -> getCurrent();
        axis -> go(speed, distance, lockCurrent);
    }
    void shaveShaft() { // s 230319 reset radiusShaft to current Y value and shave shaft to this
      radiusShaft = y.getCurrent();
      go('y', speed[6], radiusBorder);
      go('x', speed[8], borderLeft);
      go('y', speed[6], radiusShaft + 1);
      go('x', speed[4], borderRight);
      go('y', speed[2], radiusShaft);
      go('x', speed[2], borderLeft);
      go('y', speed[6], radiusBorder);
      go('x', speed[8], leftSlot);
      go('y', speed[4], radiusShaft);
    }
    void sliceShaft() { // c 230325 cuts all slots
      int32_t xGap = widthGap;
      int32_t yCurrent = y.getCurrent();
      int32_t yTarget = radiusShaft;
      while (yCurrent > radiusSlot) {
        yTarget = (yTarget - slice < radiusSlot) ? radiusSlot : yTarget - slice;
        xGap = -xGap; // reverse moves into the slots depth (may be unusefull)
        int32_t xTarget = leftSlot;
        for (int i = 0; i < slots; i++) { // cut slot
          xTarget = leftSlot + (i * widthSlot);
          go('y', speed[6], radiusBorder);
          go('x', speed[5], xTarget);
          go('y', speed[6], yCurrent);
          go('y', speed[2], yTarget);
          go('x', speed[2], xTarget + xGap);
          go('x', speed[2], xTarget - xGap);
          go('x', speed[4], xTarget);
          go('y', speed[6], radiusBorder);
        }
        yCurrent = yTarget;
      }
      // come close to the left side of tail
        go('x', speed[6], leftSlot + (slots * widthSlot));
        go('y', speed[4], radiusShaft);
    }
    void cutTail() {    // t 230319
      int32_t left = x.getCurrent();
      int32_t right = borderRight;
      int32_t bottom = radiusTail;
      int32_t current = radiusShaft;
      int32_t target = current - slice;
      go('x', speed[8], right);
      while (target > bottom + 1) {
        go('x', speed[8], right);
        go('y', speed[2], target);
        go('x', speed[2], left + 1);
        target += slice;
      }
      // come close to precise cut of tail
        go('x', speed[8], right);
        go('y', speed[2], radiusTail + 1);
        go('x', speed[2], left + 1);
      // precise cut of fail
        finaliseTale();
    }
    void finaliseTale() {// p 230319 - make corner (1 tenth left & 1 tenth deep from current)
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
    }
    void makeAlarm() {
      // remember current position:
        ble.commit(getCurrentStr());
        int32_t alarmY = y.getCurrent();
      // unroll and wait next BLE command:
        go('y', speed[6], radiusBorder);
        ble.push(" ALARM !!!");
        while (!ble.available())
          delay(100);
      // rollback to remembered Y and continue:
        go('y', speed[2], alarmY);      
    }
    void makePause() {
      ble.commit(getCurrentStr());
      ble.push(" PAUSE");
      while (!ble.available())
        delay(100);
    }
    std::string getCurrentStr() {
      std::stringstream stream;
      stream << std::right << std::setfill('0') << std::setw(4) << x.getCurrent() << "_"
                           << std::setfill('0') << std::setw(3) << y.getCurrent();
      return stream.str();
    }
};

void setup() {
  SerialX.begin(band, SERIAL_8N1, pinRxX, pinTxX);
  SerialY.begin(band, SERIAL_8N1, pinRxY, pinTxY);
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}
// objects:
  Driver57 driverX{SerialX};
  Driver57 driverY{SerialY};
  Axis x{driverX, 0x01, powerX, borderLeft, borderRight};
  Axis y{driverY, 0x02, powerY, radiusSlot, radiusBorder};
  Cutter cutter{x, y, ble};

void loop() {
  // delay();  // 0.2ms - minimum delay to correct working of BLE and UART contractors
  cutter.checkServo();
  cutter.checkBLE();
  delay(10);
}
// 260401
// tasks:
  // separate class Driver57 to discrete file