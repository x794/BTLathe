HardwareSerial SerialX(1);
HardwareSerial SerialY(2);
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

// common:
  const uint32_t band = 38400;

// axes:
  const uint8_t pinTxY = 4;
  const uint8_t pinRxY = 5;
  const uint8_t pinTxX = 6;
  const uint8_t pinRxX = 7;
  const uint32_t powerX = 4 * 16; // 4 (pulses per 1 tenth) * 16 (microstep)
  const uint32_t powerY = 4 * 64;
  const uint8_t speed[5] = {0x01, 0x02, 0x04, 0x30, 0x60};  // ... + 0x80 -> backward

// BLE:
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>

  BLEServer *pServer = NULL;
  BLECharacteristic * pTxCharacteristic;
  bool deviceConnected = false;
  bool oldDeviceConnected = false;
  #define txSize 10
  uint8_t txData[txSize];
  uint8_t rxSize = 0;
  uint8_t rxData[10];

  #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
  #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
  #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// lathe structs and classes:
struct commandStruct {
  uint8_t driverIndex;
  uint8_t function;
  uint8_t dataByte;
  int32_t data;
};

class Driver57 {
  public:
    Driver57(HardwareSerial &SerialServo_) : SerialServo{SerialServo_} {
      Serial.println("init readNextByteRecursive..");
    }
    bool pull(struct commandStruct &answer) {
      Serial.println("\ndriver.getRecord) - call readBytecode()"); //marker
      uint8_t bytecode[10];
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
        Serial.println("driver.pull(&answer) - parsed uart bytecode stream, packed it to answer, return true"); //marker
        return true;
      }
      Serial.println("driver.pull(&answer) - can't find/parse uart bytecode stream, return false"); //marker
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
      Serial.println("  readBytecode() start, call clear(bytecode), recursion readNextByteRecursive()"); //marker
      // readNextByteRecursive(bytecodeHolder, iteration, indexCRC_default)
      uint8_t bytecodeSize = readNextByteRecursive(bytecode, 0, 9);
      reportOutput("  readBytecode() - bytecodeSize: " + (String) bytecodeSize + ", bytecode: ", bytecode, bytecodeSize);
      return (bytecodeSize > 0);
    }
    uint8_t readNextByteRecursive(uint8_t bytecode[10], int i, int indexCRC) {  // get target bytecode array, index of current uint8_t to read, size of array to read
      uint32_t waiter = 0;
      while ((waiter++ < 10000) && !SerialServo.available())  // wait next byte
        delayMicroseconds(1);
      Serial.print("    readNextByteRecursive() - waiter: " + (String) waiter);
      if (SerialServo.available()) {
        uint8_t incoming = SerialServo.read();
        bytecode[i] = incoming;
        Serial.print(", read: ");       //marker
        Serial.println(incoming, HEX);  //marker
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
      Serial.println("\n    readNextByteRecursive() - correct bytecode not founded"); //marker
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
      Serial.println("    readNextByteRecursivecheckCRC() - return " + (String) result); //marker      
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
    Axis(Driver57& driver_, uint8_t axisNumber_, int32_t power_) : driver{driver_}, axisNumber{axisNumber_}, power{power_} {
    }
    void go(uint8_t speed, int32_t distance, bool lockCurrent) {
      if (distance == 0)
        return;
      int32_t pulses = distance * power;
      if (distance < 0) {
        pulses = -pulses;
        speed += 0x80;
      }
      driver.push(commandStruct {axisNumber, 0xFD, speed, pulses});
      Serial.println("axis.go() - exit bytecode founded! Call setCurrent(getCurrent() + distance), finished"); //marker
      setCurrent(lockCurrent ? getCurrent() : getCurrent() + distance);
      setStatus(SEND);
    }
    void setCurrent(int32_t current_) {
      current = current_;
    }
    int32_t getCurrent() {
      return current;
    }
    uint8_t getStatus() {
      return status;
    }
    void setStatus(uint8_t status_) {
      status = status_;
    }
    bool checkAxis() {
      struct commandStruct answer;
      if (driver.pull(answer)) {    // try to read driver. In success case driver put answer and return true
        if (answer.function == 0xFD && answer.dataByte == 1) {
          if (answer.dataByte == 1)
            setStatus(BUSY);
          if (answer.dataByte == 2)
            setStatus(GOUP);
        }
        if (answer.function == 0x39) {
          if (answer.data >= 1)
            setStatus(GOUP);
          if (answer.data < 1 && getStatus() == GOUP)
            setStatus(FREE);            
        }
        return true;
      }
      return false;
    }
  private:
    Driver57 &driver;
    uint8_t axisNumber;   // used to mark rs485 commands
    int32_t power;        // quantity of pulses per one tenth (to shaft and motor where 5mm shift per 200 pulses power = 4 * microstep)
    int32_t current = 12345;
    uint8_t status = GOUP;
};

class Cutter {
  public:
    Cutter (Axis& x_, Axis& y_) : x{x_}, y{y_} {
      x.setCurrent(center);
      y.setCurrent(radiusBorder);
      leftSlot = center - widthSlot * (slots / 2);
      leftSlot += (slots % 2 == 0) ? (widthSlot / 2) : 0;
    }
    bool checkBLE() {
      // clear rxData
        for (int i = 0; i < rxSize; i++)
          rxData[i] = 0x00;

      // check BLE connection
      if (deviceConnected) {
        pTxCharacteristic->getValue();  // put incoming value to rxData[]
        
        // check alarm and pause:
        if (rxData[0] == 'a' || rxData[0] == 'p') {
          makePause(rxData[0]);
        }

        // check lockCurrent:
        bool lockCurrent = (rxData[0] == 'l');
        if (rxData[0] = 'l')
          rxData[0] = 0x32;
        
        // check single move command:
        if ((rxData[0] > 0x2F) && (rxData[0] < 0x35)) {
          uint32_t speedIndex = (uint8_t) (rxData[0] - 0x30); // 0 = BLE 0x30 = uint 0
          char direction = (char) rxData[1];
          int32_t target = 0;
          for (int i = 2; i < rxSize; i++)
            target = target * 10 + (int32_t) (rxData[i] - 0x30);
          Serial.println("\nmanipulator.checkBLE() - received rxData[] - call cutter.go(" + (String) direction + ", " + (String) speed[speedIndex] + ", " + (String) target + ", " + lockCurrent + ");"); // marker
          go(direction, speed[speedIndex], target, lockCurrent);
        }

        // check multi move commands:
        if (rxData[0] == 's')
          shaveShaft();
        if (rxData[0] == 'c')
          sliceShaft();
        if (rxData[0] == 't')
          cutTail();
        if (rxData[0] == 'f')
          finaliseTale();
        // send current coordinates to BLE
          int32_t coordinates[2];
          getCurrent(coordinates);
          fillCurrentToRxData(coordinates);
          pTxCharacteristic->setValue((uint8_t*) &txData, (size_t) txSize);
          pTxCharacteristic->notify();
          delay(10); // bluetooth stack will go into congestion, if too many packets are sent
      }
      // disconnecting
        if (!deviceConnected && oldDeviceConnected) {
          delay(1000); // give the bluetooth stack the chance to get things ready
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
    void checkServo() { // repeat motors reading while successful
      while (!(x.checkAxis() || y.checkAxis()));
    }
    uint8_t getStatus() {
      return max(x.getStatus(), y.getStatus());
    }
  private:
    Axis &x;
    Axis &y;

    void go(char dir, uint8_t speed, int32_t target) {  // call unlocked go()
      go(dir, speed, target, false);
    }
    void go(char dir, uint8_t speed, int32_t target, bool lockCurrent) { // 230321 edjast target, edjast if target is out of border, call Axis.go(...)
      // break in case of zero distance:
        if (target == 0)
          return;
      // set new pointer, min and max limits to selected axis; or break in case of the erroneous direction:
        Axis* axis;
        int32_t min;
        int32_t max;
        if (dir == 'x' || dir == 'l' || dir == 'r') {
          axis = &x;
          min = borderLeft;
          max = borderRight;
        }
        else if (dir == 'y' || dir == 'i' || dir == 'o') {
          axis = &y;
          min = radiusSlot;
          max = radiusBorder;
        }
        else
          return;
      // calculate target in case relative coordinate
        int32_t distance = target;
        if (dir == 'r' || dir == 'o') // coordinate rise case
          target = axis -> getCurrent() + distance;
        if (dir == 'l' || dir == 'i') // coordinate reduce case
          target = axis -> getCurrent() - distance;
      // adjust target in out of border cases:
        if (target < min)
          target = min;
        if (target > max)
          target = max;
      // call adjasted axis.go()
      distance = target - axis -> getCurrent();
      axis -> go(speed, distance, lockCurrent);
    }
    void shaveShaft() { // s 230319 reset radiusShaft to current Y value and shave shaft to this
      radiusShaft = y.getCurrent();
      go('y', speed[3], radiusBorder);
      go('x', speed[4], borderLeft);
      go('y', speed[3], radiusShaft + 1);
      go('x', speed[2], borderRight);
      go('y', speed[1], radiusShaft);
      go('x', speed[1], borderLeft);
      go('y', speed[3], radiusBorder);
      go('x', speed[4], leftSlot);
      go('y', speed[2], radiusShaft);
    }
    void sliceShaft() { // c 230325 cuts all slots
      int32_t xGap = widthGap;
      int32_t yCurrent = y.getCurrent();
      int32_t yTarget = radiusShaft;
      while (yCurrent < radiusSlot) {
        yTarget = (yTarget - slice < radiusSlot) ? radiusSlot : yTarget - slice;
        xGap = -xGap; // reverse moves into the slots depth (may be unusefull)
        int32_t xTarget = leftSlot;
        for (int i = 0; i < slots; i++) { // cut slot
          xTarget = leftSlot + (i * widthSlot);
          go('y', speed[3], radiusBorder);
          go('x', speed[4], xTarget);
          go('y', speed[3], yCurrent);
          go('y', speed[1], yTarget);
          go('x', speed[1], xTarget + xGap);
          go('x', speed[1], xTarget - xGap);
          go('x', speed[3], xTarget);
          go('y', speed[3], radiusBorder);
        }
        yCurrent = yTarget;
      }
      // come close to the left side of tail
        go('x', speed[3], leftSlot + (slots * widthSlot));
        go('y', speed[2], radiusShaft);
    }
    void cutTail() {    // t 230319
      int32_t left = x.getCurrent();
      int32_t right = borderRight;
      int32_t bottom = radiusTail;
      int32_t current = radiusShaft;
      int32_t target = current - slice;
      go('x', speed[4], right);
      while (target > bottom + 1) {
        go('x', speed[4], right);
        go('y', speed[1], target);
        go('x', speed[1], left + 1);
        target += slice;
      }
      // come close to precise cut of tail
        go('x', speed[4], right);
        go('y', speed[1], radiusTail + 1);
        go('x', speed[1], left + 1);
      // precise cut of fail
        finaliseTale();
    }
    void finaliseTale() {// p 230319 - make corner (1 tenth left & 1 tenth deep from current)
      int32_t left = x.getCurrent() - 1;
      int32_t right = borderRight;
      int32_t bottom = y.getCurrent() - 1;
      // finish walk
        go('y', speed[3], radiusShaft);
        go('x', speed[1], left);
        go('y', speed[1], bottom);
        go('x', speed[1], right);
      // back to new corner
        go('y', speed[3], radiusShaft);
        go('x', speed[3], left);
    }
    void fillCurrentToRxData(int32_t * coordinates) {
      txData[0] = 0x0D; // CR - new line
      txData[1] = ((coordinates[0] %  10000) /  1000) + 0x30;
      txData[2] = ((coordinates[0] %   1000) /   100) + 0x30;
      txData[3] = ((coordinates[0] %    100) /    10) + 0x30;
      txData[4] = ((coordinates[0] %     10) /     1) + 0x30;
      txData[5] = 0x5f; // space
      txData[6] = ((coordinates[1] %   1000) /   100) + 0x30;
      txData[7] = ((coordinates[1] %    100) /    10) + 0x30;
      txData[8] = ((coordinates[1] %     10) /     1) + 0x30;
      txData[9] = 0x0D; // CR - new line
    }
    void getCurrent(int32_t * coordinates) {
      coordinates[0] = x.getCurrent();
      coordinates[1] = y.getCurrent();
    }
    void makePause(char c) {
      int32_t coordinates[2];
      getCurrent(coordinates);

      if (c == 'a')
        go('y', speed[3], radiusBorder);
      
      fillCurrentToRxData(coordinates);
      txData[9] = 0x5f;
      pTxCharacteristic->setValue((uint8_t*) &txData, (size_t) txSize);
      pTxCharacteristic->notify();
      delay(10); // bluetooth stack will go into congestion, if too many packets are sent

      txData[0] = (c == 'a') ? 'A' : 'P';
      txData[1] = (c == 'a') ? 'L' : 'A';
      txData[2] = (c == 'a') ? 'A' : 'U';
      txData[3] = (c == 'a') ? 'R' : 'S';
      txData[4] = (c == 'a') ? 'M' : 'E';
      txData[5] = 0x0D;
      pTxCharacteristic->setValue((uint8_t*) &txData, (size_t) 6);
      pTxCharacteristic->notify();
      delay(10); // bluetooth stack will go into congestion, if too many packets are sent

      while (rxData[0] == 'a' || rxData[0] == 'p') {
        delay(100);
        checkBLE();
      }
      go('y', speed[1], coordinates[1]);      
    }
};

// BLE classes:
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
        if (rxValue.length() > 0) {
          rxSize = rxValue.length(); // rxSize = rxValue.length() - 1; // circumcise LR-symbol
          for (int i = 0; i < rxSize; i++)
            rxData[i] = rxValue[i];
        }
      }
  };

void setup() {
  Serial.begin(band);
  SerialX.begin(band, SERIAL_8N1, pinRxX, pinTxX);
  SerialY.begin(band, SERIAL_8N1, pinRxY, pinTxY);

  // BLE setup
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
  Axis x{driverX, 0x01, powerX};
  Axis y{driverY, 0x02, powerY};
  Cutter cutter{x, y};

void loop() {
  cutter.checkServo();
  if (cutter.getStatus() == FREE)
    cutter.checkBLE();
}
// 260325
// tasks:
  // add sinchronous motor turning
  // separate class Driver57 to discrete file
  // test alarm, pause, moves, indications
  // cut of reports to serial monitor