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

// common:
  const uint32_t band = 38400;

// UART:
  const uint8_t pinTxY = 4;
  const uint8_t pinRxY = 5;
  const uint8_t pinTxX = 6;
  const uint8_t pinRxX = 7;

// rs485:
  // const uint8_t pinEnableTx = 5;
  // const uint32_t delayOneByteSendBase = 11520000;           // microseconds to send one byte with band = 1
  // uint32_t delayOneByteSend = delayOneByteSendBase / band;  // microseconds to send one byte with current band. It take about 100 microseconds per one byte sending on band == 115200

// axis:
  // const uint8_t pinDirX = ;
  // const uint8_t pinDirY = ;
  // const uint8_t pinPulseX = ;
  // const uint8_t pinPulseY = ;
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
struct requestStruct {
  uint8_t head;
  uint8_t address;
  uint8_t function;
  uint8_t dataByte;
  int32_t data;
};

class LinkUART {
  public:
    LinkUART(HardwareSerial &SerialServo_) : SerialServo{SerialServo_} {
      Serial.println("init uart...");
      Serial.println(SerialServo_);
      Serial.println(SerialServo);      
    }
    bool read(uint8_t bytecode[10]) {      // try reading to incomed array, and return true if success
      Serial.println("uart.read() start, zeroise bytecode, call recursion uart.readServo()"); //marker
      for (int i = 0; i < 10; i++)
        bytecode[i] = 0x00;
      if (readServo(bytecode, 0, 9)) { // readServo - recursion - return true if reading to bytecode[] successfull, start writing from buffer[0], indexCRC == 9 (will been corrected in i=2 itteration)
        reportOutput("uart.read() return true, bytecode: ", bytecode, 10);
        return true;
      }
      reportOutput("uart.read() return false, bytecode: ", bytecode, 10);
      return false;
    }
    void write(uint8_t bytecode[10], uint8_t bytecodeSize) {
      SerialServo.write(bytecode, bytecodeSize);  // send bytecode to UART
      reportOutput("UART send:", bytecode, bytecodeSize); // send bytecode to console
    }
  private:
    HardwareSerial & SerialServo;
    void clearBytecode(uint8_t bytecode[10]) {
      Serial.println("uart.clearBytecode"); //marker
      for (int i = 0; i < 10; i++)
          bytecode[i] = 0x00;
    }
    bool checkCRC(uint8_t bytecode[10], int indexCRC) {
      uint8_t crc = 0;
      for (int i = 0; i < indexCRC; i++)
        crc += bytecode[i];
      bool result = (crc == bytecode[indexCRC]);
      Serial.println("uart.checkCRC() - return " + (String) result); //marker      
      return (result);
    }
    bool readServo(uint8_t bytecode[10], int i, int indexCRC) {  // get target bytecode array, index of current uint8_t to read, size of array to read
      uint32_t waiter = 0;
      while ((waiter++ < 1000000) && !SerialServo.available())  // delay, if the reading has already been started and interrupted
        delayMicroseconds(1);
      Serial.println ("uart.readServo() - waiter: " + (String) waiter);
      if (SerialServo.available()) {
        uint8_t incoming = SerialServo.read();
        bytecode[i] = incoming;
        Serial.print(", read: "); //marker
        Serial.println(incoming, HEX);          //marker
        if (i == 0 && incoming == 0xFB)     // continue the reading only if flag 0xFB catched
          return readServo(bytecode, i + 1, indexCRC);
        else if (i == 1 && (incoming == 0x01 || incoming == 0x02)) // continue the reading only if motors number < 3
          return readServo(bytecode, i + 1, indexCRC);
        else if (i == 2) {                  // continue the reading only if correct command detected
          if (incoming == 0x30)
            return readServo(bytecode, i + 1, 9); // 3+6, send indexCRC = 9
          else if (incoming == 0x33)
            return readServo(bytecode, i + 1, 7); // 3+4, send indexCRC = 7
          else if (incoming == 0x39)
            return readServo(bytecode, i + 1, 5); // 3+2, send indexCRC = 5
          else if (incoming >= 0x3A || incoming == 0x3B || incoming == 0x3D || incoming == 0x3E ||
                   incoming == 0x80 || incoming == 0x82 || incoming == 0x83 || incoming == 0x84 ||
                   incoming == 0x85 || incoming == 0x86 || incoming == 0x88 || incoming == 0x89 ||
                   incoming == 0x8A || incoming == 0x8B || incoming == 0x3F || incoming == 0xF3 ||
                   incoming == 0xF6 || incoming == 0xF7 || incoming == 0xFF || incoming == 0xFD )
            return readServo(bytecode, i + 1, 4); // 3+1, send indexCRC = 4
        }
        else if (i > 2 && i < indexCRC)          // continue the reading only if the expected data is receiving
          return readServo(bytecode, i + 1, indexCRC);
        else if (i == indexCRC)                  // it is the last recursive itteration - it checks the CRC
            return checkCRC(bytecode, indexCRC);
      }
      Serial.println("uart.readServo() - return false - not found command to bytecode"); //marker
      return false;                   // if correct command was not detected - return false
    }
    void reportOutput(String str, uint8_t* bytecode, uint8_t bytecodeSize) {
      Serial.print(str);
      for (int i = 0; i < bytecodeSize; i++) {
        Serial.print(" ");
        Serial.print(bytecode[i], HEX);
      }
      Serial.println();
    }
};

class Link485 { /*
  public:
    Link485() {
    }
    Link485(LinkUART& uart_, uint8_t pinEnableTx_, int delayOneByteSend_) {
      uart = uart_;
      pinEnableTx = pinEnableTx_;
      delayOneByteSend = delayOneByteSend_;
      pinMode(pinEnableTx, OUTPUT);
    }
    bool read(uint8_t bytecode[10]) {
      Serial.println("485.read"); //marker
      return uart.read(bytecode);
    }
    void write(uint8_t bytecode[10], uint8_t bytecodeSize) {
      digitalWrite(pinEnableTx, HIGH);                    // start RS485 broadcast
      uart.write(bytecode, bytecodeSize);                 // RS485 broadcast
      delayMicroseconds(bytecodeSize * delayOneByteSend); // support RS485 broadcast
      digitalWrite(pinEnableTx, LOW);                     // finish RS485 broadcast
    }
  private:
    LinkUART uart;
    uint8_t pinEnableTx;
    int delayOneByteSend;
*/};

class Codec57c {
  public:
    Codec57c(LinkUART& link_) : link{link_} {
    }
    bool read(struct requestStruct& request) {
      Serial.println("codec.read() start and call link.read()"); //marker
      uint8_t bytecode[10];
      if (link.read(bytecode)) {
        request.head = bytecode[0];
        request.address = bytecode[1];
        request.function = bytecode[2];
        request.dataByte = 0;
        request.data = 0;
        switch (request.function) {          
          case 0x30:
            request.data = getInt32FromFFFF4FFF(&bytecode[5]);
            break;
          case 0x33:
            request.data = getInt32FromBytecode32(&bytecode[3]);
            break;
          case 0x39:
            request.data = getInt32FromBytecode16(&bytecode[3]);
            break;
          default:
            request.dataByte = (uint8_t) bytecode[3];
        }
        Serial.println("codec.read() - repack bytecode to struct, return true"); //marker
        return true;
      }
      Serial.println("codec.read() - return false"); //marker
      return false;
    }
    void write(struct requestStruct request) {

      Serial.println("codec.write " + (String) request.data); //marker
    
      uint8_t bytecode[10];
      uint8_t bytecodeSize = 0;
      bytecode[0] = request.head;
      bytecode[1] = request.address;
      bytecode[2] = request.function;
      
      switch (bytecode[2]) {
        case 0x30:
        case 0x33:
        case 0x39:
        case 0x3A:
        case 0x3B:
        case 0x3D:
        case 0x3E:
          bytecode[3] = getCRC(bytecode, 3);
          bytecodeSize = 4;
          break;
        case 0x80:
        case 0x82:
        case 0x83:
        case 0x84:
        case 0x85:
        case 0x86:
        case 0x88:
        case 0x89:
        case 0x8A:
        case 0x8B:
        case 0x3F:
        case 0xF3:
        case 0xF6:
        case 0xF7:
        case 0xFF:
          bytecode[3] = request.dataByte;
          bytecode[4] = getCRC(bytecode, 4);
          bytecodeSize = 5;
          break;
        case 0xFD:
          bytecode[3] = request.dataByte;
          bytecode[4] = (request.data % 0x100000000) / 0x1000000;
          bytecode[5] = (request.data % 0x1000000) / 0x10000;
          bytecode[6] = (request.data % 0x10000) / 0x100;
          bytecode[7] = (request.data % 0x100) / 0x1;          
          bytecode[8] = getCRC(bytecode, 8);
          bytecodeSize = 9;
          break;
      }
      for (int i = 0; i < bytecodeSize; i++) //marker
        Serial.println(bytecode[i], HEX); //marker
      link.write(bytecode, bytecodeSize);
    }
  private:
    LinkUART &link;
    uint8_t getCRC(uint8_t bytecode[10], int indexCRC) {
      uint8_t crc = 0;
      for (int i = 0; i < indexCRC; i++)
        crc += bytecode[i];
      Serial.println("codec.getCRC caclulate and return CRC = " + (String) crc); //marker
      return crc;
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
};

class Axis {
  public:
    Axis(Codec57c& codec_, uint8_t axisNumber_, int32_t power_) : codec{codec_}, axisNumber{axisNumber_}, power{power_} {
    }
    void go(uint8_t speed, int32_t distance) {
      Serial.println("axis.go"); //marker
      if (distance == 0)
        return;
      int32_t pulses = distance * power;
      if (distance < 0) {
        pulses = -pulses;
        speed += 0x80;
      }
      // send request to go
      codec.write(requestStruct {0xFA, axisNumber, 0xFD, speed, pulses});
      // wait fot response to go
      while(!(codec.read(request) && (request.head == 0xFB) 
                                  && (request.address == axisNumber)
                                  && (request.function == 0xFD)
                                  && (request.dataByte == 0x02))) {
        Serial.println("waiting loop " + (String) request.head
                                 + " " + (String) request.address
                                 + " " + (String) request.function
                                 + " " + (String) request.dataByte); //marker
      }
      Serial.println("axis.go() - exit bytecode founded! Call setCurrent(getCurrent() + distance)    and finish"); //marker
      setCurrent(getCurrent() + distance);
    }
    void setCurrent(int32_t curr) {
      current = curr;
    }
    int32_t getCurrent() {
      return current;
    }
  private:
    Codec57c &codec;
    uint8_t axisNumber;   // used to mark rs485 requests
    int32_t power;        // quantity of pulses per one tenth (to shaft and motor where 5mm shift per 200 pulses power = 4 * microstep)
    int32_t current = 12345;
    struct requestStruct request;
};

class Cutter {
  public:
    Cutter (Axis& x_, Axis& y_) : x{x_}, y{y_} {
      x.setCurrent(center);
      y.setCurrent(radiusBorder);
      leftSlot = center - widthSlot * (slots / 2);
      leftSlot += (slots % 2 == 0) ? (widthSlot / 2) : 0;
    }
    void go(char dir, uint8_t speed, int32_t target) { // 230321 edjast target, edjast if target is out of border, call Axis.go(...)
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
      Serial.println("axis.go(" + (String) speed + ", " + (String) distance + ")"); //marker
      axis -> go(speed, distance);
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
    void sliceShaft() { // c 230317 cuts all slots
      int32_t xGap = widthGap;
      int32_t yCurrent = y.getCurrent();
      int32_t yTarget = radiusShaft;
      while (yCurrent < radiusSlot) {
        yTarget = (yTarget - slice < radiusSlot) ? radiusSlot : yTarget - slice;
        xGap = -xGap; // reverse moves into the slots depth (may be unusefull)
        for (int i = 0; i < slots; i++)
          cutSlot(leftSlot + i * widthSlot, xGap, yCurrent, yTarget);
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
      // precise cut of tail
        preciseTail();
    }
    void preciseTail() {// p 230319 - make corner (1 tenth left & 1 tenth deep from current)
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
    void getCurrent(int32_t * coordinates) {
      coordinates[0] = x.getCurrent();
      coordinates[1] = y.getCurrent();
    }
  private:
    Axis &x;
    Axis &y;
    void cutSlot(int32_t xTarget, int32_t xGap, int32_t yCurrent, int32_t yTarget) { // 230317
      go('y', speed[3], radiusBorder);
      go('x', speed[4], xTarget);
      go('y', speed[3], yCurrent);
      go('y', speed[1], yTarget);
      go('x', speed[1], xTarget + xGap);
      go('x', speed[1], xTarget - xGap);
      go('x', speed[3], xTarget);
      go('y', speed[3], radiusBorder);
    }
};

class Manipulator {
  public:
    Manipulator(Cutter& cutter_) : cutter{cutter_} {
    }
    void readBLE() {
      if (deviceConnected) {
        pTxCharacteristic->getValue();  // put incoming value to rxData[]

        if (rxData[0] > 0x2F) {
          if (rxData[0] < 0x35) {
            uint32_t speedIndex = (uint8_t) (rxData[0] - 0x30); // 0 = BLE 0x30 = uint 0
            char direction = (char) rxData[1];
            int32_t target = 0;
            for (int i = 2; i < rxSize; i++)
              target = target * 10 + (int32_t) (rxData[i] - 0x30);
            Serial.println("manipulator.readBLE() - received rxData[] - call cutter.go(" + (String) direction + ", " + (String) speed[speedIndex] + ", " + (String) target + ");"); // marker
            cutter.go(direction, speed[speedIndex], target);
          }
          if (rxData[0] == 's')
            cutter.shaveShaft();
          if (rxData[0] == 'c')
            cutter.sliceShaft();
          if (rxData[0] == 't')
            cutter.cutTail();
          if (rxData[0] == 'p')
            cutter.preciseTail();

          // send current coordinates to BLE
            int32_t coordinates[2];
            cutter.getCurrent(coordinates);
            Serial.println("readBLE() send to BLE " + (String) coordinates[0] + '.' + (String) coordinates[1] + ", zeroise rxData[], finish"); // marker

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

            pTxCharacteristic->setValue((uint8_t*) &txData, (size_t) txSize);
            pTxCharacteristic->notify();
            delay(10); // bluetooth stack will go into congestion, if too many packets are sent
            for (int i = 0; i < rxSize; i++)
              rxData[i] = 0x00;
        }
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
    void pollSRV() {
    }
  private:
    Cutter &cutter;
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
  LinkUART uartX{SerialX};
  LinkUART uartY{SerialY};
  // Link485 rs485(uart, pinEnableTx, delayOneByteSend);  // disable this line to use uart instead of rs485
  Codec57c codecX{uartX};                               // specify rs485 or uart
  Codec57c codecY{uartY};                               // specify rs485 or uart
  Axis x{codecX, 0x01, powerX};
  Axis y{codecY, 0x02, powerY};
  Cutter cutter{x, y};
  Manipulator manipulator{cutter};

void loop() {
  manipulator.readBLE();
}
// 230325
// tasks:
  // write zeroise(struct) before read
  // pack bytecode zeroise to zeroize(bytecode)
  // write error checker
  // add statuses to cutter and to axes: FREE, SEND, BUSY, GOUP