HardwareSerial Servo(1);

//common parameters:
  const uint32_t band = 38400;

// UART parameters:
  const uint8_t pinTx = 17;
  const uint8_t pinRx = 18;

// rs485 parameters:
  const uint8_t pinEnableTx = 3;
  const uint32_t delayOneByteSendBase = 11520000;           // microseconds to send one byte with band = 1
  uint32_t delayOneByteSend = delayOneByteSendBase / band;  // microseconds to send one byte with current band. It take about 100 microseconds per one byte sending on band == 115200

// axis parametres:
  const uint8_t pinDirX = 4;
  const uint8_t pinDirY = 5;
  const uint8_t pinPulseX = 6;
  const uint8_t pinPulseY = 7;
  const uint32_t powerX = 4 * 16; // 4 (pulses per 1 tenth) * 16 (microstep)
  const uint32_t powerY = 4 * 16;
  const uint8_t speed[5] = {0x05, 0x0f, 0x2f, 0x4f, 0x7f};  // ... + 0x80 -> backward

// structs and classes:
struct requestStruct {
  uint8_t head;
  uint8_t address;
  uint8_t function;
  uint8_t dataByte;
  int32_t data;
};

class LinkUART {
  public:
    LinkUART() {
    }
    LinkUART(uint8_t pinRx, uint8_t pinTx, int band) {
      Servo.begin(band, SERIAL_8N1, pinRx, pinTx);
      Serial.begin(band);
    }
    bool read(uint8_t bytecode[10]) {      // try reading to incomed array, and return true if success
      if (readServo(bytecode, 0, sizeof(bytecode) - 1)) { // readServo - recursion - return true if reading to bytecode[] successfull, start writing from buffer[0], indexCRC == 9 (by default)
        reportOutput("get: ", bytecode, 10);
        return true;
      }
      return false;
    }
    void write(uint8_t bytecode[10], uint8_t bytecodeSize) {
      Servo.write(bytecode, bytecodeSize);  // send bytecode to UART
      reportOutput("send:", bytecode, 10); // send bytecode to console
    }
  private:
    void clearBytecode(uint8_t bytecode[10]) {
      for (int i = 0; i < 10; i++)
          bytecode[i] = 0x00;
    }
    bool checkCRC(uint8_t bytecode[10], int indexCRC) {
      uint8_t crc = 0;
      for (int i = 0; i < indexCRC; i++)
        crc += bytecode[i];
      return (crc == bytecode[indexCRC]);
    }
    bool readServo(uint8_t bytecode[10], int i, int indexCRC) {  // get target bytecode array, index of current uint8_t to read, size of array to read
      if (i > 0 && !Servo.available())  // delay, if the reading has already been started and interrupted
        delayMicroseconds(10);

      if (Servo.available()) {
        uint8_t incoming = Servo.read();
        bytecode[i] = incoming;

        if (i == 0 && incoming == 0xFB) {     // continue the reading only if flag 0xFB catched
          return readServo(bytecode, i + 1, indexCRC);
        }

        else if (i == 1 && incoming < 0x03) { // continue the reading only if motors number < 3
          return readServo(bytecode, i + 1, indexCRC);
        }
        
        else if (i == 2) {                    // continue the reading only if correct command detected
        
          if (incoming == 0x30)
            return readServo(bytecode, i + 1, 9); // 3+6, send indexCRC = 9

          else if (incoming == 0x33)
            return readServo(bytecode, i + 1, 7); // 3+4, send indexCRC = 7

          else if (incoming == 0x39)
            return readServo(bytecode, i + 1, 5); // 3+2, send indexCRC = 5

          else if (incoming >= 0x3A ||
                   incoming == 0x3B ||
                   incoming == 0x3D ||
                   incoming == 0x3E ||
                   incoming == 0x80 ||
                   incoming == 0x82 ||
                   incoming == 0x83 ||
                   incoming == 0x84 ||
                   incoming == 0x85 ||
                   incoming == 0x86 ||
                   incoming == 0x88 ||
                   incoming == 0x89 ||
                   incoming == 0x8A ||
                   incoming == 0x8B ||
                   incoming == 0x3F ||
                   incoming == 0xF3 ||
                   incoming == 0xF6 ||
                   incoming == 0xF7 ||
                   incoming == 0xFF ||
                   incoming == 0xFD )
            return readServo(bytecode, i + 1, 4); // 3+1, send indexCRC = 4
        }
        
        else if (i > 2 && i < indexCRC) {         // continue the reading only if the expected data is receiving
          return readServo(bytecode, i + 1, indexCRC);
        }

        else if (i == indexCRC) {                 // it is the last recursive itteration - it checks the CRC
            return checkCRC(bytecode, indexCRC);
        }
      }
      
      else {                      // if correct command was not detected - return false
        return false;
      }
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

class Link485 {
  public:
    Link485() {
    }
    Link485(LinkUART uart, uint8_t pinEnableTx, int delayOneByteSend) {
      this->uart = uart;
      this->pinEnableTx = pinEnableTx;
      this->delayOneByteSend = delayOneByteSend;
      pinMode(pinEnableTx, OUTPUT);
    }
    bool read(uint8_t bytecode[10]) {
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
};

class Codec57c {
  public:
    Codec57c() {
    }
    Codec57c(Link485 link) {
      this->link = link;
    }
    bool read(struct requestStruct request) {
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
        return true;
      }
      return false;
    }
    void write(struct requestStruct request) {
      uint8_t bytecode[10];
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
          break;
        case 0xFD:
          bytecode[3] = request.dataByte;
          byte* reverse = (byte*)(void*)request.dataByte;
          bytecode[4] = reverse[3];
          bytecode[5] = reverse[2];
          bytecode[6] = reverse[1];
          bytecode[7] = reverse[0];
          // bytecode[4] = (request.data % 0x100000000) / 0x1000000;
          // bytecode[5] = (request.data % 0x1000000) / 0x10000;
          // bytecode[6] = (request.data % 0x10000) / 0x100;
          // bytecode[7] = (request.data % 0x100) / 0x1;
          bytecode[8] = getCRC(bytecode, 8);
          break;
      }
      link.write(bytecode, 10);
    }
  private:
    Link485 link;
    uint8_t getCRC(uint8_t bytecode[10], int indexCRC) {
      uint8_t crc = 0;
      for (int i = 0; i < indexCRC; i++)
        crc += bytecode[i];
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
    Axis() {
    }
    Axis(Codec57c codec, uint8_t axisNumber, int32_t power) {
      this -> codec = codec;
      this -> axisNumber = axisNumber;
      this -> power = power;
    }
    void goTo(uint8_t speed, int32_t target) {
      go(speed, target - current);
    }
    void setCurrent(int32_t curr) {
      current = curr;
    }
    int32_t getCurrent() {
      return current;
    }
  private:
    Codec57c codec;
    uint8_t axisNumber;   // used to mark rs485 requests
    int32_t power;        // quantity of pulses per one tenth (to shaft and motor where 5mm shift per 200 pulses power = 4 * microstep)
    int32_t current;
    struct requestStruct request;

    void go(uint8_t speed, int32_t distance) {
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
      while(!codec.read(request) && request.head == 0xFB && request.address == axisNumber && request.function == 0xFD)
        delay(100);
      current += distance;
    }
};

class Cutter {
  public:
    Cutter() {
    }
    Cutter (Axis x, Axis y) {
      this -> x = x;
      this -> y = y;
      x.setCurrent(center);
      y.setCurrent(radiusBorder);
      leftSlot = center - widthSlot * (slots / 2);
      leftSlot += (slots % 2 == 0) ? (widthSlot / 2) : 0;
    }
  private:
    Axis x;
    Axis y;
    // shafts and machines parametres
      int32_t radiusBorder = 260;
      int32_t radiusShaft = 252;
      int32_t radiusSlot = 120;
      int32_t radiusTale = 140;
      int32_t slice = 5;

      int32_t center = 2500;
      int32_t widthSlot = 44;
      int32_t widthGap = 5;
      int32_t slots = 92;

      int32_t borderLeft = 400;
      int32_t borderRight = 5050;
      int32_t leftSlot;

    void shaveShaft() { // 230319 reset radiusShaft to current Y value and shave shaft to this
      radiusShaft = y.getCurrent();
      y.goTo(speed[3], radiusBorder);
      x.goTo(speed[4], borderLeft);
      y.goTo(speed[0], radiusShaft + 1);
      x.goTo(speed[2], borderRight);
      y.goTo(speed[1], radiusShaft);
      x.goTo(speed[1], borderLeft);
      y.goTo(speed[3], radiusBorder);
      x.goTo(speed[4], leftSlot);
    }
    void sliceShaft() { // 230317 cuts all slots
      int32_t xGap = widthGap;
      int32_t yCurrent = radiusShaft;
      int32_t yTarget = radiusShaft;
      while (yCurrent < radiusSlot) {
        yTarget = (yTarget - slice < radiusSlot) ? radiusSlot : yTarget - slice;
        xGap = -xGap; // reverse moves into the slots depth (may be unusefull)
        for (int i = 0; i < slots; i++)
          cutSlot(leftSlot + i * widthSlot, xGap, yCurrent, yTarget);
        yCurrent = yTarget;
      }
      x.goTo(speed[3], leftSlot + (slots * widthSlot));  // come to tales left side
    }
    void cutSlot(int32_t xTarget, int32_t xGap, int32_t yCurrent, int32_t yTarget) { // 230317
      y.goTo(speed[3], radiusBorder);
      x.goTo(speed[4], xTarget);
      y.goTo(speed[3], yCurrent);
      y.goTo(speed[1], yTarget);
      x.goTo(speed[1], xTarget + xGap);
      x.goTo(speed[1], xTarget - xGap);
      x.goTo(speed[3], xTarget);
      y.goTo(speed[3], radiusBorder);
    }
    void cutTale() { // 230319
      int32_t left = x.getCurrent();
      int32_t right = borderRight;
      int32_t bottom = radiusTale;
      int32_t current = radiusShaft;
      int32_t target = current - slice;
      x.goTo(speed[4], right);
      while (target > bottom + 1) {
        y.goTo(speed[1], target);
        x.goTo(speed[1], left + 1);
        x.goTo(speed[4], right);
        target += slice;
      }
      // prefinish walk
        y.goTo(speed[1], radiusTale + 1);
        x.goTo(speed[1], left + 1);
      // finish walk
        preciseTale();
    }
    void preciseTale() { // 230319 - make corner (1 tenth left & 1 tenth deep from current)
      int32_t left = x.getCurrent() - 1;
      int32_t right = borderRight;
      int32_t bottom = y.getCurrent() - 1;
      // finish walk
        y.goTo(speed[4], radiusShaft);
        x.goTo(speed[1], left);
        y.goTo(speed[1], bottom);
        x.goTo(speed[1], right);
      // back to new corner
        y.goTo(speed[4], radiusShaft);
        x.goTo(speed[4], left);
    }
};

// objects creation:
  LinkUART uart(pinRx, pinTx, band);
  Link485 rs485(uart, pinEnableTx, delayOneByteSend);  // disable this line to use uart instead of rs485
  Codec57c codec(rs485);                               // specify rs485 or uart
  Axis x(codec, 0x01, powerX);
  Axis y(codec, 0x02, powerY);
  Cutter cutter(x, y);

void setup() {
  delay(2000);
}

void loop() {
  delay(2000);
}
