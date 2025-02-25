/*
    MODBUS RTU - AS A SLAVE DEVICE
*/

#define MODBUS_SLAVE_ID 0x01
#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03
#define MODBUS_FUNCTION_WRITE_HOLDING_REGISTERS 0x10
#define MODBUS_REGISTER_COUNT 10
#define MODBUS_BAUD_RATE 9600
#define RS485_DE_RE_PIN 4  // DE and RE pins tied together and connected to GPIO 4

HardwareSerial RS485Serial(1);
uint16_t HoldingRegisters[MODBUS_REGISTER_COUNT] = { 3, 6, 9, 12, 15, 18, 21, 24, 27, 30 };  // Array for holding registers

void setup() {
  Serial.begin(115200);
  RS485Serial.begin(MODBUS_BAUD_RATE, SERIAL_8N1, 16, 17);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
}

unsigned long lastMsg = 0;
uint8_t bufferSize = 9 + (2 * MODBUS_REGISTER_COUNT);  // for FC : 0x10
uint8_t len = 0;

void loop() {
  if (RS485Serial.available()) {
    // incomingData[len++] = RS485Serial.read();
    uint8_t incomingData[bufferSize], len = RS485Serial.readBytes(incomingData, bufferSize);

    if (len > 0) {
      // Validate the slave ID
      if (incomingData[0] == MODBUS_SLAVE_ID) {
        // Verify CRC
        uint16_t receivedCRC = (incomingData[len - 1] << 8) | incomingData[len - 2];
        uint16_t calculatedCRC = calculateCRC(incomingData, len - 2);
        if (receivedCRC == calculatedCRC) {
          // Extract the function code
          uint8_t functionCode = incomingData[1];
          if (functionCode == MODBUS_FUNCTION_READ_HOLDING_REGISTERS) {
            Serial.println("Read Holding Registers");
            processRequest03(incomingData, len);
          } else if (functionCode == MODBUS_FUNCTION_WRITE_HOLDING_REGISTERS) {
            Serial.println("Write Holding Registers");
            // processRequest16(incomingData, len);
          }
        } else {
          Serial.println("Invalid CRC");  // CRC mismatch
        }
      } else {
        Serial.println("Invalid Slave ID");  // Ignore request not meant for this slave
      }
    }
  }
}


uint16_t calculateCRC(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void sendResponse(uint8_t *response, uint8_t length) {
  RS485Serial.write(response, length);
  RS485Serial.flush();
}

// Function Code ; 0x03 as a Slave device
void processRequest03(uint8_t *request, uint8_t length) {
  // Parse the starting address and quantity of registers
  // uint16_t startAddress = (request[2] << 8) | request[3];
  // uint16_t quantity = (request[4] << 8) | request[5];
  digitalWrite(RS485_DE_RE_PIN, HIGH);  // Set DE/RE pin high for transmit mode
  uint16_t startAddress = word(request[2], request[3]);
  uint16_t quantity = word(request[4], request[5]);

  Serial.println(startAddress);
  Serial.println(quantity);

  // Validate the request
  if (startAddress + quantity > MODBUS_REGISTER_COUNT) {
    Serial.println("Invalid address range");  //
  }

  // Build the response
  uint8_t response[5 + 2 * quantity];
  response[0] = MODBUS_SLAVE_ID;  // Slave ID
  response[1] = request[1];       // Function Code
  response[2] = quantity * 2;     // Byte Count
  for (uint16_t i = startAddress; i < (startAddress + quantity); i++) {
    // response[3 + 2 * i] = HoldingRegisters[startAddress + i] >> 8;    // High byte
    // response[4 + 2 * i] = HoldingRegisters[startAddress + i] & 0xFF;  // Low byte
    response[3 + (i - startAddress) * 2] = highByte(HoldingRegisters[i]);
    response[4 + ((i - startAddress) * 2)] = lowByte(HoldingRegisters[i]);
  }

  // Append CRC
  uint16_t crc = calculateCRC(response, 3 + 2 * quantity);
  response[3 + 2 * quantity] = crc & 0xFF;  // CRC low byte
  response[4 + 2 * quantity] = crc >> 8;    // CRC high byte

  // Send the response
  sendResponse(response, 5 + 2 * quantity);
}

// Function Code ; 0x10 as a Slave device
void processRequest16(uint8_t *request, uint8_t length) {
  // Parse the starting address and quantity of registers
  uint16_t startAddress = (request[2] << 8) | request[3];
  uint16_t quantity = (request[4] << 8) | request[5];
  uint8_t byteCount = request[6];

  // Validate the request
  if (startAddress + quantity > 10 || byteCount != quantity * 2) {
    Serial.println("Invalid Data length");  // Invalid address or data length
  }
  // else {
  //   Serial.print(startAddress);
  //   Serial.print(" ");
  //   Serial.println(byteCount);
  // }

  // Write the data to the holding registers
  for (uint16_t i = 0; i < quantity; i++) {
    HoldingRegisters[startAddress + i] = (request[7 + 2 * i] << 8) | request[8 + 2 * i];
    Serial.print(HoldingRegisters[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Build the response
  uint8_t response[8];
  response[0] = MODBUS_SLAVE_ID;  // Slave ID
  response[1] = 0x10;             // Function code
  response[2] = request[2];       // Starting address high byte
  response[3] = request[3];       // Starting address low byte
  response[4] = request[4];       // Quantity high byte
  response[5] = request[5];       // Quantity low byte
  uint16_t crc = calculateCRC(response, 6);
  response[6] = crc & 0xFF;  // CRC low byte
  response[7] = crc >> 8;    // CRC high byte

  // Send the response
  sendResponse(response, 8);
}

/* LED Test
  // digitalWrite(2, HIGH);
  // delay(1000);
  // digitalWrite(2, LOW);
  // delay(1000);
*/
