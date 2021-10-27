#include <ModbusSlave.h>

#define four_input_board
//#define twelve_input_board

#define RS_Enable_Pin PB1

bool serial_degub_enb = false;

union ByteFloatUnion
{
  uint16_t byteformat[2];
  float floatformat;
};

union ByteINT64Union
{
  uint16_t byteformat[4];
  int64_t int64format;
};

ByteFloatUnion val;
ByteINT64Union val2;

uint16_t data[4];
int16_t reg[5000];

uint8_t station_address = 0;
Modbus slave(Serial3, station_address, RS_Enable_Pin);

///////////////////////////////////////////////////////////////////////
#ifdef four_input_board
byte INPUT_PINS[4] = {PB12, PB13, PB14, PB15};

unsigned long now;
unsigned long last_sent;
const unsigned long interval = 100;
bool Heart_Beat_state = 0;

#define Serial_debug_port Serial1
#define Heart_Beat_LED_Pin PC13
#endif
///////////////////////////////////////////////////////////////////////
//**********************************************************************
#ifdef twelve_input_board
#define Heart_Beat_LED_Pin PA0

#define sensor_1 PB12
#define sensor_2 PB13
#define sensor_3 PB14
#define sensor_4 PB15
#define sensor_5 PA8
#define sensor_6 PA9
#define sensor_7 PA10
#define sensor_8 PA11
#define sensor_9 PA12
#define sensor_10 PA15
#define sensor_11 PB3
#define sensor_12 PB4

#define DIP_SW_ADDRESS_0_LSB PB0
#define DIP_SW_ADDRESS_1     PA7
#define DIP_SW_ADDRESS_2     PA6
#define DIP_SW_ADDRESS_3     PA5
#define DIP_SW_ADDRESS_4_MSB PA4

bool ADDRESS_BIT_0 = 0;
bool ADDRESS_BIT_1 = 0;
bool ADDRESS_BIT_2 = 0;
bool ADDRESS_BIT_3 = 0;
bool ADDRESS_BIT_4 = 0;

unsigned long now;
unsigned long last_sent;
const unsigned long interval = 100;
bool Heart_Beat_state = 0;

#define Serial_debug_port Serial2
#endif
//**********************************************************************
void setup()
{
  enableDebugPorts();
  Serial.end();

  if (serial_degub_enb)
  {
    Serial_debug_port.begin(115200);
  }
  pinMode(RS_Enable_Pin, OUTPUT);

  ////////////////////////////////////////////////////////////
#ifdef four_input_board
  for (int i = 0; i <= 3; i ++)
  {
    pinMode(INPUT_PINS[i], INPUT_PULLDOWN);
  }

  pinMode(Heart_Beat_LED_Pin, OUTPUT);

  station_address = 4;
  slave.setUnitAddress(station_address);
#endif
  ////////////////////////////////////////////////////////////

  //**********************************************************************
#ifdef twelve_input_board
  pinMode(sensor_1, INPUT_PULLDOWN);  //Sensor 1
  pinMode(sensor_2, INPUT_PULLDOWN);  //Sensor 2
  pinMode(sensor_3, INPUT_PULLDOWN);  //Sensor 3
  pinMode(sensor_4, INPUT_PULLDOWN);  //Sensor 4
  pinMode(sensor_5, INPUT_PULLDOWN);  //Sensor 5
  pinMode(sensor_6, INPUT_PULLDOWN);  //Sensor 6
  pinMode(sensor_7, INPUT_PULLDOWN);  //Sensor 7
  pinMode(sensor_8, INPUT_PULLDOWN);  //Sensor 8
  pinMode(sensor_9, INPUT_PULLDOWN);  //Sensor 9
  pinMode(sensor_10, INPUT_PULLDOWN); //Sensor 10
  pinMode(sensor_11, INPUT_PULLDOWN); //Sensor 11
  pinMode(sensor_12, INPUT_PULLDOWN); //Sensor 12

  pinMode(DIP_SW_ADDRESS_0_LSB, INPUT_PULLDOWN);
  pinMode(DIP_SW_ADDRESS_1,     INPUT_PULLDOWN);
  pinMode(DIP_SW_ADDRESS_2,     INPUT_PULLDOWN);
  pinMode(DIP_SW_ADDRESS_3,     INPUT_PULLDOWN);
  pinMode(DIP_SW_ADDRESS_4_MSB, INPUT_PULLDOWN);

  pinMode(Heart_Beat_LED_Pin, OUTPUT);

  station_address = calc_address_from_dipSW();
  slave.setUnitAddress(station_address);

  while (0 == 1)
  {
    //    station_address = calc_address_from_dipSW();
    if (serial_degub_enb)
    {
      Serial_debug_port.print("Hello from new board : ");
      Serial_debug_port.println(station_address);
    }
    delay(100);
  }
#endif
  //**********************************************************************

  slave.cbVector[CB_READ_HOLDING_REGISTERS] = READ_HOLDING_REGISTERS;
  slave.cbVector[CB_READ_INPUT_REGISTERS] = READ_INPUT_REGISTERS;
  slave.cbVector[CB_WRITE_COILS] = WRITE_COILS;
  slave.cbVector[CB_READ_COILS] = READ_COILS;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = WRITE_HOLDING_REGISTERS;

  //  slave.setUnitAddress(4);

  Serial3.begin(19200, SERIAL_8E1);
  slave.begin(19200);

  update_reg_values(0, 0.1);
}

void loop()
{

  now = millis();
  if (now - last_sent >= interval)
  {
    last_sent = now;
    Heart_Beat_state ^= 1;
    digitalWrite(Heart_Beat_LED_Pin, Heart_Beat_state);
  }

  slave.poll();

  delay(10);
}

uint8_t WRITE_HOLDING_REGISTERS(uint8_t fc, uint16_t address, uint16_t length)
{
  //  digitalWrite(PC13, LOW);
  if (serial_degub_enb)
  {
    Serial_debug_port.print("fc : ");
    Serial_debug_port.print(fc);
    Serial_debug_port.print(" | address : ");
    Serial_debug_port.print(address);
    Serial_debug_port.print(" | length : ");
    Serial_debug_port.println(length);
  }

  for (int i = address; i < length + address; i++)
  {
    reg[i] = slave.readRegisterFromBuffer(i - address);
    if (serial_degub_enb)
    {
      Serial_debug_port.print(i);
      Serial_debug_port.print(" | ");
      Serial_debug_port.println(reg[i]);
    }
  }
  //  digitalWrite(PC13, HIGH);
  return STATUS_OK;
}




uint8_t READ_COILS(uint8_t fc, uint16_t address, uint16_t length)
{
  //  digitalWrite(PC13, LOW);
  if (serial_degub_enb)
  {
    Serial_debug_port.print("fc : ");
    Serial_debug_port.print(fc);
    Serial_debug_port.print(" | address : ");
    Serial_debug_port.print(address);
    Serial_debug_port.print(" | length : ");
    Serial_debug_port.println(length);
  }
  ////////////////////////////////////////////////////////////
#ifdef four_input_board
  reg[0] = digitalRead(INPUT_PINS[0]);
  reg[1] = digitalRead(INPUT_PINS[1]);
  reg[2] = digitalRead(INPUT_PINS[2]);
  reg[3] = digitalRead(INPUT_PINS[3]);
#endif
  ////////////////////////////////////////////////////////////

  reg[4] = 0;
  reg[5] = 1;
  reg[6] = 0;
  reg[7] = 1;
  reg[8] = 0;
  reg[9] = 1;

  for (int i = address; i < length + address; i++)
  {
    slave.writeCoilToBuffer(i - address, reg[i]);
  }
  //  digitalWrite(PC13, HIGH);
  return STATUS_OK;
}


uint8_t WRITE_COILS(uint8_t fc, uint16_t address, uint16_t length)
{
  //  digitalWrite(PC13, LOW);
  {
    Serial_debug_port.print("fc : ");
    Serial_debug_port.print(fc);
    Serial_debug_port.print(" | address : ");
    Serial_debug_port.print(address);
    Serial_debug_port.print(" | length : ");
    Serial_debug_port.println(length);
  }
  if (address == 13)
  {
    //    pinMode(PC13, OUTPUT);
    //    digitalWrite(PC13, slave.readCoilFromBuffer(0));
  }

  for (int i = address; i < length + address; i++)
  {
    if (serial_degub_enb)
    {
      Serial_debug_port.print(i);
      Serial_debug_port.print(" | ");
      Serial_debug_port.println(slave.readCoilFromBuffer(i - address));
    }
  }
  //  digitalWrite(PC13, HIGH);
  return STATUS_OK;
}


uint8_t READ_HOLDING_REGISTERS(uint8_t fc, uint16_t address, uint16_t length)
{
  //  digitalWrite(PC13, LOW);
  if (serial_degub_enb)
  {
    Serial_debug_port.print("fc : ");
    Serial_debug_port.print(fc);
    Serial_debug_port.print(" | address : ");
    Serial_debug_port.print(address);
    Serial_debug_port.print(" | length : ");
    Serial_debug_port.println(length);
  }

  for (int i = address; i < length + address; i++)
  {
    slave.writeRegisterToBuffer(i - address, reg[i]);
  }
  //  digitalWrite(PC13, HIGH);
  return STATUS_OK;
}

uint8_t READ_INPUT_REGISTERS(uint8_t fc, uint16_t address, uint16_t length)
{
  //  digitalWrite(PC13, LOW);
  if (serial_degub_enb)
  {
    Serial_debug_port.print("fc : ");
    Serial_debug_port.print(fc);
    Serial_debug_port.print(" | address : ");
    Serial_debug_port.print(address);
    Serial_debug_port.print(" | length : ");
    Serial_debug_port.println(length);
  }
  data[0] = analogRead(0);
  data[1] = millis();
  val.floatformat = 3.3 * (data[0] / 4096.0);
  data[2] = val.byteformat[0];
  data[3] = val.byteformat[1];

  reg[4000] = data[0];
  reg[4001] = data[1];
  reg[4002] = data[2];
  reg[4003] = data[3];

  reg[2000] = 40;
  reg[2001] = 41;
  reg[2002] = 42;
  reg[2003] = 43;
  reg[2004] = 44;
  reg[2005] = 45;
  reg[2006] = 46;
  reg[2007] = 47;
  reg[2008] = 48;
  reg[2009] = 49;

  val.floatformat = 4.1;
  reg[3000] = val.byteformat[0];
  reg[3001] = val.byteformat[1];

  val.floatformat = 4.2;
  reg[3002] = val.byteformat[0];
  reg[3003] = val.byteformat[1];

  val.floatformat = 4.3;
  reg[3004] = val.byteformat[0];
  reg[3005] = val.byteformat[1];

  val.floatformat = 4.4;
  reg[3006] = val.byteformat[0];
  reg[3007] = val.byteformat[1];

  for (int i = address; i < length + address; i++)
  {
    slave.writeRegisterToBuffer(i - address, reg[i]);
  }
  //  digitalWrite(PC13, HIGH);
  return STATUS_OK;
}

//**********************************************************************
#ifdef twelve_input_board
uint16_t calc_address_from_dipSW()
{
  uint16_t adr = 0;

  ADDRESS_BIT_0 = digitalRead(DIP_SW_ADDRESS_0_LSB);
  ADDRESS_BIT_1 = digitalRead(DIP_SW_ADDRESS_1);
  ADDRESS_BIT_2 = digitalRead(DIP_SW_ADDRESS_2);
  ADDRESS_BIT_3 = digitalRead(DIP_SW_ADDRESS_3);
  ADDRESS_BIT_4 = digitalRead(DIP_SW_ADDRESS_4_MSB);

  adr = ADDRESS_BIT_0 + ADDRESS_BIT_1 * 2 + ADDRESS_BIT_2 * 4 + ADDRESS_BIT_3 * 8 + ADDRESS_BIT_4 * 16;

  return adr;
}
#endif
//**********************************************************************

void update_reg_values(float start_val, float step_val)
{
  int start_add = 3000;
  int inc_index = 0;

  for (int i = start_add; i <= 3194; i += 2)
  {
    if (i == 3034)
    {}
    else if (i == 3192)
    {
      val.floatformat = -1;
      reg[i]     = val.byteformat[0];
      reg[i + 1] = val.byteformat[1];
    }
    else if (i >= 3094 && i <= 3108)
    {}
    else if (i >= 3112 && i <= 3190)
    {}
    else
    {
      inc_index ++;

      val.floatformat = start_val + inc_index * step_val;
      reg[i]     = val.byteformat[0];
      reg[i + 1] = val.byteformat[1];
      if (serial_degub_enb)
      {
        Serial_debug_port.print(inc_index);
        Serial_debug_port.print(" | ");
        Serial_debug_port.print(i);
        Serial_debug_port.print(" : ");
        Serial_debug_port.println(val.floatformat);
      }
    }
  }

  //  Serial_debug_port.println("***********************");

  for (int i = 3196; i <= 3197; i += 1)
  {
    if (i == 3196 || i == 3197)
    {
      reg[i] = -1;
    }
    else
    {
      inc_index ++;
      reg[i] = start_val + inc_index;
      if (serial_degub_enb)
      {
        Serial_debug_port.print(inc_index);
        Serial_debug_port.print(" | ");
        Serial_debug_port.print(i);
        Serial_debug_port.print(" : ");
        Serial_debug_port.println(reg[i]);
      }
    }
  }

  //  Serial_debug_port.println("***********************");

  for (int i = 3200; i <= 3550; i += 4)
  {
    if (i >= 3252 && i <= 3268)
    {}

    else if (i >= 3288 && i <= 3300)
    {}

    else if (i >= 3352 && i <= 3510)
    {
      i += 2;
    }

    else if (i == 3514 || i == 3200)
    {
      val2.int64format = -1;
      reg[i]     = val2.byteformat[0];
      reg[i + 1] = val2.byteformat[1];
      reg[i + 2] = val2.byteformat[2];
      reg[i + 3] = val2.byteformat[3];
    }

    else
    {
      inc_index ++;

      val2.int64format = start_val + inc_index;
      reg[i]     = val2.byteformat[0];
      reg[i + 1] = val2.byteformat[1];
      reg[i + 2] = val2.byteformat[2];
      reg[i + 3] = val2.byteformat[3];
      if (serial_degub_enb)
      {
        Serial_debug_port.print(inc_index);
        Serial_debug_port.print(" | ");
        Serial_debug_port.print(i);
        Serial_debug_port.print(" : ");
        Serial_debug_port.println(reg[i]);
      }
    }
  }
}
