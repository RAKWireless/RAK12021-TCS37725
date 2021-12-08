/**
   @file TCS3772.cpp
   @author rakwireless.com
   @brief AMS TCS3772 Color Light-to-Digital Converter with Proximity Sensing IC library.
          Operation function implementation.
   @version 0.1
   @date 2021-11-08
   @copyright Copyright (c) 2021
**/

#include "TCS3772.h"

/*!
 *  @brief  Initialize the class.
 *  @param  addr: The variable address should be 0x29. 
 */
TCS3772::TCS3772(byte addr)
{

#if defined(USE_SW_IIC)

  IIC = new IOIIC(PIN_WIRE_SDA, PIN_WIRE_SCL);
  IIC->IIC_Init(addr);
  _writeAddress = (addr<<1);
  _readAddress = ((addr<<1) + 1 );
  
#else

  _deviceAddress = addr;
  
#endif
}

/*!
 *  @brief  I2c bus write.
 *  @param  registerAddress : Register address.
 *  @param  writeData		    : Write data pointer.
 *  @param  size			      : The length of the written data.
 */
void TCS3772::writeRegister(uint8_t registerAddress, uint8_t *writeData, uint8_t size)
{

#if defined(USE_SW_IIC)
  IIC->IIC_Start();  
  IIC->IIC_Send_Byte( _writeAddress );
  IIC->IIC_Wait_Ack();
//  if(IIC->IIC_Wait_Ack())
//    return NACK;
    
  IIC->IIC_Send_Byte( registerAddress );  
  IIC->IIC_Wait_Ack();     
//  if(IIC->IIC_Wait_Ack())
//    return NACK;

  for(uint8_t i = 0; i < size; i++)
  {      
    IIC->IIC_Send_Byte( writeData[i] ); 
    IIC->IIC_Wait_Ack();   
  }   
//  if(IIC->IIC_Wait_Ack())
//    return NACK;
  IIC->IIC_Stop();
//  return ACK; 
#else
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(registerAddress);
  for (size_t i = 0; i < size; i++)
  {
    _i2cPort->write(writeData[i]);
  }
  _i2cPort->endTransmission();
#endif
}

/*!
 *  @brief  I2c bus read.
 *  @param  registerAddress : Register address.
 *  @param  readData		    : Read data pointer.
 *  @param  size			      : The length of the written data.
 */
void TCS3772::readRegister(uint8_t registerAddress, uint8_t *readData, uint8_t size)
{  
#if defined(USE_SW_IIC)
  IIC->IIC_Start();  
  IIC->IIC_Send_Byte( _writeAddress );
  IIC->IIC_Wait_Ack();
//  if(IIC->IIC_Wait_Ack())
//    return NACK;
  IIC->IIC_Send_Byte( registerAddress );   
  IIC->IIC_Wait_Ack();   
//  if(IIC->IIC_Wait_Ack())
//    return NACK;
  IIC->IIC_Stop();
  delayMicroseconds(DELYA); 
  IIC->IIC_Start();
  IIC->IIC_Send_Byte( _readAddress );
  IIC->IIC_Wait_Ack();
//  if(IIC->IIC_Wait_Ack())
//    return NACK; 
    
  for(uint8_t i = 0; i < size; i++)
  {    
    readData[i] = IIC->IIC_Read_Byte(0);
  } 
       
  IIC->IIC_Stop();
#else

  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(registerAddress);
  _i2cPort->endTransmission(true); 
  _i2cPort->requestFrom(_deviceAddress, size);
  size_t i = 0;
  while ( _i2cPort->available() ) // slave may send less than requested
  {
    readData[i] = _i2cPort->read(); // receive a byte as a proper uint8_t
    i++;
  }
#endif
}

/*!
 *  @brief  Set bits of the register without changing the value of other bits.
 *  @param  address : Register address.
 *  @param  mask	: Set mask.
 *  @param  val		: Set value.
 */
void TCS3772::setRegister(uint8_t address, uint8_t mask, uint8_t val)
{
  uint8_t i2cData = 0;
  readRegister(address , &i2cData , 1);
  i2cData &= ~mask;
  i2cData |= (val & mask);
  writeRegister(address, &i2cData , 1);  // Set the internal integration time as 2.4*64 ms.;
}

/*!
 *  @brief  Initalizes the TCS3772 sensor.
 *  @param  wirePort      : IIC interface used.
 *  @param  deviceAddress : Device address should be 0x29. 
 *  @return If the device init successful return true else return false.
 */
bool TCS3772::begin(TwoWire &wirePort, uint8_t deviceAddress)
{
  uint8_t i2cData = 0;
  uint8_t _sensor_id;
  
//#if defined(_VARIANT_RAK11300_)
//  gpio_set_function(I2C_SDA,  GPIO_FUNC_SIO); 
//  gpio_set_function(I2C_SCL,  GPIO_FUNC_SIO); 
//#endif 
  
  // Set device address and wire port to private variable
  _deviceAddress = deviceAddress;

  _i2cPort = &wirePort;

  LIB_LOG("begin", "DeviceAddress = 0x%X", _deviceAddress);
//  if (isConnected() == false)
//  {
//    return false;
//  }

  readRegister(REG_ID | CMD_INCREMENT , &_sensor_id , 1);      
  LIB_LOG("begin", "ID = 0x%X", _sensor_id);
  delay(1000);  

  i2cData = MASK_AEN | MASK_PON | MASK_PEN;
  setRegister(REG_ENABLE | CMD_INCREMENT, MASK_AEN | MASK_PON | MASK_PEN, i2cData);

  i2cData = 0xFF;
  setRegister(0x02 | CMD_INCREMENT , i2cData , i2cData);// Set Proximity Time Register (0x02) to 0xFF.

  i2cData = 0x5A;
  setRegister(REG_PULSE_COUNT | CMD_INCREMENT , i2cData , i2cData);// Set Proximity Time Register (0x02) to 0xFF.

//  i2cData = 0x00;
//  setRegister(0x0F | CMD_INCREMENT , 0xC0 , i2cData);

  setATime(2.4*64);  // Set the internal integration time as 2.4*64 ms.
  setAGain(1);
  setWaitTime(0);

  if(_sensor_id == VAL_DEVICE_ID_A) 
    return true;
  else if(_sensor_id == VAL_DEVICE_ID_B) 
    return true;
  else                                    
    return false;
}

/*!
 *  @brief  Initalizes the TCS3772 sensor.
 *  @param  NULL.
 *  @return Red, green, blue, and clear (RGBC) light date. @Structure TCS3772_DataScaled
 */
TCS3772_DataScaled TCS3772::getMeasurement(void)
{  
  uint8_t i2cData[1];
  TCS3772_DataScaled scaledData;
  
  readRegister(REG_CLEAR_DATAL | CMD_INCREMENT, i2cData, 1);
  scaledData.clear  = i2cData[0];

  readRegister(REG_CLEAR_DATAH | CMD_INCREMENT, i2cData, 1);
  scaledData.clear  += (i2cData[0]<<8);

  readRegister(REG_RED_DATAL | CMD_INCREMENT, i2cData, 1);
  scaledData.red  = i2cData[0];

  readRegister(REG_RED_DATAH | CMD_INCREMENT, i2cData, 1);
  scaledData.red  += (i2cData[0]<<8);

  readRegister(REG_GREEN_DATAL | CMD_INCREMENT, i2cData, 1);
  scaledData.green  = i2cData[0];

  readRegister(REG_GREEN_DATAH | CMD_INCREMENT, i2cData, 1);
  scaledData.green  += (i2cData[0]<<8);

  readRegister(REG_BLUE_DATAL | CMD_INCREMENT, i2cData, 1);
  scaledData.blue  = i2cData[0];

  readRegister(REG_BLUE_DATAH | CMD_INCREMENT, i2cData, 1);
  scaledData.blue  += (i2cData[0]<<8);

  readRegister(REG_PROX_DATAL | CMD_INCREMENT, i2cData, 1);
  scaledData.prox  = i2cData[0];

  readRegister(REG_PROX_DATAH | CMD_INCREMENT, i2cData, 1);
  scaledData.prox  += (i2cData[0]<<8);

//  readRegister(REG_PROX_DATAL | CMD_INCREMENT, i2cData, 2);
//  scaledData.prox   = (i2cData[1]<<8) + i2cData[0];
  
  return scaledData;
}

/*!
 *  @brief  Set automatic gain.
 *  @param  Clear value.
 *  @return Gain value for automatic gain selection.
 */
uint8_t TCS3772::autoGain(uint16_t val_clear)
{
  static uint16_t val_last;

  static const uint16_t  MARGIN_LOW   =(5000);
  static const uint16_t  MARGIN_HIGH  =(0xFFFF - MARGIN_LOW);

  // val_gain: 0=G1,  1=G4,   2=G16, 3=G60
  // val_time: 0=i64, 1=i128, 2=i256
  if ((val_clear != val_last) || (val_clear == 0xFFFF))
  {
    static uint8_t val_gain, val_time, gain;
    val_last = val_clear;
    if(val_clear < MARGIN_LOW)
    {
      if (val_gain<3) // first try to raise gain, before raising integrationtime
      {
        val_gain++;
        gain = 1<<(2*val_gain);
        gain = setAGain(gain);
        return 0;
      }
      else if (val_time<2)
      {
        val_time++;
        uint16_t time;
        time = 1<<(val_time);
        setATime(2.4*64*time);
        return 0;
      }
    }
    else if (val_clear > MARGIN_HIGH)
    {
      if (val_time>0)
      {
        val_time--;
        uint16_t time;
        time = 1<<(val_time);
        setATime(2.4*64*time);
        return 0;
      }
      else if (val_gain>0)
      {
        val_gain--;
        gain = 1<<(2*val_gain);
        gain = setAGain(gain);
        return 0;
      }
    }
    return (gain*(1<<val_time));
  }
  return 0;
}

uint8_t TCS3772::MSEC_TO_ATIME(const uint16_t msec)
{
  return (256 - (msec/2.4));
}

uint8_t TCS3772::MSEC_TO_ATIMELONG(const uint16_t msec)
{
  return (256 - (msec/28.8));
}

/*!
 *  @brief  Set integration time.
 *  @param  Integration time, unit:ms.
 *  @return NULL.
 */
void TCS3772::setATime(const uint16_t integrationtime_ms)
{
  uint8_t _value;
  if (integrationtime_ms > 614)      _value = VAL_MAX;
  else if (integrationtime_ms < 4)   _value = VAL_MIN;
  else                               _value = (uint8_t) MSEC_TO_ATIME(integrationtime_ms);
  writeRegister( REG_ATIME  | CMD_INCREMENT, &_value , 1);
}

/*!
 *  @brief  Set gain.
 *  @param  Gain value.
 *  @return Gain value.
 */
uint8_t TCS3772::setAGain(uint8_t gain)
{
  uint8_t _valueA, _valueB ;

  if(gain < 4)
  {
    _valueA = VAL_AGAIN_01;
    _valueB = 1;
  }
  else if(gain < 16)
  {
    _valueA = VAL_AGAIN_04;
    _valueB = 4;
  }
  else if(gain < 60)
  {
    _valueA = VAL_AGAIN_16;
    _valueB = 16;
  }
  else
  {
    _valueA = VAL_AGAIN_60;
    _valueB = 60;
  }
  setRegister(TCS3772_REG_CONTROL | CMD_INCREMENT, MASK_AGAIN, _valueA);
  return _valueB;
}

/*!
 *  @brief  Set waiting time.
 *  @param  Wait value.
 *  @return NULL.
 */
void TCS3772::setWaitTime(uint16_t wait)
{
  uint8_t _valueA, _valueB, _valueC;
  if (wait > 7372)
  {
    _valueA = 255;
    _valueB = 255;
    _valueC = VAL_MAX;
  }
  else if (wait > 614)
  {
    _valueA =  255;
    _valueB =  255;
    _valueC =  MSEC_TO_ATIMELONG(wait);
  }
  else if (wait < 4)
  {
    _valueA =  0;
    _valueB =  0;
    _valueC =  VAL_MIN;
  }
  else
  {
    _valueA = 255;
    _valueB = 0;
    _valueC = MSEC_TO_ATIME(wait);
  }
  
  setRegister(REG_ENABLE | CMD_INCREMENT, MASK_WEN,   _valueA);
  setRegister(REG_CONFIG | CMD_INCREMENT, MASK_WLONG, _valueB);
  writeRegister(REG_WTIME | CMD_INCREMENT, &_valueC, 1);
}

/*!
 *  @brief  Set the lower limit of the proximity interrupt trigger threshold.
 *  @param  Lower threshold value.
 *  @return NULL.
 */
void TCS3772::setProxLowThreshold(uint16_t para)
{
  uint8_t i2cData = 0;
  uint8_t ProxLowH;
  uint8_t ProxLowL; 
  
  i2cData = (uint8_t)(para & 0xff);   
  writeRegister(REG_PILTL | CMD_INCREMENT, &i2cData, 1);
  i2cData = (uint8_t)(para / 256U) & 0x7FU; 
  writeRegister(REG_PILTH | CMD_INCREMENT, &i2cData, 1);

  readRegister(REG_PILTH | CMD_INCREMENT , &ProxLowH , 1);
  LIB_LOG("setProxLowThreshold", "ProxLowH = 0x%X", ProxLowH);

  readRegister(REG_PILTL | CMD_INCREMENT , &ProxLowL , 1);
  LIB_LOG("setProxLowThreshold", "ProxLowL = 0x%X", ProxLowL);
}

/*!
 *  @brief  Set the upper limit of the proximity interrupt trigger threshold.
 *  @param  Upper threshold value.
 *  @return NULL.
 */
void TCS3772::setProxHighThreshold(uint16_t para)
{
  uint8_t i2cData = 0;
  uint8_t ProxHighH;
  uint8_t ProxHighL; 
  
  i2cData = (uint8_t)(para & 0xff);   
  writeRegister(REG_PIHTL | CMD_INCREMENT, &i2cData, 1);
  i2cData = (uint8_t)(para / 256U) & 0x7FU; 
  writeRegister(REG_PIHTH | CMD_INCREMENT, &i2cData, 1);

  readRegister(REG_PIHTH | CMD_INCREMENT , &ProxHighH , 1);
  LIB_LOG("setProxHighThreshold", "ProxHighH = 0x%X", ProxHighH);

  readRegister(REG_PIHTL | CMD_INCREMENT , &ProxHighL , 1);
  LIB_LOG("setProxHighThreshold", "ProxHighL = 0x%X", ProxHighL);
}

/*!
 *  @brief  Set the lower limit of the light intensity trigger threshold.
 *  @param  Lower threshold value.
 *  @return NULL.
 */
void TCS3772::setClearLowThreshold(uint16_t para)
{
  uint8_t i2cData = 0;
  uint8_t ClearLowH;
  uint8_t ClearLowL; 
  
  i2cData = (uint8_t)(para & 0xff);   
  writeRegister(REG_AILTL | CMD_INCREMENT, &i2cData, 1);
  i2cData = (uint8_t)(para / 256U) & 0x7FU; 
  writeRegister(REG_AILTH | CMD_INCREMENT, &i2cData, 1);

  readRegister(REG_AILTH | CMD_INCREMENT , &ClearLowH , 1);
  LIB_LOG("setClearLowThreshold", "ClearLowH = 0x%X", ClearLowH);

  readRegister(REG_AILTL | CMD_INCREMENT , &ClearLowL , 1);
  LIB_LOG("setClearLowThreshold", "ClearLowL = 0x%X", ClearLowL);
}

/*!
 *  @brief  Set the upper limit of the light intensity trigger threshold.
 *  @param  Upper threshold value.
 *  @return NULL.
 */
void TCS3772::setClearHighThreshold(uint16_t para)
{
  uint8_t i2cData = 0;
  uint8_t ClearHighH;
  uint8_t ClearHighL; 
  
  i2cData = (uint8_t)(para & 0xff);   
  writeRegister(REG_AIHTL | CMD_INCREMENT, &i2cData, 1);
  i2cData = (uint8_t)(para / 256U) & 0x7FU; 
  writeRegister(REG_AIHTH | CMD_INCREMENT, &i2cData, 1);

  readRegister(REG_AIHTH | CMD_INCREMENT , &ClearHighH , 1);
  LIB_LOG("setClearLowThreshold", "ClearHighH = 0x%X", ClearHighH);

  readRegister(REG_AIHTL | CMD_INCREMENT , &ClearHighL , 1);
  LIB_LOG("setClearLowThreshold", "ClearHighL = 0x%X", ClearHighL);
}

/*!
 *  @brief  Enable proximity interrupt.
 *  @param  NULL.
 *  @return NULL.
 */
void TCS3772::enableProxINT(void)
{
  uint8_t i2cData = 0;
  uint8_t EnableReg;

/*  Set persistence filter to generate an interrupt for  
 *    1 clear channel value outside of threshold range.
 *    1 proximity value out of range.
 */
  i2cData = 0x11;
  writeRegister(REG_PERS | CMD_INCREMENT, &i2cData, 1);
  
  i2cData = MASK_PIEN;
  setRegister(REG_ENABLE | CMD_INCREMENT, MASK_PIEN, i2cData);

  readRegister(REG_ENABLE | CMD_INCREMENT , &EnableReg , 1);
  LIB_LOG("enableClearINT", "EnableReg = 0x%X", EnableReg);
  
  readRegister(REG_PERS | CMD_INCREMENT , &EnableReg , 1);
  LIB_LOG("enableClearINT", "REG PERS = 0x%X", EnableReg);
}

/*!
 *  @brief  Disable proximity interrupt.
 *  @param  NULL.
 *  @return NULL.
 */
void TCS3772::disableProxINT(void)
{
  uint8_t i2cData = 0;
  setRegister(REG_ENABLE | CMD_INCREMENT, MASK_PIEN, i2cData);
}

/*!
 *  @brief  Enable light intensity interrupt.
 *  @param  NULL.
 *  @return NULL.
 */
void TCS3772::enableClearINT(void)
{
  uint8_t i2cData = 0;
  uint8_t EnableReg;

/*  Set persistence filter to generate an interrupt for  
 *    1 clear channel value outside of threshold range.
 *    1 proximity value out of range.
 */
  i2cData = 0x11;
  writeRegister(REG_PERS | CMD_INCREMENT, &i2cData, 1);
  
  i2cData = MASK_AIEN;
  setRegister(REG_ENABLE | CMD_INCREMENT, MASK_AIEN, i2cData);

//  i2cData = 0x08;
//  setRegister(REG_ENABLE | CMD_INCREMENT, 0x08, i2cData);

  readRegister(REG_ENABLE | CMD_INCREMENT , &EnableReg , 1);
  LIB_LOG("enableClearINT", "EnableReg = 0x%X", EnableReg);
  
  readRegister(REG_PERS | CMD_INCREMENT , &EnableReg , 1);
  LIB_LOG("enableClearINT", "REG PERS = 0x%X", EnableReg);
}

/*!
 *  @brief  Disable light intensity interrupt.
 *  @param  NULL.
 *  @return NULL.
 */
void TCS3772::disableClearINT(void)
{
  uint8_t i2cData = 0;
  setRegister(REG_ENABLE | CMD_INCREMENT, MASK_AIEN, i2cData);
}

/*!
 *  @brief  Get interrupt status.
 *  @param  NULL.
 *  @return Return the value of Status Register(0x13).
 */
uint8_t TCS3772::getInterruptSrc(void)
{
  uint8_t i2cData = 0;
  
  readRegister(TCS3772_REG_STATUS | CMD_INCREMENT , &i2cData , 1);
  LIB_LOG("getInterruptSrc", "STATUS REG = 0x%X", i2cData);
  return i2cData ;
}

/*!
 *  @brief Clear all interrupt flags.
 *  @param  NULL.
 *  @return NULL.
 */
void TCS3772::clearAllInterrupt(void)
{
  uint8_t i2cData = 1;  
//  writeRegister(PROX_INT_CLEAR | CMD_SPECIAL , &i2cData , 1);
//
//  writeRegister(CLEAR_INT_CLEAR | CMD_SPECIAL , &i2cData , 1);
//
  writeRegister(0x67 | CMD_INCREMENT , &i2cData , 0);
}

/*!
 *  @brief  Determine if the device is connected.
 *  @param  NULL.
 *  @return If the device is connected return true else return false.
 */
bool TCS3772::isConnected()
{
  for (byte i = 0; i < 5; i++)
  {
    /* After inspecting with logic analyzer, the device fails
       to connect for unknown reasons. The device typically connects
       after two calls. We included a for loop to allow for 
       multiple calls to the device.
    */
    _i2cPort->beginTransmission((uint8_t)_deviceAddress);
    if (_i2cPort->endTransmission() == 0)
      return (true); 
  }
  return (false);
}
