/**
   @file TCS3772.h
   @author rakwireless.com
   @brief AMS TCS3772 Color Light-to-Digital Converter with Proximity Sensing IC library. 
          TCS3772 Types and Enumerations.
   @version 0.1
   @date 2021-11-08
   @copyright Copyright (c) 2021
**/

#ifndef __TCS3772_H__
#define __TCS3772_H__

#include <Arduino.h>
#include <Wire.h>
#include "stdint.h"


/*
 *	Because the hardware IIC cannot get a valid response and communication on the RAK11300.
 *	So use the software IIC. This can be improved.
 */
#if defined(_VARIANT_RAK11300_) 
  #define USE_SW_IIC    // Use software IIC.
#endif

#if defined(USE_SW_IIC)
  #include "SoftwareIIC.h"
#endif

#ifndef LIB_DEBUG
#define LIB_DEBUG 0   // Debug output set to 0 to disable app debug output.
#endif

#if LIB_DEBUG > 0
  #define LIB_LOG(tag, ...)              \
    do                                   \
    {                                    \
      if (tag)                           \
      Serial.printf("<%s> ", tag);       \
      Serial.printf(__VA_ARGS__);        \
      Serial.printf("\n");               \
    } while (0)
#else
  #define LIB_LOG(...)
#endif

// Define I2C Address
#define    I2C_ADDRESS         (0x29)

// Keept Out: Proximity Feature, Interrupts,
#define    CMD_REPEAT          (B10000000)
#define    CMD_INCREMENT       (B10100000)
#define    CMD_SPECIAL         (B11100000)
#define    PROX_INT_CLEAR      (B00101)
#define    CLEAR_INT_CLEAR     (B00110)

#define    REG_ENABLE          (0x00)
#define    VAL_PWR_ON          (0x01)
#define    VAL_PWR_OFF         (0x00)
#define    MASK_PON            (0x01)     // POWER ON
#define    MASK_AEN            (0x02)     // RGBC-Sensor Enable
#define    MASK_PEN            (0x04)     // Proximity Enable
#define    MASK_WEN            (0x08)     // WAIT Enable
#define    MASK_AIEN           (0x10)     // Clear channel interrupt enable
#define    MASK_PIEN           (0x20)     // Proximity interrupt enable

#define    REG_ATIME           (0x01)     // Integration time in 2.4ms Steps
#define    VAL_MIN             (0xFF)     // 2.4ms
#define    VAL_MAX             (0x00)     // 614ms
#define    REG_WTIME           (0x03)     // WLONG0 2.4ms Steps, WLONG1 28.8ms Steps

#define    REG_AILTL           (0x04)     // Clear channel low threshold lower byte
#define    REG_AILTH           (0x05)     // Clear channel low threshold upper byte
#define    REG_AIHTL           (0x06)     // Clear channel high threshold lower byte
#define    REG_AIHTH           (0x07)     // Clear channel high threshold upper byte

#define    REG_PILTL           (0x08)     // Proximity ADC channel low threshold lower byte
#define    REG_PILTH           (0x09)     // Proximity ADC channel low threshold upper byte
#define    REG_PIHTL           (0x0A)     // Proximity ADC channel high threshold lower byte
#define    REG_PIHTH           (0x0B)     // Proximity ADC channel high threshold upper byte

#define    REG_CONFIG          (0x0D)
#define    MASK_WLONG          (bit(1))    // Factor 12x for WTIME
#define    REG_PERS            (0x0C)
#define    REG_PULSE_COUNT     (0x0E)
#define    TCS3772_REG_CONTROL         (0x0F)
#define    MASK_AGAIN          (0x03)
#define    VAL_AGAIN_01        (0x00)
#define    VAL_AGAIN_04        (0x01)
#define    VAL_AGAIN_16        (0x02)
#define    VAL_AGAIN_60        (0x03)

#define    REG_ID              (0x12)
#define    VAL_DEVICE_ID_A     (0x40)       // TCS37725
#define    VAL_DEVICE_ID_B     (0x49)       // TCS37727

#define    TCS3772_REG_STATUS          (0x13)
#define    MASK_AVALID         (bit(0))     // cylce completed since AEN1

#define    REG_CLEAR_DATAL     (0x14)
#define    REG_CLEAR_DATAH     (0x15)

#define    REG_RED_DATAL       (0x16)
#define    REG_RED_DATAH       (0x17)

#define    REG_GREEN_DATAL     (0x18)
#define    REG_GREEN_DATAH     (0x19)

#define    REG_BLUE_DATAL      (0x1A)
#define    REG_BLUE_DATAH      (0x1B)

#define    REG_PROX_DATAL      (0x1C)
#define    REG_PROX_DATAH      (0x1D)

// Interrupt source.
typedef enum TCS3772_INT_CTR
{
  TCS3772_AINT  = bit(4),    // Clear channel Interrupt.
  TCS3772_PINT  = bit(5),    // Proximity Interrupt. 
}TCS3772_INT_CTR;

typedef struct
{
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t clear;
  uint16_t prox;
}TCS3772_DataScaled;

class TCS3772
{
public:
  TCS3772(byte addr = I2C_ADDRESS);
  bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = I2C_ADDRESS);

  void setClearLowThreshold(uint16_t para);
  void setClearHighThreshold(uint16_t para);

  void enableClearINT(void);
  void disableClearINT(void);
  void clearAllInterrupt(void);
  uint8_t getInterruptSrc(void);
  
  TCS3772_DataScaled getMeasurement(void);

  uint8_t autoGain(uint16_t val_clear);

private:
  uint8_t setAGain(uint8_t gain);
  void setATime(const uint16_t integrationtime_ms);
  void setWaitTime(uint16_t wait);
  bool isConnected(void);
  uint8_t MSEC_TO_ATIME(const uint16_t msec);
  uint8_t MSEC_TO_ATIMELONG(const uint16_t msec);
  void readRegister(uint8_t registerAddress, uint8_t *readData, uint8_t size);
  void writeRegister(uint8_t registerAddress, uint8_t *writeData, uint8_t size);
  
  void enableProxINT(void);
  void disableProxINT(void);
  void setProxLowThreshold(uint16_t para);
  void setProxHighThreshold(uint16_t para);
  void setRegister(uint8_t address, uint8_t mask, uint8_t val);
  TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
  uint8_t _deviceAddress;
#if defined(USE_SW_IIC)
  class IOIIC * IIC;
  uint8_t _writeAddress;
  uint8_t _readAddress;
#endif
};

#endif
