| <center><img src="./assets/rakstar.jpg" alt="RAKstar" width=25%></center>  | ![RAKWireless](./assets/RAK-Whirls.png) | [![Build Status](https://github.com/RAKWireless/RAK12021-TCS37725/workflows/RAK%20Library%20Build%20CI/badge.svg)](https://github.com/RAKWireless/RAK12021-TCS37725/actions) |
| -- | -- | -- |

# RAK12021 RGBC Sensor

Simple Arduino Library for the WisBlock I2C RGBC Sensor version from [RAKwireless](https://docs.rakwireless.com/Product-Categories/WisBlock) which works really great and is ready to use with I2C.

[*RAKwireless <RAK#> <function>*](https://store.rakwireless.com/products/rgb-sensor-rak12021)

# Documentation

* **[Product Repository](https://github.com/RAKWireless/RAK12021-TCS37725)** - Product repository for the RAKWireless RAK12021 RGB Light Sensor.
* **[Documentation](https://docs.rakwireless.com/Product-Categories/WisBlock/RAK12021/Overview/)** - Documentation and Quick Start Guide for the RAK12021 RGB Light Sensor.

# Installation

In Arduino IDE open Sketch->Include Library->Manage Libraries then search for RAK12021.    

In PlatformIO open PlatformIO Home, switch to libraries and search for RAK12021. 
Or install the library project depend by adding 

```log
lib_deps =
  rakwireless/RAK12021 TCS37725
```
into **`platformio.ini`**

For manual installation download the archive, unzip it and place the RAK12021-TCS37725 folder into the library directory.    
In Arduino IDE this is usually <arduinosketchfolder>/libraries/    
In PlatformIO this is usually <user/.platformio/lib>     

# Usage

The library provides an interface class, which allows communication to the TCS37725 Soil Moisture sensor over I2C. Check out the examples how to use the RGBC Sensor.  

- [RAK12021_TCS37725_RGBC](./examples/RAK12021_TCS37725_RGBC) simply reads the red, green, blue, and clear (RGBC) value.  If the light intensity exceeds a certain threshold, an interrupt is triggered.

## This class provides the following methods:
**TCS3772(byte addr = I2C_ADDRESS);**     
Constructor for TCS3772 sensor. Initialize TCS3772 object. 
Parameters:    

| Direction | Name | Function |
| --------- | ---- | -------- |
| in        | I2C_ADDRESS | Sensor IIC address. Default is 0x29. |
|  return |  | none  |

**bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = I2C_ADDRESS);**    
Initialize the device. 
Parameters:    

| Direction | Name | Function |
| --------- | ---- | -------- |
| in          | wirePort | IIC port ,Default is Wire. |
| in | I2C_ADDRESS | Sensor IIC address. Default is 0x29. |
|  return | | If the initialization succeeds return true else return false  |

**void setClearLowThreshold(uint16_t para);**    
Set the lower limit of the light intensity trigger threshold.    
Parameters:    

| Direction | Name | Function |
| --------- | ---- | -------- |
| in          | para | Lower threshold value |
|  return | | none |

**void setClearHighThreshold(uint16_t para);**    
Set the upper limit of the light intensity trigger threshold.  
Parameters:    

| Direction | Name | Function               |
| --------- | ---- | ---------------------- |
| in        | para | Upper threshold value. |
| return    |      | none                   |

**void enableClearINT(void);**    
Enable light intensity interrupt.  
Parameters:    

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void disableClearINT(void);**      
Disable light intensity interrupt.  
Parameters:    

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**void clearAllInterrupt(void);**     
Clear all interrupt flags.  
Parameters:    

| Direction | Name | Function |
| --------- | ---- | -------- |
| return    |      | none     |

**uint8_t getInterruptSrc(void);**    
Get interrupt status.  
Parameters:    

| Direction | Name | Function |
| --------- | ---- | -------- |
| in          |  | none |
|  return | | Return the value of Status Register(0x13). |

**TCS3772_DataScaled getMeasurement(void);**    
Get sensor RGBC data. 
Parameters:    

| Direction | Name | Function |
| --------- | ---- | -------- |
| in          |  | none |
|  return | | Red, green, blue, and clear (RGBC) light date. <br />@Structure TCS3772_DataScaled |

**uint8_t autoGain(uint16_t val_clear);**    
Automatically set the gain according to the clear light intensity.
Parameters:    

| Direction | Name | Function |
| --------- | ---- | -------- |
| in          | val_clear | Clear value. |
|  return | | Gain value for automatic gain selection. |
