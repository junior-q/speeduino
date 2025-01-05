/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
/** @file
 * Lower level ConfigPage*, Table2D, Table3D and EEPROM storage operations.
 */

#include "globals.h"
#include EEPROM_LIB_H //This is defined in the board .h files
#include "config.h"
#include "storage.h"
#include "pages.h"
#include "tables/table3d_axis_io.h"


#if defined(CORE_AVR)
#pragma GCC push_options
// This minimizes RAM usage at no performance cost
#pragma GCC optimize ("Os") 
#endif


uint32_t deferEEPROMWritesUntil = 0;

bool isEepromWritePending(void)
{
  return BIT_CHECK(currentStatus.status4, BIT_STATUS4_BURNPENDING);
}


void storageControl(void)
{
	if(currentLoopTime > micros_safe())
	{
		//Occurs when micros() has overflowed
		deferEEPROMWritesUntil = 0; //Required to ensure that EEPROM writes are not deferred indefinitely
	}
}


//Simply an alias for EEPROM.update()
void EEPROMWriteRaw(uint16_t address, uint8_t data)
{
	EEPROM.update(address, data);
}

uint8_t EEPROMReadRaw(uint16_t address)
{
	return EEPROM.read(address);
}


static eeprom_address_t compute_crc_address(uint8_t pageNum)
{
  return EEPROM_LAST_BARO-((getPageCount() - pageNum)*sizeof(uint32_t));
}

/** Write CRC32 checksum to EEPROM.
Takes a page number and CRC32 value then stores it in the relevant place in EEPROM
@param pageNum - Config page number
@param crcValue - CRC32 checksum
*/
void storePageCRC32(uint8_t pageNum, uint32_t crcValue)
{
  EEPROM.put(compute_crc_address(pageNum), crcValue);
}

/** Retrieves and returns the 4 byte CRC32 checksum for a given page from EEPROM.
@param pageNum - Config page number
*/
uint32_t readPageCRC32(uint8_t pageNum)
{
  uint32_t crc32_val;
  return EEPROM.get(compute_crc_address(pageNum), crc32_val);
}

/** Same as above, but writes the CRC32 for the calibration page rather than tune data
@param calibrationPageNum - Calibration page number
@param calibrationCRC - CRC32 checksum
*/
void storeCalibrationCRC32(uint8_t calibrationPageNum, uint32_t calibrationCRC)
{
  uint16_t targetAddress;
  switch(calibrationPageNum)
  {
    case O2_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_O2_CRC;
      break;
    case IAT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_IAT_CRC;
      break;
    case CLT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC;
      break;
    default:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC; //Obviously should never happen
      break;
  }

  EEPROM.put(targetAddress, calibrationCRC);
}

/** Retrieves and returns the 4 byte CRC32 checksum for a given calibration page from EEPROM.
@param calibrationPageNum - Config page number
*/
uint32_t readCalibrationCRC32(uint8_t calibrationPageNum)
{
  uint32_t crc32_val;
  uint16_t targetAddress;
  switch(calibrationPageNum)
  {
    case O2_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_O2_CRC;
      break;
    case IAT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_IAT_CRC;
      break;
    case CLT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC;
      break;
    default:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC; //Obviously should never happen
      break;
  }

  EEPROM.get(targetAddress, crc32_val);
  return crc32_val;
}

uint16_t getEEPROMSize(void)
{
  return EEPROM.length();
}



#if defined(CORE_AVR)
#pragma GCC pop_options
#endif
