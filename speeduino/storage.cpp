/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart

small changes by junior-q:
- encapsulated function into a class in order to protect static funct
- inserted embedded storage for direct access to an external fram:
  since this is not a library but a storage service of the application,
  the fram access has been embedded

A full copy of the license may be found in the projects root directory
*/
/** @file
 * Lower level ConfigPage*, Table2D, Table3D and EEPROM storage operations.
 */

#include "globals.h"
#include "config.h"
#include "pages.h"
#include "tables/table3d_axis_io.h"

#if defined(USE_EEPROM_FOR_STORAGE)
#include <EEPROM.h>
#endif

#include "storage.h"


#if defined(CORE_AVR)
#pragma GCC push_options
// This minimizes RAM usage at no performance cost
#pragma GCC optimize ("Os") 
#endif


#define FRAM_DUMMYBYTE 0xFE	//dummy bytes to make easier to sniff

#define FRAM_CMD_WREN  0x06	//write enable
#define FRAM_CMD_WRDI  0x04	//write disable
#define FRAM_CMD_RDSR  0x05	//read status reg
#define FRAM_CMD_WRSR  0x01	//write status reg
#define FRAM_CMD_READ  0x03
#define FRAM_CMD_WRITE 0x02

StorageClass Storage;

uint32_t deferEEPROMWritesUntil;

StorageClass::StorageClass()
{
	deferEEPROMWritesUntil = 0;
}

bool StorageClass::isEepromWritePending(void)
{
  return BIT_CHECK(currentStatus.status4, BIT_STATUS4_BURNPENDING);
}


void StorageClass::storageControl(void)
{
	if(currentLoopTime > micros_safe())
	{
		//Occurs when micros() has overflowed
		deferEEPROMWritesUntil = 0; //Required to ensure that EEPROM writes are not deferred indefinitely
	}

	// Check for any outstanding EEPROM writes.
//    if( (isEepromWritePending() == true) && (serialStatusFlag == SERIAL_INACTIVE) && (micros() > deferEEPROMWritesUntil))

		// TODO: verificare comunicazione seriale
	if( (isEepromWritePending() == true) )
    {
    	writeAllConfig();
    }

}


//Simply an alias for EEPROM.update()
void StorageClass::EEPROMWriteRaw(uint16_t address, uint8_t data)
{
#if defined(USE_FRAM_FOR_STORAGE)
uint8_t buf[6];

	  digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 0);
	  digitalWrite(FRAM_PIN_WE_FOR_STORAGE, 1);

	  buf[0] = FRAM_CMD_WRITE;
	  buf[1] = address >> 8;
	  buf[2] = address;
	  buf[3] = data;

	  FRAM_SPI_FOR_STORAGE.transfer(buf, 4, SPI_CONTINUE);

	  digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 1);
	  digitalWrite(FRAM_PIN_WE_FOR_STORAGE, 0);

#endif

#if defined(USE_EEPROM_FOR_STORAGE)
	EEPROM.update(address, data);
#endif
}

uint8_t StorageClass::EEPROMReadRaw(uint16_t address)
{
#if defined(USE_FRAM_FOR_STORAGE)
uint8_t wr_buf[6];

	  digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 0);

	  wr_buf[0] = FRAM_CMD_READ;
	  wr_buf[1] = address >> 8;
	  wr_buf[2] = address;

	  FRAM_SPI_FOR_STORAGE.transfer(wr_buf, 3, &wr_buf[3], 1, SPI_CONTINUE);

	  digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 1);

	  return wr_buf[3];
#endif

#if defined(USE_EEPROM_FOR_STORAGE)
	return EEPROM.read(address);
#endif
}


eeprom_address_t  StorageClass::load_block(eeprom_address_t address, void *__p_block, uint16_t _blk_size)
{
 #if defined(CORE_AVR)
   // The generic code in the #else branch works but this provides a 45% speed up on AVR
   size_t size = pLast-pFirst;
   eeprom_read_block(pFirst, (const void*)(size_t)address, size);
   return address+size;
 #else

#if defined(USE_FRAM_FOR_STORAGE)
   uint8_t wr_buf[6];

   	  digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 0);

   	  wr_buf[0] = FRAM_CMD_READ;
   	  wr_buf[1] = address >> 8;
   	  wr_buf[2] = address;

   	  FRAM_SPI_FOR_STORAGE.transfer(wr_buf, 3, __p_block, _blk_size, SPI_CONTINUE);

   	  digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 1);
   	return address;
#endif

#if defined(USE_EEPROM_FOR_STORAGE)
   while( __blk_size )
   {
     *__p_block++ = EEPROM.read(address++);

     __blk_size--;
   }
   return address;
#endif

 #endif
}


/** Load range of bytes form EEPROM offset to memory.
  * @param address - start offset in EEPROM
  * @param pFirst - Start memory address
  * @param pLast - End memory address
  */
eeprom_address_t StorageClass::load_range(eeprom_address_t address, byte *pFirst, const byte *pLast)
{
 #if defined(CORE_AVR)
   // The generic code in the #else branch works but this provides a 45% speed up on AVR
   size_t size = pLast-pFirst;
   eeprom_read_block(pFirst, (const void*)(size_t)address, size);
   return address+size;
 #else

#if defined(USE_FRAM_FOR_STORAGE)
   size_t rd_size = pLast - pFirst;

   	return load_block(address, pFirst, rd_size);
#endif

#if defined(USE_EEPROM_FOR_STORAGE)
   for (; pFirst != pLast; ++address, (void)++pFirst)
   {
     *pFirst = EEPROM.read(address);
   }
   return address;
#endif

 #endif
 }



eeprom_address_t StorageClass::load(table_row_iterator row, eeprom_address_t address)
{
  return load_range(address, &*row, row.end());
}

eeprom_address_t StorageClass::load(table_value_iterator it, eeprom_address_t address)
{
  while (!it.at_end())
  {
    address = load(*it, address);
    ++it;
  }
  return address;
}

eeprom_address_t StorageClass::load(table_axis_iterator it, eeprom_address_t address)
{
const table3d_axis_io_converter converter = get_table3d_axis_converter(it.get_domain());

#if defined(USE_FRAM_FOR_STORAGE)
uint8_t buf[6];
byte val;

	  digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 0);

	  buf[0] = FRAM_CMD_READ;
	  buf[1] = address >> 8;
	  buf[2] = address;

	  FRAM_SPI_FOR_STORAGE.transfer(buf, 3, SPI_CONTINUE);

	  while( !it.at_end() )
	  {
		  val = FRAM_SPI_FOR_STORAGE.transfer(0x88, SPI_CONTINUE);

		  *it = converter.from_byte( val );

		  ++address;
		  ++it;
	  }

	  digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 1);

#endif

#if defined(USE_EEPROM_FOR_STORAGE)
  while( !it.at_end() )
  {
	  *it = converter.from_byte( EEPROM.read(address) );

	  ++address;
	  ++it;
  }
#endif

  return address;
}


eeprom_address_t StorageClass::loadTable(void *pTable, table_type_t key, eeprom_address_t address)
{
  return load(y_rbegin(pTable, key),
                load(x_begin(pTable, key),
                  load(rows_begin(pTable, key), address)));
}


eeprom_address_t StorageClass::write_block(eeprom_address_t address, const void *__p_block, uint16_t _blk_size)
{
#if defined(USE_FRAM_FOR_STORAGE)
	uint8_t buf[6];

	digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 0);
	digitalWrite(FRAM_PIN_WE_FOR_STORAGE, 1);

	buf[0] = FRAM_CMD_WRITE;
	buf[1] = address >> 8;
	buf[2] = address;

	FRAM_SPI_FOR_STORAGE.transfer(buf, 3, SPI_CONTINUE);

	FRAM_SPI_FOR_STORAGE.transfer(__p_block, (size_t) _blk_size, SPI_CONTINUE);

	digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 1);
	digitalWrite(FRAM_PIN_WE_FOR_STORAGE, 0);

#endif

#if defined(USE_EEPROM_FOR_STORAGE)
   while ( location.can_write() && pStart!=pEnd)
   {
     location.update(*pStart);
     ++pStart;
     ++location;
   }
#endif
   return address;

}


write_location StorageClass::write_range(const byte *pStart, const byte *pEnd, write_location location)
{

#if defined(USE_FRAM_FOR_STORAGE)
	size_t wr_size = pEnd - pStart;

	write_block(location.address, pStart, wr_size);
#endif

#if defined(USE_EEPROM_FOR_STORAGE)
   while ( location.can_write() && pStart!=pEnd)
   {
     location.update(*pStart);
     ++pStart;
     ++location;
   }
#endif

   return location;
 }

write_location StorageClass::write(const table_row_iterator &row, write_location location)
 {
   return write_range(&*row, row.end(), location);
 }

write_location StorageClass::write(table_value_iterator it, write_location location)
 {
   while (location.can_write() && !it.at_end())
   {
     location = write(*it, location);
     ++it;
   }
   return location;
 }

write_location StorageClass::write(table_axis_iterator it, write_location location)
 {
   const table3d_axis_io_converter converter = get_table3d_axis_converter(it.get_domain());

#if defined(USE_FRAM_FOR_STORAGE)
   uint8_t wr_buf[6];

   	  digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 0);
  	  digitalWrite(FRAM_PIN_WE_FOR_STORAGE, 1);

   	  wr_buf[0] = FRAM_CMD_WRITE;
   	  wr_buf[1] = location.address >> 8;
   	  wr_buf[2] = location.address;

   	  FRAM_SPI_FOR_STORAGE.transfer(wr_buf, 3, SPI_CONTINUE);

   	  while (location.can_write() && !it.at_end())
      {
   		wr_buf[0] = converter.to_byte(*it);

   	   	FRAM_SPI_FOR_STORAGE.transfer(wr_buf, 1, SPI_CONTINUE);

   	   	++location;
        ++it;
      }

   	  digitalWrite(FRAM_PIN_CS_FOR_STORAGE, 1);
   	  digitalWrite(FRAM_PIN_WE_FOR_STORAGE, 0);
#endif

#if defined(USE_EEPROM_FOR_STORAGE)

   while (location.can_write() && !it.at_end())
   {
     location.update(converter.to_byte(*it));
     ++location;
     ++it;
   }
#endif

   return location;
 }


write_location StorageClass::writeTable(void *pTable, table_type_t key, write_location location)
 {
   return write(y_rbegin(pTable, key),
                 write(x_begin(pTable, key),
                   write(rows_begin(pTable, key), location)));
 }





eeprom_address_t StorageClass::compute_crc_address(uint8_t pageNum)
{
  return EEPROM_LAST_BARO-((getPageCount() - pageNum)*sizeof(uint32_t));
}

/** Write CRC32 checksum to EEPROM.
Takes a page number and CRC32 value then stores it in the relevant place in EEPROM
@param pageNum - Config page number
@param crcValue - CRC32 checksum
*/
void StorageClass::storePageCRC32(uint8_t pageNum, uint32_t crcValue)
{

#if defined(USE_FRAM_FOR_STORAGE)
	write_block(compute_crc_address(pageNum), &crcValue, sizeof(crcValue));
#endif


#if defined(USE_EEPROM_FOR_STORAGE)
  EEPROM.put(compute_crc_address(pageNum), crcValue);
#endif
}

/** Retrieves and returns the 4 byte CRC32 checksum for a given page from EEPROM.
@param pageNum - Config page number
*/
uint32_t StorageClass::readPageCRC32(uint8_t pageNum)
{
uint32_t crc32_val = 0;

#if defined(USE_FRAM_FOR_STORAGE)
	load_block(compute_crc_address(pageNum), &crc32_val, sizeof(crc32_val));
#endif

#if defined(USE_EEPROM_FOR_STORAGE)
	crc32_val = EEPROM.get(compute_crc_address(pageNum), crc32_val);
#endif

  return crc32_val;
}

/** Same as above, but writes the CRC32 for the calibration page rather than tune data
@param calibrationPageNum - Calibration page number
@param calibrationCRC - CRC32 checksum
*/
void StorageClass::storeCalibrationCRC32(uint8_t calibrationPageNum, uint32_t calibrationCRC)
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

#if defined(USE_FRAM_FOR_STORAGE)
	write_block(targetAddress, &calibrationCRC, sizeof(calibrationCRC));
#endif

#if defined(USE_EEPROM_FOR_STORAGE)
  EEPROM.put(targetAddress, calibrationCRC);
#endif
}

/** Retrieves and returns the 4 byte CRC32 checksum for a given calibration page from EEPROM.
@param calibrationPageNum - Config page number
*/
uint32_t StorageClass::readCalibrationCRC32(uint8_t calibrationPageNum)
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

#if defined(USE_FRAM_FOR_STORAGE)
	load_block(targetAddress, &crc32_val, sizeof(crc32_val));
#endif

#if defined(USE_EEPROM_FOR_STORAGE)
  EEPROM.get(targetAddress, crc32_val);
#endif

  return crc32_val;
}

uint16_t StorageClass::getEEPROMSize(void)
{
#if defined(USE_FRAM_FOR_STORAGE)
	return FRAM_LAST_ADDR + 1;
#endif

#if defined(USE_EEPROM_FOR_STORAGE)
  return EEPROM.length();
#endif
}



#if defined(CORE_AVR)
#pragma GCC pop_options
#endif
