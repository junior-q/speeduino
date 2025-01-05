/** @file
 * Instantiation of various (table2D, table3D) tables, volatile (interrupt modified) variables, Injector (1...8) enablement flags, etc.
 */

#include "globals.h"
#include "config.h"
#include "storage.h"
#include "pages.h"



const byte data_structure_version = 2; //This identifies the data structure when reading / writing. (outdated ?)

struct table3d16RpmLoad fuelTable; 				///< 16x16 fuel map
struct table3d16RpmLoad fuelTable2; 			///< 16x16 fuel map
struct table3d16RpmLoad ignitionTable; 			///< 16x16 ignition map
struct table3d16RpmLoad ignitionTable2; 		///< 16x16 ignition map
struct table3d16RpmLoad afrTable; 				///< 16x16 afr target map
struct table3d8RpmLoad stagingTable; 			///< 8x8 fuel staging table

struct table3d8RpmLoad boostTable; 			 	///< 8x8 boost map
struct table3d8RpmLoad boostTableLookupDuty; 	///< 8x8 boost map lookup table

struct table3d8RpmLoad vvtTable; 				///< 8x8 vvt map
struct table3d8RpmLoad vvt2Table; 				///< 8x8 vvt2 map
struct table3d8RpmLoad wmiTable; 				///< 8x8 wmi map

trimTable3d trim1Table; 						///< 6x6 Fuel trim 1 map
trimTable3d trim2Table; 						///< 6x6 Fuel trim 2 map
trimTable3d trim3Table; 						///< 6x6 Fuel trim 3 map
trimTable3d trim4Table; 						///< 6x6 Fuel trim 4 map
trimTable3d trim5Table; 						///< 6x6 Fuel trim 5 map
trimTable3d trim6Table; 						///< 6x6 Fuel trim 6 map
trimTable3d trim7Table; 						///< 6x6 Fuel trim 7 map
trimTable3d trim8Table; 						///< 6x6 Fuel trim 8 map

struct table3d4RpmLoad dwellTable; 				///< 4x4 Dwell map

struct table2D taeTable; 						///< 4 bin TPS Acceleration Enrichment map (2D)

struct table2D maeTable;

struct table2D WUETable; 						///< 10 bin Warm Up Enrichment map (2D)
struct table2D ASETable; 						///< 4 bin After Start Enrichment map (2D)
struct table2D ASECountTable; 					///< 4 bin After Start duration map (2D)
struct table2D PrimingPulseTable; 				///< 4 bin Priming pulsewidth map (2D)
struct table2D crankingEnrichTable; 			///< 4 bin cranking Enrichment map (2D)
struct table2D dwellVCorrectionTable; 			///< 6 bin dwell voltage correction (2D)
struct table2D injectorVCorrectionTable; 		///< 6 bin injector voltage correction (2D)
struct table2D injectorAngleTable; 				///< 4 bin injector angle curve (2D)
struct table2D IATDensityCorrectionTable; 		///< 9 bin inlet air temperature density correction (2D)
struct table2D baroFuelTable; 					///< 8 bin baro correction curve (2D)
struct table2D IATRetardTable; 					///< 6 bin ignition adjustment based on inlet air temperature  (2D)
struct table2D idleTargetTable; 				///< 10 bin idle target table for idle timing (2D)
struct table2D idleAdvanceTable; 				///< 6 bin idle advance adjustment table based on RPM difference  (2D)
struct table2D CLTAdvanceTable; 				///< 6 bin ignition adjustment based on coolant temperature  (2D)
struct table2D rotarySplitTable; 				///< 8 bin ignition split curve for rotary leading/trailing  (2D)
struct table2D flexFuelTable;  					///< 6 bin flex fuel correction table for fuel adjustments (2D)
struct table2D flexAdvTable;   					///< 6 bin flex fuel correction table for timing advance (2D)
struct table2D flexBoostTable; 					///< 6 bin flex fuel correction table for boost adjustments (2D)
struct table2D fuelTempTable;  					///< 6 bin flex fuel correction table for fuel adjustments (2D)
struct table2D knockWindowStartTable;
struct table2D knockWindowDurationTable;
struct table2D oilPressureProtectTable;
struct table2D wmiAdvTable; 					//6 bin wmi correction table for timing advance (2D)
struct table2D coolantProtectTable;
struct table2D fanPWMTable;
struct table2D rollingCutTable;

struct config2 configPage2;
struct config4 configPage4;
struct config6 configPage6;
struct config9 configPage9;
struct config10 configPage10;
struct config13 configPage13;
struct config15 configPage15;

uint16_t cltCalibration_bins[32];
uint16_t cltCalibration_values[32];
struct table2D cltCalibrationTable;

uint16_t iatCalibration_bins[32];
uint16_t iatCalibration_values[32];
struct table2D iatCalibrationTable;

uint16_t o2Calibration_bins[32];
uint8_t o2Calibration_values[32];
struct table2D o2CalibrationTable; 

// Utility functions.
// By having these in this file, it prevents other files from calling EEPROM functions directly. This is useful due to differences in the EEPROM libraries on different devces
/// Read last stored barometer reading from EEPROM.
byte readLastBaro(void) { return EEPROM.read(EEPROM_LAST_BARO); }
/// Write last acquired arometer reading to EEPROM.
void storeLastBaro(byte newValue) { EEPROM.update(EEPROM_LAST_BARO, newValue); }
/// Read EEPROM current data format version (from offset EEPROM_DATA_VERSION).
byte readEEPROMVersion(void) { return EEPROM.read(EEPROM_DATA_VERSION); }
/// Store EEPROM current data format version (to offset EEPROM_DATA_VERSION).
void storeEEPROMVersion(byte newVersion) { EEPROM.update(EEPROM_DATA_VERSION, newVersion); }


/** Load all config tables from storage.
 */
void loadConfig(void)
{
  loadTable(&fuelTable, decltype(fuelTable)::type_key, EEPROM_CONFIG1_MAP);
  load_range(EEPROM_CONFIG2_START, (byte *)&configPage2, (byte *)&configPage2+sizeof(configPage2));

  //*********************************************************************************************************************************************************************************
  //IGNITION CONFIG PAGE (2)

  loadTable(&ignitionTable, decltype(ignitionTable)::type_key, EEPROM_CONFIG3_MAP);
  load_range(EEPROM_CONFIG4_START, (byte *)&configPage4, (byte *)&configPage4+sizeof(configPage4));

  //*********************************************************************************************************************************************************************************
  //AFR TARGET CONFIG PAGE (3)

  loadTable(&afrTable, decltype(afrTable)::type_key, EEPROM_CONFIG5_MAP);
  load_range(EEPROM_CONFIG6_START, (byte *)&configPage6, (byte *)&configPage6+sizeof(configPage6));

  //*********************************************************************************************************************************************************************************
  // Boost and vvt tables load
  loadTable(&boostTable, decltype(boostTable)::type_key, EEPROM_CONFIG7_MAP1);
  loadTable(&vvtTable, decltype(vvtTable)::type_key,  EEPROM_CONFIG7_MAP2);
  loadTable(&stagingTable, decltype(stagingTable)::type_key, EEPROM_CONFIG7_MAP3);

  //*********************************************************************************************************************************************************************************
  // Fuel trim tables load
  loadTable(&trim1Table, decltype(trim1Table)::type_key, EEPROM_CONFIG8_MAP1);
  loadTable(&trim2Table, decltype(trim2Table)::type_key, EEPROM_CONFIG8_MAP2);
  loadTable(&trim3Table, decltype(trim3Table)::type_key, EEPROM_CONFIG8_MAP3);
  loadTable(&trim4Table, decltype(trim4Table)::type_key, EEPROM_CONFIG8_MAP4);
  loadTable(&trim5Table, decltype(trim5Table)::type_key, EEPROM_CONFIG8_MAP5);
  loadTable(&trim6Table, decltype(trim6Table)::type_key, EEPROM_CONFIG8_MAP6);
  loadTable(&trim7Table, decltype(trim7Table)::type_key, EEPROM_CONFIG8_MAP7);
  loadTable(&trim8Table, decltype(trim8Table)::type_key, EEPROM_CONFIG8_MAP8);

  //*********************************************************************************************************************************************************************************
  //canbus control page load
  load_range(EEPROM_CONFIG9_START, (byte *)&configPage9, (byte *)&configPage9+sizeof(configPage9));

  //*********************************************************************************************************************************************************************************

  //CONFIG PAGE (10)
  load_range(EEPROM_CONFIG10_START, (byte *)&configPage10, (byte *)&configPage10+sizeof(configPage10));

  //*********************************************************************************************************************************************************************************
  //Fuel table 2 (See storage.h for data layout)
  loadTable(&fuelTable2, decltype(fuelTable2)::type_key, EEPROM_CONFIG11_MAP);

  //*********************************************************************************************************************************************************************************
  // WMI, VVT2 and Dwell table load
  loadTable(&wmiTable, decltype(wmiTable)::type_key, EEPROM_CONFIG12_MAP);
  loadTable(&vvt2Table, decltype(vvt2Table)::type_key, EEPROM_CONFIG12_MAP2);
  loadTable(&dwellTable, decltype(dwellTable)::type_key, EEPROM_CONFIG12_MAP3);

  //*********************************************************************************************************************************************************************************
  //CONFIG PAGE (13)
  load_range(EEPROM_CONFIG13_START, (byte *)&configPage13, (byte *)&configPage13+sizeof(configPage13));

  //*********************************************************************************************************************************************************************************
  //SECOND IGNITION CONFIG PAGE (14)

  loadTable(&ignitionTable2, decltype(ignitionTable2)::type_key, EEPROM_CONFIG14_MAP);

  //*********************************************************************************************************************************************************************************
  //CONFIG PAGE (15) + boost duty lookup table (LUT)
  loadTable(&boostTableLookupDuty, decltype(boostTableLookupDuty)::type_key, EEPROM_CONFIG15_MAP);
  load_range(EEPROM_CONFIG15_START, (byte *)&configPage15, (byte *)&configPage15+sizeof(configPage15));

  //*********************************************************************************************************************************************************************************
}

/** Write a table or map to EEPROM storage.
Takes the current configuration (config pages and maps)
and writes them to EEPROM as per the layout defined in storage.h.
*/
void writeConfig(uint8_t pageNum)
{
//The maximum number of write operations that will be performed in one go.
//If we try to write to the EEPROM too fast (Eg Each write takes ~3ms on the AVR) then
//the rest of the system can hang)
#if defined(USE_SPI_EEPROM)
  //For use with common Winbond SPI EEPROMs Eg W25Q16JV
  uint8_t EEPROM_MAX_WRITE_BLOCK = 20; //This needs tuning
#elif defined(CORE_STM32) || defined(CORE_TEENSY)
  uint8_t EEPROM_MAX_WRITE_BLOCK = 64;
#else
  uint8_t EEPROM_MAX_WRITE_BLOCK = 18;
  if(BIT_CHECK(currentStatus.status4, BIT_STATUS4_COMMS_COMPAT)) { EEPROM_MAX_WRITE_BLOCK = 8; } //If comms compatibility mode is on, slow the burn rate down even further

  #ifdef CORE_AVR
    //In order to prevent missed pulses during EEPROM writes on AVR, scale the
    //maximum write block size based on the RPM.
    //This calculation is based on EEPROM writes taking approximately 4ms per byte
    //(Actual value is 3.8ms, so 4ms has some safety margin)
    if(currentStatus.RPM > 65) //Min RPM of 65 prevents overflow of uint8_t
    {
      EEPROM_MAX_WRITE_BLOCK = (uint8_t)(15000U / currentStatus.RPM);
      EEPROM_MAX_WRITE_BLOCK = max(EEPROM_MAX_WRITE_BLOCK, 1);
      EEPROM_MAX_WRITE_BLOCK = min(EEPROM_MAX_WRITE_BLOCK, 15); //Any higher than this will cause comms timeouts on AVR
    }
  #endif

#endif

  write_location result = { 0, 0, EEPROM_MAX_WRITE_BLOCK };

  switch(pageNum)
  {
    case veMapPage:
      /*---------------------------------------------------
      | Fuel table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&fuelTable, decltype(fuelTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG1_MAP));
      break;

    case veSetPage:
      /*---------------------------------------------------
      | Config page 2 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage2, (byte *)&configPage2+sizeof(configPage2), result.changeWriteAddress(EEPROM_CONFIG2_START));
      break;

    case ignMapPage:
      /*---------------------------------------------------
      | Ignition table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&ignitionTable, decltype(ignitionTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG3_MAP));
      break;

    case ignSetPage:
      /*---------------------------------------------------
      | Config page 2 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage4, (byte *)&configPage4+sizeof(configPage4), result.changeWriteAddress(EEPROM_CONFIG4_START));
      break;

    case afrMapPage:
      /*---------------------------------------------------
      | AFR table (See storage.h for data layout) - Page 5
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&afrTable, decltype(afrTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG5_MAP));
      break;

    case afrSetPage:
      /*---------------------------------------------------
      | Config page 3 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage6, (byte *)&configPage6+sizeof(configPage6), result.changeWriteAddress(EEPROM_CONFIG6_START));
      break;

    case boostvvtPage:
      /*---------------------------------------------------
      | Boost and vvt tables (See storage.h for data layout) - Page 8
      | 8x8 table itself + the 8 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&boostTable, decltype(boostTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG7_MAP1));
      result = writeTable(&vvtTable, decltype(vvtTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG7_MAP2));
      result = writeTable(&stagingTable, decltype(stagingTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG7_MAP3));
      break;

    case seqFuelPage:
      /*---------------------------------------------------
      | Fuel trim tables (See storage.h for data layout) - Page 9
      | 6x6 tables itself + the 6 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&trim1Table, decltype(trim1Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP1));
      result = writeTable(&trim2Table, decltype(trim2Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP2));
      result = writeTable(&trim3Table, decltype(trim3Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP3));
      result = writeTable(&trim4Table, decltype(trim4Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP4));
      result = writeTable(&trim5Table, decltype(trim5Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP5));
      result = writeTable(&trim6Table, decltype(trim6Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP6));
      result = writeTable(&trim7Table, decltype(trim7Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP7));
      result = writeTable(&trim8Table, decltype(trim8Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP8));
      break;

    case canbusPage:
      /*---------------------------------------------------
      | Config page 10 (See storage.h for data layout)
      | 192 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage9, (byte *)&configPage9+sizeof(configPage9), result.changeWriteAddress(EEPROM_CONFIG9_START));
      break;

    case warmupPage:
      /*---------------------------------------------------
      | Config page 11 (See storage.h for data layout)
      | 192 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage10, (byte *)&configPage10+sizeof(configPage10), result.changeWriteAddress(EEPROM_CONFIG10_START));
      break;

    case fuelMap2Page:
      /*---------------------------------------------------
      | Fuel table 2 (See storage.h for data layout)
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&fuelTable2, decltype(fuelTable2)::type_key, result.changeWriteAddress(EEPROM_CONFIG11_MAP));
      break;

    case wmiMapPage:
      /*---------------------------------------------------
      | WMI and Dwell tables (See storage.h for data layout) - Page 12
      | 8x8 WMI table itself + the 8 values along each of the axis
      | 8x8 VVT2 table + the 8 values along each of the axis
      | 4x4 Dwell table itself + the 4 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&wmiTable, decltype(wmiTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG12_MAP));
      result = writeTable(&vvt2Table, decltype(vvt2Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG12_MAP2));
      result = writeTable(&dwellTable, decltype(dwellTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG12_MAP3));
      break;

    case progOutsPage:
      /*---------------------------------------------------
      | Config page 13 (See storage.h for data layout)
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage13, (byte *)&configPage13+sizeof(configPage13), result.changeWriteAddress(EEPROM_CONFIG13_START));
      break;

    case ignMap2Page:
      /*---------------------------------------------------
      | Ignition table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&ignitionTable2, decltype(ignitionTable2)::type_key, result.changeWriteAddress(EEPROM_CONFIG14_MAP));
      break;

    case boostvvtPage2:
      /*---------------------------------------------------
      | Boost duty cycle lookuptable (See storage.h for data layout) - Page 15
      | 8x8 table itself + the 8 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&boostTableLookupDuty, decltype(boostTableLookupDuty)::type_key, result.changeWriteAddress(EEPROM_CONFIG15_MAP));

      /*---------------------------------------------------
      | Config page 15 (See storage.h for data layout)
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage15, (byte *)&configPage15+sizeof(configPage15), result.changeWriteAddress(EEPROM_CONFIG15_START));
      break;

    default:
      break;
  }

  BIT_WRITE(currentStatus.status4, BIT_STATUS4_BURNPENDING, !result.can_write());
}

/** Reset all configPage* structs (2,4,6,9,10,13) and write them full of null-bytes.
 */
void resetConfigPages(void)
{
  for (uint8_t page=1; page<getPageCount(); ++page)
  {
    page_iterator_t entity = page_begin(page);
    while (entity.type!=End)
    {
      if (entity.type==Raw)
      {
        memset(entity.pData, 0, entity.size);
      }
      entity = advance(entity);
    }
  }
}

/** Write all config pages to EEPROM.
 */
void writeAllConfig(void)
{
  uint8_t pageCount = getPageCount();
  uint8_t page = 1U;
  writeConfig(page);
  page = page + 1;
  while (page<pageCount && !isEepromWritePending())
  {
    writeConfig(page);
    page = page + 1;
  }
}


/** Read the calibration information from EEPROM.
This is separate from the config load as the calibrations do not exist as pages within the ini file for Tuner Studio.
*/
void loadCalibration(void)
{
  // If you modify this function be sure to also modify writeCalibration();
  // it should be a mirror image of this function.

  EEPROM.get(EEPROM_CALIBRATION_O2_BINS, o2Calibration_bins);
  EEPROM.get(EEPROM_CALIBRATION_O2_VALUES, o2Calibration_values);

  EEPROM.get(EEPROM_CALIBRATION_IAT_BINS, iatCalibration_bins);
  EEPROM.get(EEPROM_CALIBRATION_IAT_VALUES, iatCalibration_values);

  EEPROM.get(EEPROM_CALIBRATION_CLT_BINS, cltCalibration_bins);
  EEPROM.get(EEPROM_CALIBRATION_CLT_VALUES, cltCalibration_values);
}

/** Write calibration tables to EEPROM.
This takes the values in the 3 calibration tables (Coolant, Inlet temp and O2)
and saves them to the EEPROM.
*/
void writeCalibration(void)
{
  // If you modify this function be sure to also modify loadCalibration();
  // it should be a mirror image of this function.

  EEPROM.put(EEPROM_CALIBRATION_O2_BINS, o2Calibration_bins);
  EEPROM.put(EEPROM_CALIBRATION_O2_VALUES, o2Calibration_values);

  EEPROM.put(EEPROM_CALIBRATION_IAT_BINS, iatCalibration_bins);
  EEPROM.put(EEPROM_CALIBRATION_IAT_VALUES, iatCalibration_values);

  EEPROM.put(EEPROM_CALIBRATION_CLT_BINS, cltCalibration_bins);
  EEPROM.put(EEPROM_CALIBRATION_CLT_VALUES, cltCalibration_values);
}

void writeCalibrationPage(uint8_t pageNum)
{
  if(pageNum == O2_CALIBRATION_PAGE)
  {
    EEPROM.put(EEPROM_CALIBRATION_O2_BINS, o2Calibration_bins);
    EEPROM.put(EEPROM_CALIBRATION_O2_VALUES, o2Calibration_values);
  }
  else if(pageNum == IAT_CALIBRATION_PAGE)
  {
    EEPROM.put(EEPROM_CALIBRATION_IAT_BINS, iatCalibration_bins);
    EEPROM.put(EEPROM_CALIBRATION_IAT_VALUES, iatCalibration_values);
  }
  else if(pageNum == CLT_CALIBRATION_PAGE)
  {
    EEPROM.put(EEPROM_CALIBRATION_CLT_BINS, cltCalibration_bins);
    EEPROM.put(EEPROM_CALIBRATION_CLT_VALUES, cltCalibration_values);
  }
}

