/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart

A full copy of the license may be found in the projects root directory

- small changes by junior-q:
- encapsulated function into a class in order to protect static funct
- all data processed using buffers and not serial calls
- serial call used only for input-output
- dedicated module for M451 series: subst of comms.cpp & comms_legacy.cpp

*/



/** @file
   * Process Incoming and outgoing serial communications.
 */
#include "globals.h"

#if defined(CORE_M451)

#include "storage.h"
#include "config.h"
#include "maths.h"
#include "utilities.h"
#include "decoders.h"

#include "TS_CommandButtonHandler.h"

#include "errors.h"
#include "pages.h"
#include "page_crc.h"
#include "logger.h"
#include "src/FastCRC/FastCRC.h"

#include "TSComms.h"

// Forward declarations

TSCommClass tsComm;


//!@{
/** @brief Hard coded response for some TS messages.
 * @attention Stored in flash (.text segment) and loaded on demand.
 */
const char serialVersion[] = "002";
const byte canId = 0;
const char codeVersion[] = "speeduino 202405-dev";
const char productString[] = "Speeduino 2023.11-IE_4C-M451";
//!@}




// ====================================== Endianess Support =============================

/**
 * @brief      Flush all remaining bytes from the rx serial buffer
 */
void flushRXbuffer(void)
{
  while (Serial.available() > 0) { Serial.read(); }
}

TSCommClass::TSCommClass()
{
	currentPage = 1;			//Not the same as the speeduino config page numbers

}


// ====================================== Multibyte Primitive IO Support =============================


/** Processes the incoming data on the serial buffer based on the command sent.
Can be either data for a new command or a continuation of data for command that is already in progress:

Comands are single byte (letter symbol) commands.
*/

void TSCommClass::serialReceive(void)
{
int c;
int isLegacyCommand = 0;
static uint8_t incomingCrcBuf[4];


	if( serialTransmitInProgress() )
		return;

	while( Serial.available() )		// just process all available chars ...
	{
	    c = Serial.read();

	    serialReceiveStartTime = millis();

	    switch( serialStatusFlag )
		{
			case	SERIAL_COMMAND_INPROGRESS_LEGACY:	//Check for an existing legacy command in progress
			    legacySerialCommand(c);

			    if( serialTransmitInProgress() )
					return;
			    break;

			case	SERIAL_TRANSMIT_INPROGRESS:
			/** A partial write is in progress (legacy send). */
			case	SERIAL_TRANSMIT_INPROGRESS_LEGACY:
			/** We are part way through transmitting the tooth log */
			case	SERIAL_TRANSMIT_TOOTH_INPROGRESS:
			/** We are part way through transmitting the tooth log (legacy send) */
			case	SERIAL_TRANSMIT_TOOTH_INPROGRESS_LEGACY:
			/** We are part way through transmitting the composite log */
			case	SERIAL_TRANSMIT_COMPOSITE_INPROGRESS:
			/** We are part way through transmitting the composite log (legacy send) */
			case	SERIAL_TRANSMIT_COMPOSITE_INPROGRESS_LEGACY:
				return;

			case	SERIAL_COMMAND_INPROGRESS:
				break;

			case	SERIAL_INACTIVE:

				if( c == 'F')
					isLegacyCommand = 1;

				if( (((c >= 'A') && (c <= 'z')) || (c == '?')) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_ALLOW_LEGACY_COMMS)) )
					isLegacyCommand = 1;


				if( isLegacyCommand )
				{
					legacySerialCommand(c);

					if( serialTransmitInProgress() )
						return;
				}
				else
				{
					serialPayloadRxLength = c;		// high byte first ?
					serialPayloadRxLength <<= 8;

					serialStatusFlag = SERIAL_RECEIVE_WAIT_FOR_LEN;
				}
				break;

			case	SERIAL_RECEIVE_WAIT_FOR_LEN:

				serialPayloadRxLength += c;		// low byte

				serialBytesRx = 0;			// reset buffer idx

				if( serialBytesRx >= serialPayloadRxLength )	// no payload ?
				{
					serialStatusFlag = SERIAL_RECEIVE_CRC0;
				}
				else
				{
					serialStatusFlag = SERIAL_RECEIVE_PAYLOAD;
				}
				break;

			case	SERIAL_RECEIVE_PAYLOAD:

				serialPayloadRx[serialBytesRx++] = c;

		    	if( serialBytesRx >= serialPayloadRxLength )	// no payload ?
				{
					serialStatusFlag = SERIAL_RECEIVE_CRC0;
				}

				break;

			case	SERIAL_RECEIVE_CRC0:	// crc is big endian: need to reverse
				incomingCrcBuf[3] = c;
				serialStatusFlag = SERIAL_RECEIVE_CRC1;
				break;
			case	SERIAL_RECEIVE_CRC1:
				incomingCrcBuf[2] = c;
				serialStatusFlag = SERIAL_RECEIVE_CRC2;
				break;
			case	SERIAL_RECEIVE_CRC2:
				incomingCrcBuf[1] = c;
				serialStatusFlag = SERIAL_RECEIVE_CRC3;
				break;
			case	SERIAL_RECEIVE_CRC3:
				incomingCrcBuf[0] = c;

				uint32_t incomingCrc;
				uint32_t payloadCrc;

			    memcpy(&incomingCrc, incomingCrcBuf, sizeof(incomingCrc));


			    payloadCrc = CRC32_serial.crc32(serialPayloadRx, serialPayloadRxLength);

				serialStatusFlag = SERIAL_INACTIVE; //The serial receive is now complete


		        if( incomingCrc == payloadCrc )
		        {
		          
		          processSerialCommand(serialPayloadRx[0]);		//CRC is correct. Process the command

		          BIT_CLEAR(currentStatus.status4, BIT_STATUS4_ALLOW_LEGACY_COMMS); //Lock out legacy commands until next power cycle
		        }
		        else
		        {
					//CRC Error. Need to send an error message
					sendReturnCodeMsg(SERIAL_RC_CRC_ERR);
//					flushRXbuffer();
		        }
				break;

		}

	} //Data in serial buffer and serial receive in progress

  //Check for a timeout

    if( serialStatusFlag == SERIAL_RECEIVE_WAIT_FOR_LEN ||
		serialStatusFlag == SERIAL_RECEIVE_PAYLOAD ||
		serialStatusFlag == SERIAL_RECEIVE_CRC0 ||
		serialStatusFlag == SERIAL_RECEIVE_CRC1 ||
		serialStatusFlag == SERIAL_RECEIVE_CRC2 ||
		serialStatusFlag == SERIAL_RECEIVE_CRC3 )
    {
		if( isTimeout() )
		{
			serialStatusFlag = SERIAL_INACTIVE; //Reset the serial receive

			flushRXbuffer();
			//sendReturnCodeMsg(SERIAL_RC_TIMEOUT);

			Serial.write("timeout !!\r\n");
		} //Timeout
    }
}



void TSCommClass::processSerialCommand(char __c)
{
static const TsCmdHandler *pCmdHandler;	//



	if( serialStatusFlag == SERIAL_INACTIVE )
	{
		currentCommand = __c;		// new cmd

		if( __c >= '?' && __c <= 'z' )	// valid cmd ?
		{
			uint8_t cmdIndex = __c - '?';

			pCmdHandler = &cmdHandlerTable[cmdIndex];	//	points to cmd handler

			if( pCmdHandler->pf_fun != NULL )
			{
				serialStatusPhase = SERIAL_PHASE_IDLE;	// reset sequence
				(this->*pCmdHandler->pf_fun)();
			}
		}
	}
	else
	{
		if( serialStatusFlag == SERIAL_COMMAND_INPROGRESS )
		{
			if( pCmdHandler->pf_fun != NULL )
			{
				(this->*pCmdHandler->pf_fun)();
			}
			else
			{
				serialStatusFlag = SERIAL_INACTIVE;
			}
		}
	}

}



/** Processes the incoming data on the serial buffer based on the command sent.
Can be either data for a new command or a continuation of data for command that is already in progress:
- cmdPending = If a command has started but is waiting on further data to complete
- chunkPending = Specifically for the new receive value method where TS will send a known number of contiguous bytes to be written to a table

Commands are single byte (letter symbol) commands.
*/
void TSCommClass::legacySerialCommand(char __c)
{

static const TsCmdHandler *pLegacyHandler;	//


	if( serialStatusFlag == SERIAL_INACTIVE )
	{
		currentCommand = __c;		// new cmd

		if( __c >= '?' && __c <= 'z' )	// valid cmd ?
		{
			uint8_t legacyIndex = __c - '?';

			pLegacyHandler = &legacyHandlerTable[legacyIndex];	//	points to legacy handler

			if( pLegacyHandler->pf_fun != NULL )
			{
				serialStatusPhase = SERIAL_PHASE_IDLE;	// reset sequence
				(this->*pLegacyHandler->pf_fun)();
			}
		}
	}
	else
	{
		if( serialStatusFlag == SERIAL_COMMAND_INPROGRESS_LEGACY )
		{
			if( pLegacyHandler->pf_fun != NULL )
			{
				(this->*pLegacyHandler->pf_fun)();
			}
			else
			{
				serialStatusFlag = SERIAL_INACTIVE;
			}
		}
	}


}



void TSCommClass::serialTransmit(void)
{
  switch ( serialStatusFlag )
  {
    case SERIAL_TRANSMIT_TOOTH_INPROGRESS:
      sendToothLog();
      break;

    case SERIAL_TRANSMIT_TOOTH_INPROGRESS_LEGACY:
    	sendBufferNonBlocking();
      	break;

    case SERIAL_TRANSMIT_COMPOSITE_INPROGRESS:
      sendCompositeLog();
      break;

    case SERIAL_TRANSMIT_INPROGRESS_LEGACY:
    case SERIAL_TRANSMIT_INPROGRESS:
    	sendBufferNonBlocking();
      	break;

    default: // Nothing to do
      	break;
  }
}



// ====================================== Non-blocking IO Support =============================

int TSCommClass::sendBufferNonBlocking(void)
{

	if( serialTransmitInProgress() )	// tx active ?
	{
		  do
		  {
			  if( serialBytesTx < serialPayloadTxLength )	// not endof buf ?
			  {
					if( Serial.availableForWrite() )
					{
						Serial.write(serialPayloadTx[serialBytesTx]);

						serialBytesTx++;		// next char index
					}
					else
					{
						break;
					}
			  }
			  else
			  {
				  serialStatusFlag = SERIAL_INACTIVE;
			  }

		  }while( serialTransmitInProgress() );

	}

	return 0;
}


// ====================================== TS Message Support =============================

/** @brief Send a message to TS containing only a return code.
 * 
 * This is used when TS asks for an action to happen (E.g. start a logger) or
 * to signal an error condition to TS
 * 
 * @attention This is a blocking operation
 */
void TSCommClass::sendReturnCodeMsg(byte __returnCode)
{

	serialPayloadTx[0] = 0;
	serialPayloadTx[1] = 1;				// packet len: 1 byte
	  
	serialPayloadTx[2] = __returnCode;	// requested code

	sendSerialPayload(1);
}


void TSCommClass::sendReturnStringMsg(const char *__string)
{
uint16_t packetLength = 1 + strlen(__string);	// packet len: 1 byte + string

	serialPayloadTx[0] = packetLength >> 8;
	serialPayloadTx[1] = packetLength;
	  
	serialPayloadTx[2] = SERIAL_RC_OK;	// requested code
	strcpy( (char *) &serialPayloadTx[3], __string);	// requested version

	sendSerialPayload(packetLength);
}

void TSCommClass::sendReturnCRCMsg(const byte *__p_crc)
{
	uint16_t payLen = 5;
	
	serialPayloadTx[0] = payLen >> 8;
	serialPayloadTx[1] = payLen;

	serialPayloadTx[2] = SERIAL_RC_OK;

	serialPayloadTx[3] = __p_crc[3];
	serialPayloadTx[4] = __p_crc[2];
	serialPayloadTx[5] = __p_crc[1];
	serialPayloadTx[6] = __p_crc[0];

	sendSerialPayload(payLen);
}


/** @brief Send a status record back to tuning/logging SW.
 * This will "live" information from @ref currentStatus struct.
 * @param offset - Start field number
 * @param packetLength - Length of actual message (after possible ack/confirm headers)
 * E.g. tuning sw command 'A' (Send all values) will send data from field number 0, LOG_ENTRY_SIZE fields.
 */
void TSCommClass::sendReturnLiveValuesMsg(uint16_t __offset, uint16_t __packetLength)
{
	if( firstCommsRequest ) 
	{ 
		firstCommsRequest = false;
		currentStatus.secl = 0; 
	}

	currentStatus.status2 ^= (-currentStatus.hasSync ^ currentStatus.status2) & (1U << BIT_STATUS2_SYNC); //Set the sync bit of the Spark variable to match the hasSync variable

	uint16_t payLen = __packetLength + 1;
	
	serialPayloadTx[0] = payLen >> 8;
	serialPayloadTx[1] = payLen;
	serialPayloadTx[2] = SERIAL_RC_OK;

	for(byte x=0; x<__packetLength; x++)
	{
		serialPayloadTx[x+3] = getTSLogEntry(__offset + x);
	}

	sendSerialPayload(payLen);
}



/** @brief Start sending the shared serialPayload buffer.
 * 
 * ::serialStatusFlag will be signal the result of the send:<br>
 * ::serialStatusFlag == SERIAL_INACTIVE: send is complete <br>
 * ::serialStatusFlag == SERIAL_TRANSMIT_INPROGRESS: partial send, subsequent calls to continueSerialTransmission
 * will finish sending serialPayload
 * 
 * @param payloadLength How many bytes to send [0, sizeof(serialPayload))
*/
void TSCommClass::sendSerialPayload(uint16_t __payloadLength)
{
	register uint16_t sizeofLen = 2;

	// skips first 2 bytes from crc calc: len is out of CRC
	uint32_t payloadCrc = CRC32_serial.crc32(serialPayloadTx+sizeofLen, __payloadLength);

	byte *pCrc = (byte *) &payloadCrc;

	uint8_t *pPay = &serialPayloadTx[__payloadLength+sizeofLen];
		
	  // append now CRC
	pPay[0] = pCrc[3];
	pPay[1] = pCrc[2];
	pPay[2] = pCrc[1];
	pPay[3] = pCrc[0];

	  // total byte= len size + len data + len crc 
	serialPayloadTxLength = sizeofLen + __payloadLength + sizeof(payloadCrc);	// totals bytes to be tx
	serialBytesTx = 0;							// reset tx index

	//Start new transmission session
	serialStatusFlag = SERIAL_TRANSMIT_INPROGRESS;
	sendBufferNonBlocking();

}


void TSCommClass::sendSerialPayloadLegacy(uint16_t __payloadLength)
{
	serialPayloadTxLength = __payloadLength;
	serialBytesTx = 0;							// reset tx index

	//Start new transmission session
	serialStatusFlag = SERIAL_TRANSMIT_INPROGRESS_LEGACY;
	sendBufferNonBlocking();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Standard zone

void TSCommClass::cmdHandler_qest_mark(void){
//case '?':
}

			// standard command handlers
void TSCommClass::cmdHandler_A(void)
{

/** @brief Send a status record back to tuning/logging SW.
 * This will "live" information from @ref currentStatus struct.
 * @param offset - Start field number
 * @param packetLength - Length of actual message (after possible ack/confirm headers)
 * E.g. tuning sw command 'A' (Send all values) will send data from field number 0, LOG_ENTRY_SIZE fields.
static void generateLiveValues(uint16_t offset, uint16_t packetLength)
 */
//    case 'A': // send x bytes of realtime values in legacy support format
//      generateLiveValues(0, LOG_ENTRY_SIZE); 

uint16_t offset = 0;
uint16_t packetLength = LOG_ENTRY_SIZE;

	sendReturnLiveValuesMsg(offset, packetLength);
}

void TSCommClass::cmdHandler_B(void)
{
//	case 'B': // Same as above, but for the comms compat mode. Slows down the burn rate and increases the defer time
	BIT_SET(currentStatus.status4, BIT_STATUS4_COMMS_COMPAT); //Force the compat mode

// using embedded Fram we do not need take account of eeprom delay

	uint8_t page_num = serialPayloadRx[2];

  	writeConfig(page_num); 

	 //Read the table number and perform burn. Note that byte 1 in the array is unused
  	sendReturnCodeMsg(SERIAL_RC_BURN_OK);
}	


void TSCommClass::cmdHandler_C(void){
//    case 'C': // test communications. This is used by Tunerstudio to see whether there is an ECU on a given serial port

	serialPayloadTx[0] = 0;
	serialPayloadTx[1] = 2;	// packet len: 2 bytes

	serialPayloadTx[2] = SERIAL_RC_OK;	// requested code
	serialPayloadTx[3] = 255;

    sendSerialPayload(2);
}

void TSCommClass::cmdHandler_D(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_E(void){
//    case 'E': // receive command button commands
	TS_CommandButtonsHandler(word(serialPayloadRx[1], serialPayloadRx[2]));
	sendReturnCodeMsg(SERIAL_RC_OK);
}


void TSCommClass::cmdHandler_F(void){
	//    case 'F': // send serial protocol version

	sendReturnStringMsg(serialVersion);
	
}


void TSCommClass::cmdHandler_G(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}


void TSCommClass::cmdHandler_H(void){
//    case 'H': //Start the tooth logger
	startToothLogger();
	sendReturnCodeMsg(SERIAL_RC_OK);
}

void TSCommClass::cmdHandler_I(void){
//	    case 'I': // send CAN ID
	serialPayloadTx[0] = 0;
	serialPayloadTx[1] = 2;	// packet len: 2 bytes

	serialPayloadTx[2] = SERIAL_RC_OK;	// requested code
	serialPayloadTx[3] = canId;

    sendSerialPayload(2);
}



void TSCommClass::cmdHandler_J(void){
	//    case 'J': //Start the composite logger
  startCompositeLogger();
  sendReturnCodeMsg(SERIAL_RC_OK);
}

void TSCommClass::cmdHandler_K(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_L(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_M(void){
//    case 'M':
/*New write command
//7 bytes required:
//2 - Page identifier
//2 - offset
//2 - Length
//1 - 1st New value
*/

// The functions in this section are abstracted out to prevent enlarging callers stack frame, 
// which in turn throws off free ram reporting.

/**
 * @brief Update a pages contents from a buffer
 * 
 * @param pageNum The index of the page to update
 * @param offset Offset into the page
 * @param buffer The buffer to read from
 * @param length The buffer length
 * @return true if page updated successfully
 * @return false if page cannot be updated
 */
//inline bool updatePageValues(uint8_t pageNum, uint16_t offset, const byte *buffer, uint16_t length)
//{


uint8_t pageNum = serialPayloadRx[2];
uint16_t offset = word(serialPayloadRx[4], serialPayloadRx[3]);
const byte *buffer = &serialPayloadRx[7];
uint16_t length = word(serialPayloadRx[6], serialPayloadRx[5]);

	if ( (offset + length) <= getPageSize(pageNum) )
	{
		for(uint16_t i = 0; i < length; i++)
		{
			setPageValue(pageNum, (offset + i), buffer[i]);
		}

// no fram 
//		deferEEPROMWritesUntil = micros() + EEPROM_DEFER_DELAY;
		sendReturnCodeMsg(SERIAL_RC_OK);    
	}
	else
	{
	//This should never happen, but just in case
		sendReturnCodeMsg(SERIAL_RC_RANGE_ERR);
	}

}


void TSCommClass::cmdHandler_N(void){
//	case 'O': //Start the composite logger 2nd cam (teritary)
	startCompositeLoggerTertiary();
	sendReturnCodeMsg(SERIAL_RC_OK);
}

void TSCommClass::cmdHandler_O(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_P(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_Q(void){
//    case 'Q': // send code version
	sendReturnStringMsg(codeVersion);
}

void TSCommClass::cmdHandler_R(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_S(void){
//    case 'S': // send code version
	sendReturnStringMsg(productString);
	currentStatus.secl = 0; //This is required in TS3 due to its stricter timings
}

void TSCommClass::cmdHandler_T(void){
//    case 'T': //Send 256 tooth log entries to Tuner Studios tooth logger

    logItemsTransmitted = 0;

	if( currentStatus.toothLogEnabled == true ) 
	{ 
		sendToothLog(); 
	} //Sends tooth log values as ints
    else 
	{
		if ( currentStatus.compositeTriggerUsed > 0 ) 
		{ 
			sendCompositeLog(); 
		}
	    else 
		{ 
		/* MISRA no-op */ 
		}
	}
      
}

void TSCommClass::cmdHandler_U(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);

/* command not used

    case 'U': //User wants to reset the Arduino (probably for FW update)
      if (resetControl != RESET_CONTROL_DISABLED)
      {
      #ifndef SMALL_FLASH_MODE
        if (serialStatusFlag == SERIAL_INACTIVE) { Serial.println(F("Comms halted. Next byte will reset the Arduino.")); }
      #endif

        while (Serial.available() == 0) { }
        digitalWrite(pinResetControl, LOW);
      }
      else
      {
      #ifndef SMALL_FLASH_MODE
        if (serialStatusFlag == SERIAL_INACTIVE) { Serial.println(F("Reset control is currently disabled.")); }
      #endif
      }
      break;
*/

}

void TSCommClass::cmdHandler_V (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_W (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_X (void){
//    case 'X': //Start the composite logger 2nd cam (teritary)
	startCompositeLoggerCams();
	sendReturnCodeMsg(SERIAL_RC_OK);
}

void TSCommClass::cmdHandler_Y(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_Z(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}


void TSCommClass::cmdHandler_96(void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_a (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}


void TSCommClass::cmdHandler_b (void){

//    case 'b': // New EEPROM burn command to only burn a single page at a time 
// using embedded Fram we do not need take account of eeprom delay

	uint8_t pageNum = serialPayloadRx[2];
  	writeConfig(pageNum); 
	 //Read the table number and perform burn. Note that byte 1 in the array is unused
  	sendReturnCodeMsg(SERIAL_RC_BURN_OK);
}



void TSCommClass::cmdHandler_c (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}


void TSCommClass::cmdHandler_d (void){
//		    case 'd': // Send a CRC32 hash of a given page
	byte pageNum = serialPayloadRx[2];

	uint32_t CRC32_val = calculatePageCRC32(pageNum);

	sendReturnCRCMsg((byte *) &CRC32_val);
}

void TSCommClass::cmdHandler_e (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_f (void){
//    case 'f': //Send serial capability details
	uint16_t payLen = 6;
	
	serialPayloadTx[0] = payLen >> 8;
	serialPayloadTx[1] = payLen;
	serialPayloadTx[3] = SERIAL_RC_OK;
	serialPayloadTx[4] = 2; //Serial protocol version
	serialPayloadTx[5] = highByte(BLOCKING_FACTOR);
	serialPayloadTx[6] = lowByte(BLOCKING_FACTOR);
	serialPayloadTx[7] = highByte(TABLE_BLOCKING_FACTOR);
	serialPayloadTx[8] = lowByte(TABLE_BLOCKING_FACTOR);
  
	sendSerialPayload(payLen);
}


void TSCommClass::cmdHandler_g (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_h (void){
	//    case 'h': //Stop the tooth logger
    stopToothLogger();
	sendReturnCodeMsg(SERIAL_RC_OK);
}

void TSCommClass::cmdHandler_i (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}


void TSCommClass::cmdHandler_j (void){
//    case 'j': //Stop the composite logger
	stopCompositeLogger();
	sendReturnCodeMsg(SERIAL_RC_OK);

}

void TSCommClass::cmdHandler_k (void){
	//case 'k': //Send CRC values for the calibration pages
	byte calibrationPageNum = serialPayloadRx[2];
	uint32_t CRC32_val = Storage.readCalibrationCRC32(calibrationPageNum);

	sendReturnCRCMsg((byte *) &CRC32_val);
}

void TSCommClass::cmdHandler_l (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_m (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_n (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_o (void){
//   case 'o': //Stop the composite logger 2nd cam (tertiary)
	stopCompositeLoggerTertiary();
	sendReturnCodeMsg(SERIAL_RC_OK);
}

void TSCommClass::cmdHandler_p (void){
    /*
    * New method for sending page values (MS command equivalent is 'r')
    */
//    case 'p':
	/*
      //6 bytes required:
      //2 - Page identifier
      //2 - offset
      //2 - Length
      */

/**
 * @brief Loads a pages contents into a buffer
 * 
 * @param pageNum The index of the page to update
 * @param offset Offset into the page
 * @param buffer The buffer to read from
 * @param length The buffer length
 */
//void loadPageValuesToBuffer(uint8_t pageNum, uint16_t offset, byte *buffer, uint16_t length)

uint16_t length = word(serialPayloadRx[6], serialPayloadRx[5]);
uint8_t pageNum = serialPayloadRx[2];
uint16_t offset =  word(serialPayloadRx[4], serialPayloadRx[3]);

	//Setup the transmit buffer

	uint16_t payLen = length + 1;

	serialPayloadTx[0] = payLen >> 8;
	serialPayloadTx[1] = payLen;
	serialPayloadTx[2] = SERIAL_RC_OK;

	byte *buffer = &serialPayloadTx[3];

	for(uint16_t i = 0; i < length; i++)
	{
		buffer[i] = getPageValue(pageNum, offset + i);
	}

	sendSerialPayload(payLen);
}

void TSCommClass::cmdHandler_q (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_r (void){

//    case 'r': //New format for the optimised OutputChannels
    
	uint8_t cmd = serialPayloadRx[2];
	uint16_t offset = word(serialPayloadRx[4], serialPayloadRx[3]);
	uint16_t length = word(serialPayloadRx[6], serialPayloadRx[5]);

	uint16_t payLen = length + 1;

	if(cmd == SEND_OUTPUT_CHANNELS) //Send output channels command 0x30 is 48dec
	{
		sendReturnLiveValuesMsg(offset, length);
		sendSerialPayload(payLen);
	}
	else 
	{
		if( cmd == 0x0f )
		{
			//Request for signature
			sendReturnStringMsg(codeVersion);
		}
		else
		{
			//No other r/ commands should be called
		}
	}

}

void TSCommClass::cmdHandler_s (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_t (void){
//    case 't': // receive new Calibration info. Command structure: "t", <tble_idx> <data array>.

uint8_t cmd = serialPayloadRx[2];
uint16_t offset = word(serialPayloadRx[3], serialPayloadRx[4]);
uint16_t calibrationLength = word(serialPayloadRx[5], serialPayloadRx[6]); // Should be 256

	if(cmd == O2_CALIBRATION_PAGE)
	{
		loadO2CalibrationChunk(offset, calibrationLength);
		sendReturnCodeMsg(SERIAL_RC_OK);
		Serial.flush(); //This is safe because engine is assumed to not be running during calibration
	}
	else if(cmd == IAT_CALIBRATION_PAGE)
	{
		processTemperatureCalibrationTableUpdate(calibrationLength, IAT_CALIBRATION_PAGE, iatCalibration_values, iatCalibration_bins);
	}
	else if(cmd == CLT_CALIBRATION_PAGE)
	{
		processTemperatureCalibrationTableUpdate(calibrationLength, CLT_CALIBRATION_PAGE, cltCalibration_values, cltCalibration_bins);
	}
	else
	{
		sendReturnCodeMsg(SERIAL_RC_RANGE_ERR);
	}

}

void TSCommClass::cmdHandler_u (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_v (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_w (void){

/* sd card and rtc not available

*/
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_x (void){
//    case 'x': //Stop the composite logger 2nd cam (tertiary)
	stopCompositeLoggerCams();
    sendReturnCodeMsg(SERIAL_RC_OK);
}

void TSCommClass::cmdHandler_y (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}

void TSCommClass::cmdHandler_z (void){
	sendReturnCodeMsg(SERIAL_RC_UKWN_ERR);
}


// ====================================== Command/Action Support =============================


/**
 * @brief Update the oxygen sensor table from serialPayload
 * 
 * @param offset Offset into serialPayload and the table
 * @param chunkSize Number of bytes available in serialPayload
 */
void TSCommClass::loadO2CalibrationChunk(uint16_t offset, uint16_t chunkSize)
{
  using pCrcCalc = uint32_t (FastCRC32::*)(const uint8_t *, const uint16_t, bool);
  // First pass through the loop, we need to INITIALIZE the CRC
  pCrcCalc pCrcFun = offset==0U ? &FastCRC32::crc32 : &FastCRC32::crc32_upd;
  uint32_t calibrationCRC = 0U;

  //Read through the current chunk (Should be 256 bytes long)
  // Note there are 2 loops here: 
  //    [x, chunkSize)
  //    [offset, offset+chunkSize)
  for(uint16_t x = 0; x < chunkSize; ++x, ++offset)
  {
    //TS sends a total of 1024 bytes of calibration data, broken up into 256 byte chunks
    //As we're using an interpolated 2D table, we only need to store 32 values out of this 1024
    if( (x % 32U) == 0U )
    {
      o2Calibration_values[offset/32U] = serialPayloadRx[x+7U]; //O2 table stores 8 bit values
      o2Calibration_bins[offset/32U]   = offset;
    }

    //Update the CRC
    calibrationCRC = (CRC32_serial.*pCrcFun)(&serialPayloadRx[x+7U], 1, false);
    // Subsequent passes through the loop, we need to UPDATE the CRC
    pCrcFun = &FastCRC32::crc32_upd;
  }
 
  if( offset >= 1023 ) 
  {
    //All chunks have been received (1024 values). Finalise the CRC and burn to EEPROM
	Storage.storeCalibrationCRC32(O2_CALIBRATION_PAGE, ~calibrationCRC);
    writeCalibrationPage(O2_CALIBRATION_PAGE);
  }
}

/**
 * @brief Convert 2 bytes into an offset temperature in degrees Celcius
 * @attention Returned value will be offset CALIBRATION_TEMPERATURE_OFFSET
 */
uint16_t TSCommClass::toTemperature(byte lo, byte hi)
{
  int16_t tempValue = (int16_t)(word(hi, lo)); //Combine the 2 bytes into a single, signed 16-bit value
  tempValue = tempValue / 10; //TS sends values multiplied by 10 so divide back to whole degrees. 
  tempValue = ((tempValue - 32) * 5) / 9; //Convert from F to C
  //Apply the temp offset and check that it results in all values being positive
  return max( tempValue + CALIBRATION_TEMPERATURE_OFFSET, 0 );
}

/**
 * @brief Update a temperature calibration table from serialPayload
  * 
 * @param calibrationLength The chunk size received from TS
 * @param calibrationPage Index of the table
 * @param values The table values
 * @param bins The table bin values
 */
void TSCommClass::processTemperatureCalibrationTableUpdate(uint16_t calibrationLength, uint8_t calibrationPage, uint16_t *values, uint16_t *bins)
{
  //Temperature calibrations are sent as 32 16-bit values
  if(calibrationLength == 64U)
  {
    for (uint16_t x = 0; x < 32U; x++)
    {
      values[x] = toTemperature(serialPayloadRx[(2U * x) + 7U], serialPayloadRx[(2U * x) + 8U]);
      bins[x] = (x * 33U); // 0*33=0 to 31*33=1023
    }
    Storage.storeCalibrationCRC32(calibrationPage, CRC32_serial.crc32(&serialPayloadRx[7], 64));
    writeCalibrationPage(calibrationPage);
	
    sendReturnCodeMsg(SERIAL_RC_OK);
  }
  else 
  { 
    sendReturnCodeMsg(SERIAL_RC_RANGE_ERR); 
  }
}

// ====================================== End Internal Functions =============================


/** 
 * 
*/
void TSCommClass::sendToothLog(void)
{
	//We need TOOTH_LOG_SIZE number of records to send to TunerStudio. If there aren't that many in the buffer then we just return and wait for the next call
	if (BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY) == false)
	{
		//If the buffer is not yet full but TS has timed out, pad the rest of the buffer with 0s
		while(toothHistoryIndex < TOOTH_LOG_SIZE)
		{
		  toothHistory[toothHistoryIndex] = 0;
		  toothHistoryIndex++;
		}
	}

		//Setup the transmit buffer
	uint16_t payLen = sizeof(toothHistory)  + 1;	//Size of the tooth log (uint32_t values) plus the return code

	serialPayloadTx[0] = payLen >> 8;
	serialPayloadTx[1] = payLen;
	serialPayloadTx[2] = SERIAL_RC_OK;

	memcpy( (void *) &serialPayloadTx[3], toothHistory, sizeof(toothHistory));

	sendSerialPayload(payLen);

//	BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
//	serialStatusFlag = SERIAL_INACTIVE;
//	toothHistoryIndex = 0;
//	logItemsTransmitted = 0;
}

void TSCommClass::sendCompositeLog(void)
{
	if ( BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY) == false )
	{
		//If the buffer is not yet full but TS has timed out, pad the rest of the buffer with 0s
		while( toothHistoryIndex < TOOTH_LOG_SIZE)
		{
		  toothHistory[toothHistoryIndex] = toothHistory[toothHistoryIndex-1]; //Composite logger needs a realistic time value to display correctly. Copy the last value
		  compositeLogHistory[toothHistoryIndex] = 0;
		  toothHistoryIndex++;
		}
	}

   //Setup the transmit buffer
	uint16_t payLen = sizeof(toothHistory) + sizeof(compositeLogHistory) + 1;	//Size of the tooth log (uint32_t values) plus the return code

	serialPayloadTx[0] = payLen >> 8;
	serialPayloadTx[1] = payLen;
	serialPayloadTx[2] = SERIAL_RC_OK;

	uint8_t *pPayLoad = &serialPayloadTx[3];

	for(logItemsTransmitted=0; logItemsTransmitted< TOOTH_LOG_SIZE; logItemsTransmitted++)
    {
		memcpy( (void *) pPayLoad, &toothHistory[logItemsTransmitted], 4);	// edge info

		pPayLoad += 4;

		*pPayLoad++ = compositeLogHistory[logItemsTransmitted];	// composite info
    }

	sendSerialPayload(payLen);

      //tx buffer is full. Store the current state so it can be resumed later
// serialStatusFlag = SERIAL_TRANSMIT_COMPOSITE_INPROGRESS;
// BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
// toothHistoryIndex = 0;
// serialStatusFlag = SERIAL_INACTIVE;
// logItemsTransmitted = 0;

}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Legacy zone

void TSCommClass::legacyHandler_qest_mark(void){
//case '?':

const char helpString[] = "\n"\
	   "===Command Help===\n\n"\
	   "All commands are single character and are concatenated with their parameters \n"\
	   "without spaces."\
	   "Syntax:  <command>+<parameter1>+<parameter2>+<parameterN>\n\n"\
	   "===List of Commands===\n\n"\
	   "A - Displays 31 bytes of currentStatus values in binary (live data)\n"\
	   "B - Burn current map and configPage values to eeprom\n"\
	   "C - Test COM port.	Used by Tunerstudio to see whether an ECU is on a given serial \n"\
	   "	port. Returns a binary number.\n"
	   "N - Print new line.\n"
	   "P - Set current page.  Syntax:	P+<pageNumber>\n"
	   "R - Same as A command\n"
	   "S - Display signature number\n"
	   "Q - Same as S command\n"
	   "V - Display map or configPage values in binary\n"
	   "W - Set one byte in map or configPage.	Expects binary parameters. \n"
	   "	Syntax:  W+<offset>+<newbyte>\n"
	   "t - Set calibration values.  Expects binary parameters.  Table index is either 0, \n"
	   "	1, or 2.  Syntax:  t+<tble_idx>+<newValue1>+<newValue2>+<newValueN>\n"
	   "Z - Display calibration values\n"
	   "T - Displays 256 tooth log entries in binary\n"
	   "r - Displays 256 tooth log entries\n"
	   "U - Prepare for firmware update. The next byte received will cause the Arduino to reset.\n"
	   "? - Displays this help page" ;

	memcpy(serialPayloadTx, helpString, sizeof(helpString) );

	sendSerialPayloadLegacy( sizeof(helpString) );
}

void TSCommClass::legacyHandler_A(void){
}

void TSCommClass::legacyHandler_B(void){
}

void TSCommClass::legacyHandler_C(void){
}

void TSCommClass::legacyHandler_D(void){
}

void TSCommClass::legacyHandler_E(void){
}

void TSCommClass::legacyHandler_F(void){

	strcpy( (char *) serialPayloadTx, serialVersion);

	sendSerialPayloadLegacy( strlen(serialVersion) );
}

void TSCommClass::legacyHandler_G(void){
}

void TSCommClass::legacyHandler_H(void){

}

void TSCommClass::legacyHandler_I(void){
}

void TSCommClass::legacyHandler_J(void){
}

void TSCommClass::legacyHandler_K(void){

}

void TSCommClass::legacyHandler_L(void){
}

void TSCommClass::legacyHandler_M(void){

}

void TSCommClass::legacyHandler_N(void){
}

void TSCommClass::legacyHandler_O(void){
}

void TSCommClass::legacyHandler_P(void){
}

void TSCommClass::legacyHandler_Q(void){

	//case 'Q': // send code version

	memcpy(serialPayloadTx, codeVersion, sizeof(codeVersion) );

	sendSerialPayloadLegacy(sizeof(codeVersion));
}

void TSCommClass::legacyHandler_R(void){
}

void TSCommClass::legacyHandler_S(void){
}

void TSCommClass::legacyHandler_T(void){
}

void TSCommClass::legacyHandler_U(void){
}

void TSCommClass::legacyHandler_V (void){
}

void TSCommClass::legacyHandler_W (void){

}

void TSCommClass::legacyHandler_X (void){
}

void TSCommClass::legacyHandler_Y(void){
}

void TSCommClass::legacyHandler_Z(void){
}


void TSCommClass::legacyHandler_96(void){
}

void TSCommClass::legacyHandler_a (void){
}

void TSCommClass::legacyHandler_b (void){
}

void TSCommClass::legacyHandler_c (void){
}

void TSCommClass::legacyHandler_d (void){
}

void TSCommClass::legacyHandler_e (void){
}

void TSCommClass::legacyHandler_f (void){
}

void TSCommClass::legacyHandler_g (void){
}

void TSCommClass::legacyHandler_h (void){
}

void TSCommClass::legacyHandler_i (void){
}

void TSCommClass::legacyHandler_j (void){
	/*
	case 'j': //Stop the composite logger
		stopCompositeLogger();
		break;
*/
}

void TSCommClass::legacyHandler_k (void){

}

void TSCommClass::legacyHandler_l (void){
}

void TSCommClass::legacyHandler_m (void){
	/*
case 'm': //Send the current free memory
	currentStatus.freeRAM = freeRam();
	Serial.write(lowByte(currentStatus.freeRAM));
	Serial.write(highByte(currentStatus.freeRAM));
*/

}

void TSCommClass::legacyHandler_n (void){
}

void TSCommClass::legacyHandler_o (void){
	/*
		  case 'o': //Stop the composite logger 2nd cam (tertiary)
			stopCompositeLoggerTertiary();
			break;
*/
}

void TSCommClass::legacyHandler_p (void){
}

void TSCommClass::legacyHandler_q (void){
}

void TSCommClass::legacyHandler_r (void){
}

void TSCommClass::legacyHandler_s (void){
}

void TSCommClass::legacyHandler_t (void){
}

void TSCommClass::legacyHandler_u (void){

}

void TSCommClass::legacyHandler_v (void){
}


void TSCommClass::legacyHandler_w (void){

}

void TSCommClass::legacyHandler_x (void){
}

void TSCommClass::legacyHandler_y (void){
}

void TSCommClass::legacyHandler_z (void){
}

const TSCommClass::TsCmdHandler TSCommClass::cmdHandlerTable[] = {
		{'?', &TSCommClass::cmdHandler_qest_mark },
		{'@', NULL },
		{'A', &TSCommClass::cmdHandler_A },
		{'B', &TSCommClass::cmdHandler_B },
		{'C', &TSCommClass::cmdHandler_C },
		{'D', &TSCommClass::cmdHandler_D },
		{'E', &TSCommClass::cmdHandler_E },
		{'F', &TSCommClass::cmdHandler_F },
		{'G', &TSCommClass::cmdHandler_G },
		{'H', &TSCommClass::cmdHandler_H },
		{'I', &TSCommClass::cmdHandler_I },
		{'J', &TSCommClass::cmdHandler_J },
		{'J', &TSCommClass::cmdHandler_K },
		{'L', &TSCommClass::cmdHandler_L },
		{'M', &TSCommClass::cmdHandler_M },
		{'N', &TSCommClass::cmdHandler_N },
		{'O', &TSCommClass::cmdHandler_O },
		{'P', &TSCommClass::cmdHandler_P },
		{'Q', &TSCommClass::cmdHandler_Q },
		{'R', &TSCommClass::cmdHandler_R },
		{'S', &TSCommClass::cmdHandler_S },
		{'T', &TSCommClass::cmdHandler_T },
		{'U', &TSCommClass::cmdHandler_U },
		{'V', &TSCommClass::cmdHandler_V },
		{'W', &TSCommClass::cmdHandler_W },
		{'X', &TSCommClass::cmdHandler_X },
		{'Y', &TSCommClass::cmdHandler_Y },
		{'Z', &TSCommClass::cmdHandler_Z },
		{'[', NULL },
		{'\\',NULL },
		{']',  NULL },
		{'^',  NULL },
		{'_',  NULL },
		{' ', &TSCommClass::cmdHandler_96 },
		{'a', &TSCommClass::cmdHandler_a },
		{'b', &TSCommClass::cmdHandler_b },
		{'c', &TSCommClass::cmdHandler_c },
		{'d', &TSCommClass::cmdHandler_d },
		{'e', &TSCommClass::cmdHandler_e },
		{'f', &TSCommClass::cmdHandler_f },
		{'g', &TSCommClass::cmdHandler_g },
		{'h', &TSCommClass::cmdHandler_h },
		{'i', &TSCommClass::cmdHandler_i },
		{'j', &TSCommClass::cmdHandler_j },
		{'j', &TSCommClass::cmdHandler_k },
		{'l', &TSCommClass::cmdHandler_l },
		{'m', &TSCommClass::cmdHandler_m },
		{'n', &TSCommClass::cmdHandler_n },
		{'o', &TSCommClass::cmdHandler_o },
		{'p', &TSCommClass::cmdHandler_p },
		{'q', &TSCommClass::cmdHandler_q },
		{'r', &TSCommClass::cmdHandler_r },
		{'s', &TSCommClass::cmdHandler_s },
		{'t', &TSCommClass::cmdHandler_t },
		{'u', &TSCommClass::cmdHandler_u },
		{'v', &TSCommClass::cmdHandler_v },
		{'w', &TSCommClass::cmdHandler_w },
		{'x', &TSCommClass::cmdHandler_x },
		{'y', &TSCommClass::cmdHandler_y },
		{'z', &TSCommClass::cmdHandler_z },
};

const TSCommClass::TsCmdHandler TSCommClass::legacyHandlerTable[] = {
	{'?', &TSCommClass::legacyHandler_qest_mark },
	{'@', NULL },
	{'A', &TSCommClass::legacyHandler_A },
	{'B', &TSCommClass::legacyHandler_B },
	{'C', &TSCommClass::legacyHandler_C },
	{'D', &TSCommClass::legacyHandler_D },
	{'E', &TSCommClass::legacyHandler_E },
	{'F', &TSCommClass::legacyHandler_F },
	{'G', &TSCommClass::legacyHandler_G },
	{'H', &TSCommClass::legacyHandler_H },
	{'I', &TSCommClass::legacyHandler_I },
	{'J', &TSCommClass::legacyHandler_J },
	{'J', &TSCommClass::legacyHandler_K },
	{'L', &TSCommClass::legacyHandler_L },
	{'M', &TSCommClass::legacyHandler_M },
	{'N', &TSCommClass::legacyHandler_N },
	{'O', &TSCommClass::legacyHandler_O },
	{'P', &TSCommClass::legacyHandler_P },
	{'Q', &TSCommClass::legacyHandler_Q },
	{'R', &TSCommClass::legacyHandler_R },
	{'S', &TSCommClass::legacyHandler_S },
	{'T', &TSCommClass::legacyHandler_T },
	{'U', &TSCommClass::legacyHandler_U },
	{'V', &TSCommClass::legacyHandler_V },
	{'W', &TSCommClass::legacyHandler_W },
	{'X', &TSCommClass::legacyHandler_X },
	{'Y', &TSCommClass::legacyHandler_Y },
	{'Z', &TSCommClass::legacyHandler_Z },
	{'[',  NULL },
	{'\\', NULL },
	{']',  NULL },
	{'^',  NULL },
	{'_',  NULL },
	{' ', &TSCommClass::legacyHandler_96 },
	{'a', &TSCommClass::legacyHandler_a },
	{'b', &TSCommClass::legacyHandler_b },
	{'c', &TSCommClass::legacyHandler_c },
	{'d', &TSCommClass::legacyHandler_d },
	{'e', &TSCommClass::legacyHandler_e },
	{'f', &TSCommClass::legacyHandler_f },
	{'g', &TSCommClass::legacyHandler_g },
	{'h', &TSCommClass::legacyHandler_h },
	{'i', &TSCommClass::legacyHandler_i },
	{'j', &TSCommClass::legacyHandler_j },
	{'j', &TSCommClass::legacyHandler_k },
	{'l', &TSCommClass::legacyHandler_l },
	{'m', &TSCommClass::legacyHandler_m },
	{'n', &TSCommClass::legacyHandler_n },
	{'o', &TSCommClass::legacyHandler_o },
	{'p', &TSCommClass::legacyHandler_p },
	{'q', &TSCommClass::legacyHandler_q },
	{'r', &TSCommClass::legacyHandler_r },
	{'s', &TSCommClass::legacyHandler_s },
	{'t', &TSCommClass::legacyHandler_t },
	{'u', &TSCommClass::legacyHandler_u },
	{'v', &TSCommClass::legacyHandler_v },
	{'w', &TSCommClass::legacyHandler_w },
	{'x', &TSCommClass::legacyHandler_x },
	{'y', &TSCommClass::legacyHandler_y },
	{'z', &TSCommClass::legacyHandler_z },

};




#endif


