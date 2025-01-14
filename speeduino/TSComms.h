/** \file comms.h
 * @brief File for handling all serial requests 
 * @author Josh Stewart
 * 
 * This file contains all the functions associated with serial comms.
 * This includes sending of live data, sending/receiving current page data, sending CRC values of pages, receiving sensor calibration data etc
 * 
 */

#ifndef NEW_COMMS_H
#define NEW_COMMS_H

#if defined(CORE_M451)
  #define BLOCKING_FACTOR       121
  #define TABLE_BLOCKING_FACTOR 64
#endif


#define SERIAL_RC_OK        0x00 //!< Success
#define SERIAL_RC_REALTIME  0x01 //!< Unused
#define SERIAL_RC_PAGE      0x02 //!< Unused

#define SERIAL_RC_BURN_OK   0x04 //!< EEPROM write succeeded

#define SERIAL_RC_TIMEOUT   0x80 //!< Timeout error
#define SERIAL_RC_CRC_ERR   0x82 //!< CRC mismatch
#define SERIAL_RC_UKWN_ERR  0x83 //!< Unknown command
#define SERIAL_RC_RANGE_ERR 0x84 //!< Incorrect range. TS will not retry command
#define SERIAL_RC_BUSY_ERR  0x85 //!< TS will wait and retry

#define SERIAL_LEN_SIZE     2U
#define SERIAL_TIMEOUT      3000 //ms

#define SEND_OUTPUT_CHANNELS 48U


// tuner studio communication class

class TSCommClass
{

public:


	/** \enum SerialStatus
	 * @brief The current state of serial communication
	 * */
	enum SerialStatus {
	  /** No serial comms is in progress */
	  SERIAL_INACTIVE,
	  /** A partial write is in progress. */
	  SERIAL_TRANSMIT_INPROGRESS,
	  /** A partial write is in progress (legacy send). */
	  SERIAL_TRANSMIT_INPROGRESS_LEGACY,
	  /** We are part way through transmitting the tooth log */
	  SERIAL_TRANSMIT_TOOTH_INPROGRESS,
	  /** We are part way through transmitting the tooth log (legacy send) */
	  SERIAL_TRANSMIT_TOOTH_INPROGRESS_LEGACY,
	  /** We are part way through transmitting the composite log */
	  SERIAL_TRANSMIT_COMPOSITE_INPROGRESS,
	  /** We are part way through transmitting the composite log (legacy send) */
	  SERIAL_TRANSMIT_COMPOSITE_INPROGRESS_LEGACY,
	  /** Whether or not a serial request has only been partially received.
	   * This occurs when a the length has been received in the serial buffer,
	   * but not all of the payload or CRC has yet been received.
	   *
	   * Expectation is that ::serialReceive is called  until the status reverts
	   * to SERIAL_INACTIVE
	  */
	  SERIAL_RECEIVE_WAIT_FOR_LEN,
	  SERIAL_RECEIVE_PAYLOAD,
	//  SERIAL_RECEIVE_INPROGRESS,
	  SERIAL_RECEIVE_CRC0,
	  SERIAL_RECEIVE_CRC1,
	  SERIAL_RECEIVE_CRC2,
	  SERIAL_RECEIVE_CRC3,

	  SERIAL_COMMAND_INPROGRESS,

	  /** We are part way through processing a legacy serial commang: call ::serialReceive */
	  SERIAL_COMMAND_INPROGRESS_LEGACY
	};


	enum SerialPhase {
		SERIAL_PHASE_IDLE,
		SERIAL_PHASE_DATARX,

	};

	struct TsCmdHandler{
		char	cmd;
		void 	(TSCommClass::*pf_fun)(void);
	};

	 TSCommClass();

	/**
	 * @brief The serial receive pump. Should be called whenever the serial port
	 * has data available to read.
	 */
	void serialReceive(void);

	/** @brief The serial transmit pump. Should be called when ::serialStatusFlag indicates a transmit
	 * operation is in progress */
	void serialTransmit(void);

	/**
	 * @brief Is a serial write in progress?
	 *
	 * Expectation is that ::serialTransmit is called until this
	 * returns false
	 */
	inline bool serialTransmitInProgress(void) {
	    return serialStatusFlag==SERIAL_TRANSMIT_INPROGRESS
	    || serialStatusFlag==SERIAL_TRANSMIT_INPROGRESS_LEGACY
	    || serialStatusFlag==SERIAL_TRANSMIT_TOOTH_INPROGRESS
	    || serialStatusFlag==SERIAL_TRANSMIT_TOOTH_INPROGRESS_LEGACY
	    || serialStatusFlag==SERIAL_TRANSMIT_COMPOSITE_INPROGRESS
	    || serialStatusFlag==SERIAL_TRANSMIT_COMPOSITE_INPROGRESS_LEGACY;
	}
	
protected:

	/** @brief Processes a message once it has been fully recieved */
	void processSerialCommand(char __c);
	void legacySerialCommand(char __c);

	inline bool isMap(void) {
	    // Detecting if the current page is a table/map
	  return (currentPage == veMapPage) || (currentPage == ignMapPage) || (currentPage == afrMapPage) || (currentPage == fuelMap2Page) || (currentPage == ignMap2Page);
	}

	/** @brief The number of bytes received or transmitted to date during nonblocking I/O.
	 *
	 * @attention We can share one variable between rx & tx because we only support simpex serial comms.
	 * I.e. we can only be receiving or transmitting at any one time.
	 */

	inline bool isTimeout(void) {
	  return (millis() - serialReceiveStartTime) > SERIAL_TIMEOUT;
	}

	uint16_t toTemperature(byte lo, byte hi);
	void processTemperatureCalibrationTableUpdate(uint16_t calibrationLength, uint8_t calibrationPage, uint16_t *values, uint16_t *bins);
	void loadO2CalibrationChunk(uint16_t offset, uint16_t chunkSize);
	void sendReturnCodeMsg(byte returnCode);
	void sendReturnStringMsg(const char *__string);
	void sendReturnCRCMsg(const byte *__p_crc);
	void sendReturnLiveValuesMsg(uint16_t __offset, uint16_t __packetLength);

	void sendToothLog_legacy(byte startOffset);
	int  sendBufferNonBlocking(void);

	void sendSerialPayload(uint16_t __payloadLength);
	void sendSerialPayloadLegacy(uint16_t __payloadLength);

	/** @brief Should be called when ::serialStatusFlag == SERIAL_TRANSMIT_TOOTH_INPROGRESS, */
	void sendToothLog(void);

	/** @brief Should be called when ::serialStatusFlag == LOG_SEND_COMPOSITE */
	void sendCompositeLog(void);




	uint32_t serialReceiveStartTime = 0; 		//!< The time at which the serial receive started. Used for calculating whether a timeout has occurred */
	FastCRC32 CRC32_serial; 					//!< Support accumulation of a CRC during non-blocking operations */

	uint16_t serialBytesRx = 0;
	uint8_t serialPayloadRx[TSCOMM_RX_SERIAL_BUFFER_SIZE]; 		//!< Serial payload buffer. */
	uint16_t serialPayloadRxLength = 0; //!< How many bytes in serialPayload were received */

	uint16_t serialBytesTx = 0;
	uint8_t serialPayloadTx[TSCOMM_TX_SERIAL_BUFFER_SIZE]; //!< Serial payload buffer. */
	uint16_t serialPayloadTxLength = 0; //!< How many bytes in serialPayload are ready to send */

	byte currentPage = 1;			//Not the same as the speeduino config page numbers

	bool firstCommsRequest = true; 	/**< The number of times the A command has been issued. This is used to track whether a reset has recently been performed on the controller */

	byte currentCommand; 			/**< The serial command that is currently being processed. This is only useful when cmdPending=True */
	bool chunkPending = false; 		/**< Whether or not the current chunk write is complete or not */

	uint16_t chunkComplete = 0; 	/**< The number of bytes in a chunk write that have been written so far */
	uint16_t chunkSize = 0; 		/**< The complete size of the requested chunk write */
	int valueOffset; 				/**< The memory offset within a given page for a value to be read from or written to. Note that we cannot use 'offset' as a variable name, it is a reserved word for several teensy libraries */

	byte logItemsTransmitted;
	byte inProgressLength;


	/** @brief Current status of serial comms. */
	SerialStatus serialStatusFlag;

	SerialPhase serialStatusPhase;

private:

		// standard command handlers
		
	void cmdHandler_qest_mark(void);

	void cmdHandler_A(void);
	void cmdHandler_B(void);
	void cmdHandler_C(void);
	void cmdHandler_D(void);
	void cmdHandler_E(void);
	void cmdHandler_F(void);
	void cmdHandler_G(void);
	void cmdHandler_H(void);
	void cmdHandler_I(void);

	void cmdHandler_J(void);
	void cmdHandler_K(void);
	void cmdHandler_L(void);
	void cmdHandler_M(void);
	void cmdHandler_N(void);
	void cmdHandler_O(void);
	void cmdHandler_P(void);
	void cmdHandler_Q(void);
	void cmdHandler_R(void);
	void cmdHandler_S(void);
	void cmdHandler_T(void);
	void cmdHandler_U(void);
	void cmdHandler_V (void);
	void cmdHandler_W (void);
	void cmdHandler_X (void);
	void cmdHandler_Y(void);
	void cmdHandler_Z(void);
	void cmdHandler_96(void);
	void cmdHandler_a (void);
	void cmdHandler_b (void);
	void cmdHandler_c (void);
	void cmdHandler_d (void);
	void cmdHandler_e (void);
	void cmdHandler_f (void);
	void cmdHandler_g (void);
	void cmdHandler_h (void);
	void cmdHandler_i (void);
	void cmdHandler_j (void);
	void cmdHandler_k (void);
	void cmdHandler_l (void);
	void cmdHandler_m (void);
	void cmdHandler_n (void);
	void cmdHandler_o (void);
	void cmdHandler_p (void);
	void cmdHandler_q (void);
	void cmdHandler_r (void);
	void cmdHandler_s (void);
	void cmdHandler_t (void);
	void cmdHandler_u (void);
	void cmdHandler_v (void);
	void cmdHandler_w (void);
	void cmdHandler_x (void);
	void cmdHandler_y (void);
	void cmdHandler_z (void);

		// legacy command handlers
	void legacyHandler_qest_mark(void);

	void legacyHandler_A(void);
	void legacyHandler_B(void);
	void legacyHandler_C(void);
	void legacyHandler_D(void);
	void legacyHandler_E(void);
	void legacyHandler_F(void);
	void legacyHandler_G(void);
	void legacyHandler_H(void);
	void legacyHandler_I(void);

	void legacyHandler_J(void);
	void legacyHandler_K(void);
	void legacyHandler_L(void);
	void legacyHandler_M(void);
	void legacyHandler_N(void);
	void legacyHandler_O(void);
	void legacyHandler_P(void);
	void legacyHandler_Q(void);
	void legacyHandler_R(void);
	void legacyHandler_S(void);
	void legacyHandler_T(void);
	void legacyHandler_U(void);
	void legacyHandler_V (void);
	void legacyHandler_W (void);
	void legacyHandler_X (void);
	void legacyHandler_Y(void);
	void legacyHandler_Z(void);
	void legacyHandler_96(void);
	void legacyHandler_a (void);
	void legacyHandler_b (void);
	void legacyHandler_c (void);
	void legacyHandler_d (void);
	void legacyHandler_e (void);
	void legacyHandler_f (void);
	void legacyHandler_g (void);
	void legacyHandler_h (void);
	void legacyHandler_i (void);
	void legacyHandler_j (void);
	void legacyHandler_k (void);
	void legacyHandler_l (void);
	void legacyHandler_m (void);
	void legacyHandler_n (void);
	void legacyHandler_o (void);
	void legacyHandler_p (void);
	void legacyHandler_q (void);
	void legacyHandler_r (void);
	void legacyHandler_s (void);
	void legacyHandler_t (void);
	void legacyHandler_u (void);
	void legacyHandler_v (void);
	void legacyHandler_w (void);
	void legacyHandler_x (void);
	void legacyHandler_y (void);
	void legacyHandler_z (void);


	static const TsCmdHandler cmdHandlerTable[60];
	static const TsCmdHandler legacyHandlerTable[60];

};


extern TSCommClass tsComm;



#endif // COMMS_H
