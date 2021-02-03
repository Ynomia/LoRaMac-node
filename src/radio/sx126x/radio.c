/*!
 * \file      radio.c
 *
 * \brief     Radio driver API definition
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "FreeRTOS.h"
#include "semphr.h"

#include "board.h"
#include "log.h"
#include "radio.h"
#include "sx126x-board.h"
#include "sx126x.h"
#include "timer.h"
#include "utilities.h"
#include <math.h>
#include <string.h>

/*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 */
void RadioInit( RadioEvents_t *events );

/*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t RadioGetStatus( void );

/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
void RadioSetModem( RadioModems_t modem );

/*!
 * \brief Sets the channel frequency
 *
 * \param [IN] freq         Channel RF frequency
 */
void RadioSetChannel( uint32_t freq );

/*!
 * \brief Checks if the channel is free for the given time
 *
 * \remark The FSK modem is always used for this task as we can select the Rx bandwidth at will.
 *
 * \param [IN] freq                Channel RF frequency in Hertz
 * \param [IN] rxBandwidth         Rx bandwidth in Hertz
 * \param [IN] rssiThresh          RSSI threshold in dBm
 * \param [IN] maxCarrierSenseTime Max time in milliseconds while the RSSI is measured
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
bool RadioIsChannelFree( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime );

/*!
 * \brief Generates a 32 bits random value based on the RSSI readings
 *
 * \remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either Radio.SetRxConfig or
 *         Radio.SetTxConfig functions must be called.
 *
 * \retval randomValue    32 bits random value
 */
uint32_t RadioRandom( void );

/*!
 * \brief Sets the reception parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value
 *                          FSK : timeout in number of bytes
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
					   uint32_t datarate, uint8_t coderate,
					   uint32_t bandwidthAfc, uint16_t preambleLen,
					   uint16_t symbTimeout, bool fixLen,
					   uint8_t payloadLen,
					   bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
					   bool iqInverted, bool rxContinuous );

/*!
 * \brief Sets the transmission parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [ms]
 */
void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
					   uint32_t bandwidth, uint32_t datarate,
					   uint8_t coderate, uint16_t preambleLen,
					   bool fixLen, bool crcOn, bool FreqHopOn,
					   uint8_t HopPeriod, bool iqInverted, uint32_t timeout );

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool RadioCheckRfFrequency( uint32_t frequency );

/*!
 * \brief Computes the packet time on air in ms for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 *
 * \retval airTime        Computed airTime (ms) for the given packet payload length
 */
uint32_t RadioTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
						 uint32_t datarate, uint8_t coderate,
						 uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
						 bool crcOn );

/*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void RadioSend( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the radio in sleep mode
 */
void RadioSleep( void );

/*!
 * \brief Sets the radio in standby mode
 */
void RadioStandby( void );

/*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRx( uint32_t timeout );

/*!
 * \brief Start a Channel Activity Detection
 */
void RadioStartCad( void );

/*!
 * \brief Sets the radio in continuous wave transmission mode
 *
 * \param [IN]: freq       Channel RF frequency
 * \param [IN]: power      Sets the output power [dBm]
 * \param [IN]: time       Transmission mode timeout [s]
 */
void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time );

/*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
int16_t RadioRssi( RadioModems_t modem );

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void RadioWrite( uint32_t addr, uint8_t data );

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \retval data Register value
 */
uint8_t RadioRead( uint32_t addr );

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void RadioWriteBuffer( uint32_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void RadioReadBuffer( uint32_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the maximum payload length.
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] max        Maximum payload length in bytes
 */
void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max );

/*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \remark Applies to LoRa modem only
 *
 * \param [IN] enable if true, it enables a public network
 */
void RadioSetPublicNetwork( bool enable );

/*!
 * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * \retval time Radio plus board wakeup time in ms.
 */
uint32_t RadioGetWakeupTime( void );

/*!
 * \brief Process radio irq
 */
void RadioIrqProcess( void );

/*!
 * \brief Sets the radio in reception mode with Max LNA gain for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRxBoosted( uint32_t timeout );

/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
void RadioSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime );

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio = {
	RadioInit,
	RadioGetStatus,
	RadioSetModem,
	RadioSetChannel,
	RadioIsChannelFree,
	RadioRandom,
	RadioSetRxConfig,
	RadioSetTxConfig,
	RadioCheckRfFrequency,
	RadioTimeOnAir,
	RadioSend,
	RadioSleep,
	RadioStandby,
	RadioRx,
	RadioStartCad,
	RadioSetTxContinuousWave,
	RadioRssi,
	RadioWrite,
	RadioRead,
	RadioWriteBuffer,
	RadioReadBuffer,
	RadioSetMaxPayloadLength,
	RadioSetPublicNetwork,
	RadioGetWakeupTime,
	RadioIrqProcess,
	// Available on SX126x only
	RadioRxBoosted,
	RadioSetRxDutyCycle
};

/*
 * Local types definition
 */

/*!
 * FSK bandwidth definition
 */
typedef struct
{
	uint32_t bandwidth;
	uint8_t	 RegValue;
} FskBandwidth_t;

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] = {
	{ 4800, 0x1F },
	{ 5800, 0x17 },
	{ 7300, 0x0F },
	{ 9700, 0x1E },
	{ 11700, 0x16 },
	{ 14600, 0x0E },
	{ 19500, 0x1D },
	{ 23400, 0x15 },
	{ 29300, 0x0D },
	{ 39000, 0x1C },
	{ 46900, 0x14 },
	{ 58600, 0x0C },
	{ 78200, 0x1B },
	{ 93800, 0x13 },
	{ 117300, 0x0B },
	{ 156200, 0x1A },
	{ 187200, 0x12 },
	{ 234300, 0x0A },
	{ 312000, 0x19 },
	{ 373600, 0x11 },
	{ 467000, 0x09 },
	{ 500000, 0x00 }, // Invalid Bandwidth
};

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };

uint8_t MaxPayloadLength = 0xFF;

uint32_t TxTimeout = 0;
uint32_t RxTimeout = 0;

bool RxContinuous = false;

xPacketStatus_t RadioPktStatus;
uint8_t			RadioRxPayload[255];

/*
 * SX126x DIO IRQ callback functions prototype
 */
bool IrqFired = false;

/*!
 * \brief DIO 0 IRQ callback
 */
void RadioOnDioIrq( void *context );

/*!
 * \brief Tx timeout timer callback
 */
void RadioOnTxTimeoutIrq( void *context );

/*!
 * \brief Rx timeout timer callback
 */
void RadioOnRxTimeoutIrq( void *context );

/*
 * Private global variables
 */

/*!
 * Holds the current network type for the radio
 */
typedef struct
{
	bool Previous;
	bool Current;
} RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = { false };

/*!
 * Radio callbacks variable
 */
static RadioEvents_t *RadioEvents;

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
SX126x_t SX126x;

/*!
 * Tx and Rx timers
 */
TimerEvent_t TxTimeoutTimer;
TimerEvent_t RxTimeoutTimer;

STATIC_SEMAPHORE_STRUCTURES( xLorawanInteruptHandle );

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t RadioGetFskBandwidthRegValue( uint32_t bandwidth )
{
	uint8_t i;

	if ( bandwidth == 0 ) {
		return ( 0x1F );
	}

	for ( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ ) {
		if ( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) ) {
			return FskBandwidths[i + 1].RegValue;
		}
	}
	// ERROR: Value not found
	while ( 1 )
		;
}

void RadioInit( RadioEvents_t *events )
{
	RadioEvents = events;
	vSX126xInit( RadioOnDioIrq );
	SX126xSetStandby( STDBY_RC );
	vSX126xSetRegulatorMode( USE_DCDC );
	vSX126xSetBufferBaseAddress( 0x00, 0x00 );
	vSX126xSetTxParams( 0, RADIO_RAMP_200_US );
	vSX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
	vSX126xAntSwOn(); //TODO

	// Initialize driver timeout timers
	TimerInit( &TxTimeoutTimer, RadioOnTxTimeoutIrq );
	TimerInit( &RxTimeoutTimer, RadioOnRxTimeoutIrq );

	// set the irq to false
	STATIC_SEMAPHORE_CREATE_BINARY( xLorawanInteruptHandle );
	IrqFired = false;
}

RadioState_t RadioGetStatus( void )
{
	switch ( eSX126xGetOperatingMode() ) {
		case MODE_TX:
			return RF_TX_RUNNING;
		case MODE_RX:
			return RF_RX_RUNNING;
		case MODE_CAD:
			return RF_CAD;
		default:
			return RF_IDLE;
	}
}

void RadioSetModem( RadioModems_t modem )
{
	switch ( modem ) {
		default:
		case MODEM_FSK:
			vSX126xSetPacketType( PACKET_TYPE_GFSK );
			// When switching to GFSK mode the LoRa SyncWord register value is reset
			// Thus, we also reset the RadioPublicNetwork variable
			RadioPublicNetwork.Current = false;
			break;
		case MODEM_LORA:
			vSX126xSetPacketType( PACKET_TYPE_LORA );
			// Public/Private network register is reset when switching modems
			if ( RadioPublicNetwork.Current != RadioPublicNetwork.Previous ) {
				RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
				RadioSetPublicNetwork( RadioPublicNetwork.Current );
			}
			break;
	}
}

void RadioSetChannel( uint32_t freq )
{
	vSX126xSetRfFrequency( freq );
}

bool RadioIsChannelFree( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
	bool	 status			  = true;
	int16_t	 rssi			  = 0;
	uint32_t carrierSenseTime = 0;

	RadioSetModem( MODEM_FSK );

	RadioSetChannel( freq );

	// TODO Set Rx bandwidth. Other parameters are not used.
	// RadioSetRxConfig( MODEM_FSK, rxBandwidth, 600, 0, rxBandwidth, 3, 0, false,
	// 				  0, false, 0, 0, false, true );
	RadioRx( 0 );

	vTaskDelay( pdMS_TO_TICKS( 1 ) );

	carrierSenseTime = TimerGetCurrentTime();

	// Perform carrier sense for maxCarrierSenseTime
	while ( TimerGetElapsedTime( carrierSenseTime ) < maxCarrierSenseTime ) {
		rssi = RadioRssi( MODEM_FSK );

		if ( rssi > rssiThresh ) {
			status = false;
			break;
		}
	}
	RadioSleep();
	return status;
}

uint32_t RadioRandom( void )
{
	uint32_t rnd = 0;

	/*
     * Radio setup for random number generation
     */
	// Set LoRa modem ON
	RadioSetModem( MODEM_LORA );

	// Disable LoRa modem interrupts
	vSX126xSetDioIrqParams( IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

	rnd = SX126xGetRandom();

	return rnd;
}

void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
					   uint32_t datarate, uint8_t coderate,
					   uint32_t bandwidthAfc, uint16_t preambleLen,
					   uint16_t symbTimeout, bool fixLen,
					   uint8_t payloadLen,
					   bool crcOn, bool freqHopOn, uint8_t hopPeriod,
					   bool iqInverted, bool rxContinuous )
{
	UNUSED( bandwidthAfc );
	UNUSED( freqHopOn );
	UNUSED( hopPeriod );
	RxContinuous = rxContinuous;
	if ( rxContinuous == true ) {
		symbTimeout = 0;
	}
	if ( fixLen == true ) {
		MaxPayloadLength = payloadLen;
	}
	else {
		MaxPayloadLength = 0xFF;
	}

	switch ( modem ) {
		case MODEM_FSK:
			vSX126xSetStopRxTimerOnPreambleDetect( false );
			pxSx126xModule->xModulationParams.ePacketType = PACKET_TYPE_GFSK;

			pxSx126xModule->xModulationParams.xParams.xGfsk.ulBitRate		   = datarate;
			pxSx126xModule->xModulationParams.xParams.xGfsk.xModulationShaping = MOD_SHAPING_G_BT_1;
			pxSx126xModule->xModulationParams.xParams.xGfsk.ucBandwidth		   = RadioGetFskBandwidthRegValue( bandwidth << 1 ); // SX126x bandwidth is double sided

			pxSx126xModule->xPacketParams.ePacketType					   = PACKET_TYPE_GFSK;
			pxSx126xModule->xPacketParams.xParams.xGfsk.usPreambleLength   = ( preambleLen << 3 ); // convert byte into bit
			pxSx126xModule->xPacketParams.xParams.xGfsk.ePreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
			pxSx126xModule->xPacketParams.xParams.xGfsk.ucSyncWordLength   = 3 << 3; // convert byte into bit
			pxSx126xModule->xPacketParams.xParams.xGfsk.eAddrComp		   = RADIO_ADDRESSCOMP_FILT_OFF;
			pxSx126xModule->xPacketParams.xParams.xGfsk.eHeaderType		   = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;
			pxSx126xModule->xPacketParams.xParams.xGfsk.ucPayloadLength	= MaxPayloadLength;
			if ( crcOn == true ) {
				pxSx126xModule->xPacketParams.xParams.xGfsk.eCrcLength = RADIO_CRC_2_BYTES_CCIT;
			}
			else {
				pxSx126xModule->xPacketParams.xParams.xGfsk.eCrcLength = RADIO_CRC_OFF;
			}
			pxSx126xModule->xPacketParams.xParams.xGfsk.eDcFree = RADIO_DC_FREEWHITENING;

			RadioStandby();
			RadioSetModem( ( pxSx126xModule->xModulationParams.ePacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
			vSX126xSetModulationParams( &pxSx126xModule->xModulationParams );
			vSX126xSetPacketParams( &pxSx126xModule->xPacketParams );
			SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
			SX126xSetWhiteningSeed( 0x01FF );

			RxTimeout = (uint32_t) symbTimeout * 8000UL / datarate;
			break;

		case MODEM_LORA:
			vSX126xSetStopRxTimerOnPreambleDetect( false );
			pxSx126xModule->xModulationParams.ePacketType					 = PACKET_TYPE_LORA;
			pxSx126xModule->xModulationParams.xParams.xLoRa.eSpreadingFactor = (RadioLoRaSpreadingFactors_t) datarate;
			pxSx126xModule->xModulationParams.xParams.xLoRa.eBandwidth		 = Bandwidths[bandwidth];
			pxSx126xModule->xModulationParams.xParams.xLoRa.eCodingRate		 = (RadioLoRaCodingRates_t) coderate;

			if ( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
				 ( ( bandwidth == 1 ) && ( datarate == 12 ) ) ) {
				pxSx126xModule->xModulationParams.xParams.xLoRa.ucLowDatarateOptimize = 0x01;
			}
			else {
				pxSx126xModule->xModulationParams.xParams.xLoRa.ucLowDatarateOptimize = 0x00;
			}

			pxSx126xModule->xPacketParams.ePacketType = PACKET_TYPE_LORA;

			if ( ( pxSx126xModule->xModulationParams.xParams.xLoRa.eSpreadingFactor == LORA_SF5 ) ||
				 ( pxSx126xModule->xModulationParams.xParams.xLoRa.eSpreadingFactor == LORA_SF6 ) ) {
				if ( preambleLen < 12 ) {
					pxSx126xModule->xPacketParams.xParams.xLoRa.usPreambleLength = 12;
				}
				else {
					pxSx126xModule->xPacketParams.xParams.xLoRa.usPreambleLength = preambleLen;
				}
			}
			else {
				pxSx126xModule->xPacketParams.xParams.xLoRa.usPreambleLength = preambleLen;
			}

			pxSx126xModule->xPacketParams.xParams.xLoRa.eHeaderType = (RadioLoRaPacketLengthsMode_t) fixLen;

			pxSx126xModule->xPacketParams.xParams.xLoRa.ucPayloadLength = MaxPayloadLength;
			pxSx126xModule->xPacketParams.xParams.xLoRa.eCrcMode		= (RadioLoRaCrcModes_t) crcOn;
			pxSx126xModule->xPacketParams.xParams.xLoRa.eInvertIQ		= (RadioLoRaIQModes_t) iqInverted;

			RadioStandby();
			RadioSetModem( ( pxSx126xModule->xModulationParams.ePacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
			vSX126xSetModulationParams( &pxSx126xModule->xModulationParams );
			vSX126xSetPacketParams( &pxSx126xModule->xPacketParams );
			SX126xSetLoRaSymbNumTimeout( symbTimeout );

			// WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
			if ( pxSx126xModule->xPacketParams.xParams.xLoRa.eInvertIQ == LORA_IQ_INVERTED ) {
				// RegIqPolaritySetup = @address 0x0736
				vSX126xWriteRegister( 0x0736, ucSX126xReadRegister( 0x0736 ) & ~( 1 << 2 ) );
			}
			else {
				// RegIqPolaritySetup @address 0x0736
				vSX126xWriteRegister( 0x0736, ucSX126xReadRegister( 0x0736 ) | ( 1 << 2 ) );
			}
			// WORKAROUND END

			// Timeout Max, Timeout handled directly in SetRx function
			RxTimeout = 0xFFFF;

			break;
	}
}

void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
					   uint32_t bandwidth, uint32_t datarate,
					   uint8_t coderate, uint16_t preambleLen,
					   bool fixLen, bool crcOn, bool freqHopOn,
					   uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
	UNUSED( freqHopOn );
	UNUSED( hopPeriod );
	switch ( modem ) {
		case MODEM_FSK:
			pxSx126xModule->xModulationParams.ePacketType			  = PACKET_TYPE_GFSK;
			pxSx126xModule->xModulationParams.xParams.xGfsk.ulBitRate = datarate;

			pxSx126xModule->xModulationParams.xParams.xGfsk.xModulationShaping = MOD_SHAPING_G_BT_1;
			pxSx126xModule->xModulationParams.xParams.xGfsk.ucBandwidth		   = RadioGetFskBandwidthRegValue( bandwidth << 1 ); // SX126x bandwidth is double sided
			pxSx126xModule->xModulationParams.xParams.xGfsk.ulFdev			   = fdev;

			pxSx126xModule->xPacketParams.ePacketType					   = PACKET_TYPE_GFSK;
			pxSx126xModule->xPacketParams.xParams.xGfsk.usPreambleLength   = ( preambleLen << 3 ); // convert byte into bit
			pxSx126xModule->xPacketParams.xParams.xGfsk.ePreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
			pxSx126xModule->xPacketParams.xParams.xGfsk.ucSyncWordLength   = 3 << 3; // convert byte into bit
			pxSx126xModule->xPacketParams.xParams.xGfsk.eAddrComp		   = RADIO_ADDRESSCOMP_FILT_OFF;
			pxSx126xModule->xPacketParams.xParams.xGfsk.eHeaderType		   = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;

			if ( crcOn == true ) {
				pxSx126xModule->xPacketParams.xParams.xGfsk.eCrcLength = RADIO_CRC_2_BYTES_CCIT;
			}
			else {
				pxSx126xModule->xPacketParams.xParams.xGfsk.eCrcLength = RADIO_CRC_OFF;
			}
			pxSx126xModule->xPacketParams.xParams.xGfsk.eDcFree = RADIO_DC_FREEWHITENING;

			RadioStandby();
			RadioSetModem( ( pxSx126xModule->xModulationParams.ePacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
			vSX126xSetModulationParams( &pxSx126xModule->xModulationParams );
			vSX126xSetPacketParams( &pxSx126xModule->xPacketParams );
			SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
			SX126xSetWhiteningSeed( 0x01FF );
			break;

		case MODEM_LORA:
			pxSx126xModule->xModulationParams.ePacketType					 = PACKET_TYPE_LORA;
			pxSx126xModule->xModulationParams.xParams.xLoRa.eSpreadingFactor = (RadioLoRaSpreadingFactors_t) datarate;
			pxSx126xModule->xModulationParams.xParams.xLoRa.eBandwidth		 = Bandwidths[bandwidth];
			pxSx126xModule->xModulationParams.xParams.xLoRa.eCodingRate		 = (RadioLoRaCodingRates_t) coderate;

			if ( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
				 ( ( bandwidth == 1 ) && ( datarate == 12 ) ) ) {
				pxSx126xModule->xModulationParams.xParams.xLoRa.ucLowDatarateOptimize = 0x01;
			}
			else {
				pxSx126xModule->xModulationParams.xParams.xLoRa.ucLowDatarateOptimize = 0x00;
			}

			pxSx126xModule->xPacketParams.ePacketType = PACKET_TYPE_LORA;

			if ( ( pxSx126xModule->xModulationParams.xParams.xLoRa.eSpreadingFactor == LORA_SF5 ) ||
				 ( pxSx126xModule->xModulationParams.xParams.xLoRa.eSpreadingFactor == LORA_SF6 ) ) {
				if ( preambleLen < 12 ) {
					pxSx126xModule->xPacketParams.xParams.xLoRa.usPreambleLength = 12;
				}
				else {
					pxSx126xModule->xPacketParams.xParams.xLoRa.usPreambleLength = preambleLen;
				}
			}
			else {
				pxSx126xModule->xPacketParams.xParams.xLoRa.usPreambleLength = preambleLen;
			}

			pxSx126xModule->xPacketParams.xParams.xLoRa.eHeaderType		= (RadioLoRaPacketLengthsMode_t) fixLen;
			pxSx126xModule->xPacketParams.xParams.xLoRa.ucPayloadLength = MaxPayloadLength;
			pxSx126xModule->xPacketParams.xParams.xLoRa.eCrcMode		= (RadioLoRaCrcModes_t) crcOn;
			pxSx126xModule->xPacketParams.xParams.xLoRa.eInvertIQ		= (RadioLoRaIQModes_t) iqInverted;

			RadioStandby();
			RadioSetModem( ( pxSx126xModule->xModulationParams.ePacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
			vSX126xSetModulationParams( &pxSx126xModule->xModulationParams );
			vSX126xSetPacketParams( &pxSx126xModule->xPacketParams );

			break;
		default:
			break;
	}

	// WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
	if ( ( modem == MODEM_LORA ) && ( pxSx126xModule->xModulationParams.xParams.xLoRa.eBandwidth == LORA_BW_500 ) ) {
		// RegTxModulation = @address 0x0889
		vSX126xWriteRegister( 0x0889, ucSX126xReadRegister( 0x0889 ) & ~( 1 << 2 ) );
	}
	else {
		// RegTxModulation = @address 0x0889
		vSX126xWriteRegister( 0x0889, ucSX126xReadRegister( 0x0889 ) | ( 1 << 2 ) );
	}
	// WORKAROUND END

	vSX126xSetRfTxPower( power );
	TxTimeout = timeout;
}

bool RadioCheckRfFrequency( uint32_t frequency )
{
	UNUSED( frequency );
	return true;
}

static uint32_t RadioGetLoRaBandwidthInHz( RadioLoRaBandwidths_t bw )
{
	uint32_t bandwidthInHz = 0;

	switch ( bw ) {
		case LORA_BW_007:
			bandwidthInHz = 7812UL;
			break;
		case LORA_BW_010:
			bandwidthInHz = 10417UL;
			break;
		case LORA_BW_015:
			bandwidthInHz = 15625UL;
			break;
		case LORA_BW_020:
			bandwidthInHz = 20833UL;
			break;
		case LORA_BW_031:
			bandwidthInHz = 31250UL;
			break;
		case LORA_BW_041:
			bandwidthInHz = 41667UL;
			break;
		case LORA_BW_062:
			bandwidthInHz = 62500UL;
			break;
		case LORA_BW_125:
			bandwidthInHz = 125000UL;
			break;
		case LORA_BW_250:
			bandwidthInHz = 250000UL;
			break;
		case LORA_BW_500:
			bandwidthInHz = 500000UL;
			break;
	}

	return bandwidthInHz;
}

static uint32_t RadioGetGfskTimeOnAirNumerator( uint32_t datarate, uint8_t coderate,
												uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
												bool crcOn )
{
	const RadioAddressComp_t addrComp		= RADIO_ADDRESSCOMP_FILT_OFF;
	const uint8_t			 syncWordLength = 3;

	return ( preambleLen << 3 ) +
		   ( ( fixLen == false ) ? 8 : 0 ) +
		   ( syncWordLength << 3 ) +
		   ( ( payloadLen +
			   ( addrComp == RADIO_ADDRESSCOMP_FILT_OFF ? 0 : 1 ) +
			   ( ( crcOn == true ) ? 2 : 0 ) )
			 << 3 );
}

static uint32_t RadioGetLoRaTimeOnAirNumerator( uint32_t bandwidth,
												uint32_t datarate, uint8_t coderate,
												uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
												bool crcOn )
{
	int32_t crDenom			  = coderate + 4;
	bool	lowDatareOptimize = false;

	// Ensure that the preamble length is at least 12 symbols when using SF5 or
	// SF6
	if ( ( datarate == 5 ) || ( datarate == 6 ) ) {
		if ( preambleLen < 12 ) {
			preambleLen = 12;
		}
	}

	if ( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
		 ( ( bandwidth == 1 ) && ( datarate == 12 ) ) ) {
		lowDatareOptimize = true;
	}

	int32_t ceilDenominator;
	int32_t ceilNumerator = ( payloadLen << 3 ) +
							( crcOn ? 16 : 0 ) -
							( 4 * datarate ) +
							( fixLen ? 0 : 20 );

	if ( datarate <= 6 ) {
		ceilDenominator = 4 * datarate;
	}
	else {
		ceilNumerator += 8;

		if ( lowDatareOptimize == true ) {
			ceilDenominator = 4 * ( datarate - 2 );
		}
		else {
			ceilDenominator = 4 * datarate;
		}
	}

	if ( ceilNumerator < 0 ) {
		ceilNumerator = 0;
	}

	// Perform integral ceil()
	int32_t intermediate =
		( ( ceilNumerator + ceilDenominator - 1 ) / ceilDenominator ) * crDenom + preambleLen + 12;

	if ( datarate <= 6 ) {
		intermediate += 2;
	}

	return ( uint32_t )( ( 4 * intermediate + 1 ) * ( 1 << ( datarate - 2 ) ) );
}

uint32_t RadioTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
						 uint32_t datarate, uint8_t coderate,
						 uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
						 bool crcOn )
{
	uint32_t numerator	 = 0;
	uint32_t denominator = 1;

	switch ( modem ) {
		case MODEM_FSK: {
			numerator	= 1000U * RadioGetGfskTimeOnAirNumerator( datarate, coderate,
																  preambleLen, fixLen,
																  payloadLen, crcOn );
			denominator = datarate;
		} break;
		case MODEM_LORA: {
			numerator	= 1000U * RadioGetLoRaTimeOnAirNumerator( bandwidth, datarate,
																  coderate, preambleLen,
																  fixLen, payloadLen, crcOn );
			denominator = RadioGetLoRaBandwidthInHz( Bandwidths[bandwidth] );
		} break;
	}
	// Perform integral ceil()
	return ( numerator + denominator - 1 ) / denominator;
}

void RadioSend( uint8_t *buffer, uint8_t size )
{
	vSX126xSetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						   IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						   IRQ_RADIO_NONE,
						   IRQ_RADIO_NONE );
	vSX126xSetModulationParams( &pxSx126xModule->xModulationParams );
	if ( eSX126xGetPacketType() == PACKET_TYPE_LORA ) {
		pxSx126xModule->xPacketParams.xParams.xLoRa.ucPayloadLength = size;
	}
	else {
		pxSx126xModule->xPacketParams.xParams.xGfsk.ucPayloadLength = size;
	}
	vSX126xSetPacketParams( &pxSx126xModule->xPacketParams );
	
	vSX126xSendPayload( buffer, size, 0 );
	// TODO
	// TimerSetValue( &TxTimeoutTimer, TxTimeout );
	// TimerStart( &TxTimeoutTimer );
}

void RadioSleep( void )
{
	SleepParams_t params = { 0 };

	params.Fields.WarmStart = 1;
	SX126xSetSleep( params );

	vTaskDelay( pdMS_TO_TICKS( 2 ) );
}

void RadioStandby( void )
{
	SX126xSetStandby( STDBY_RC );
}

void LorawanRadioRx( uint32_t timeout )
{
	UNUSED( timeout );
	vSX126xSetDioIrqParams( IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
							IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
							IRQ_RADIO_NONE,
							IRQ_RADIO_NONE );

	SX126xSetRx( 0xFFFFFF );
}

void RadioRx( uint32_t timeout )
{
	vSX126xSetDioIrqParams( IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
							IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
							IRQ_RADIO_NONE,
							IRQ_RADIO_NONE );

	if ( timeout != 0 ) {
		TimerSetValue( &RxTimeoutTimer, timeout );
		TimerStart( &RxTimeoutTimer );
	}

	if ( RxContinuous == true ) {
		SX126xSetRx( 0xFFFFFF ); // Rx Continuous
	}
	else {
		SX126xSetRx( RxTimeout << 6 );
	}
}

void RadioRxBoosted( uint32_t timeout )
{
	vSX126xSetDioIrqParams( IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
							IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
							IRQ_RADIO_NONE,
							IRQ_RADIO_NONE );

	if ( timeout != 0 ) {
		TimerSetValue( &RxTimeoutTimer, timeout );
		TimerStart( &RxTimeoutTimer );
	}

	if ( RxContinuous == true ) {
		vSX126xSetRxBoosted( 0xFFFFFF ); // Rx Continuous
	}
	else {
		vSX126xSetRxBoosted( RxTimeout << 6 );
	}
}

void RadioSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime )
{
	SX126xSetRxDutyCycle( rxTime, sleepTime );
}

void RadioStartCad( void )
{
	vSX126xSetDioIrqParams( IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
	SX126xSetCad();
}

void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
	uint32_t timeout = (uint32_t) time * 1000;

	vSX126xSetRfFrequency( freq );
	vSX126xSetRfTxPower( power );
	vSX126xSetTxContinuousWave();
	TimerSetValue( &TxTimeoutTimer, timeout );
	TimerStart( &TxTimeoutTimer );
}

int16_t RadioRssi( RadioModems_t modem )
{
	UNUSED( modem );
	return cSX126xGetRssiInst();
}

void RadioWrite( uint32_t addr, uint8_t data )
{
	vSX126xWriteRegister( addr, data );
}

uint8_t RadioRead( uint32_t addr )
{
	return ucSX126xReadRegister( addr );
}

void RadioWriteBuffer( uint32_t addr, uint8_t *buffer, uint8_t size )
{
	vSX126xWriteRegisters( addr, buffer, size );
}

void RadioReadBuffer( uint32_t addr, uint8_t *buffer, uint8_t size )
{
	vSX126xReadRegisters( addr, buffer, size );
}

void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
	if ( modem == MODEM_LORA ) {
		pxSx126xModule->xPacketParams.xParams.xLoRa.ucPayloadLength = MaxPayloadLength = max;
		vSX126xSetPacketParams( &pxSx126xModule->xPacketParams );
	}
	else {
		if ( pxSx126xModule->xPacketParams.xParams.xGfsk.eHeaderType == RADIO_PACKET_VARIABLE_LENGTH ) {
			pxSx126xModule->xPacketParams.xParams.xGfsk.ucPayloadLength = MaxPayloadLength = max;
			vSX126xSetPacketParams( &pxSx126xModule->xPacketParams );
		}
	}
}

void RadioSetPublicNetwork( bool enable )
{
	RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

	RadioSetModem( MODEM_LORA );
	if ( enable == true ) {
		// Change LoRa modem SyncWord
		vSX126xWriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF );
		vSX126xWriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF );
	}
	else {
		// Change LoRa modem SyncWord
		vSX126xWriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
		vSX126xWriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );
	}
}

uint32_t RadioGetWakeupTime( void )
{
	return ulSX126xGetBoardTcxoWakeupTime() + RADIO_WAKEUP_TIME;
}

void RadioOnTxTimeoutIrq( void *context )
{
	UNUSED( context );
	if ( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) ) {
		RadioEvents->TxTimeout();
	}
}

void RadioOnRxTimeoutIrq( void *context )
{
	UNUSED( context );
	if ( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) ) {
		RadioEvents->RxTimeout();
	}
}

void RadioOnDioIrq( void *context )
{
	UNUSED( context );
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( xLorawanInteruptHandle, &xHigherPriorityTaskWoken );
	IrqFired = true;
}

void vWaitforInterrupt( void )
{
	xSemaphoreTake( xLorawanInteruptHandle, portMAX_DELAY );
}

void RadioIrqProcess( void )
{
	if ( IrqFired == true ) {
		CRITICAL_SECTION_BEGIN();
		// Clear IRQ flag
		IrqFired = false;
		CRITICAL_SECTION_END();

		uint16_t irqRegs = usSX126xGetIrqStatus();
		vSX126xClearIrqStatus( irqRegs );

		if ( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE ) {
			TimerStop( &TxTimeoutTimer );
			//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
			vSX126xSetOperatingMode( MODE_STDBY_RC );
			if ( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) ) {
				RadioEvents->TxDone();
			}
		}
		if ( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE ) {
			if ( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR ) {
				if ( RxContinuous == false ) {
					//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
					vSX126xSetOperatingMode( MODE_STDBY_RC );
				}
				if ( ( RadioEvents != NULL ) && ( RadioEvents->RxError ) ) {
					RadioEvents->RxError();
				}
			}
			else {
				uint8_t size;

				TimerStop( &RxTimeoutTimer );
				if ( RxContinuous == false ) {
					//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
					vSX126xSetOperatingMode( MODE_STDBY_RC );

					// WORKAROUND - Implicit Header Mode Timeout Behavior, see DS_SX1261-2_V1.2 datasheet chapter 15.3
					// RegRtcControl = @address 0x0902
					vSX126xWriteRegister( 0x0902, 0x00 );
					// RegEventMask = @address 0x0944
					vSX126xWriteRegister( 0x0944, ucSX126xReadRegister( 0x0944 ) | ( 1 << 1 ) );
					// WORKAROUND END
				}
				ucSX126xGetPayload( RadioRxPayload, &size, 255 );
				vSX126xGetPacketStatus( &RadioPktStatus );
				if ( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) ) {
					RadioEvents->RxDone( RadioRxPayload, size, RadioPktStatus.xParams.xLoRa.cRssiPkt, RadioPktStatus.xParams.xLoRa.cSnrPkt );
				}
			}
		}

		if ( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE ) {
			//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
			vSX126xSetOperatingMode( MODE_STDBY_RC );
			if ( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) ) {
				RadioEvents->CadDone( ( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED ) );
			}
		}

		if ( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT ) {
			if ( eSX126xGetOperatingMode() == MODE_TX ) {
				TimerStop( &TxTimeoutTimer );
				//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
				vSX126xSetOperatingMode( MODE_STDBY_RC );
				if ( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) ) {
					RadioEvents->TxTimeout();
				}
			}
			else if ( eSX126xGetOperatingMode() == MODE_RX ) {
				TimerStop( &RxTimeoutTimer );
				//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
				vSX126xSetOperatingMode( MODE_STDBY_RC );
				if ( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) ) {
					RadioEvents->RxTimeout();
				}
			}
		}

		if ( ( irqRegs & IRQ_PREAMBLE_DETECTED ) == IRQ_PREAMBLE_DETECTED ) {
			//__NOP( );
		}

		if ( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID ) {
			//__NOP( );
		}

		if ( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID ) {
			//__NOP( );
		}

		if ( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR ) {
			TimerStop( &RxTimeoutTimer );
			if ( RxContinuous == false ) {
				//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
				vSX126xSetOperatingMode( MODE_STDBY_RC );
			}
			if ( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) ) {
				RadioEvents->RxTimeout();
			}
		}
	}
}
