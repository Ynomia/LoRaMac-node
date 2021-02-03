/*!
 * \file      sx126x.c
 *
 * \brief     SX126x driver implementation
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
#include "sx126x.h"
#include "csiro_math.h"
// #include "delay.h"
#include "log.h"
#include "radio.h"
#include "sx126x-board.h"
#include "timer.h"
#include "utilities.h"
#include <string.h>

/*!
 * \brief Internal frequency of the radio
 */
#define SX126X_XTAL_FREQ 32000000UL

/*!
 * \brief Scaling factor used to perform fixed-point operations
 */
#define SX126X_PLL_STEP_SHIFT_AMOUNT ( 14 )

/*!
 * \brief PLL step - scaled with SX126X_PLL_STEP_SHIFT_AMOUNT
 */
#define SX126X_PLL_STEP_SCALED ( SX126X_XTAL_FREQ >> ( 25 - SX126X_PLL_STEP_SHIFT_AMOUNT ) )

/*!
 * \brief Maximum value for parameter symbNum in \ref SX126xSetLoRaSymbNumTimeout
 */
#define SX126X_MAX_LORA_SYMB_NUM_TIMEOUT 248

/*!
 * \brief Radio registers definition
 */
typedef struct
{
	uint16_t Addr;	//!< The address of the register
	uint8_t	 Value; //!< The value of the register
} RadioRegisters_t;

/*!
 * \brief Stores the current packet type set in the radio
 */
static RadioPacketTypes_t PacketType;

/*!
 * \brief Stores the current packet header type set in the radio
 */
static volatile RadioLoRaPacketLengthsMode_t LoRaHeaderType;

/*!
 * \brief Stores the last frequency error measured on LoRa received packet
 */
volatile uint32_t FrequencyError = 0;

/*!
 * \brief Hold the status of the Image calibration
 */
static bool ImageCalibrated = false;

/*!
 * \brief Get the number of PLL steps for a given frequency in Hertz
 *
 * \param [in] freqInHz Frequency in Hertz
 *
 * \returns Number of PLL steps
 */
static uint32_t SX126xConvertFreqInHzToPllStep( uint32_t freqInHz );

/*
 * SX126x DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void SX126xOnDioIrq( void );

/*!
 * \brief DIO 0 IRQ callback
 */
void SX126xSetPollingMode( void );

/*!
 * \brief DIO 0 IRQ callback
 */
void SX126xSetInterruptMode( void );

/*
 * \brief Process the IRQ if handled by the driver
 */
void SX126xProcessIrqs( void );

void SX126xInit( DioIrqHandler dioIrq )
{
	SX126xReset();

	SX126xIoIrqInit( dioIrq );

	SX126xWakeup();
	SX126xSetStandby( STDBY_RC );

	// Initialize TCXO control
	SX126xIoTcxoInit();

	// Initialize RF switch control
	// SX126xIoRfSwitchInit();

	SX126xSetOperatingMode( MODE_STDBY_RC );
}

void SX126xCheckDeviceReady( void )
{
	if ( ( SX126xGetOperatingMode() == MODE_SLEEP ) || ( SX126xGetOperatingMode() == MODE_RX_DC ) ) {
		SX126xWakeup();
		// Switch is turned off when device is in sleep mode and turned on is all other modes
		SX126xAntSwOn();
	}
	SX126xWaitOnBusy();
}

void SX126xSetPayload( uint8_t *payload, uint8_t size )
{
	SX126xWriteBuffer( 0x00, payload, size );
}

uint8_t SX126xGetPayload( uint8_t *buffer, uint8_t *size, uint8_t maxSize )
{
	uint8_t offset = 0;

	SX126xGetRxBufferStatus( size, &offset );
	if ( *size > maxSize ) {
		return 1;
	}
	SX126xReadBuffer( offset, buffer, *size );
	return 0;
}

void SX126xSendPayload( uint8_t *payload, uint8_t size, uint32_t timeout )
{
	SX126xSetPayload( payload, size );
	SX126xSetTx( timeout );
}

uint8_t SX126xSetSyncWord( uint8_t *syncWord )
{
	vSX126xWriteRegisters( REG_LR_SYNCWORDBASEADDRESS, syncWord, 8 );
	return 0;
}

void SX126xSetCrcSeed( uint16_t seed )
{
	uint8_t buf[2];

	buf[0] = ( uint8_t )( ( seed >> 8 ) & 0xFF );
	buf[1] = ( uint8_t )( seed & 0xFF );

	switch ( eSX126xGetPacketType() ) {
		case PACKET_TYPE_GFSK:
			vSX126xWriteRegisters( REG_LR_CRCSEEDBASEADDR, buf, 2 );
			break;

		default:
			break;
	}
}

void SX126xSetCrcPolynomial( uint16_t polynomial )
{
	uint8_t buf[2];

	buf[0] = ( uint8_t )( ( polynomial >> 8 ) & 0xFF );
	buf[1] = ( uint8_t )( polynomial & 0xFF );

	switch ( eSX126xGetPacketType() ) {
		case PACKET_TYPE_GFSK:
			vSX126xWriteRegisters( REG_LR_CRCPOLYBASEADDR, buf, 2 );
			break;

		default:
			break;
	}
}

void SX126xSetWhiteningSeed( uint16_t seed )
{
	uint8_t regValue = 0;

	switch ( eSX126xGetPacketType() ) {
		case PACKET_TYPE_GFSK:
			regValue = ucSX126xReadRegister( REG_LR_WHITSEEDBASEADDR_MSB ) & 0xFE;
			regValue = ( ( seed >> 8 ) & 0x01 ) | regValue;
			vSX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_MSB, regValue ); // only 1 bit.
			vSX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_LSB, (uint8_t) seed );
			break;

		default:
			break;
	}
}

uint32_t SX126xGetRandom( void )
{
	uint32_t number		 = 0;
	uint8_t	 regAnaLna	 = 0;
	uint8_t	 regAnaMixer = 0;

	regAnaLna = ucSX126xReadRegister( REG_ANA_LNA );
	vSX126xWriteRegister( REG_ANA_LNA, regAnaLna & ~( 1 << 0 ) );

	regAnaMixer = ucSX126xReadRegister( REG_ANA_MIXER );
	vSX126xWriteRegister( REG_ANA_MIXER, regAnaMixer & ~( 1 << 7 ) );

	// Set radio in continuous reception
	vSX126xSetRx( 0xFFFFFF ); // Rx Continuous

	vSX126xReadRegisters( RANDOM_NUMBER_GENERATORBASEADDR, (uint8_t *) &number, 4 );

	SX126xSetStandby( STDBY_RC );

	vSX126xWriteRegister( REG_ANA_LNA, regAnaLna );
	vSX126xWriteRegister( REG_ANA_MIXER, regAnaMixer );

	return number;
}

void SX126xSetSleep( SleepParams_t sleepConfig )
{
	vSX126xAntSwOff();

	uint8_t value = ( ( (uint8_t) sleepConfig.Fields.WarmStart << 2 ) |
					  ( (uint8_t) sleepConfig.Fields.Reset << 1 ) |
					  ( (uint8_t) sleepConfig.Fields.WakeUpRTC ) );
	vSX126xWriteCommand( RADIO_SET_SLEEP, &value, 1 );
	vSX126xSetOperatingMode( MODE_SLEEP );
}

void SX126xSetStandby( RadioStandbyModes_t standbyConfig )
{
	vSX126xWriteCommand( RADIO_SET_STANDBY, (uint8_t *) &standbyConfig, 1 );
	if ( standbyConfig == STDBY_RC ) {
		vSX126xSetOperatingMode( MODE_STDBY_RC );
	}
	else {
		vSX126xSetOperatingMode( MODE_STDBY_XOSC );
	}
}

void SX126xSetFs( void )
{
	vSX126xWriteCommand( RADIO_SET_FS, 0, 0 );
	vSX126xSetOperatingMode( MODE_FS );
}

void SX126xSetTx( uint32_t timeout )
{
	uint8_t buf[3];

	SX126xSetOperatingMode( MODE_TX );

	buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
	buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
	buf[2] = ( uint8_t )( timeout & 0xFF );
	SX126xWriteCommand( RADIO_SET_TX, buf, 3 );
}

void SX126xSetRx( uint32_t timeout )
{
	uint8_t buf[3];

	SX126xSetOperatingMode( MODE_RX );

	buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
	buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
	buf[2] = ( uint8_t )( timeout & 0xFF );
	SX126xWriteCommand( RADIO_SET_RX, buf, 3 );
}

void SX126xSetRxBoosted( uint32_t timeout )
{
	uint8_t buf[3];

	SX126xSetOperatingMode( MODE_RX );

	SX126xWriteRegister( REG_RX_GAIN, 0x96 ); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity

	buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
	buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
	buf[2] = ( uint8_t )( timeout & 0xFF );
	SX126xWriteCommand( RADIO_SET_RX, buf, 3 );
}

void SX126xSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime )
{
	uint8_t buf[6];

	buf[0] = ( uint8_t )( ( rxTime >> 16 ) & 0xFF );
	buf[1] = ( uint8_t )( ( rxTime >> 8 ) & 0xFF );
	buf[2] = ( uint8_t )( rxTime & 0xFF );
	buf[3] = ( uint8_t )( ( sleepTime >> 16 ) & 0xFF );
	buf[4] = ( uint8_t )( ( sleepTime >> 8 ) & 0xFF );
	buf[5] = ( uint8_t )( sleepTime & 0xFF );
	vSX126xWriteCommand( RADIO_SET_RXDUTYCYCLE, buf, 6 );
	vSX126xSetOperatingMode( MODE_RX_DC );
}

void SX126xSetCad( void )
{
	vSX126xWriteCommand( RADIO_SET_CAD, 0, 0 );
	vSX126xSetOperatingMode( MODE_CAD );
}

void SX126xSetTxContinuousWave( void )
{
	SX126xWriteCommand( RADIO_SET_TXCONTINUOUSWAVE, 0, 0 );
	SX126xSetOperatingMode( MODE_TX );
}

void SX126xSetTxInfinitePreamble( void )
{
	SX126xWriteCommand( RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
	SX126xSetOperatingMode( MODE_TX );
}

void SX126xSetStopRxTimerOnPreambleDetect( bool enable )
{
	SX126xWriteCommand( RADIO_SET_STOPRXTIMERONPREAMBLE, (uint8_t *) &enable, 1 );
}

void SX126xSetLoRaSymbNumTimeout( uint8_t symbNum )
{
	uint8_t mant = ( ( ( symbNum > SX126X_MAX_LORA_SYMB_NUM_TIMEOUT ) ? SX126X_MAX_LORA_SYMB_NUM_TIMEOUT : symbNum ) + 1 ) >> 1;
	uint8_t exp	 = 0;
	uint8_t reg	 = 0;

	while ( mant > 31 ) {
		mant = ( mant + 3 ) >> 2;
		exp++;
	}

	reg = mant << ( 2 * exp + 1 );
	vSX126xWriteCommand( RADIO_SET_LORASYMBTIMEOUT, &reg, 1 );

	if ( symbNum != 0 ) {
		reg = exp + ( mant << 3 );
		vSX126xWriteRegister( REG_LR_SYNCH_TIMEOUT, reg );
	}
}

void SX126xSetRegulatorMode( RadioRegulatorMode_t mode )
{
	SX126xWriteCommand( RADIO_SET_REGULATORMODE, (uint8_t *) &mode, 1 );
}

void SX126xCalibrate( CalibrationParams_t calibParam )
{
	uint8_t value = ( ( (uint8_t) calibParam.Fields.ImgEnable << 6 ) |
					  ( (uint8_t) calibParam.Fields.ADCBulkPEnable << 5 ) |
					  ( (uint8_t) calibParam.Fields.ADCBulkNEnable << 4 ) |
					  ( (uint8_t) calibParam.Fields.ADCPulseEnable << 3 ) |
					  ( (uint8_t) calibParam.Fields.PLLEnable << 2 ) |
					  ( (uint8_t) calibParam.Fields.RC13MEnable << 1 ) |
					  ( (uint8_t) calibParam.Fields.RC64KEnable ) );

	SX126xWriteCommand( RADIO_CALIBRATE, &value, 1 );
}

void SX126xCalibrateImage( uint32_t freq )
{
	uint8_t calFreq[2];

	if ( freq > 900000000 ) {
		calFreq[0] = 0xE1;
		calFreq[1] = 0xE9;
	}
	else if ( freq > 850000000 ) {
		calFreq[0] = 0xD7;
		calFreq[1] = 0xDB;
	}
	else if ( freq > 770000000 ) {
		calFreq[0] = 0xC1;
		calFreq[1] = 0xC5;
	}
	else if ( freq > 460000000 ) {
		calFreq[0] = 0x75;
		calFreq[1] = 0x81;
	}
	else if ( freq > 425000000 ) {
		calFreq[0] = 0x6B;
		calFreq[1] = 0x6F;
	}
	SX126xWriteCommand( RADIO_CALIBRATEIMAGE, calFreq, 2 );
}

void SX126xSetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut )
{
	uint8_t buf[4];

	buf[0] = paDutyCycle;
	buf[1] = hpMax;
	buf[2] = deviceSel;
	buf[3] = paLut;
	SX126xWriteCommand( RADIO_SET_PACONFIG, buf, 4 );
}

void SX126xSetRxTxFallbackMode( uint8_t fallbackMode )
{
	SX126xWriteCommand( RADIO_SET_TXFALLBACKMODE, &fallbackMode, 1 );
}

void SX126xSetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
	uint8_t buf[8];

	buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
	buf[1] = ( uint8_t )( irqMask & 0x00FF );
	buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
	buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
	buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
	buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
	buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
	buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
	SX126xWriteCommand( RADIO_CFG_DIOIRQ, buf, 8 );
}

uint16_t SX126xGetIrqStatus( void )
{
	uint8_t irqStatus[2];

	SX126xReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
	return ( irqStatus[0] << 8 ) | irqStatus[1];
}

void SX126xSetDio2AsRfSwitchCtrl( uint8_t enable )
{
	SX126xWriteCommand( RADIO_SET_RFSWITCHMODE, &enable, 1 );
}

void SX126xSetDio3AsTcxoCtrl( RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout )
{
	uint8_t buf[4];

	buf[0] = tcxoVoltage & 0x07;
	buf[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
	buf[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
	buf[3] = ( uint8_t )( timeout & 0xFF );

	SX126xWriteCommand( RADIO_SET_TCXOMODE, buf, 4 );
}

void SX126xSetRfFrequency( uint32_t frequency )
{
	uint8_t buf[4];

	if ( ImageCalibrated == false ) {
		SX126xCalibrateImage( frequency );
		ImageCalibrated = true;
	}

	uint32_t freqInPllSteps = SX126xConvertFreqInHzToPllStep( frequency );

	buf[0] = ( uint8_t )( ( freqInPllSteps >> 24 ) & 0xFF );
	buf[1] = ( uint8_t )( ( freqInPllSteps >> 16 ) & 0xFF );
	buf[2] = ( uint8_t )( ( freqInPllSteps >> 8 ) & 0xFF );
	buf[3] = ( uint8_t )( freqInPllSteps & 0xFF );
	SX126xWriteCommand( RADIO_SET_RFFREQUENCY, buf, 4 );
}

void SX126xSetPacketType( RadioPacketTypes_t packetType )
{
	// Save packet type internally to avoid questioning the radio
	PacketType = packetType;
	vSX126xWriteCommand( RADIO_SET_PACKETTYPE, (uint8_t *) &packetType, 1 );
}

RadioPacketTypes_t SX126xGetPacketType( void )
{
	return PacketType;
}

void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
	uint8_t buf[2];

	if ( SX126xGetDeviceId() == SX1261 ) {
		if ( power == 15 ) {
			SX126xSetPaConfig( 0x06, 0x00, 0x01, 0x01 );
		}
		else {
			SX126xSetPaConfig( 0x04, 0x00, 0x01, 0x01 );
		}
		if ( power >= 14 ) {
			power = 14;
		}
		else if ( power < -17 ) {
			power = -17;
		}
	}
	else // sx1262
	{
		// WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
		// RegTxClampConfig = @address 0x08D8
		SX126xWriteRegister( 0x08D8, SX126xReadRegister( 0x08D8 ) | ( 0x0F << 1 ) );
		// WORKAROUND END

		SX126xSetPaConfig( 0x04, 0x07, 0x00, 0x01 );
		if ( power > 22 ) {
			power = 22;
		}
		else if ( power < -9 ) {
			power = -9;
		}
	}
	buf[0] = power;
	buf[1] = (uint8_t) rampTime;
	SX126xWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

void SX126xSetModulationParams( ModulationParams_t *modulationParams )
{
	uint8_t	 n;
	uint32_t tempVal = 0;
	uint8_t	 buf[8]	 = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	// Check if required configuration corresponds to the stored packet type
	// If not, silently update radio packet type
	if ( PacketType != modulationParams->PacketType ) {
		SX126xSetPacketType( modulationParams->PacketType );
	}

	switch ( modulationParams->PacketType ) {
		case PACKET_TYPE_GFSK:
			n		= 8;
			tempVal = ( uint32_t )( 32 * SX126X_XTAL_FREQ / modulationParams->Params.Gfsk.BitRate );
			buf[0]	= ( tempVal >> 16 ) & 0xFF;
			buf[1]	= ( tempVal >> 8 ) & 0xFF;
			buf[2]	= tempVal & 0xFF;
			buf[3]	= modulationParams->Params.Gfsk.ModulationShaping;
			buf[4]	= modulationParams->Params.Gfsk.Bandwidth;
			tempVal = SX126xConvertFreqInHzToPllStep( modulationParams->Params.Gfsk.Fdev );
			buf[5]	= ( tempVal >> 16 ) & 0xFF;
			buf[6]	= ( tempVal >> 8 ) & 0xFF;
			buf[7]	= ( tempVal & 0xFF );
			SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );
			break;
		case PACKET_TYPE_LORA:
			n	   = 4;
			buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
			buf[1] = modulationParams->Params.LoRa.Bandwidth;
			buf[2] = modulationParams->Params.LoRa.CodingRate;
			buf[3] = modulationParams->Params.LoRa.LowDatarateOptimize;

			SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );

			break;
		default:
		case PACKET_TYPE_NONE:
			return;
	}
}

void SX126xSetPacketParams( PacketParams_t *packetParams )
{
	uint8_t n;
	uint8_t crcVal = 0;
	uint8_t buf[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	// Check if required configuration corresponds to the stored packet type
	// If not, silently update radio packet type
	if ( PacketType != packetParams->PacketType ) {
		SX126xSetPacketType( packetParams->PacketType );
	}

	switch ( packetParams->PacketType ) {
		case PACKET_TYPE_GFSK:
			if ( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_IBM ) {
				SX126xSetCrcSeed( CRC_IBM_SEED );
				SX126xSetCrcPolynomial( CRC_POLYNOMIAL_IBM );
				crcVal = RADIO_CRC_2_BYTES;
			}
			else if ( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_CCIT ) {
				SX126xSetCrcSeed( CRC_CCITT_SEED );
				SX126xSetCrcPolynomial( CRC_POLYNOMIAL_CCITT );
				crcVal = RADIO_CRC_2_BYTES_INV;
			}
			else {
				crcVal = packetParams->Params.Gfsk.CrcLength;
			}
			n	   = 9;
			buf[0] = ( packetParams->Params.Gfsk.PreambleLength >> 8 ) & 0xFF;
			buf[1] = packetParams->Params.Gfsk.PreambleLength;
			buf[2] = packetParams->Params.Gfsk.PreambleMinDetect;
			buf[3] = ( packetParams->Params.Gfsk.SyncWordLength /*<< 3*/ ); // convert from byte to bit
			buf[4] = packetParams->Params.Gfsk.AddrComp;
			buf[5] = packetParams->Params.Gfsk.HeaderType;
			buf[6] = packetParams->Params.Gfsk.PayloadLength;
			buf[7] = crcVal;
			buf[8] = packetParams->Params.Gfsk.DcFree;
			break;
		case PACKET_TYPE_LORA:
			n	   = 6;
			buf[0] = ( packetParams->Params.LoRa.PreambleLength >> 8 ) & 0xFF;
			buf[1] = packetParams->Params.LoRa.PreambleLength;
			buf[2] = LoRaHeaderType = packetParams->Params.LoRa.HeaderType;
			buf[3]					= packetParams->Params.LoRa.PayloadLength;
			buf[4]					= packetParams->Params.LoRa.CrcMode;
			buf[5]					= packetParams->Params.LoRa.InvertIQ;
			break;
		default:
		case PACKET_TYPE_NONE:
			return;
	}
	vSX126xWriteCommand( RADIO_SET_PACKETPARAMS, buf, n );
}

void SX126xSetCadParams( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode, uint32_t cadTimeout )
{
	uint8_t buf[7];

	buf[0] = (uint8_t) cadSymbolNum;
	buf[1] = cadDetPeak;
	buf[2] = cadDetMin;
	buf[3] = (uint8_t) cadExitMode;
	buf[4] = ( uint8_t )( ( cadTimeout >> 16 ) & 0xFF );
	buf[5] = ( uint8_t )( ( cadTimeout >> 8 ) & 0xFF );
	buf[6] = ( uint8_t )( cadTimeout & 0xFF );
	SX126xWriteCommand( RADIO_SET_CADPARAMS, buf, 7 );
	SX126xSetOperatingMode( MODE_CAD );
}

void SX126xSetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
	uint8_t buf[2];

	buf[0] = txBaseAddress;
	buf[1] = rxBaseAddress;
	SX126xWriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}

RadioStatus_t SX126xGetStatus( void )
{
	uint8_t		  stat	 = 0;
	RadioStatus_t status = { .Value = 0 };

	stat					= SX126xReadCommand( RADIO_GET_STATUS, NULL, 0 );
	status.Fields.CmdStatus = ( stat & ( 0x07 << 1 ) ) >> 1;
	status.Fields.ChipMode	= ( stat & ( 0x07 << 4 ) ) >> 4;
	return status;
}

int8_t SX126xGetRssiInst( void )
{
	uint8_t buf[1];
	int8_t	rssi = 0;

	SX126xReadCommand( RADIO_GET_RSSIINST, buf, 1 );
	rssi = -buf[0] >> 1;
	return rssi;
}

void SX126xGetRxBufferStatus( uint8_t *payloadLength, uint8_t *rxStartBufferPointer )
{
	uint8_t status[2];

	SX126xReadCommand( RADIO_GET_RXBUFFERSTATUS, status, 2 );

	// In case of LORA fixed header, the payloadLength is obtained by reading
	// the register REG_LR_PAYLOADLENGTH
	if ( ( SX126xGetPacketType() == PACKET_TYPE_LORA ) && ( LoRaHeaderType == LORA_PACKET_FIXED_LENGTH ) ) {
		*payloadLength = SX126xReadRegister( REG_LR_PAYLOADLENGTH );
	}
	else {
		*payloadLength = status[0];
	}
	*rxStartBufferPointer = status[1];
}

void SX126xGetPacketStatus( PacketStatus_t *pktStatus )
{
	uint8_t status[3];

	SX126xReadCommand( RADIO_GET_PACKETSTATUS, status, 3 );

	pktStatus->packetType = SX126xGetPacketType();
	switch ( pktStatus->packetType ) {
		case PACKET_TYPE_GFSK:
			pktStatus->Params.Gfsk.RxStatus	 = status[0];
			pktStatus->Params.Gfsk.RssiSync	 = -status[1] >> 1;
			pktStatus->Params.Gfsk.RssiAvg	 = -status[2] >> 1;
			pktStatus->Params.Gfsk.FreqError = 0;
			break;

		case PACKET_TYPE_LORA:
			pktStatus->Params.LoRa.RssiPkt = -status[0] >> 1;
			// Returns SNR value [dB] rounded to the nearest integer value
			pktStatus->Params.LoRa.SnrPkt		 = ( ( (int8_t) status[1] ) + 2 ) >> 2;
			pktStatus->Params.LoRa.SignalRssiPkt = -status[2] >> 1;
			pktStatus->Params.LoRa.FreqError	 = FrequencyError;
			break;

		default:
		case PACKET_TYPE_NONE:
			// In that specific case, we set everything in the pktStatus to zeros
			// and reset the packet type accordingly
			memset( pktStatus, 0, sizeof( PacketStatus_t ) );
			pktStatus->packetType = PACKET_TYPE_NONE;
			break;
	}
}

RadioError_t SX126xGetDeviceErrors( void )
{
	uint8_t		 err[] = { 0, 0 };
	RadioError_t error = { .Value = 0 };

	SX126xReadCommand( RADIO_GET_ERROR, (uint8_t *) err, 2 );
	error.Fields.PaRamp		= ( err[0] & ( 1 << 0 ) ) >> 0;
	error.Fields.PllLock	= ( err[1] & ( 1 << 6 ) ) >> 6;
	error.Fields.XoscStart	= ( err[1] & ( 1 << 5 ) ) >> 5;
	error.Fields.ImgCalib	= ( err[1] & ( 1 << 4 ) ) >> 4;
	error.Fields.AdcCalib	= ( err[1] & ( 1 << 3 ) ) >> 3;
	error.Fields.PllCalib	= ( err[1] & ( 1 << 2 ) ) >> 2;
	error.Fields.Rc13mCalib = ( err[1] & ( 1 << 1 ) ) >> 1;
	error.Fields.Rc64kCalib = ( err[1] & ( 1 << 0 ) ) >> 0;
	return error;
}

void SX126xClearDeviceErrors( void )
{
	uint8_t buf[2] = { 0x00, 0x00 };
	SX126xWriteCommand( RADIO_CLR_ERROR, buf, 2 );
}

void SX126xClearIrqStatus( uint16_t irq )
{
	uint8_t buf[2];

	buf[0] = ( uint8_t )( ( (uint16_t) irq >> 8 ) & 0x00FF );
	buf[1] = ( uint8_t )( (uint16_t) irq & 0x00FF );
	SX126xWriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
}

static uint32_t SX126xConvertFreqInHzToPllStep( uint32_t freqInHz )
{
	uint32_t stepsInt;
	uint32_t stepsFrac;

	// pllSteps = freqInHz / (SX126X_XTAL_FREQ / 2^19 )
	// Get integer and fractional parts of the frequency computed with a PLL step scaled value
	stepsInt  = freqInHz / SX126X_PLL_STEP_SCALED;
	stepsFrac = freqInHz - ( stepsInt * SX126X_PLL_STEP_SCALED );

	// Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
	return ( stepsInt << SX126X_PLL_STEP_SHIFT_AMOUNT ) +
		   ( ( ( stepsFrac << SX126X_PLL_STEP_SHIFT_AMOUNT ) + ( SX126X_PLL_STEP_SCALED >> 1 ) ) /
			 SX126X_PLL_STEP_SCALED );
}

/*!
 * \brief Radio registers definition
 */
typedef struct
{
	uint16_t usAddr;  //!< The address of the register
	uint8_t  ucValue; //!< The value of the register
} xRadioRegisters_t;

/*!
 * \brief Holds the internal operating mode of the radio
 */
static RadioOperatingModes_t eOperatingMode;

/*!
 * \brief Stores the current packet type set in the radio
 */
static RadioPacketTypes_t ePacketType;

/*!
 * \brief Stores the last frequency error measured on LoRa received packet
 */
volatile uint32_t ulFrequencyError = 0;

/*!
 * \brief Hold the status of the Image calibration
 */
static bool bImageCalibrated = false;

/*
 * SX126x DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void vSX126xOnDioIrq( void );

/*!
 * \brief DIO 0 IRQ callback
 */
void vSX126xSetPollingMode( void );

/*!
 * \brief DIO 0 IRQ callback
 */
void vSX126xSetInterruptMode( void );

/*
 * \brief Process the IRQ if handled by the driver
 */
void vSX126xProcessIrqs( void );

/*
* \If you need the TCXO add a setup to it here. It is not currently being used by our platform.
*/
void vSX126xInit( DioIrqHandler fnDioIrq )
{
	vSX126xIoIrqInit( fnDioIrq );
}

RadioOperatingModes_t eSX126xGetOperatingMode( void )
{
	return eOperatingMode;
}

void vSX126xSetOperatingMode( RadioOperatingModes_t eMode )
{
	eOperatingMode = eMode;
}

void vSX126xCheckDeviceReady( void )
{
	if ( ( eSX126xGetOperatingMode() == MODE_SLEEP ) || ( eSX126xGetOperatingMode() == MODE_RX_DC ) ) {
		vSX126xWakeup();
		// Switch is turned off when device is in sleep mode and turned on is all other modes
		// this is commented out as if the wakeup is not respected and you switch io2 it crashes the device
	}
	vSX126xWaitOnBusy();
}

void vSX126xSetPayload( uint8_t *pucPayload, uint8_t ucSize )
{
	vSX126xWriteBuffer( 0x00, pucPayload, ucSize );
}

uint8_t ucSX126xGetPayload( uint8_t *pucPuffer, uint8_t *pucSize, uint8_t uxMaxSize )
{
	uint8_t ucOffset = 0;
	vSX126xGetRxBufferStatus( pucSize, &ucOffset );
	if ( *pucSize > uxMaxSize ) {
		return 1;
	}
	vSX126xReadBuffer( ucOffset, pucPuffer, *pucSize );
	return 0;
}

void vSX126xSendPayload( uint8_t *pucPayload, uint8_t ucSize, uint32_t ulTimeout )
{
	vSX126xSetPayload( pucPayload, ucSize );

	vSX126xSetTx( ulTimeout );
}

void vSX126xSetTx( uint32_t ulTimeout )
{
	uint8_t ucBuf[3];

	vSX126xSetOperatingMode( MODE_TX );

	ucBuf[0] = ( uint8_t )( ( ulTimeout >> 16 ) & 0xFF );
	ucBuf[1] = ( uint8_t )( ( ulTimeout >> 8 ) & 0xFF );
	ucBuf[2] = ( uint8_t )( ulTimeout & 0xFF );
	vSX126xWriteCommand( RADIO_SET_TX, ucBuf, 3 );
}

void vSX126xSetRx( uint32_t ulTimeout )
{
	uint8_t ucBuf[3];
	vSX126xSetOperatingMode( MODE_RX );

	ucBuf[0] = ( uint8_t )( ( ulTimeout >> 16 ) & 0xFF );
	ucBuf[1] = ( uint8_t )( ( ulTimeout >> 8 ) & 0xFF );
	ucBuf[2] = ( uint8_t )( ulTimeout & 0xFF );
	vSX126xWriteCommand( RADIO_SET_RX, ucBuf, 3 );
}

void vSX126xSetRxBoosted( uint32_t ulTimeout )
{
	uint8_t ucBuf[3];

	vSX126xSetOperatingMode( MODE_RX );

	vSX126xWriteRegister( REG_RX_GAIN, 0x96 ); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity

	ucBuf[0] = ( uint8_t )( ( ulTimeout >> 16 ) & 0xFF );
	ucBuf[1] = ( uint8_t )( ( ulTimeout >> 8 ) & 0xFF );
	ucBuf[2] = ( uint8_t )( ulTimeout & 0xFF );
	vSX126xWriteCommand( RADIO_SET_RX, ucBuf, 3 );
}

void vSX126xSetTxContinuousWave( void )
{
	vSX126xWriteCommand( RADIO_SET_TXCONTINUOUSWAVE, 0, 0 );
}

void vSX126xSetTxInfinitePreamble( void )
{
	vSX126xWriteCommand( RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
}

void vSX126xSetStopRxTimerOnPreambleDetect( bool bEnable )
{
	vSX126xWriteCommand( RADIO_SET_STOPRXTIMERONPREAMBLE, (uint8_t *) &bEnable, 1 );
}

void vSX126xSetLoRaSymbNumTimeout( uint8_t ucSymbNum )
{
	vSX126xWriteCommand( RADIO_SET_LORASYMBTIMEOUT, &ucSymbNum, 1 );
}

void vSX126xSetRegulatorMode( RadioRegulatorMode_t eMode )
{
	vSX126xWriteCommand( RADIO_SET_REGULATORMODE, (uint8_t *) &eMode, 1 );
}

void eSX126xCalibrate( xCalibrationParams_t xCalibParam )
{
	vSX126xWriteCommand( RADIO_CALIBRATE, (uint8_t *) &xCalibParam, 1 );
}

void vSX126xCalibrateImage( uint32_t ulFreq )
{
	uint8_t ucCalFreq[2];

	if ( ulFreq > 900000000 ) {
		ucCalFreq[0] = 0xE1;
		ucCalFreq[1] = 0xE9;
	}
	else if ( ulFreq > 850000000 ) {
		ucCalFreq[0] = 0xD7;
		ucCalFreq[1] = 0xDB;
	}
	else if ( ulFreq > 770000000 ) {
		ucCalFreq[0] = 0xC1;
		ucCalFreq[1] = 0xC5;
	}
	else if ( ulFreq > 460000000 ) {
		ucCalFreq[0] = 0x75;
		ucCalFreq[1] = 0x81;
	}
	else if ( ulFreq > 425000000 ) {
		ucCalFreq[0] = 0x6B;
		ucCalFreq[1] = 0x6F;
	}
	vSX126xWriteCommand( RADIO_CALIBRATEIMAGE, ucCalFreq, 2 );
}

void vSX126xSetPaConfig( uint8_t ucPaDutyCycle, uint8_t ucHpMax, uint8_t ucDeviceSel, uint8_t ucPaLut )
{
	uint8_t ucBuf[4];

	ucBuf[0] = ucPaDutyCycle;
	ucBuf[1] = ucHpMax;
	ucBuf[2] = ucDeviceSel;
	ucBuf[3] = ucPaLut;
	vSX126xWriteCommand( RADIO_SET_PACONFIG, ucBuf, 4 );
}

void vSX126xSetRxTxFallbackMode( uint8_t ucFallbackMode )
{
	vSX126xWriteCommand( RADIO_SET_TXFALLBACKMODE, &ucFallbackMode, 1 );
}

void vSX126xSetDioIrqParams( uint16_t usIrqMask, uint16_t usDio1Mask, uint16_t usDio2Mask, uint16_t usDio3Mask )
{
	uint8_t ucBuf[8];

	ucBuf[0] = ( uint8_t )( ( usIrqMask >> 8 ) & 0x00FF );
	ucBuf[1] = ( uint8_t )( usIrqMask & 0x00FF );
	ucBuf[2] = ( uint8_t )( ( usDio1Mask >> 8 ) & 0x00FF );
	ucBuf[3] = ( uint8_t )( usDio1Mask & 0x00FF );
	ucBuf[4] = ( uint8_t )( ( usDio2Mask >> 8 ) & 0x00FF );
	ucBuf[5] = ( uint8_t )( usDio2Mask & 0x00FF );
	ucBuf[6] = ( uint8_t )( ( usDio3Mask >> 8 ) & 0x00FF );
	ucBuf[7] = ( uint8_t )( usDio3Mask & 0x00FF );
	vSX126xWriteCommand( RADIO_CFG_DIOIRQ, ucBuf, 8 );
}

uint16_t usSX126xGetIrqStatus( void )
{
	uint8_t ucIrqStatus[2];

	vSX126xReadCommand( RADIO_GET_IRQSTATUS, ucIrqStatus, 2 );
	return ( ucIrqStatus[0] << 8 ) | ucIrqStatus[1];
}

void vSX126xSetDio2AsRfSwitchCtrl( uint8_t ucEnable )
{
	vSX126xWriteCommand( RADIO_SET_RFSWITCHMODE, &ucEnable, 1 );
}

void vSX126xSetDio3AsTcxoCtrl( RadioTcxoCtrlVoltage_t eTcxoVoltage, uint32_t ulTimeout )
{
	uint8_t ucBuf[4];

	ucBuf[0] = eTcxoVoltage & 0x07;
	ucBuf[1] = ( uint8_t )( ( ulTimeout >> 16 ) & 0xFF );
	ucBuf[2] = ( uint8_t )( ( ulTimeout >> 8 ) & 0xFF );
	ucBuf[3] = ( uint8_t )( ulTimeout & 0xFF );

	vSX126xWriteCommand( RADIO_SET_TCXOMODE, ucBuf, 4 );
}

void vSX126xSetRfFrequency( uint32_t ulFrequency )
{
	uint8_t  ucBuf[4];
	uint32_t ulFreq = 0;

	if ( bImageCalibrated == false ) {
		vSX126xCalibrateImage( ulFrequency );
		bImageCalibrated = true;
	}

	ulFreq   = ( uint32_t )( (double) ulFrequency / (double) FREQ_STEP );
	ucBuf[0] = ( uint8_t )( ( ulFreq >> 24 ) & 0xFF );
	ucBuf[1] = ( uint8_t )( ( ulFreq >> 16 ) & 0xFF );
	ucBuf[2] = ( uint8_t )( ( ulFreq >> 8 ) & 0xFF );
	ucBuf[3] = ( uint8_t )( ulFreq & 0xFF );
	vSX126xWriteCommand( RADIO_SET_RFFREQUENCY, ucBuf, 4 );
}

void vSX126xSetPacketType( RadioPacketTypes_t ePacketTypeToSet )
{
	// Save packet type internally to avoid questioning the radio
	ePacketType = ePacketTypeToSet;
	vSX126xWriteCommand( RADIO_SET_PACKETTYPE, (uint8_t *) &ePacketTypeToSet, 1 );
}

RadioPacketTypes_t eSX126xGetPacketType( void )
{
	return ePacketType;
}

void vSX126xSetTxParams( int8_t cPower, RadioRampTimes_t eRampTime )
{
	uint8_t ucBuf[2];

	if ( ucSX126xGetDeviceId() == SX1261 ) {
		if ( cPower == 15 ) {
			vSX126xSetPaConfig( 0x06, 0x00, 0x01, 0x01 );
		}
		else {
			vSX126xSetPaConfig( 0x04, 0x00, 0x01, 0x01 );
		}
		if ( cPower >= 14 ) {
			cPower = 14;
		}
		else if ( cPower < -17 ) {
			cPower = -17;
		}
		vSX126xWriteRegister( REG_OCP, 0x18 ); // current max is 80 mA for the whole device
	}
	else // sx1262
	{
		vSX126xSetPaConfig( 0x04, 0x07, 0x00, 0x01 );
		if ( cPower > 22 ) {
			cPower = 22;
		}
		else if ( cPower < -9 ) {
			cPower = -9;
		}
		vSX126xWriteRegister( REG_OCP, 0x38 ); // current max 160mA for the whole device
	}
	ucBuf[0] = cPower;
	ucBuf[1] = (uint8_t) eRampTime;
	vSX126xWriteCommand( RADIO_SET_TXPARAMS, ucBuf, 2 );
}

void vSX126xSetModulationParams( xModulationParams_t *pxModulationParams )
{
	uint32_t ulTempVal = 0;
	uint8_t  ucBuf[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	// Check if required configuration corresponds to the stored packet type
	// If not, silently update radio packet type
	if ( ePacketType != pxModulationParams->ePacketType ) {
		SX126xSetPacketType( pxModulationParams->ePacketType );
	}

	switch ( pxModulationParams->ePacketType ) {
		case PACKET_TYPE_GFSK:
			ulTempVal = ( uint32_t )( 32 * ( (double) XTAL_FREQ / (double) pxModulationParams->xParams.xGfsk.ulBitRate ) );
			ucBuf[0]  = ( ulTempVal >> 16 ) & 0xFF;
			ucBuf[1]  = ( ulTempVal >> 8 ) & 0xFF;
			ucBuf[2]  = ulTempVal & 0xFF;
			ucBuf[3]  = pxModulationParams->xParams.xGfsk.xModulationShaping;
			ucBuf[4]  = pxModulationParams->xParams.xGfsk.ucBandwidth;
			ulTempVal = ( uint32_t )( (double) pxModulationParams->xParams.xGfsk.ulFdev / (double) FREQ_STEP );
			ucBuf[5]  = ( ulTempVal >> 16 ) & 0xFF;
			ucBuf[6]  = ( ulTempVal >> 8 ) & 0xFF;
			ucBuf[7]  = ( ulTempVal & 0xFF );
			vSX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, ucBuf, 8 );
			break;
		case PACKET_TYPE_LORA:
			ucBuf[0] = pxModulationParams->xParams.xLoRa.eSpreadingFactor;
			ucBuf[1] = pxModulationParams->xParams.xLoRa.eBandwidth;
			ucBuf[2] = pxModulationParams->xParams.xLoRa.eCodingRate;
			ucBuf[3] = pxModulationParams->xParams.xLoRa.ucLowDatarateOptimize;
			vSX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, ucBuf, 4 );

			break;
		default:
		case PACKET_TYPE_NONE:
			return;
	}
}

void vSX126xSetPacketParams( xPacketParams_t *pxPacketParams )
{
	uint8_t ucPacketSize;
	uint8_t ucCrcVal = 0;
	uint8_t ucBuf[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	// Check if required configuration corresponds to the stored packet type
	// If not, silently update radio packet type
	if ( ePacketType != pxPacketParams->ePacketType ) {
		SX126xSetPacketType( pxPacketParams->ePacketType );
	}

	switch ( pxPacketParams->ePacketType ) {
		case PACKET_TYPE_GFSK:
			if ( pxPacketParams->xParams.xGfsk.eCrcLength == RADIO_CRC_2_BYTES_IBM ) {
				SX126xSetCrcSeed( CRC_IBM_SEED );
				SX126xSetCrcPolynomial( CRC_POLYNOMIAL_IBM );
				ucCrcVal = RADIO_CRC_2_BYTES;
			}
			else if ( pxPacketParams->xParams.xGfsk.eCrcLength == RADIO_CRC_2_BYTES_CCIT ) {
				SX126xSetCrcSeed( CRC_CCITT_SEED );
				SX126xSetCrcPolynomial( CRC_POLYNOMIAL_CCITT );
				ucCrcVal = RADIO_CRC_2_BYTES_INV;
			}
			else {
				ucCrcVal = pxPacketParams->xParams.xGfsk.eCrcLength;
			}
			ucPacketSize = 9;
			ucBuf[0]	 = ( pxPacketParams->xParams.xGfsk.usPreambleLength >> 8 ) & 0xFF;
			ucBuf[1]	 = pxPacketParams->xParams.xGfsk.usPreambleLength;
			ucBuf[2]	 = pxPacketParams->xParams.xGfsk.ePreambleMinDetect;
			ucBuf[3]	 = ( pxPacketParams->xParams.xGfsk.ucSyncWordLength /*<< 3*/ ); // convert from byte to bit
			ucBuf[4]	 = pxPacketParams->xParams.xGfsk.eAddrComp;
			ucBuf[5]	 = pxPacketParams->xParams.xGfsk.eHeaderType;
			ucBuf[6]	 = pxPacketParams->xParams.xGfsk.ucPayloadLength;
			ucBuf[7]	 = ucCrcVal;
			ucBuf[8]	 = pxPacketParams->xParams.xGfsk.eDcFree;
			break;
		case PACKET_TYPE_LORA:
			ucPacketSize = 6;
			ucBuf[0]	 = ( pxPacketParams->xParams.xLoRa.usPreambleLength >> 8 ) & 0xFF;
			ucBuf[1]	 = pxPacketParams->xParams.xLoRa.usPreambleLength;
			ucBuf[2]	 = pxPacketParams->xParams.xLoRa.eHeaderType;
			ucBuf[3]	 = pxPacketParams->xParams.xLoRa.ucPayloadLength;
			ucBuf[4]	 = pxPacketParams->xParams.xLoRa.eCrcMode;
			ucBuf[5]	 = pxPacketParams->xParams.xLoRa.eInvertIQ;
			break;
		default:
		case PACKET_TYPE_NONE:
			return;
	}
	vSX126xWriteCommand( RADIO_SET_PACKETPARAMS, ucBuf, ucPacketSize );
}

void vSX126xSetCadParams( RadioLoRaCadSymbols_t eCadSymbolNum, uint8_t ucCadDetPeak, uint8_t ucCadDetMin, RadioCadExitModes_t eCadExitMode, uint32_t ulCadTimeout )
{
	uint8_t ucBuf[7];

	ucBuf[0] = (uint8_t) eCadSymbolNum;
	ucBuf[1] = ucCadDetPeak;
	ucBuf[2] = ucCadDetMin;
	ucBuf[3] = (uint8_t) eCadExitMode;
	ucBuf[4] = ( uint8_t )( ( ulCadTimeout >> 16 ) & 0xFF );
	ucBuf[5] = ( uint8_t )( ( ulCadTimeout >> 8 ) & 0xFF );
	ucBuf[6] = ( uint8_t )( ulCadTimeout & 0xFF );
	vSX126xWriteCommand( RADIO_SET_CADPARAMS, ucBuf, 7 );
	vSX126xSetOperatingMode( MODE_CAD );
}

void vSX126xSetBufferBaseAddress( uint8_t ucTxBaseAddress, uint8_t ucRxBaseAddress )
{
	uint8_t ucBuf[2];

	ucBuf[0] = ucTxBaseAddress;
	ucBuf[1] = ucRxBaseAddress;
	vSX126xWriteCommand( RADIO_SET_BUFFERBASEADDRESS, ucBuf, 2 );
}

xRadioStatus_t eSX126xGetStatus( void )
{
	uint8_t		   ucStat = 0;
	xRadioStatus_t eStatus;
	vSX126xReadCommand( RADIO_GET_STATUS, (uint8_t *) &ucStat, 1 );
	eStatus.ucValue = ucStat;
	return eStatus;
}

int8_t cSX126xGetRssiInst( void )
{
	uint8_t ucBuf[1];
	int8_t  cRssi = 0;

	vSX126xReadCommand( RADIO_GET_RSSIINST, ucBuf, 1 );
	cRssi = -ucBuf[0] >> 1;
	return cRssi;
}

void vSX126xGetRxBufferStatus( uint8_t *pucPayloadLength, uint8_t *pucRxStartucBufferPointer )
{
	uint8_t ucStatus[2];

	vSX126xReadCommand( RADIO_GET_RXBUFFERSTATUS, ucStatus, 2 );
	// In case of LORA fixed header, the payloadLength is obtained by reading
	// the register REG_LR_PAYLOADLENGTH
	if ( ( eSX126xGetPacketType() == PACKET_TYPE_LORA ) && ( ucSX126xReadRegister( REG_LR_PACKETPARAMS ) >> 7 == 1 ) ) {
		*pucPayloadLength = ucSX126xReadRegister( REG_LR_PAYLOADLENGTH );
	}
	else {
		*pucPayloadLength = ucStatus[0];
	}
	*pucRxStartucBufferPointer = ucStatus[1];
}

void vSX126xGetPacketStatus( xPacketStatus_t *pxPktStatus )
{
	uint8_t ucStatus[3];

	vSX126xReadCommand( RADIO_GET_PACKETSTATUS, ucStatus, 3 );

	pxPktStatus->ePacketType = eSX126xGetPacketType();
	switch ( pxPktStatus->ePacketType ) {
		case PACKET_TYPE_GFSK:
			pxPktStatus->xParams.xGfsk.ucRxStatus  = ucStatus[0];
			pxPktStatus->xParams.xGfsk.cRssiSync   = -ucStatus[1] >> 1;
			pxPktStatus->xParams.xGfsk.cRssiAvg	= -ucStatus[2] >> 1;
			pxPktStatus->xParams.xGfsk.ulFreqError = 0;
			break;

		case PACKET_TYPE_LORA:
			pxPktStatus->xParams.xLoRa.cRssiPkt = -ucStatus[0] >> 1;
			// Returns SNR value [dB] rounded to the nearest integer value
			pxPktStatus->xParams.xLoRa.cSnrPkt		  = ( ( (int8_t) ucStatus[1] ) + 2 ) >> 2;
			pxPktStatus->xParams.xLoRa.cSignalRssiPkt = -ucStatus[2] >> 1;
			pxPktStatus->xParams.xLoRa.ulFreqError	= ulFrequencyError;
			break;

		default:
		case PACKET_TYPE_NONE:
			// In that specific case, we set everything in the pxPktStatus to zeros
			// and reset the packet type accordingly
			memset( pxPktStatus, 0, sizeof( xPacketStatus_t ) );
			pxPktStatus->ePacketType = PACKET_TYPE_NONE;
			break;
	}
}

xRadioError_t xSX126xGetDeviceErrors( void )
{
	xRadioError_t xError;

	vSX126xReadCommand( RADIO_GET_ERROR, (uint8_t *) &xError, 2 );
	return xError;
}

void vSX126xClearDeviceErrors( void )
{
	uint8_t ucBuf[2] = { 0x00, 0x00 };
	vSX126xWriteCommand( RADIO_CLR_ERROR, ucBuf, 2 );
}

void vSX126xClearIrqStatus( uint16_t usIrq )
{
	uint8_t ucBuf[2];

	ucBuf[0] = ( uint8_t )( ( (uint16_t) usIrq >> 8 ) & 0x00FF );
	ucBuf[1] = ( uint8_t )( (uint16_t) usIrq & 0x00FF );
	vSX126xWriteCommand( RADIO_CLR_IRQSTATUS, ucBuf, 2 );
}
