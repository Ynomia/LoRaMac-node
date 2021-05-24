/*!
 * \file      RegionCommon.c
 *
 * \brief     LoRa MAC common region implementation
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 */
#include <math.h>
#include "radio.h"
#include "utilities.h"
#include "RegionCommon.h"
#include "systime.h"

#define BACKOFF_DC_1_HOUR 100
#define BACKOFF_DC_10_HOURS 1000
#define BACKOFF_DC_24_HOURS 10000

#define BACKOFF_DUTY_CYCLE_1_HOUR_IN_S      3600
#define BACKOFF_DUTY_CYCLE_10_HOURS_IN_S    ( BACKOFF_DUTY_CYCLE_1_HOUR_IN_S + ( BACKOFF_DUTY_CYCLE_1_HOUR_IN_S * 10 ) )
#define BACKOFF_DUTY_CYCLE_24_HOURS_IN_S    ( BACKOFF_DUTY_CYCLE_10_HOURS_IN_S + ( BACKOFF_DUTY_CYCLE_1_HOUR_IN_S * 24 ) )
#define BACKOFF_24_HOURS_IN_S               ( BACKOFF_DUTY_CYCLE_1_HOUR_IN_S * 24 )

#ifndef DUTY_CYCLE_TIME_PERIOD
/*!
 * Default duty cycle observation time period
 *
 * \remark The ETSI observation time period is 1 hour (3600000 ms) but, the implemented algorithm may violate the
 *         defined duty-cycle restrictions. In order to ensure that these restrictions never get violated we changed the
 *         default duty cycle observation time period to 1/2 hour (1800000 ms).
 */
#define DUTY_CYCLE_TIME_PERIOD              1800000
#endif

/*!
 * \brief Returns `N / D` rounded to the smallest integer value greater than or equal to `N / D`
 *
 * \warning when `D == 0`, the result is undefined
 *
 * \remark `N` and `D` can be signed or unsigned
 *
 * \param [IN] N the numerator, which can have any sign
 * \param [IN] D the denominator, which can have any sign
 * \retval N / D with any fractional part rounded to the smallest integer value greater than or equal to `N / D`
 */
#define DIV_CEIL( N, D )                                                       \
    (                                                                          \
        ( N > 0 ) ?                                                            \
        ( ( ( N ) + ( D ) - 1 ) / ( D ) ) :                                    \
        ( ( N ) / ( D ) )                                                      \
    )


static uint8_t CountChannels( uint16_t mask, uint8_t nbBits )
{
	uint8_t nbActiveBits = 0;

	for ( uint8_t j = 0; j < nbBits; j++ ) {
		if ( ( mask & ( 1 << j ) ) == ( 1 << j ) ) {
			nbActiveBits++;
		}
	}
	return nbActiveBits;
}

uint16_t RegionCommonGetJoinDc( SysTime_t elapsedTime )
{
	uint16_t dutyCycle = 0;

	if ( elapsedTime.Seconds < 3600 ) {
		dutyCycle = BACKOFF_DC_1_HOUR;
	}
	else if ( elapsedTime.Seconds < ( 3600 + 36000 ) ) {
		dutyCycle = BACKOFF_DC_10_HOURS;
	}
	else {
		dutyCycle = BACKOFF_DC_24_HOURS;
	}
	return dutyCycle;
}

bool RegionCommonChanVerifyDr( uint8_t nbChannels, uint16_t *channelsMask, int8_t dr, int8_t minDr, int8_t maxDr, ChannelParams_t *channels )
{
	if ( RegionCommonValueInRange( dr, minDr, maxDr ) == 0 ) {
		return false;
	}

	for ( uint8_t i = 0, k = 0; i < nbChannels; i += 16, k++ ) {
		for ( uint8_t j = 0; j < 16; j++ ) {
			if ( ( ( channelsMask[k] & ( 1 << j ) ) != 0 ) ) { // Check datarate validity for enabled channels
				if ( RegionCommonValueInRange( dr, ( channels[i + j].DrRange.Fields.Min & 0x0F ),
											   ( channels[i + j].DrRange.Fields.Max & 0x0F ) ) == 1 ) {
					// At least 1 channel has been found we can return OK.
					return true;
				}
			}
		}
	}
	return false;
}

uint8_t RegionCommonValueInRange( int8_t value, int8_t min, int8_t max )
{
	if ( ( value >= min ) && ( value <= max ) ) {
		return 1;
	}
	return 0;
}

bool RegionCommonChanDisable( uint16_t *channelsMask, uint8_t id, uint8_t maxChannels )
{
	uint8_t index = id / 16;

	if ( ( index > ( maxChannels / 16 ) ) || ( id >= maxChannels ) ) {
		return false;
	}

	// Deactivate channel
	channelsMask[index] &= ~( 1 << ( id % 16 ) );

	return true;
}

uint8_t RegionCommonCountChannels( uint16_t *channelsMask, uint8_t startIdx, uint8_t stopIdx )
{
	uint8_t nbChannels = 0;

	if ( channelsMask == NULL ) {
		return 0;
	}

	for ( uint8_t i = startIdx; i < stopIdx; i++ ) {
		nbChannels += CountChannels( channelsMask[i], 16 );
	}

	return nbChannels;
}

void RegionCommonChanMaskCopy( uint16_t *channelsMaskDest, uint16_t *channelsMaskSrc, uint8_t len )
{
	if ( ( channelsMaskDest != NULL ) && ( channelsMaskSrc != NULL ) ) {
		for ( uint8_t i = 0; i < len; i++ ) {
			channelsMaskDest[i] = channelsMaskSrc[i];
		}
	}
}

void RegionCommonSetBandTxDone( bool joined, Band_t *band, TimerTime_t lastTxDone )
{
	if ( joined == true ) {
		band->LastTxDoneTime = lastTxDone;
	}
	else {
		band->LastTxDoneTime	 = lastTxDone;
		band->LastJoinTxDoneTime = lastTxDone;
	}
}

TimerTime_t RegionCommonUpdateBandTimeOff( bool joined, bool dutyCycle, Band_t *bands, uint8_t nbBands )
{
	TimerTime_t nextTxDelay = TIMERTIME_T_MAX;

	// Update bands Time OFF
	for ( uint8_t i = 0; i < nbBands; i++ ) {
		if ( joined == false ) {
			TimerTime_t elapsedJoin = TimerGetElapsedTime( bands[i].LastJoinTxDoneTime );
			TimerTime_t elapsedTx   = TimerGetElapsedTime( bands[i].LastTxDoneTime );
			TimerTime_t txDoneTime  = MAX( elapsedJoin,
										   ( dutyCycle == true ) ? elapsedTx : 0 );

			if ( bands[i].TimeOff <= txDoneTime ) {
				bands[i].TimeOff = 0;
			}
			if ( bands[i].TimeOff != 0 ) {
				nextTxDelay = MIN( bands[i].TimeOff - txDoneTime, nextTxDelay );
			}
		}
		else {
			if ( dutyCycle == true ) {
				TimerTime_t elapsed = TimerGetElapsedTime( bands[i].LastTxDoneTime );
				if ( bands[i].TimeOff <= elapsed ) {
					bands[i].TimeOff = 0;
				}
				if ( bands[i].TimeOff != 0 ) {
					nextTxDelay = MIN( bands[i].TimeOff - elapsed, nextTxDelay );
				}
			}
			else {
				nextTxDelay		 = 0;
				bands[i].TimeOff = 0;
			}
		}
	}

	return ( nextTxDelay == TIMERTIME_T_MAX ) ? 0 : nextTxDelay;
}

uint8_t RegionCommonParseLinkAdrReq( uint8_t *payload, RegionCommonLinkAdrParams_t *linkAdrParams )
{
	uint8_t retIndex = 0;

	if ( payload[0] == SRV_MAC_LINK_ADR_REQ ) {
		// Parse datarate and tx power
		linkAdrParams->Datarate = payload[1];
		linkAdrParams->TxPower  = linkAdrParams->Datarate & 0x0F;
		linkAdrParams->Datarate = ( linkAdrParams->Datarate >> 4 ) & 0x0F;
		// Parse ChMask
		linkAdrParams->ChMask = (uint16_t) payload[2];
		linkAdrParams->ChMask |= (uint16_t) payload[3] << 8;
		// Parse ChMaskCtrl and nbRep
		linkAdrParams->NbRep	  = payload[4];
		linkAdrParams->ChMaskCtrl = ( linkAdrParams->NbRep >> 4 ) & 0x07;
		linkAdrParams->NbRep &= 0x0F;

		// LinkAdrReq has 4 bytes length + 1 byte CMD
		retIndex = 5;
	}
	return retIndex;
}

uint8_t RegionCommonLinkAdrReqVerifyParams( RegionCommonLinkAdrReqVerifyParams_t *verifyParams, int8_t *dr, int8_t *txPow, uint8_t *nbRep )
{
	uint8_t status		  = verifyParams->Status;
	int8_t  datarate	  = verifyParams->Datarate;
	int8_t  txPower		  = verifyParams->TxPower;
	int8_t  nbRepetitions = verifyParams->NbRep;

	// Handle the case when ADR is off.
	if ( verifyParams->AdrEnabled == false ) {
		// When ADR is off, we are allowed to change the channels mask
		nbRepetitions = verifyParams->CurrentNbRep;
		datarate	  = verifyParams->CurrentDatarate;
		txPower		  = verifyParams->CurrentTxPower;
	}

	if ( status != 0 ) {
		// Verify datarate. The variable phyParam. Value contains the minimum allowed datarate.
		if ( RegionCommonChanVerifyDr( verifyParams->NbChannels, verifyParams->ChannelsMask, datarate,
									   verifyParams->MinDatarate, verifyParams->MaxDatarate, verifyParams->Channels ) == false ) {
			status &= 0xFD; // Datarate KO
		}

		// Verify tx power
		if ( RegionCommonValueInRange( txPower, verifyParams->MaxTxPower, verifyParams->MinTxPower ) == 0 ) {
			// Verify if the maximum TX power is exceeded
			if ( verifyParams->MaxTxPower > txPower ) { // Apply maximum TX power. Accept TX power.
				txPower = verifyParams->MaxTxPower;
			}
			else {
				status &= 0xFB; // TxPower KO
			}
		}
	}

	// If the status is ok, verify the NbRep
	if ( status == 0x07 ) {
		if ( nbRepetitions == 0 ) { // Restore the default value according to the LoRaWAN specification
			nbRepetitions = 1;
		}
	}

	// Apply changes
	*dr	= datarate;
	*txPow = txPower;
	*nbRep = nbRepetitions;

	return status;
}

uint32_t RegionCommonComputeSymbolTimeLoRa( uint8_t phyDr, uint32_t bandwidthInHz )
{
    return ( 1 << phyDr ) * 1000000 / bandwidthInHz;
}

double RegionCommonComputeSymbolTimeFsk( uint8_t phyDr )
{
	return ( 8.0 / (double) phyDr ); // 1 symbol equals 1 byte
}

void RegionCommonComputeRxWindowParameters( uint32_t tSymbolInUs, uint8_t minRxSymbols, uint32_t rxErrorInMs, uint32_t wakeUpTimeInMs, uint32_t* windowTimeoutInSymbols, int32_t* windowOffsetInMs )
{
    *windowTimeoutInSymbols = MAX( DIV_CEIL( ( ( 2 * minRxSymbols - 8 ) * tSymbolInUs + 2 * ( rxErrorInMs * 1000 ) ),  tSymbolInUs ), minRxSymbols ); // Computed number of symbols
    *windowOffsetInMs = ( int32_t )DIV_CEIL( ( int32_t )( 4 * tSymbolInUs ) -
                                               ( int32_t )DIV_CEIL( ( *windowTimeoutInSymbols * tSymbolInUs ), 2 ) -
                                               ( int32_t )( wakeUpTimeInMs * 1000 ), 1000 );
}

int8_t RegionCommonComputeTxPower( int8_t txPowerIndex, float maxEirp, float antennaGain )
{
	int8_t phyTxPower = 0;

	phyTxPower = (int8_t) floor( ( maxEirp - ( txPowerIndex * 2U ) ) - antennaGain );

	return phyTxPower;
}

void RegionCommonCalcBackOff( RegionCommonCalcBackOffParams_t *calcBackOffParams )
{
	uint8_t  bandIdx	   = calcBackOffParams->Channels[calcBackOffParams->Channel].Band;
	uint16_t dutyCycle	 = calcBackOffParams->Bands[bandIdx].DCycle;
	uint16_t joinDutyCycle = 0;

	// Reset time-off to initial value.
	calcBackOffParams->Bands[bandIdx].TimeOff = 0;

	if ( calcBackOffParams->Joined == false ) {
		// Get the join duty cycle
		joinDutyCycle = RegionCommonGetJoinDc( calcBackOffParams->ElapsedTime );
		// Apply the most restricting duty cycle
		dutyCycle = MAX( dutyCycle, joinDutyCycle );
		// Reset the timeoff if the last frame was not a join request and when the duty cycle is not enabled
		if ( ( calcBackOffParams->DutyCycleEnabled == false ) && ( calcBackOffParams->LastTxIsJoinRequest == false ) ) {
			// This is the case when the duty cycle is off and the last uplink frame was not a join.
			// This could happen in case of a rejoin, e.g. in compliance test mode.
			// In this special case we have to set the time off to 0, since the join duty cycle shall only
			// be applied after the first join request.
			calcBackOffParams->Bands[bandIdx].TimeOff = 0;
		}
		else {
			// Apply band time-off.
			calcBackOffParams->Bands[bandIdx].TimeOff = calcBackOffParams->TxTimeOnAir * dutyCycle - calcBackOffParams->TxTimeOnAir;
		}
	}
	else {
		if ( calcBackOffParams->DutyCycleEnabled == true ) {
			calcBackOffParams->Bands[bandIdx].TimeOff = calcBackOffParams->TxTimeOnAir * dutyCycle - calcBackOffParams->TxTimeOnAir;
		}
		else {
			calcBackOffParams->Bands[bandIdx].TimeOff = 0;
		}
	}
}

void RegionCommonRxBeaconSetup( RegionCommonRxBeaconSetupParams_t *rxBeaconSetupParams )
{
	bool	rxContinuous = true;
	uint8_t datarate;

	// Set the radio into sleep mode
	Radio.Sleep();

	// Setup frequency and payload length
	Radio.SetChannel( rxBeaconSetupParams->Frequency );
	Radio.SetMaxPayloadLength( MODEM_LORA, rxBeaconSetupParams->BeaconSize );

	// Check the RX continuous mode
	if ( rxBeaconSetupParams->RxTime != 0 ) {
		rxContinuous = false;
	}

	// Get region specific datarate
	datarate = rxBeaconSetupParams->Datarates[rxBeaconSetupParams->BeaconDatarate];

	// Setup radio
	Radio.SetRxConfig( MODEM_LORA, rxBeaconSetupParams->BeaconChannelBW, datarate,
					   1, 0, 10, rxBeaconSetupParams->SymbolTimeout, true, rxBeaconSetupParams->BeaconSize, false, 0, 0, false, rxContinuous );

	Radio.Rx( rxBeaconSetupParams->RxTime );
}

int8_t RegionCommonGetNextLowerTxDr( int8_t dr, int8_t minDr )
{
    if( dr == minDr )
    {
        return minDr;
    }
    else
    {
        return( dr - 1 );
    }
}

int8_t RegionCommonLimitTxPower( int8_t txPower, int8_t maxBandTxPower )
{
    // Limit tx power to the band max
    return MAX( txPower, maxBandTxPower );
}

uint32_t RegionCommonGetBandwidth( uint32_t drIndex, const uint32_t* bandwidths )
{
    switch( bandwidths[drIndex] )
    {
        default:
        case 125000:
            return 0;
        case 250000:
            return 1;
        case 500000:
            return 2;
    }
}
