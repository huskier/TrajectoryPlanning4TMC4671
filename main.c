
#include "boards/Board.h"
#include "hal/derivative.h"
#include "hal/HAL.h"
#include "tmc/IdDetection.h"
#include "tmc/TMCL.h"
#include "tmc/VitalSignsMonitor.h"
#include "tmc/BoardAssignment.h"

#include "tmc/ic/TMC4671/TMC4671.h"
#include "tmc/ramp/LinearRamp.h"

#include "tmc/TMCL.h"

#include <string.h>

const char *VersionString = MODULE_ID"V306"; // module id and version of the firmware shown in the TMCL-IDE

u32 updateCnt;

/* Keep as is! This lines are important for the update functionality. */
#if defined(Landungsbruecke)
	const uint8 Protection[] __attribute__ ((section(".cfmconfig")))=
	{
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  //Backdoor key
		0xFF, 0xFF, 0xFF, 0xFF,                          //Flash protection (FPPROT)
		0x7E,                                            //Flash security   (FSEC) => nach Image-Generierung manuell auf 0x40 setzen im Image
		0xF9,                                            //Flash option     (FOPT) (NMI ausgeschaltet, EzPort ausgeschaltet, Normal power)
		0xFF,                                            //reserved
		0xFF                                             //reserved
	};

	__attribute__ ((section(".bldata"))) uint32 BLMagic;
#endif


#define DEFAULT_MOTOR  0
#define TMC4671_MOTORS 1
#define USE_LINEAR_RAMP

static IOPinTypeDef *PIN_DRV_ENN;
static ConfigurationTypeDef *TMC4671_config;
static SPIChannelTypeDef *TMC4671_SPIChannel;

typedef struct
{
	u16  startVoltage;
	u16  initWaitTime;
	u16  actualInitWaitTime;
	u8   initState;
	u8   initMode;
	u16  torqueMeasurementFactor;  // u8.u8
	u8	 motionMode;
} TMinimalMotorConfig;

TMinimalMotorConfig motorConfig[TMC4671_MOTORS];

TMC_LinearRamp rampGenerator[TMC4671_MOTORS];
u8 actualMotionMode[TMC4671_MOTORS];


void readChipInfo()
{
		int value;
		uint8 dispByte;

		value = tmc4671_readInt(0, TMC4671_CHIPINFO_DATA);

		dispInt(value);
}

// => SPI wrapper
u8 tmc4671_readwriteByte(u8 motor, u8 data, u8 lastTransfer)
{
	if (motor == DEFAULT_MOTOR)
		return TMC4671_SPIChannel->readWrite(data, lastTransfer);
	else
		return 0;
}
// <= SPI wrapper

static uint32 rotate(uint8 motor, int32 velocity)
{
	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

//#ifdef USE_LINEAR_RAMP
	// update velocity ramp before switching from torque to velocity mode
	if (actualMotionMode[motor] == TMC4671_MOTION_MODE_TORQUE)
		rampGenerator[motor].rampVelocity = tmc4671_getActualVelocity(motor);

	// switch to velocity motion mode
	actualMotionMode[motor] = TMC4671_MOTION_MODE_VELOCITY;
	tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_VELOCITY);

	// set target velocity for ramp generator
	rampGenerator[motor].targetVelocity = velocity;
//#else
	//tmc4671_setTargetVelocity(motor, velocity);
//#endif

	return TMC_ERROR_NONE;
}

static uint32 right(uint8 motor, int32 velocity)
{
	return rotate(motor, velocity);
}

static uint32 left(uint8 motor, int32 velocity)
{
	return rotate(motor, -velocity);
}

static uint32 stop(uint8 motor)
{
	return rotate(motor, 0);
}

static uint32 moveTo(uint8 motor, int32 position)
{
	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

#ifdef USE_LINEAR_RAMP
	// update velocity ramp before switching from torque to position mode
	if (actualMotionMode[motor] == TMC4671_MOTION_MODE_TORQUE)
		rampGenerator[motor].rampVelocity = tmc4671_getActualVelocity(motor);

	// switch to position motion mode
	actualMotionMode[motor] = TMC4671_MOTION_MODE_POSITION;
	tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_POSITION);

	// set target position for ramp generator
	rampGenerator[motor].targetPosition = position;
#else
	tmc4671_setAbsolutTargetPosition(motor, position);
#endif

	return TMC_ERROR_NONE;
}

static uint32 moveBy(uint8 motor, int32 *ticks)
{
	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

	// update velocity ramp before switching from torque to position mode
	if (actualMotionMode[motor] == TMC4671_MOTION_MODE_TORQUE)
		rampGenerator[motor].rampVelocity = tmc4671_getActualVelocity(motor);

	// switch to position motion mode
	actualMotionMode[motor] = TMC4671_MOTION_MODE_POSITION;
	tmc4671_switchToMotionMode(motor, TMC4671_MOTION_MODE_POSITION);

	// set target position for ramp generator
	int actPos = tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL);
	rampGenerator[motor].targetPosition = (s32) actPos + *ticks;


	dispString("actPos is : ");
	dispInt(actPos);
	dispString("targetPos is : ");
	dispInt(rampGenerator[motor].targetPosition);

	return TMC_ERROR_NONE;
}

static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value)
{
	u32 errors = TMC_ERROR_NONE;

	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 4: // Maximum speed
		if(readWrite == READ)
		{
			*value = (u32) tmc4671_readInt(motor, TMC4671_PID_VELOCITY_LIMIT);

#ifdef USE_LINEAR_RAMP
			// update also ramp generator value
			rampGenerator[motor].maxVelocity = *value;
#endif
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_LIMIT, *value);

#ifdef USE_LINEAR_RAMP
			// update also ramp generator value
			rampGenerator[motor].maxVelocity = *value;
#endif
		}
		break;
	case 11: // acceleration
		if(readWrite == READ)
		{
			*value = (u32) tmc4671_readInt(motor, TMC4671_PID_ACCELERATION_LIMIT);

#ifdef USE_LINEAR_RAMP
			// update also ramp generator value
			rampGenerator[motor].acceleration = *value;
#endif
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_PID_ACCELERATION_LIMIT, *value);

#ifdef USE_LINEAR_RAMP
			// update also ramp generator value
			rampGenerator[motor].acceleration = *value;
#endif
		}
		break;
#ifdef USE_LINEAR_RAMP
	case 12: // enable velocity ramp
		if(readWrite == READ)
			*value = rampGenerator[motor].rampEnabled;
		else if(readWrite == WRITE)
			rampGenerator[motor].rampEnabled = *value;
		break;
#endif
	case 13: // ramp velocity
		if(readWrite == READ) {
#ifdef USE_LINEAR_RAMP
			*value = rampGenerator[motor].rampVelocity;
#else
			*value = tmc4671_readInt(motor, TMC4671_PID_VELOCITY_TARGET);
#endif
		}
		break;


	// ADC offsets
	case 50:
		// ADC_I1_OFFSET
		if(readWrite == READ)
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_I1_SCALE_OFFSET, BIT_0_TO_15);
		else if(readWrite == WRITE)
			tmc4671_writeRegister16BitValue(motor, TMC4671_ADC_I1_SCALE_OFFSET, BIT_0_TO_15, *value);
		break;
	case 51:
		// ADC_I0_OFFSET
		if(readWrite == READ)
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_I0_SCALE_OFFSET, BIT_0_TO_15);
		else if(readWrite == WRITE)
			tmc4671_writeRegister16BitValue(motor, TMC4671_ADC_I0_SCALE_OFFSET, BIT_0_TO_15, *value);
		break;
		// ADC scaler values
	case 53:
		// ADC_I1_SCALE
		if(readWrite == READ)
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_I1_SCALE_OFFSET, BIT_16_TO_31);
		else if(readWrite == WRITE)
			tmc4671_writeRegister16BitValue(motor, TMC4671_ADC_I1_SCALE_OFFSET, BIT_16_TO_31, *value);
		break;
	case 54:
		// ADC_I0_SCALE
		if(readWrite == READ)
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_I0_SCALE_OFFSET, BIT_16_TO_31);
		else if(readWrite == WRITE)
			tmc4671_writeRegister16BitValue(motor, TMC4671_ADC_I0_SCALE_OFFSET, BIT_16_TO_31, *value);
		break;
	case 100:
		// ADC_VM_RAW
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_ADC_RAW_ADDR, 1);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 101:
		// ADC_AGPI_A_RAW
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_ADC_RAW_ADDR, 1);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 102:
		// ADC_AGPI_B_RAW
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_ADC_RAW_ADDR, 2);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 111:
		// ADC_AENC_UX_RAW
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_ADC_RAW_ADDR, 2);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 112:
		// ADC_AENC_WY_RAW
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_ADC_RAW_ADDR, 3);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 113:
		// ADC_AENC_VN_RAW
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_ADC_RAW_ADDR, 3);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 116:
		// ADC_I0_scaled
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_IWY_IUX, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 117:
		// ADC_I2_scaled
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_IWY_IUX, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 118:
		// ADC_I1_scaled
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_IV, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 120:
		// AENC_UX
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_AENC_WY_UX, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 121:
		// AENC_WY
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_AENC_WY_UX, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 122:
		// AENC_VN
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_AENC_VN, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 124: // (vorher 103)
		// ADCSD_I0_RAW
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_ADC_RAW_ADDR, 0);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 125: // (vorher 104)
		// ADCSD_I1_RAW:
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_ADC_RAW_ADDR, 0);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 127:
		// ADC_I0_EXT
		if(readWrite == READ) {
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_I1_I0_EXT, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 128:
		// ADC_I1_EXT
		if(readWrite == READ) {
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_ADC_I1_I0_EXT, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 132:
		// AENC_DECODER_COUNT
		if(readWrite == READ) {
			*value = (s32) tmc4671_readInt(motor, TMC4671_AENC_DECODER_COUNT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 133:
		// AENC_DECODER_COUNT_N
		if(readWrite == READ) {
			*value = (s32) tmc4671_readInt(motor, TMC4671_AENC_DECODER_COUNT_N);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 134:
		// AENC_DECODER_PHI_E
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_AENC_DECODER_PHI_E_PHI_M, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 135:
		// AENC_DECODER_PHI_M
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_AENC_DECODER_PHI_E_PHI_M, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 136:
		// AENC_DECODER_POSITION
		if(readWrite == READ) {
			*value = (s32) tmc4671_readInt(motor, TMC4671_AENC_DECODER_POSITION);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 140:
		// OPENLOOP_VELOCITY_TARGET
		if(readWrite == READ) {
			*value = (s32) tmc4671_readInt(motor, TMC4671_OPENLOOP_VELOCITY_TARGET);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 141:
		// OPENLOOP_VELOCITY_ACTUAL
		if(readWrite == READ) {
			*value = (s32) tmc4671_readInt(motor, TMC4671_OPENLOOP_VELOCITY_ACTUAL);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 142:
		// OPENLOOP_PHI
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_OPENLOOP_PHI, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 150:
		// ABN_DECODER_COUNT
		if(readWrite == READ) {
			*value = tmc4671_readInt(motor, TMC4671_ABN_DECODER_COUNT) & 0x00FFFFFF;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 151:
		// ABN_DECODER_COUNT_N
		if(readWrite == READ) {
			*value = tmc4671_readInt(motor, TMC4671_ABN_DECODER_COUNT_N) & 0x00FFFFFF;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 152:
		// ABN_DECODER_PHI_E
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 153:
		// ABN_DECODER_PHI_M
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_ABN_DECODER_PHI_E_PHI_M, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 154:
		// STATUS_REG_0
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 212);
			*value =  (s32) tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 212);
			tmc4671_writeInt(motor, TMC4671_INTERIM_DATA, *value);
		}
		break;
	case 155:
		// STATUS_REG_1
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 213);
			*value =  (s32) tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 213);
			tmc4671_writeInt(motor, TMC4671_INTERIM_DATA, *value);
		}
		break;
	case 156:
		// STATUS_PARAM_0
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 214);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 214);
			tmc4671_writeRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15, *value);
		}
		break;
	case 157:
		// STATUS_PARAM_1
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 214);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 214);
			tmc4671_writeRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31, *value);
		}
		break;
	case 158:
		// STATUS_PARAM_2
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 215);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 215);
			tmc4671_writeRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15, *value);
		}
		break;
	case 159:
		// STATUS_PARAM_3
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 215);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 215);
			tmc4671_writeRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31, *value);
		}
		break;
	case 160:
		// HALL_PHI_E
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_HALL_PHI_E_INTERPOLATED_PHI_E, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 161:
		// HALL_PHI_E_INTERPOLATED
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_HALL_PHI_E_INTERPOLATED_PHI_E, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 162:
		// AENC_DECODER_PHI_A_RAW
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_AENC_DECODER_PHI_A_RAW, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 163:
		// AENC_DECODER_PHI_A
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_AENC_DECODER_PHI_A, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 164:
		// HALL_PHI_M
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_HALL_PHI_M, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 170:
		// PHI_E
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_PHI_E, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 171:
		// PID_TORQUE_TARGET
		if(readWrite == READ)
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_PID_TORQUE_FLUX_TARGET, BIT_16_TO_31);
		else if(readWrite == WRITE)
			tmc4671_writeRegister16BitValue(motor, TMC4671_PID_TORQUE_FLUX_TARGET, BIT_16_TO_31, *value);
		break;
	case 172:
		// PID_FLUX_TARGET
		if(readWrite == READ)
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_PID_TORQUE_FLUX_TARGET, BIT_0_TO_15);
		else if(readWrite == WRITE)
			tmc4671_writeRegister16BitValue(motor, TMC4671_PID_TORQUE_FLUX_TARGET, BIT_0_TO_15, *value);
		break;
	case 173:
		// PID_VELOCITY_TARGET
		if(readWrite == READ) {
			*value = (s32) tmc4671_readInt(motor, TMC4671_PID_VELOCITY_TARGET);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 174:
		// PID_POSITION_TARGET
		if(readWrite == READ) {
			*value = (s32) tmc4671_readInt(motor, TMC4671_PID_POSITION_TARGET);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 175:
		// PID_TORQUE_ACTUAL
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_PID_TORQUE_FLUX_ACTUAL, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 176:
		// PID_TORQUE_ACTUAL_mA
		if(readWrite == READ) {
			*value = tmc4671_getActualTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 177:
		// PID_FLUX_ACTUAL
		if(readWrite == READ) {
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_PID_TORQUE_FLUX_ACTUAL, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 178:
		// PID_VELOCITY_ACTUAL
		if(readWrite == READ) {
			*value = tmc4671_getActualVelocity(motor);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 179:
		// PID_POSITION_ACTUAL
		if(readWrite == READ)
			*value = tmc4671_getActualPosition(motor);
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_PID_POSITION_ACTUAL, *value);
#ifdef USE_LINEAR_RAMP
			// also update linear ramp during clear of actual position
			if (actualMotionMode[motor] == TMC4671_MOTION_MODE_POSITION)
			{
				rampGenerator[motor].targetPosition = *value;
				rampGenerator[motor].rampPosition = *value;
				tmc4671_writeInt(motor, TMC4671_PID_POSITION_TARGET, *value);
			}
#endif
		}
		break;
	case 180:
		// PID_TORQUE_ERROR
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_PID_ERROR_ADDR, 0);
			*value = tmc4671_readInt(motor, TMC4671_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// PID_FLUX_ERROR
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_PID_ERROR_ADDR, 1);
			*value = tmc4671_readInt(motor, TMC4671_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 182:
		// PID_VELOCITY_ERROR
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_PID_ERROR_ADDR, 2);
			*value = tmc4671_readInt(motor, TMC4671_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 183:
		// PID_POSITION_ERROR
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_PID_ERROR_ADDR, 3);
			*value = tmc4671_readInt(motor, TMC4671_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 184:
		// PID_TORQUE_ERROR_SUM
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_PID_ERROR_ADDR, 4);
			*value = tmc4671_readInt(motor, TMC4671_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 185:
		// PID_FLUX_ERROR_SUM
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_PID_ERROR_ADDR, 5);
			*value = tmc4671_readInt(motor, TMC4671_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 186:
		// PID_VELOCITY_ERROR_SUM
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_PID_ERROR_ADDR, 6);
			*value = tmc4671_readInt(motor, TMC4671_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 187:
		// PID_POSITION_ERROR_SUM
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_PID_ERROR_ADDR, 7);
			*value = tmc4671_readInt(motor, TMC4671_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 189:
		// PIDIN_TARGET_TORQUE
		if(readWrite == READ) {
			*value = tmc4671_getTargetTorque_raw(motor);
		} else if(readWrite == WRITE) {
			tmc4671_setTargetTorque_raw(motor, *value);
		}
		break;
	case 190:
		// PIDIN_TARGET_TORQUE_mA
		if(readWrite == READ) {
			*value = tmc4671_getTargetTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor);
		} else if(readWrite == WRITE) {
			tmc4671_setTargetTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor, *value);
#ifdef USE_LINEAR_RAMP
			// remember switched motion mode by setTargetTorque_mA
			actualMotionMode[motor] = TMC4671_MOTION_MODE_TORQUE;
#endif
		}
		break;
	case 191:
		// PIDIN_TARGET_FLUX
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 1);
			*value = tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 192:
		// PIDIN_TARGET_VELOCITY
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 2);
			*value = tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 193:
		// PIDIN_TARGET_POSITION
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 3);
			*value = tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 194:
		// PIDOUT_TARGET_TORQUE
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 4);
			*value = tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 195:
		// PIDOUT_TARGET_FLUX
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 5);
			*value = tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 196:
		// PIDOUT_TARGET_VELOCITY
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 6);
			*value = tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 197:
		// PIDOUT_TARGET_POSITION
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 7);
			*value = tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 198:
		// FOC_IUX
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 8);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 199:
		// FOC_IWY
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 8);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 200:
		// FOC_IV
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 9);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 201:
		// FOC_IA
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 10);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 202:
		// FOC_IB
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 10);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 203:
		// FOC_ID
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 11);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 204:
		// FOC_IQ
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 11);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 205:
		// FOC_UD
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 12);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 206:
		// FOC_UQ
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 12);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 207:
		// FOC_UD_LIMITED
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 13);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 208:
		// FOC_UQ_LIMITED
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 13);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// FOC_UA
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 14);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 210:
		// FOC_UB
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 14);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 211:
		// FOC_UUX
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 15);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 212:
		// FOC_UWY
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 15);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 213:
		// FOC_UV
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 16);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 214:
		// PWM_UX
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 17);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 215:
		// PWM_WY
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 17);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 216:
		// PWM_UV
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 18);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 217:
		// ADC_I_0
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 19);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 218:
		// ADC_I_1
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 19);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 220:
		// CONFIG_REG_0
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 208);
			*value =  (s32) tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 208);
			tmc4671_writeInt(motor, TMC4671_INTERIM_DATA, *value);
		}
		break;
	case 221:
		// CONFIG_REG_1
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 209);
			*value =  (s32) tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 209);
			tmc4671_writeInt(motor, TMC4671_INTERIM_DATA, *value);
		}
		break;
	case 222:
		// CTRL_PARAM_0
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 210);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 210);
			tmc4671_writeRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15, *value);
		}
		break;
	case 223:
		// CTRL_PARAM_1
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 210);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 210);
			tmc4671_writeRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31, *value);
		}
		break;
	case 224:
		// CTRL_PARAM_2
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 211);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 211);
			tmc4671_writeRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15, *value);
		}
		break;
	case 225:
		// CTRL_PARAM_3
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 211);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		}
		else if(readWrite == WRITE)
		{
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 211);
			tmc4671_writeRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31, *value);
		}
		break;
	case 226:
		// ACTUAL_VELOCITY_PPTM
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 29);
			*value = (s32) tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 230:
		// DEBUG_VALUE_0
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 192);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 231:
		// DEBUG_VALUE_1
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 192);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 232:
		// DEBUG_VALUE_2
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 193);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 233:
		// DEBUG_VALUE_3
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 193);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 234:
		// DEBUG_VALUE_4
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 194);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 235:
		// DEBUG_VALUE_5
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 194);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 236:
		// DEBUG_VALUE_6
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 195);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 237:
		// DEBUG_VALUE_7
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 195);
			*value = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 238:
		// DEBUG_VALUE_8
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 196);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 239:
		// DEBUG_VALUE_9
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 196);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 240:
		// DEBUG_VALUE_10
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 197);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 241:
		// DEBUG_VALUE_11
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 197);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 242:
		// DEBUG_VALUE_12
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 198);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 243:
		// DEBUG_VALUE_13
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 198);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 244:
		// DEBUG_VALUE_14
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 199);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 245:
		// DEBUG_VALUE_15
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 199);
			*value = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 246:
		// DEBUG_VALUE_16
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 200);
			*value = (s32) tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 247:
		// DEBUG_VALUE_17
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 201);
			*value = (s32) tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 248:
		// DEBUG_VALUE_18
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 202);
			*value =  (s32) tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 249:
		// DEBUG_VALUE_19
		if(readWrite == READ) {
			tmc4671_writeInt(motor, TMC4671_INTERIM_ADDR, 203);
			*value =  (s32) tmc4671_readInt(motor, TMC4671_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 251:
		// Torque measurement factor
		if(readWrite == READ) {
			*value = motorConfig[motor].torqueMeasurementFactor;
		} else if(readWrite == WRITE) {
			motorConfig[motor].torqueMeasurementFactor = *value;
		}
		break;
	case 252:
		// Start encoder initialization
		if(readWrite == READ) {
			*value = motorConfig[motor].initMode;
		} else if(readWrite == WRITE) {
			tmc4671_startEncoderInitialization(*value, &motorConfig[motor].initMode, &motorConfig[motor].initState);
		}
		break;
	case 253:
		// Encoder init state
		if(readWrite == READ) {
			*value = motorConfig[motor].initState;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 254:
		// Actual encoder wait time
		if(readWrite == READ) {
			*value = motorConfig[motor].actualInitWaitTime;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static uint32 getMeasuredSpeed(uint8 motor, int32 *value)
{
	if(motor >= TMC4671_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = tmc4671_getActualVelocity(motor);

	return TMC_ERROR_NONE;
}

static void periodicJob_InMain(uint32 actualSystick)
{
	int motor;

	// 1ms velocity ramp handling
	static u32 lastSystick;
	if (lastSystick != actualSystick)
	{
		// do velocity / position ramping for every motor
		for (motor = 0; motor < TMC4671_MOTORS; motor++)
		{
			if (actualMotionMode[motor] == TMC4671_MOTION_MODE_POSITION)
			{
				//dispString("In POS MODE...");
				tmc_linearRamp_computeRampPosition(&rampGenerator[motor]);

				// set new target position
				tmc4671_writeInt(motor, TMC4671_PID_POSITION_TARGET, rampGenerator[motor].rampPosition);

			}
			else if (actualMotionMode[motor] == TMC4671_MOTION_MODE_VELOCITY)
			{
				//dispString("In VEL MODE...");

				tmc_linearRamp_computeRampVelocity(&rampGenerator[motor]);

				// keep position ramp in reset
				rampGenerator[motor].rampPosition = tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL);
				tmc4671_writeInt(motor, TMC4671_PID_POSITION_TARGET, rampGenerator[motor].rampPosition);
				rampGenerator[motor].lastdXRest = 0;

				// set new target velocity
				tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_TARGET, rampGenerator[motor].rampVelocity);
			}
			else if (actualMotionMode[motor] == TMC4671_MOTION_MODE_TORQUE)
			{
				//dispString("In TOR MODE...");

				// only keep position ramp in reset
				rampGenerator[motor].rampPosition = tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL);
				tmc4671_writeInt(motor, TMC4671_PID_POSITION_TARGET, rampGenerator[motor].rampPosition);
				rampGenerator[motor].lastdXRest = 0;
			}

			//updateCnt++;

			//if( updateCnt == 500)
			if(rampGenerator[motor].rampPosition != rampGenerator[motor].targetPosition)
			{
				//if(updateCnt >= 100)
				{
					dispString("MOVING...");
					dispInt(rampGenerator[motor].rampPosition);
					//updateCnt = 0;

					int actPos = tmc4671_readInt(motor, TMC4671_PID_POSITION_ACTUAL);
					dispString("actPos is : ");
					dispInt(actPos);
				}
			}
		}
		lastSystick = actualSystick;
	}
}

static void writeRegister(u8 motor, uint8 address, int32 value)
{
	UNUSED(motor);
	tmc4671_writeInt(DEFAULT_MOTOR, address, value);
}

static void readRegister(u8 motor, uint8 address, int32 *value)
{
	UNUSED(motor);
	*value = tmc4671_readInt(DEFAULT_MOTOR, address);
}

static uint32 SAP(uint8 type, uint8 motor, int32 value)
{
	return handleParameter(WRITE, motor, type, &value);
}

static uint32 GAP(uint8 type, uint8 motor, int32 *value)
{
	return handleParameter(READ, motor, type, value);
}

static uint32 userFunction(uint8 type, uint8 motor, int32 *value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);
	return 0;
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
		HAL.IOs->config->setLow(PIN_DRV_ENN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setHigh(PIN_DRV_ENN);
}

static void deInit(void)
{
	enableDriver(DRIVER_DISABLE);
	HAL.IOs->config->setLow(PIN_DRV_ENN);
};

static uint8 reset()
{
	// set default polarity for evaluation board's power stage after VW reset
	for(size_t motor = 0; motor < TMC4671_MOTORS; motor++) {
		tmc4671_writeInt(motor, TMC4671_PWM_POLARITIES, 0);
		tmc4671_writeInt(motor, TMC4671_PWM_SV_CHOP, 0x0);
	}

	return 1;
}

static uint8 restore()
{
	// set default polarity for evaluation board's power stage after VW reset
	for(size_t motor = 0; motor < TMC4671_MOTORS; motor++) {
		tmc4671_writeInt(motor, TMC4671_PWM_POLARITIES, 0);
		tmc4671_writeInt(motor, TMC4671_PWM_SV_CHOP, 0x0);
	}

	//enableDriver(DRIVER_DISABLE);
	return 1;
}

static void checkErrors(uint32 tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

void TMC4671_init(void)
{
	// configure ENABLE-PIN for TMC4671
	PIN_DRV_ENN = &HAL.IOs->pins->DIO0;
	HAL.IOs->config->toOutput(PIN_DRV_ENN);
	HAL.IOs->config->setHigh(PIN_DRV_ENN);

	TMC4671_SPIChannel = &HAL.SPI->ch1;
	TMC4671_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

	int motor;

	// set default polarity for evaluation board's power stage on init
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PWM_POLARITIES, 0x0);
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PWM_SV_CHOP, 0x0);
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_dsADC_MCLK_B, 0x0);

	// set default acceleration and max velocity
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PID_ACCELERATION_LIMIT, 800);
    tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PID_VELOCITY_LIMIT, 800);

	// init ramp generator
	for (motor = 0; motor < TMC4671_MOTORS; motor++)
	{
		tmc_linearRamp_init(&rampGenerator[motor]);
		actualMotionMode[motor] = TMC4671_MOTION_MODE_STOPPED;

		// update ramp generator default values
		rampGenerator[motor].maxVelocity = (u32) tmc4671_readInt(motor, TMC4671_PID_VELOCITY_LIMIT);
		rampGenerator[motor].acceleration = (u32) tmc4671_readInt(motor, TMC4671_PID_ACCELERATION_LIMIT);
	}

	// enable the driver
	enableDriver(DRIVER_ENABLE);
}


/* Check if jumping into bootloader is forced                                           */
/*                                                                                      */
/* In order to jump to bootloader e.g. because of an accidental infinite loop           */
/* in a modified firmware you may short ID_CLK and ID_CH0 pins on start up.             */
/* This will force the entrance into bootloader mode and allow to replace bad firmware. */
void shallForceBoot()
{
	// toggle each pin and see if you can read the state on the other
	// leave if not, because this means that the pins are not tied together
	HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CLK);
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CH0);

	HAL.IOs->config->setHigh(&HAL.IOs->pins->ID_CLK);
	if(!HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CH0))
		return;

	HAL.IOs->config->setLow(&HAL.IOs->pins->ID_CLK);
	if(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CH0))
		return;

	HAL.IOs->config->toOutput(&HAL.IOs->pins->ID_CH0);
	HAL.IOs->config->toInput(&HAL.IOs->pins->ID_CLK);

	HAL.IOs->config->setHigh(&HAL.IOs->pins->ID_CH0);
	if(!HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CLK))
		return;

	HAL.IOs->config->setLow(&HAL.IOs->pins->ID_CH0);
	if(HAL.IOs->config->isHigh(&HAL.IOs->pins->ID_CLK))
		return;

	// not returned, this means pins are tied together
	tmcl_boot();
}

/* Call all standard initialization routines. */
static void init()
{
	HAL.init();                  // Initialize Hardware Abstraction Layer
	IDDetection_init();          // Initialize board detection
	tmcl_init();                 // Initialize TMCL communication
}

void configureMotor(uint8_t motor)
{
	// Type of motor &  PWM configuration
	tmc4671_writeInt(motor, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030001);
	tmc4671_writeInt(motor, TMC4671_PWM_POLARITIES, 0x00000000);
	tmc4671_writeInt(motor, TMC4671_PWM_SV_CHOP, 0x00000007);
	tmc4671_writeInt(motor, TMC4671_PWM_MAXCNT, 0x00000F9F);
	tmc4671_writeInt(motor, TMC4671_PWM_BBM_H_BBM_L, 0x00000505);

	tmc4671_writeRegister16BitValue(motor, TMC4671_PHI_E_SELECTION, BIT_0_TO_15, 3);

	// ADC configuration
	tmc4671_writeInt(motor, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
	tmc4671_writeInt(motor, TMC4671_dsADC_MCLK_A, 0x20000000);
	tmc4671_writeInt(motor, TMC4671_dsADC_MCLK_B, 0x00000000);
	tmc4671_writeInt(motor, TMC4671_dsADC_MDEC_B_MDEC_A, 0x014E014E); // Decimation configuration register.

	//ADC scale & offset
	tmc4671_writeInt(motor, TMC4671_ADC_I0_SCALE_OFFSET, 0x01005C69);
	tmc4671_writeInt(motor, TMC4671_ADC_I1_SCALE_OFFSET, 0x01005C7F);
	tmc4671_writeInt(motor, TMC4671_ADC_I_SELECT, 0x18000100);
	tmc4671_writeInt(motor, TMC4671_ADC_I1_I0_EXT, 0x00000000);

	//Encoder configuration
	//tmc4671_writeInt(motor, TMC4671_ABN_DECODER_MODE, 0x00001000); // Control bits how to handle ABN decoder signals.
	tmc4671_writeInt(motor, TMC4671_ABN_DECODER_PPR, 0x00000FA0);
	tmc4671_writeInt(motor, TMC4671_ABN_DECODER_COUNT, 0x00000EDD);
	tmc4671_writeInt(motor, TMC4671_ABN_DECODER_COUNT_N, 0x00000484);
	tmc4671_writeInt(motor, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000);

	// selections
	tmc4671_writeInt(motor, TMC4671_VELOCITY_SELECTION, 0x00000000);  // phi_m_abn
	tmc4671_writeInt(motor, TMC4671_POSITION_SELECTION, 0x00000000);  // phi_m_abn

	// set limits
	tmc4671_writeInt(motor, TMC4671_PIDOUT_UQ_UD_LIMITS, 0x00005A81);  // 23767
	tmc4671_writeInt(motor, TMC4671_PID_TORQUE_FLUX_LIMITS, 0x00000BB8);
	//tmc4671_writeInt(motor, TMC4671_PID_ACCELERATION_LIMIT, 0x000186A0);
	//tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_LIMIT, 0x00002710);
	tmc4671_writeInt(motor, TMC4671_PID_POSITION_LIMIT_LOW, 0x80000001);
	tmc4671_writeInt(motor, TMC4671_PID_POSITION_LIMIT_HIGH, 0x7FFFFFFF);

	// set PI parameter
	tmc4671_writeInt(motor, TMC4671_PID_FLUX_P_FLUX_I, 0x01000100); 				// P and I parameter for the flux regulator.
	tmc4671_writeInt(motor, TMC4671_PID_TORQUE_P_TORQUE_I, 0x01000100); 			// P and I parameter for the torque regulator.
	tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_P_VELOCITY_I, 0x27100063); 	// P and I parameter for the velocity regulator.
	tmc4671_writeInt(motor, TMC4671_PID_POSITION_P_POSITION_I, 0x00020001); 	// P parameter for the position regulator.
	tmc4671_writeInt(motor, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS, 0x00007FFF); 	// P parameter for the position regulator.
}

void encoderInit(uint8_t motor)
{
	dispString("do Encoder initialization (Mode 0)...");
	// do Encoder initialization (Mode 0)
	tmc4671_writeInt(motor, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);	 	// use UQ_UD_EXT for motion
	tmc4671_writeInt(motor, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, 0);	 	// set ABN_DECODER_PHI_E_OFFSET to zero
	tmc4671_writeInt(motor, TMC4671_PHI_E_SELECTION, 1);					// select phi_e_ext
	tmc4671_writeInt(motor, TMC4671_PHI_E_EXT, 0);							// set the "zero" angle
	tmc4671_writeInt(motor, TMC4671_UQ_UD_EXT, 6000);		// set an initialization voltage on UD_EXT (to the flux, not the torque!)
	tmc4671_writeInt(motor, TMC4671_PID_POSITION_ACTUAL, 0);				// critical: needed to set ABN_DECODER_COUNT to zero

	uint32 reply;

	dispString("clear DECODER_COUNT and check...");
	// clear DECODER_COUNT and check
	do
	{
		// set internal encoder value to zero
		tmc4671_writeInt(motor, TMC4671_ABN_DECODER_COUNT, 0);
		reply = tmc4671_readInt(motor, TMC4671_ABN_DECODER_COUNT);
	} while (reply != 0);

	// use encoder
	tmc4671_writeInt(motor, TMC4671_PHI_E_SELECTION, 0x00000003); 			// select phi_abn for rotor position angle phi_e

	// switch to velocity mode
	tmc4671_writeInt(motor, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000002);
	tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_TARGET, 10);


	dispString("and search the N-Channel to clear the actual position...");
	// and search the N-Channel to clear the actual position
	tmc4671_writeInt(motor, TMC4671_ABN_DECODER_PHI_E_PHI_M, 0);
	tmc4671_writeInt(motor, TMC4671_ABN_DECODER_MODE, 0x00001000);

	tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_TARGET, 20); // test

/*
	dispString("clear DECODER_COUNT_N and check...");
	// clear DECODER_COUNT_N and check
	do
	{
		tmc4671_writeInt(motor, TMC4671_ABN_DECODER_COUNT_N, 0);
		reply = tmc4671_readInt(motor, TMC4671_ABN_DECODER_COUNT_N);
	} while (reply != 0);

	tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_TARGET, 30); // test

	dispString("search N-channel...");
	// search N-channel
	do
	{
		reply = tmc4671_readInt(motor, TMC4671_ABN_DECODER_COUNT_N);
		tmc4671_writeInt(motor, TMC4671_PID_POSITION_ACTUAL, 0);
	} while (reply == 0);

	tmc4671_writeInt(motor, TMC4671_PID_VELOCITY_TARGET, 40); // test
*/

	dispString("drive to zero position using position mode...");
	// drive to zero position using position mode
	spi_writeInt(motor, TMC4671_PID_POSITION_TARGET, 0);
	spi_writeInt(motor, TMC4671_MODE_RAMP_MODE_MOTION, 3);

	dispString("In the end of the initEncoder......");
}


void tmc4671_EncoderInitializationMode0(u8 motor, u16 startVoltage)
{
	static uint16 last_Phi_E_Selection = 0;
	static uint32 last_UQ_UD_EXT = 0;
	static s16 last_PHI_E_EXT = 0;

	uint32 reply;

	dispString("In tmc4671_EncoderInitializationMode0 fuction...");

	tmc4671_writeInt(motor, TMC4671_ABN_DECODER_MODE, 0x00001000); // Control bits how to handle ABN decoder signals.

	tmc4671_writeInt(motor, TMC4671_MODE_RAMP_MODE_MOTION, 0x00000008);	 	// use UQ_UD_EXT for motion

	// save actual set values for PHI_E_SELECTION, UQ_UD_EXT, and PHI_E_EXT
	last_Phi_E_Selection = (u16) tmc4671_readRegister16BitValue(motor, TMC4671_PHI_E_SELECTION, BIT_0_TO_15);
	last_UQ_UD_EXT = (u32) tmc4671_readInt(motor, TMC4671_UQ_UD_EXT);
	last_PHI_E_EXT = (s16) tmc4671_readRegister16BitValue(motor, TMC4671_PHI_E_EXT, BIT_0_TO_15);

	// set ABN_DECODER_PHI_E_OFFSET to zero
	tmc4671_writeRegister16BitValue(motor, TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET, BIT_16_TO_31, 0);

	// select phi_e_ext
	tmc4671_writeRegister16BitValue(motor, TMC4671_PHI_E_SELECTION, BIT_0_TO_15, 1);

	// set an initialization voltage on UD_EXT (to the flux, not the torque!)
	tmc4671_writeRegister16BitValue(motor, TMC4671_UQ_UD_EXT, BIT_16_TO_31, 0);
	tmc4671_writeRegister16BitValue(motor, TMC4671_UQ_UD_EXT, BIT_0_TO_15, startVoltage);

	tmc4671_writeInt(motor, TMC4671_PID_POSITION_ACTUAL, 0);				// critical: needed to set ABN_DECODER_COUNT to zero

	// set the "zero" angle
	tmc4671_writeRegister16BitValue(motor, TMC4671_PHI_E_EXT, BIT_0_TO_15, 0);

	wait(1000);

	// set internal encoder value to zero
	tmc4671_writeInt(motor, TMC4671_ABN_DECODER_COUNT, 0);

	// switch back to last used UQ_UD_EXT setting
	tmc4671_writeInt(motor, TMC4671_UQ_UD_EXT, last_UQ_UD_EXT);

	// set PHI_E_EXT back to last value
	tmc4671_writeRegister16BitValue(motor, TMC4671_PHI_E_EXT, BIT_0_TO_15, last_PHI_E_EXT);

	// switch back to last used PHI_E_SELECTION setting
	tmc4671_writeRegister16BitValue(motor, TMC4671_PHI_E_SELECTION, BIT_0_TO_15, last_Phi_E_Selection);
}


void openLoopRun(uint8_t motor)
{
	// Type of motor &  PWM configuration
	tmc4671_writeInt(motor, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030001);
	tmc4671_writeInt(motor, TMC4671_PWM_POLARITIES, 0x00000000);
	tmc4671_writeInt(motor, TMC4671_PWM_SV_CHOP, 0x00000007);
	tmc4671_writeInt(motor, TMC4671_PWM_MAXCNT, 0x00000F9F);
	tmc4671_writeInt(motor, TMC4671_PWM_BBM_H_BBM_L, 0x00000505);

	tmc4671_writeInt(motor, TMC4671_PHI_E_SELECTION, 0x00000002);
	tmc4671_switchToMotionMode(motor, 8);

	tmc4671_writeInt(motor, TMC4671_OPENLOOP_ACCELERATION, 0x0000003C);
	//tmc4671_writeInt(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, 0x0000000A);
	tmc4671_writeInt(motor, TMC4671_OPENLOOP_VELOCITY_TARGET, -10);

	// set default acceleration and max velocity
	tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PID_ACCELERATION_LIMIT, 2000);
    tmc4671_writeInt(DEFAULT_MOTOR, TMC4671_PID_VELOCITY_LIMIT, 4000);

	tmc4671_writeInt(motor, TMC4671_UQ_UD_EXT, 0x000004BD);
	tmc4671_writeInt(motor, TMC4671_OPENLOOP_MODE, 0x00000000);

	tmc4671_writeInt(motor, TMC4671_PHI_E_EXT, 0x00000000);
}

//Close loop testing......
int main(void)
{
	// Start all initialization routines
	init();

	dispString("Before init......");
	TMC4671_init();
	dispString("After init......");
	readChipInfo();

	configureMotor(0);

	tmc4671_EncoderInitializationMode0(0, 6000);
	//wait(5000);

	int ticks = 65535*20;
	moveBy(0,&ticks);

	updateCnt = 0;
	// Main loop
	while(1)
	{

		periodicJob_InMain(systick_getTick());

		int actPos = tmc4671_readInt(0, TMC4671_PID_POSITION_ACTUAL);
		dispString("actPos is : ");
		dispInt(actPos);

		// Process TMCL communication
		tmcl_process();
	}

	return 0;
}


/*
 //open loop test......
int main(void)
{
	// Start all initialization routines
	init();

	dispString("Before init......");
	TMC4671_init();
	dispString("After init......");

	readChipInfo();

	dispString("Start open loop run......");

	openLoopRun(0);


    //The following values of registers can be monitored through USB2RTMI module and TMCL-IDE......
	int value = tmc4671_readInt(0, TMC4671_MODE_RAMP_MODE_MOTION);
	dispString("Motion mode is : ");
	dispInt(value);

	value = tmc4671_readInt(0, TMC4671_PWM_SV_CHOP);
	dispString("PWM SV CHOP is : ");
	dispInt(value);

	value = tmc4671_readInt(0, TMC4671_PHI_E_SELECTION);
	dispString("PHI_E_SEL is : ");
	dispInt(value);

	value = tmc4671_readInt(0, TMC4671_OPENLOOP_ACCELERATION);
	dispString("open acc is : ");
	dispInt(value);

	value = tmc4671_readInt(0, TMC4671_OPENLOOP_VELOCITY_TARGET);
	dispString("open vel is : ");
	dispInt(value);


	// Main loop
	while(1)
	{

		wait(1000);
		//dispString("MOVing...");
		int actPos = tmc4671_readInt(0, TMC4671_PID_POSITION_ACTUAL);
		dispString("actPos is : ");
		dispInt(actPos);

		// Process TMCL communication
		tmcl_process();
	}

	return 0;
}
*/
