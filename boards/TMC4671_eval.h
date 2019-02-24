#ifndef TMC4671_EVAL_H
#define TMC4671_EVAL_H

/*
u8 tmc4671_readwriteByte(u8 motor, u8 data, u8 lastTransfer);
static uint32 rotate(uint8 motor, int32 velocity);
static uint32 right(uint8 motor, int32 velocity);
static uint32 left(uint8 motor, int32 velocity);
static uint32 stop(uint8 motor);
static uint32 moveTo(uint8 motor, int32 position);
static uint32 moveBy(uint8 motor, int32 *ticks);
static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value);
static uint32 getMeasuredSpeed(uint8 motor, int32 *value);
static void TMC4671_periodicJob(uint32 actualSystick);
static void writeRegister(u8 motor, uint8 address, int32 value);
static void readRegister(u8 motor, uint8 address, int32 *value);
static uint32 SAP(uint8 type, uint8 motor, int32 value);
static uint32 GAP(uint8 type, uint8 motor, int32 *value);
static uint32 userFunction(uint8 type, uint8 motor, int32 *value);
static void enableDriver(DriverState state);
static void deInit(void);
static uint8 reset();
static uint8 restore();
static void checkErrors(uint32 tick);
void TMC4671_init(void);
*/

#endif /* TMC4671_EVAL_H */
