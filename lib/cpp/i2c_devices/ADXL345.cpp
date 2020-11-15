/*
Sparkfun's ADXL345 Library Main Source File
SparkFun_ADXL345.cpp

E.Robert @ SparkFun Electronics
Created: Jul 13, 2016
Updated: Sep 06, 2016

Modified Bildr ADXL345 Source File @ http://code.bildr.org/download/959.zip
to support both I2C and SPI Communication

Hardware Resources:
- Arduino Development Board
- SparkFun Triple Access Accelerometer ADXL345

Development Environment Specifics:
Arduino 1.6.8
SparkFun Triple Axis Accelerometer Breakout - ADXL345
Arduino Uno
*/

#include <ADXL345.h>

#define ADXL345_DEVICE (0x53)    // Device Address for ADXL345
#define ADXL345_TO_READ (6)      // Number of uint8_ts Read - Two uint8_ts Per Axis

ADXL345::ADXL345() {
	status = ADXL345_OK;
	error_code = ADXL345_NO_ERROR;
	
	gains[0] = 0.00376390;		// Original gain 0.00376390 
	gains[1] = 0.00376009;		// Original gain 0.00376009
	gains[2] = 0.00349265;		// Original gain 0.00349265
}

ADXL345::~ADXL345()
{
  if (fI2C >= 0) {
    close(fI2C);
    fI2C = -1;
  }
}

void ADXL345::powerOn(){	
	fI2C = open("/dev/i2c-1", O_RDWR);
	if (fI2C < 0) {
		// Could not open the file
		return;
	}
	if (ioctl(fI2C, I2C_SLAVE, ADXL345_DEVICE) < 0) {
		// Could not open the device on the bus
		fI2C = -1;
		return;
	}
	//ADXL345 TURN ON - CONFIGURE TO MAX RATE
	writeTo(ADXL345_POWER_CTL, 0);	// Wakeup     
	writeTo(ADXL345_POWER_CTL, 16);	// Auto_Sleep
	writeTo(ADXL345_POWER_CTL, 8);	// Measure    (Configure the power-saving features (0X2D) register to be in write mode (0x08))
	setLowPower(false);
	set_bw(ADXL345_BW_1600);
	setRangeSetting(2); 		// set default range to +/- 16g
	writeTo(ADXL345_FIFO_CTL, 0x08);// Configure the sensor to bypass the FIFO by setting the FIFO_CTL (0x38) register to be (0x08).
}


/*********************** READING ACCELERATION ***********************/
/*    Reads Acceleration into Three Variables:  x, y and z          */

void ADXL345::readAccel(int16_t *xyz){
	readAccel(&xyz[0], &xyz[1], &xyz[2]);
}

void ADXL345::readAccel(int16_t *x, int16_t *y, int16_t *z) {
	readFrom(ADXL345_DATAX0, ADXL345_TO_READ, _buff);	// Read Accel Data from ADXL345
	
	// Each Axis @ All g Ranges: 10 Bit Resolution (2 uint8_ts)
	*x = (int16_t)((((uint16_t)_buff[1]) << 8) | _buff[0]);
	*y = (int16_t)((((uint16_t)_buff[3]) << 8) | _buff[2]);
	*z = (int16_t)((((uint16_t)_buff[5]) << 8) | _buff[4]);
}

void ADXL345::get_Gxyz(double *xyz){
	int i;
	int16_t xyz_int[3];
	readAccel(xyz_int);
	for(i=0; i<3; i++){
		xyz[i] = xyz_int[i] * gains[i];
	}
}

/***************** WRITES VALUE TO ADDRESS REGISTER *****************/
void ADXL345::writeTo(uint8_t address, uint8_t val) { 
	int err;
	err = i2c_smbus_write_byte_data(fI2C, address, val); // see documentation at https://www.kernel.org/doc/Documentation/i2c/dev-interface
	if (err < 0) {
        	err = errno; // currently this error code is just ignored
	}
}

/************************ READING NUM uint8_tS *************************/
/*    Reads Num uint8_ts. Starts from Address Reg to _buff Array        */
void ADXL345::readFrom(uint8_t address, int num, uint8_t * _buff) {
	int err;

	err = i2c_smbus_read_i2c_block_data(fI2C, address, num, _buff);  // see documentation at https://www.kernel.org/doc/Documentation/i2c/smbus-protocol
	if (err < 0) {
		status = ADXL345_ERROR;
		error_code = ADXL345_READ_ERROR;
	}
}

/*************************** RANGE SETTING **************************/
/*          ACCEPTABLE VALUES: 2g, 4g, 8g, 16g ~ GET & SET          */
void ADXL345::getRangeSetting(uint8_t* rangeSetting) {
	uint8_t _b;
	readFrom(ADXL345_DATA_FORMAT, 1, &_b);
	*rangeSetting = _b & 0b00000011;
}

void ADXL345::setRangeSetting(int val) {
	uint8_t _s;
	uint8_t _b;
	
	switch (val) {
		case 2:  
			_s = 0b00000000; 
			break;
		case 4:  
			_s = 0b00000001; 
			break;
		case 8:  
			_s = 0b00000010; 
			break;
		case 16: 
			_s = 0b00000011; 
			break;
		default: 
			_s = 0b00000000;
	}
	readFrom(ADXL345_DATA_FORMAT, 1, &_b);
	_s |= (_b & 0b11101100);
	writeTo(ADXL345_DATA_FORMAT, _s);
}

/*************************** SELF_TEST BIT **************************/
/*                            ~ GET & SET                           */
bool ADXL345::getSelfTestBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 7);
}

// If Set (1) Self-Test Applied. Electrostatic Force exerted on the sensor
//  causing a shift in the output data.
// If Set (0) Self-Test Disabled.
void ADXL345::setSelfTestBit(bool selfTestBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}

/*************************** SPI BIT STATE **************************/
/*                           ~ GET & SET                            */
bool ADXL345::getSpiBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 6);
}

// If Set (1) Puts Device in 3-wire Mode
// If Set (0) Puts Device in 4-wire SPI Mode
void ADXL345::setSpiBit(bool spiBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
}

/*********************** INT_INVERT BIT STATE ***********************/
/*                           ~ GET & SET                            */
bool ADXL345::getInterruptLevelBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 5);
}

// If Set (0) Sets the Interrupts to Active HIGH
// If Set (1) Sets the Interrupts to Active LOW
void ADXL345::setInterruptLevelBit(bool interruptLevelBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}

/************************* FULL_RES BIT STATE ***********************/
/*                           ~ GET & SET                            */
bool ADXL345::getFullResBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 3);
}

// If Set (1) Device is in Full Resolution Mode: Output Resolution Increase with G Range
//  Set by the Range Bits to Maintain a 4mg/LSB Scale Factor
// If Set (0) Device is in 10-bit Mode: Range Bits Determine Maximum G Range
//  And Scale Factor
void ADXL345::setFullResBit(bool fullResBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}

/*************************** JUSTIFY BIT STATE **************************/
/*                           ~ GET & SET                            */
bool ADXL345::getJustifyBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 2);
}

// If Set (1) Selects the Left Justified Mode
// If Set (0) Selects Right Justified Mode with Sign Extension
void ADXL345::setJustifyBit(bool justifyBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}

/*********************** THRESH_TAP uint8_t VALUE **********************/
/*                          ~ SET & GET                             */
// Should Set Between 0 and 255
// Scale Factor is 62.5 mg/LSB
// A Value of 0 May Result in Undesirable Behavior
void ADXL345::setTapThreshold(int tapThreshold) {
	tapThreshold = constrain(tapThreshold,0,255);
	uint8_t _b = uint8_t (tapThreshold);
	writeTo(ADXL345_THRESH_TAP, _b);  
}

// Return Value Between 0 and 255
// Scale Factor is 62.5 mg/LSB
int ADXL345::getTapThreshold() {
	uint8_t _b;
	readFrom(ADXL345_THRESH_TAP, 1, &_b);  
	return int (_b);
}

/****************** GAIN FOR EACH AXIS IN Gs / COUNT *****************/
/*                           ~ SET & GET                            */
void ADXL345::setAxisGains(double *_gains){
	int i;
	for(i = 0; i < 3; i++){
		gains[i] = _gains[i];
	}
}
void ADXL345::getAxisGains(double *_gains){
	int i;
	for(i = 0; i < 3; i++){
		_gains[i] = gains[i];
	}
}

/********************* OFSX, OFSY and OFSZ uint8_tS ********************/
/*                           ~ SET & GET                            */
// OFSX, OFSY and OFSZ: User Offset Adjustments in Twos Complement Format
// Scale Factor of 15.6mg/LSB
void ADXL345::setAxisOffset(int x, int y, int z) {
	writeTo(ADXL345_OFSX, uint8_t (x));  
	writeTo(ADXL345_OFSY, uint8_t (y));  
	writeTo(ADXL345_OFSZ, uint8_t (z));  
}

void ADXL345::getAxisOffset(int* x, int* y, int*z) {
	uint8_t _b;
	readFrom(ADXL345_OFSX, 1, &_b);  
	*x = int (_b);
	readFrom(ADXL345_OFSY, 1, &_b);  
	*y = int (_b);
	readFrom(ADXL345_OFSZ, 1, &_b);  
	*z = int (_b);
}

/****************************** DUR uint8_t ****************************/
/*                           ~ SET & GET                            */
// DUR uint8_t: Contains an Unsigned Time Value Representing the Max Time 
//  that an Event must be Above the THRESH_TAP Threshold to qualify 
//  as a Tap Event
// The scale factor is 625µs/LSB
// Value of 0 Disables the Tap/Double Tap Funcitons. Max value is 255.
void ADXL345::setTapDuration(int tapDuration) {
	tapDuration = constrain(tapDuration,0,255);
	uint8_t _b = uint8_t (tapDuration);
	writeTo(ADXL345_DUR, _b);  
}

int ADXL345::getTapDuration() {
	uint8_t _b;
	readFrom(ADXL345_DUR, 1, &_b);  
	return int (_b);
}

/************************** LATENT REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains Unsigned Time Value Representing the Wait Time from the Detection
//  of a Tap Event to the Start of the Time Window (defined by the Window 
//  Register) during which a possible Second Tap Even can be Detected.
// Scale Factor is 1.25ms/LSB. 
// A Value of 0 Disables the Double Tap Function.
// It Accepts a Maximum Value of 255.
void ADXL345::setDoubleTapLatency(int doubleTapLatency) {
	uint8_t _b = uint8_t (doubleTapLatency);
	writeTo(ADXL345_LATENT, _b);  
}

int ADXL345::getDoubleTapLatency() {
	uint8_t _b;
	readFrom(ADXL345_LATENT, 1, &_b);  
	return int (_b);
}

/************************** WINDOW REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains an Unsigned Time Value Representing the Amount of Time 
//  After the Expiration of the Latency Time (determined by Latent register)
//  During which a Second Valid Tape can Begin. 
// Scale Factor is 1.25ms/LSB. 
// Value of 0 Disables the Double Tap Function. 
// It Accepts a Maximum Value of 255.
void ADXL345::setDoubleTapWindow(int doubleTapWindow) {
	doubleTapWindow = constrain(doubleTapWindow,0,255);
	uint8_t _b = uint8_t (doubleTapWindow);
	writeTo(ADXL345_WINDOW, _b);  
}

int ADXL345::getDoubleTapWindow() {
	uint8_t _b;
	readFrom(ADXL345_WINDOW, 1, &_b);  
	return int (_b);
}

/*********************** THRESH_ACT REGISTER ************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Activity.
// Data Format is Unsigned, so the Magnitude of the Activity Event is Compared 
//  with the Value is Compared with the Value in the THRESH_ACT Register. 
// The Scale Factor is 62.5mg/LSB. 
// Value of 0 may Result in Undesirable Behavior if the Activity Interrupt Enabled. 
// It Accepts a Maximum Value of 255.
void ADXL345::setActivityThreshold(int activityThreshold) {
	activityThreshold = constrain(activityThreshold,0,255);
	uint8_t _b = uint8_t (activityThreshold);
	writeTo(ADXL345_THRESH_ACT, _b);  
}

// Gets the THRESH_ACT uint8_t
int ADXL345::getActivityThreshold() {
	uint8_t _b;
	readFrom(ADXL345_THRESH_ACT, 1, &_b);  
	return int (_b);
}

/********************** THRESH_INACT REGISTER ***********************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Inactivity.
// The Data Format is Unsigned, so the Magnitude of the INactivity Event is 
//  Compared with the value in the THRESH_INACT Register. 
// Scale Factor is 62.5mg/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Inactivity Interrupt Enabled. 
// It Accepts a Maximum Value of 255.
void ADXL345::setInactivityThreshold(int inactivityThreshold) {
	inactivityThreshold = constrain(inactivityThreshold,0,255);
	uint8_t _b = uint8_t (inactivityThreshold);
	writeTo(ADXL345_THRESH_INACT, _b);  
}

int ADXL345::getInactivityThreshold() {
	uint8_t _b;
	readFrom(ADXL345_THRESH_INACT, 1, &_b);  
	return int (_b);
}

/*********************** TIME_INACT RESIGER *************************/
/*                          ~ SET & GET                             */
// Contains an Unsigned Time Value Representing the Amount of Time that
//  Acceleration must be Less Than the Value in the THRESH_INACT Register
//  for Inactivity to be Declared. 
// Uses Filtered Output Data* unlike other Interrupt Functions
// Scale Factor is 1sec/LSB. 
// Value Must Be Between 0 and 255. 
void ADXL345::setTimeInactivity(int timeInactivity) {
	timeInactivity = constrain(timeInactivity,0,255);
	uint8_t _b = uint8_t (timeInactivity);
	writeTo(ADXL345_TIME_INACT, _b);  
}

int ADXL345::getTimeInactivity() {
	uint8_t _b;
	readFrom(ADXL345_TIME_INACT, 1, &_b);  
	return int (_b);
}

/*********************** THRESH_FF Register *************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value, in Unsigned Format, for Free-Fall Detection
// The Acceleration on all Axes is Compared with the Value in THRES_FF to
//  Determine if a Free-Fall Event Occurred. 
// Scale Factor is 62.5mg/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Free-Fall interrupt Enabled.
// Accepts a Maximum Value of 255.
void ADXL345::setFreeFallThreshold(int freeFallThreshold) {
	freeFallThreshold = constrain(freeFallThreshold,0,255);
	uint8_t _b = uint8_t (freeFallThreshold);
	writeTo(ADXL345_THRESH_FF, _b);  
}

int ADXL345::getFreeFallThreshold() {
	uint8_t _b;
	readFrom(ADXL345_THRESH_FF, 1, &_b);  
	return int (_b);
}

/************************ TIME_FF Register **************************/
/*                          ~ SET & GET                             */
// Stores an Unsigned Time Value Representing the Minimum Time that the Value 
//  of all Axes must be Less Than THRES_FF to Generate a Free-Fall Interrupt.
// Scale Factor is 5ms/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Free-Fall Interrupt Enabled.
// Accepts a Maximum Value of 255.
void ADXL345::setFreeFallDuration(int freeFallDuration) {
	freeFallDuration = constrain(freeFallDuration,0,255);  
	uint8_t _b = uint8_t (freeFallDuration);
	writeTo(ADXL345_TIME_FF, _b);  
}

int ADXL345::getFreeFallDuration() {
	uint8_t _b;
	readFrom(ADXL345_TIME_FF, 1, &_b);  
	return int (_b);
}

/************************** ACTIVITY BITS ***************************/
/*                                                                  */
bool ADXL345::isActivityXEnabled() {  
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 6); 
}
bool ADXL345::isActivityYEnabled() {  
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 5); 
}
bool ADXL345::isActivityZEnabled() {  
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 4); 
}
bool ADXL345::isInactivityXEnabled() {  
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 2); 
}
bool ADXL345::isInactivityYEnabled() {  
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 1); 
}
bool ADXL345::isInactivityZEnabled() {  
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 0); 
}

void ADXL345::setActivityX(bool state) {  
	setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state); 
}
void ADXL345::setActivityY(bool state) {  
	setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state); 
}
void ADXL345::setActivityZ(bool state) {  
	setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state); 
}
void ADXL345::setActivityXYZ(bool stateX, bool stateY, bool stateZ) {
	setActivityX(stateX);
	setActivityY(stateY);
	setActivityZ(stateZ);
}
void ADXL345::setInactivityX(bool state) {  
	setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state); 
}
void ADXL345::setInactivityY(bool state) {  
	setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state); 
}
void ADXL345::setInactivityZ(bool state) {  
	setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state); 
}
void ADXL345::setInactivityXYZ(bool stateX, bool stateY, bool stateZ) {
	setInactivityX(stateX);
	setInactivityY(stateY);
	setInactivityZ(stateZ);
}

bool ADXL345::isActivityAc() { 
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 7); 
}
bool ADXL345::isInactivityAc(){ 
	return getRegisterBit(ADXL345_ACT_INACT_CTL, 3); 
}

void ADXL345::setActivityAc(bool state) {  
	setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state); 
}
void ADXL345::setInactivityAc(bool state) {  
	setRegisterBit(ADXL345_ACT_INACT_CTL, 3, state); 
}

/************************* SUPPRESS BITS ****************************/
/*                                                                  */
bool ADXL345::getSuppressBit(){ 
	return getRegisterBit(ADXL345_TAP_AXES, 3); 
}
void ADXL345::setSuppressBit(bool state) {  
	setRegisterBit(ADXL345_TAP_AXES, 3, state); 
}

/**************************** TAP BITS ******************************/
/*                                                                  */
bool ADXL345::isTapDetectionOnX(){ 
	return getRegisterBit(ADXL345_TAP_AXES, 2); 
}
void ADXL345::setTapDetectionOnX(bool state) {  
	setRegisterBit(ADXL345_TAP_AXES, 2, state); 
}
bool ADXL345::isTapDetectionOnY(){ 
	return getRegisterBit(ADXL345_TAP_AXES, 1); 
}
void ADXL345::setTapDetectionOnY(bool state) {  
	setRegisterBit(ADXL345_TAP_AXES, 1, state); 
}
bool ADXL345::isTapDetectionOnZ(){ 
	return getRegisterBit(ADXL345_TAP_AXES, 0); 
}
void ADXL345::setTapDetectionOnZ(bool state) {  
	setRegisterBit(ADXL345_TAP_AXES, 0, state); 
}

void ADXL345::setTapDetectionOnXYZ(bool stateX, bool stateY, bool stateZ) {
	setTapDetectionOnX(stateX);
	setTapDetectionOnY(stateY);
	setTapDetectionOnZ(stateZ);
}

bool ADXL345::isActivitySourceOnX(){ 
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 6); 
}
bool ADXL345::isActivitySourceOnY(){ 
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 5); 
}
bool ADXL345::isActivitySourceOnZ(){ 
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 4); 
}

bool ADXL345::isTapSourceOnX(){ 
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 2); 
}
bool ADXL345::isTapSourceOnY(){ 
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 1); 
}
bool ADXL345::isTapSourceOnZ(){ 
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 0); 
}

/*************************** ASLEEP BIT *****************************/
/*                                                                  */
bool ADXL345::isAsleep(){ 
	return getRegisterBit(ADXL345_ACT_TAP_STATUS, 3); 
}

/************************** LOW POWER BIT ***************************/
/*                                                                  */
bool ADXL345::isLowPower(){ 
	return getRegisterBit(ADXL345_BW_RATE, 4); 
}
void ADXL345::setLowPower(bool state) {  
	setRegisterBit(ADXL345_BW_RATE, 4, state); 
}

/*************************** RATE BITS ******************************/
/*                                                                  */
double ADXL345::getRate(){
	uint8_t _b;
	readFrom(ADXL345_BW_RATE, 1, &_b);
	_b &= 0b00001111;
	return (pow(2,((int) _b)-6)) * 6.25;
}

void ADXL345::setRate(double rate){
	uint8_t _b,_s;
	int v = (int) (rate / 6.25);
	int r = 0;
	while (v >>= 1)
	{
		r++;
	}
	if (r <= 9) { 
		readFrom(ADXL345_BW_RATE, 1, &_b);
		_s = (uint8_t) (r + 6) | (_b & 0b11110000);
		writeTo(ADXL345_BW_RATE, _s);
	}
}

/*************************** BANDWIDTH ******************************/
/*                          ~ SET & GET                             */
void ADXL345::set_bw(uint8_t bw_code){
	if((bw_code < ADXL345_BW_0_05) || (bw_code > ADXL345_BW_1600)){
		status = false;
		error_code = ADXL345_BAD_ARG;
	}
	else{
		writeTo(ADXL345_BW_RATE, bw_code);
	}
}

uint8_t ADXL345::get_bw_code(){
	uint8_t bw_code;
	readFrom(ADXL345_BW_RATE, 1, &bw_code);
	return bw_code;
}




/************************* TRIGGER CHECK  ***************************/
/*                                                                  */
// Check if Action was Triggered in Interrupts
// Example triggered(interrupts, ADXL345_SINGLE_TAP);
bool ADXL345::triggered(uint8_t interrupts, int mask){
	return ((interrupts >> mask) & 1);
}

/*
 ADXL345_DATA_READY
 ADXL345_SINGLE_TAP
 ADXL345_DOUBLE_TAP
 ADXL345_ACTIVITY
 ADXL345_INACTIVITY
 ADXL345_FREE_FALL
 ADXL345_WATERMARK
 ADXL345_OVERRUNY
 */


uint8_t ADXL345::getInterruptSource() {
	uint8_t _b;
	readFrom(ADXL345_INT_SOURCE, 1, &_b);
	return _b;
}

bool ADXL345::getInterruptSource(uint8_t interruptBit) {
	return getRegisterBit(ADXL345_INT_SOURCE,interruptBit);
}

bool ADXL345::getInterruptMapping(uint8_t interruptBit) {
	return getRegisterBit(ADXL345_INT_MAP,interruptBit);
}

/*********************** INTERRUPT MAPPING **************************/
/*         Set the Mapping of an Interrupt to pin1 or pin2          */
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void ADXL345::setInterruptMapping(uint8_t interruptBit, bool interruptPin) {
	setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}

void ADXL345::setImportantInterruptMapping(int single_tap, int double_tap, int free_fall, int activity, int inactivity) {
	if(single_tap == 1) {
		setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );}
	else if(single_tap == 2) {
		setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT2_PIN );}

	if(double_tap == 1) {
		setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );}
	else if(double_tap == 2) {
		setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT2_PIN );}

	if(free_fall == 1) {
		setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,   ADXL345_INT1_PIN );}
	else if(free_fall == 2) {
		setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,   ADXL345_INT2_PIN );}

	if(activity == 1) {
		setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT1_PIN );}
	else if(activity == 2) {
		setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,   ADXL345_INT2_PIN );}

	if(inactivity == 1) {
		setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );}
	else if(inactivity == 2) {
		setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT2_PIN );}
}

bool ADXL345::isInterruptEnabled(uint8_t interruptBit) {
	return getRegisterBit(ADXL345_INT_ENABLE,interruptBit);
}

void ADXL345::setInterrupt(uint8_t interruptBit, bool state) {
	setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

void ADXL345::singleTapINT(bool status) {
	if(status) {
		setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
	}
	else {
		setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 0);
	}
}
void ADXL345::doubleTapINT(bool status) {
	if(status) {
		setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
	}
	else {
		setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 0);		
	}	
}
void ADXL345::FreeFallINT(bool status) {
	if(status) {
		setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
	}
	else {
		setInterrupt( ADXL345_INT_FREE_FALL_BIT,  0);
	}	
}
void ADXL345::ActivityINT(bool status) {
	if(status) {
		setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
	}
	else {
		setInterrupt( ADXL345_INT_ACTIVITY_BIT,   0);
	}
}
void ADXL345::InactivityINT(bool status) {
	if(status) {
		setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
	}
	else {
		setInterrupt( ADXL345_INT_INACTIVITY_BIT, 0);
	}
}

void ADXL345::setRegisterBit(uint8_t regAdress, int bitPos, bool state) {
	uint8_t _b;
	readFrom(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);  // Forces nth Bit of _b to 1. Other Bits Unchanged.  
	} 
	else {
		_b &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
	}
	writeTo(regAdress, _b);  
}

bool ADXL345::getRegisterBit(uint8_t regAdress, int bitPos) {
	uint8_t _b;
	readFrom(regAdress, 1, &_b);
	return ((_b >> bitPos) & 1);
}

/********************************************************************/
/*                                                                  */
// Print Register Values to Serial Output =
// Can be used to Manually Check the Current Configuration of Device
void ADXL345::printAllRegister() {
	uint8_t _b;
	printf("0x00: ");
	readFrom(0x00, 1, &_b);
	print_uint8_t(_b);
	printf("\n");
	int i;
	for (i=29;i<=57;i++){
		printf("%#02x: ", i);
		readFrom(i, 1, &_b);
		print_uint8_t(_b);
		printf("\n");  
	}
}

void print_uint8_t(uint8_t val){
	int i;
	printf("B");
	for(i=7; i>=0; i--){
		printf("%d", (val >> i & 1) ? 1 : 0);
	}
}
