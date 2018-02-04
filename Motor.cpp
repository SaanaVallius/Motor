/*
 * Motor.cpp
 *
 *  Created on: 29.9.2017
 *      Author: Saana Vallius
 */

#include "Motor.h"
#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif
#include "LimitSwitches.h"
#include "DigitalIoPin.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ITM_write.h"
#include <stdlib.h>
#include <stdio.h>
#include <cstdint>
#include <math.h>

Motor::Motor(int stepPort, int stepPin, int dirPort, int dirPin,
		bool invertDir,double axLenMm) {
	step = new DigitalIoPin(stepPort, stepPin, DigitalIoPin::output, true); // 0 24
	dir = new DigitalIoPin(dirPort, dirPin, DigitalIoPin::output, invertDir); // 1 0

	//Make sure that the first output value is false
	step->write(false);

	axisLengthMm = axLenMm;

}

void Motor::manualStep(int speedHz) {
	stepOnce();
	vTaskDelay(configTICK_RATE_HZ / 1000); /// speedHz / 2);
}

//Toggle output using last output value
void Motor::stepOnce() {

	step->write(!lastStepState);
	lastStepState = !lastStepState;

	//update position
	if (direction == MAX)
		axisPositionSteps++;
	else
		axisPositionSteps--;

	//update resettable step counter, used measuring steps run in RIT
	stepCounter++;
}

uint32_t Motor::calibrate(uint32_t includeBitMask) {
	axisLengthSteps = 0;
	isCalibrated = false;
	stepsPerMm = 0;
	int calibrationRound = 1;
	int averageOf = 1; //How many calibration rounds in total

	//Set initial direction towards home (0,0) for calibration
	dir->write(MIN);

	//!CHECK THAT NO LIMIT IS PRESSED AT START!

	//While no chosen limit switches are toggled
	//run to min direction until min limit is hit

	uint32_t limitBits = xEventGroupGetBits(xegLimitSwitches);

	while (!(limitBits & includeBitMask)) {
		manualStep(initSpeed);
		limitBits = xEventGroupGetBits(xegLimitSwitches);
	}

	//When a switch is hit, the bit that's high defines left switch for this motor
	//Other motor's limit switches can be ignored by using includeBitMask
	bitLimitMin = (xEventGroupGetBits(xegLimitSwitches) & includeBitMask);
	includeBitMask &= ((~bitLimitMin) & 15); //added extra 1111 mask to prevent setting higher than first four bits

	//give some breathing air for other tasks too
	taskYIELD();

	//Set direction towards max
	dir->write(MAX);

	//Run to max direction until max limit is hit
	//First calibration round for measuring axis length in steps
	limitBits = xEventGroupGetBits(xegLimitSwitches);

	while (!(limitBits & includeBitMask)) {
		manualStep(initSpeed);
		axisLengthSteps++;
		limitBits = xEventGroupGetBits(xegLimitSwitches);
	}

	//Bit-wise AND of set limit bits and includeBitMask
	bitLimitMax = (xEventGroupGetBits(xegLimitSwitches) & includeBitMask);
	includeBitMask &= ((~bitLimitMax) & 15);

	//give some breathing air for other tasks too
	taskYIELD();

	calibrationRound = 2;

	//Set direction towards min
	dir->write(MIN);

	while (calibrationRound <= averageOf) {
		//On odd rounds go towards max, on even rounds go towards min
		if (calibrationRound % 2 > 0) {

			while (!(xEventGroupGetBits(xegLimitSwitches) & bitLimitMax)) {
				manualStep(initSpeed);

				//Increment step counter
				axisLengthSteps++;
			}
			//Start next round and change direction to MIN
			calibrationRound++;
			setDir(MIN);

			//give some breathing air for other tasks too
			//taskYIELD();

			//Even rounds
		} else if (calibrationRound % 2 == 0) {
			while (!(xEventGroupGetBits(xegLimitSwitches) & bitLimitMin)) {

				manualStep(initSpeed);
				//Increment step counter
				axisLengthSteps++;

			}
			//Start next round and change direction to MAX
			calibrationRound++;
			setDir(MAX);

			//give some breathing air for other tasks too
			//taskYIELD();
		}

	}

	//Axis lenght in steps is calculated by the average step count of one calibration round
	axisLengthSteps = axisLengthSteps / averageOf;

	stepsPerMm = 1.0 * axisLengthSteps / axisLengthMm;
	isCalibrated = true;

	//update axis/motor position to opposite end of current direction
	if (direction == MAX) {
		//motor should be in MIN end because of calibration logic :)
		axisPositionSteps = 0;
	} else {
		axisPositionSteps = axisLengthSteps;
	}

	return includeBitMask;
}

//Returns true if calibration is finished
bool Motor::calibrated() {
	return isCalibrated;
}

//Returns the required movement lenght in steps
double Motor::mmToSteps(double mm) {
	return round(mm * stepsPerMm);
}

double Motor::stepsToMm(int steps) {
	return (steps / stepsPerMm);
}

//Set direction pin
void Motor::setDir(Direction dirEnum) {
	direction = dirEnum;
	dir->write(direction);
}

Direction Motor::getDir(){
	return direction;
}

int Motor::getPositionSteps() {
	return axisPositionSteps;
}

double Motor::getPositionMm() {
	return (axisPositionSteps / stepsPerMm);
}

double Motor::getAxisLengthMm() {
	return axisLengthMm;
}

void Motor::resetStepCounter() {
	stepCounter = 0;
}

int Motor::getStepCounter() {
	return stepCounter;
}

//Get limit bit of limit switch in current direction
uint32_t Motor::getLimitBit() {

	if (direction == MIN) {
		return bitLimitMin;

	} else {
		return bitLimitMax;
	}

}

//Set min limit bit
void Motor::setLimitBitMin(uint32_t bit) {
	bitLimitMin = bit;
}

//Set max limit bit
void Motor::setLimitBitMax(uint32_t bit) {
	bitLimitMax = bit;
}
