/*
 * StepperMotor.h
 *
 *  Created on: 29.9.2017
 *      Author: Saana Vallius
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "DigitalIoPin.h"
#include <cstdint>

enum Direction {
	MIN = false, MAX = true //DigitalIoPi dir created with inverted output
};

class Motor {
private:
	DigitalIoPin *dir;
	DigitalIoPin *step;
	double axisLengthMm;
	const int initSpeed = 600; 	// holds initial axis speed setting
	int axisLengthSteps = 0;	// updated in calibration
	int axisPositionSteps = -1;	// keeps track of current axis/motor position, initialized in calibration
	int stepCounter = 0;		// used to track actual moved steps in RIT_IRQ interrupt
	double stepsPerMm = 0;

	uint32_t bitLimitMin;		// bit mask for MIN limit switch
	uint32_t bitLimitMax;		// bit mask for MAX limit switch
	bool isCalibrated;
	Direction direction = MIN;	// current moving direction
	bool lastStepState = false; //keeps track of step pulse (high/low)

public:
	Motor(int stepPort = 100, int stepPin = 100, int dirPort = 100, int dirPin = 100, bool invertDir = true,double axLenMm=1); //Port 0, pin 24 original values for 1 motor

	void stepOnce();
	void manualStep(int speedHz);

	void setDir(Direction dirEnum);
	Direction getDir();

	uint32_t calibrate(uint32_t includeBitMask = 15);
	bool calibrated();

	int getPositionSteps();
	double getPositionMm();

	double getAxisLengthMm();

	void resetStepCounter();
	int getStepCounter();

	double mmToSteps(double mm);
	double stepsToMm(int steps);

	void setLimitBitMin(uint32_t bit);
	void setLimitBitMax(uint32_t bit);
	uint32_t getLimitBit();
};

#endif /* MOTOR_H_ */
