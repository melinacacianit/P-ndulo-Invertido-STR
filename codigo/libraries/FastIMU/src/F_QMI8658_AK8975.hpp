#pragma once

#ifndef _F_QMI8658_AK8975_H_
#define _F_QMI8658_AK8975_H_

#include "F_QMI8658.hpp"
#include "F_AK8975.hpp"

class QMI8658_AK8975 : public IMUBase {
public:
	explicit QMI8658_AK8975(TwoWire& wire = Wire) : IMU(wire), MAG(wire) {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address) override {
		int e = IMU.init(cal, address);
		if (e) { return e; }
		e = MAG.init(cal, 0x0C);
		if (e) { return -2; }
		return 0;
	}

	void update() override {
		IMU.update();
		MAG.update();
	}
	void getAccel(AccelData* out) override {
		IMU.getAccel(out);
	}
	void getGyro(GyroData* out) override{
		IMU.getGyro(out);
	}
	void getMag(MagData* out) override {
		MAG.getMag(out);
	}
	void getQuat(Quaternion* out) override {};
	float getTemp() override { return IMU.getTemp();; };

	int setGyroRange(int range) override	{ return IMU.setGyroRange(range); }
	int setAccelRange(int range) override	{ return IMU.setAccelRange(range); }
	int setIMUGeometry(int index) override 	{ return IMU.setIMUGeometry(index); }

	void calibrateAccelGyro(calData* cal) override {
		IMU.calibrateAccelGyro(cal);
	}
	virtual void calibrateMag(calData* cal) override {
		MAG.calibrateMag(cal);
	}

	bool hasMagnetometer() override {
		return true;
	}
	bool hasTemperature() override {
		return true;
	}
	bool hasQuatOutput() override {
		return false;
	}

	String IMUName() override {
		return "QMI8658 + AK8975";
	}
	String IMUType() override {
		return "QMI8658_AK8975";
	}
	String IMUManufacturer() override {
		return "AKM";
	}

private:
	QMI8658 IMU;
	AK8975 MAG;
	
	calData calibration;
	uint8_t IMUAddress;
};
#endif /* _F_QMI8658_AK8975_H_ */
