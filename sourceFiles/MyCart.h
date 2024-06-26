#pragma once

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"


using namespace chrono;

class MyCart {

private:

	ChSystemNSC system;

	ChVector3d initPosition;
	ChVector3d initVelocity;
	ChVector3d initAcceleration;

	// Body elements and parameters

	std::shared_ptr<ChBody> cartBody;

	double xBodySize = 2;
	double yBodySize = 0.5;
	double zBodySize = 1;
	double bodyDensity = 100;

	std::shared_ptr<ChContactMaterialNSC> bodyMat = chrono_types::make_shared<ChContactMaterialNSC>();
	ChColor bodyColor = ChColor(0.2f, 0.3f, 1.0f);

	//Wheels elements and parameters

	std::shared_ptr<ChBody> rightFrontWheel;
	std::shared_ptr<ChLinkMateSpherical> rightFrontLink;
	std::shared_ptr<ChLinkMotorRotationSpeed> rightFrontMotor;

	std::shared_ptr<ChBody> rightRearWheel;
	std::shared_ptr<ChLinkMateSpherical> rightRearLink;
	std::shared_ptr<ChLinkMotorRotationSpeed> rightRearMotor;

	std::shared_ptr<ChBody> leftFrontWheel;
	std::shared_ptr<ChLinkMateSpherical> leftFrontLink;
	std::shared_ptr<ChLinkMotorRotationSpeed> leftFrontMotor;

	std::shared_ptr<ChBody> leftRearWheel;
	std::shared_ptr<ChLinkMateSpherical> leftRearLink;
	std::shared_ptr<ChLinkMotorRotationSpeed> leftRearMotor;

	std::shared_ptr<ChForce> frc2 = chrono_types::make_shared<ChForce>();

	double rWheelSize = 0.5;
	double hWheelSize = 0.2;
	double wheelDensity = 50;
	double motorRotSpeed = 0.0; //CH_PI / 1.0;   // speed 180 deg/s

	std::shared_ptr<ChContactMaterialNSC> wheelMat = chrono_types::make_shared<ChContactMaterialNSC>();
	ChColor wheelColor = ChColor(1.0f, 0.3f, 0.2f);
	std::string wheelTexture = GetChronoDataFile("../../sourceFiles/textures/bluewhite.png");

	// Pendulum Elements and parameters

	std::shared_ptr<ChBody> pendulumBeam;
	std::shared_ptr<ChBody> pendulumSphere;

	double rPendulumBeam = 0.1f;
	double hPendulumBeam = 2;

	double rPendulumSphere = 0.2f;

	double pendulumBeamDensity = 1;
	double pendulumSphereDensity = 10;

	std::shared_ptr<ChContactMaterialNSC> pendulumBeamMat = chrono_types::make_shared<ChContactMaterialNSC>();
	std::shared_ptr<ChContactMaterialNSC> pendulumSphereMat = chrono_types::make_shared<ChContactMaterialNSC>();
	std::shared_ptr<ChLinkMateSpherical> spherePendBodyLink;
	std::shared_ptr<ChLinkMateSpherical> lockPendSphereLink;

	ChColor penulumBeamColor = ChColor(0.2f, 0.3f, 1.0f);
	ChColor penulumSphereColor = ChColor(0.2f, 0.3f, 1.0f);


public:

	MyCart(ChVector3d);
	void createBody();
	void createPendulum();
	void connectWheel(std::shared_ptr<ChBody>& wheel, bool right, bool front);

	void updateBodyForce(double, double);
	void setMorotRotSpeed(double);

	std::shared_ptr<ChBody> getBody();
	std::shared_ptr<ChLinkMateSpherical> attachLink(std::shared_ptr<ChBody>& wheel);
	std::shared_ptr<ChLinkMotorRotationSpeed> attachMotor(std::shared_ptr<ChBody>& wheel);
	std::shared_ptr<ChBody> createWheel();

	void fixBody();

	ChVector3d getPendulumPos();
	ChVector3d getPendulumVel();
	ChVector3d getSphereAngle();
	ChVector3d getSphereAngleDt();
	ChVector3d getBodyPos();
	ChVector3d getBodyVel();


	void addCartToSys(ChSystemNSC& sys);
};