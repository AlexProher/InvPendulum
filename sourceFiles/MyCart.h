#pragma once

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include <chrono/physics/ChLinkMotorRotationTorque.h>


using namespace chrono;
using namespace rapidjson;

class MyCart {

private:

	ChSystemSMC system;

	//std::shared_ptr<Document> localConfig;
	Document localConfig;

	ChVector3d initPosition;
	ChVector3d initVelocity;
	ChVector3d initAcceleration;

	// Body elements and parameters

	std::shared_ptr<ChBody> cartBody;

	double xBodySize = 2;
	double yBodySize = 0.5;
	double zBodySize = 1;
	double bodyDensity = 100;

	std::shared_ptr<ChContactMaterialSMC> bodyMat = chrono_types::make_shared<ChContactMaterialSMC>();
	ChColor bodyColor = ChColor(0.2f, 0.3f, 1.0f);

	//Wheels elements and parameters

	struct susWheel
	{
		std::shared_ptr<ChBody> wheel;
		std::shared_ptr<ChLinkMateSpherical> link;
		std::shared_ptr<ChLinkMotorRotationTorque> motor;
		bool motorValid;
	};

	std::map<std::string, susWheel> wheels;

	std::shared_ptr<ChForce> frc2 = chrono_types::make_shared<ChForce>();
	std::shared_ptr<ChFunctionConst> mfun = chrono_types::make_shared<ChFunctionConst>();

	double rWheelSize = 0.5;
	double hWheelSize = 0.2;
	double wheelDensity = 50;
	double motorRotTorque = 0.0;

	std::shared_ptr<ChContactMaterialSMC> wheelMat = chrono_types::make_shared<ChContactMaterialSMC>();
	ChColor wheelColor = ChColor(1.0f, 0.3f, 0.2f);
	std::string wheelTexture = GetChronoDataFile("../../sourceFiles/textures/bluewhite.png");

	// Pendulum Elements and parameters

	std::shared_ptr<ChBody> pendulumBeam;
	std::shared_ptr<ChBody> pendulumSphere;

	float rPendulumBeam = 0.1;
	double hPendulumBeam = 2;

	double rPendulumSphere = 0.2f;

	double pendulumBeamDensity = 1;
	double pendulumSphereDensity = 10;

	std::shared_ptr<ChContactMaterialSMC> pendulumBeamMat = chrono_types::make_shared<ChContactMaterialSMC>();
	std::shared_ptr<ChContactMaterialSMC> pendulumSphereMat = chrono_types::make_shared<ChContactMaterialSMC>();
	std::shared_ptr<ChLinkMateSpherical> spherePendBodyLink;
	std::shared_ptr<ChLinkMateSpherical> lockPendSphereLink;

	ChColor penulumBeamColor = ChColor(0.2f, 0.3f, 1.0f);
	ChColor penulumSphereColor = ChColor(0.2f, 0.3f, 1.0f);


public:

	//MyCart(std::shared_ptr<Document>);
	MyCart(Document&);
	void createBody();
	void createPendulum();
	void connectWheel(std::shared_ptr<ChBody>& wheel,std::string position);

	void updateBodyForce(double, double);
	void updateMotorTorque(double value);
	void setMorotRotTorque(double);

	std::shared_ptr<ChBody> getBody();
	std::shared_ptr<ChLinkMateSpherical> attachLink(std::shared_ptr<ChBody>& wheel);
	std::shared_ptr<ChLinkMotorRotationTorque> attachMotor(std::shared_ptr<ChBody>& wheel);
	std::shared_ptr<ChBody> createWheel();

	void fixBody();

	ChVector3d getPendulumPos();
	ChVector3d getPendulumVel();
	ChVector3d getSphereAngle();
	ChVector3d getSphereAngleDt();
	ChVector3d getBodyPos();
	ChVector3d getBodyVel();


	void addCartToSys(ChSystemSMC& sys);
};