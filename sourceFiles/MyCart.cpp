
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "MyCart.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;

MyCart::MyCart(ChVector3d position) {

	std::cout << "Create MyCart";
	createBody();
	initPosition = position;
	cartBody->SetPos(initPosition);
	rightFrontWheel = createWheel();
	rightRearWheel = createWheel();
	leftRearWheel = createWheel();
	leftFrontWheel = createWheel();

	connectWheel(rightFrontWheel, true, true);
	connectWheel(leftFrontWheel, true, false);
	connectWheel(rightRearWheel, false, true);
	connectWheel(leftRearWheel, false, false);

	//rightFrontMotor = attachMotor(rightFrontWheel);
	//leftFrontMotor = attachMotor(leftFrontWheel);
	//leftRearMotor = attachMotor(leftRearWheel);
	//rightRearMotor = attachMotor(rightRearWheel);

	rightFrontLink = attachLink(rightFrontWheel);
	leftFrontLink = attachLink(leftFrontWheel);
	leftRearLink = attachLink(leftRearWheel);
	rightRearLink = attachLink(rightRearWheel);


	createPendulum();
};

std::shared_ptr<ChBody> MyCart::getBody() {
	return cartBody;
}

void MyCart::createBody() {

	double mass = bodyDensity * (xBodySize * yBodySize * zBodySize);
	auto visshape = chrono_types::make_shared<ChVisualShapeBox>(xBodySize, yBodySize, zBodySize);
	visshape->SetColor(bodyColor);

	auto collshape = chrono_types::make_shared<ChCollisionShapeBox>(bodyMat, xBodySize, yBodySize, zBodySize);

	cartBody = chrono_types::make_shared<ChBody>();
	cartBody->SetMass(mass);

	cartBody->SetInertiaXX(ChVector3d((1.0 / 12.0) * mass * (pow(yBodySize, 2) + pow(zBodySize, 2)),
										(1.0 / 12.0) * mass * (pow(xBodySize, 2) + pow(zBodySize, 2)),
										(1.0 / 12.0) * mass * (pow(xBodySize, 2) + pow(yBodySize, 2))));

	cartBody->AddCollisionShape(collshape);
	cartBody->AddVisualShape(visshape);
	cartBody->EnableCollision(true);
}

std::shared_ptr<ChBody> MyCart::createWheel() {

	auto wheelBody = chrono_types::make_shared<ChBody>();

	auto collshape = chrono_types::make_shared<ChCollisionShapeCylinder>(wheelMat, rWheelSize, hWheelSize);
	auto visshape = chrono_types::make_shared<ChVisualShapeCylinder>(rWheelSize, hWheelSize);
	visshape->SetTexture(wheelTexture);

	double mass = wheelDensity * (CH_PI * pow(rWheelSize, 2) * hWheelSize);
	double I_axis = 0.5 * mass * pow(rWheelSize, 2);
	double I_orth = (1 / 12.0) * mass * (3 * pow(rWheelSize, 2) + pow(hWheelSize, 2));
	ChQuaternion<> rot;

	wheelBody->SetMass(mass);

	rot = QuatFromAngleX(CH_PI_2);
	wheelBody->SetInertiaXX(ChVector3d(I_orth, I_axis, I_orth));
	wheelBody->AddCollisionShape(collshape, ChFrame<>(VNULL, rot));
	wheelBody->AddVisualShape(visshape, ChFrame<>(VNULL, rot));
	wheelBody->SetRot(rot);
	wheelBody->EnableCollision(true);

	return wheelBody;
}
void MyCart::createPendulum() {

	//pendulum beam

	pendulumBeam = chrono_types::make_shared<ChBody>();

	double pendDistFromBody = 0.1f;

	auto collshape = chrono_types::make_shared<ChCollisionShapeCylinder>(pendulumBeamMat, rPendulumBeam, hPendulumBeam);
	auto visshape = chrono_types::make_shared<ChVisualShapeCylinder>(rPendulumBeam, hPendulumBeam);
	visshape->SetColor(penulumBeamColor);

	double mass = pendulumBeamDensity * (CH_PI * pow(rPendulumBeam, 2) * hPendulumBeam);
	double I_axis = 0.5 * mass * pow(rPendulumBeam, 2);
	double I_orth = (1 / 12.0) * mass * (3 * pow(rPendulumBeam, 2) + pow(hPendulumBeam, 2));

	pendulumBeam->SetMass(mass);

	auto rotBeam = QuatFromAngleX(CH_PI / 2);
	//pendulumBeam->SetInertiaXX(ChVector3d(I_orth, I_axis, I_orth));
	//pendulumBeam->AddCollisionShape(collshape);
	//pendulumBeam->AddVisualShape(visshape);

	pendulumBeam->AddCollisionShape(collshape, ChFrame<>(ChVector3d(0, 0, 0), rotBeam));
	pendulumBeam->AddVisualShape(visshape, ChFrame<>(ChVector3d(0, 0, 0), rotBeam));
	pendulumBeam->EnableCollision(true);

	pendulumBeam->SetPos(initPosition + ChVector3d(0, (hPendulumBeam + yBodySize) / 2 + pendDistFromBody, 0));
	//pendulumBeam->SetFixed(true);


	// Pendulum sphere
	pendulumSphere = chrono_types::make_shared<ChBody>();

	auto collshapeSphere = chrono_types::make_shared<ChCollisionShapeSphere>(pendulumSphereMat, rPendulumSphere);
	auto visshapeSphere = chrono_types::make_shared<ChVisualShapeSphere>(rPendulumSphere);
	visshapeSphere->SetColor(penulumSphereColor);

	double massSphere = pendulumSphereDensity * ((4.0 / 3.0) * CH_PI * pow(rPendulumSphere, 3));
	double inertia = (2.0 / 5.0) * massSphere * pow(rPendulumSphere, 2);

	pendulumSphere->SetMass(massSphere);

	pendulumSphere->SetInertiaXX(ChVector3d(inertia, inertia, inertia));
	pendulumSphere->AddCollisionShape(collshapeSphere);
	pendulumSphere->AddVisualShape(visshapeSphere);
	pendulumSphere->EnableCollision(true);

	pendulumSphere->SetPos(pendulumBeam->GetPos() + ChVector3d(0, hPendulumBeam / 2 + rPendulumSphere, 0));
	//pendulumSphere->SetFixed(true);

	lockPendSphereLink = chrono_types::make_shared<ChLinkMateSpherical>();
	lockPendSphereLink->Initialize(pendulumSphere, pendulumBeam, ChFrameMoving<>(pendulumBeam->GetPos() + ChVector3d(0, hPendulumBeam / 2, 0)));
	lockPendSphereLink->SetConstrainedCoords(true, true, true, true, true, true);

	spherePendBodyLink = chrono_types::make_shared<ChLinkMateSpherical>();
	spherePendBodyLink->Initialize(pendulumBeam, cartBody, ChFrameMoving<>(initPosition + ChVector3d(0, yBodySize, 0)));
	spherePendBodyLink->SetConstrainedCoords(true, true, true, true, true, false);
}

void MyCart::connectWheel(std::shared_ptr<ChBody>& wheel, bool front, bool right) {
	ChVector3d deltaVect;
	int dir = 1;
	int side = 1;

	if (!right) {
		side = -1;
	}
	if (!front) {
		dir = -1;
	}
	deltaVect = ChVector3d(dir * (xBodySize / 2), 0, side * (zBodySize / 2 + hWheelSize * 0.6));
	wheel->SetPos(initPosition + deltaVect);

}

std::shared_ptr<ChLinkMotorRotationSpeed> MyCart::attachMotor(std::shared_ptr<ChBody>& wheel) {

	auto linkPos = wheel->GetPos() - initPosition + ChVector3d(0, 0, hWheelSize / 2);

	auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
	motor->Initialize(wheel, cartBody, ChFrame<>(initPosition + linkPos, QuatFromAngleZ(CH_PI_2)));
	auto mfun = chrono_types::make_shared<ChFunctionConst>(motorRotSpeed);
	motor->SetSpeedFunction(mfun);

	return motor;
}

std::shared_ptr<ChLinkMateSpherical> MyCart::attachLink(std::shared_ptr<ChBody>& wheel) {

	auto linkPos = wheel->GetPos() - initPosition + ChVector3d(0, 0, hWheelSize / 2);

	auto link = chrono_types::make_shared<ChLinkMateSpherical>();
	link->Initialize(wheel, cartBody, ChFrame<>(initPosition + linkPos));
	link->SetConstrainedCoords(true, true, true, true, true, false);

	return link;
}

void MyCart::addCartToSys(ChSystemNSC& sys) {
	std::cout << "ADD MyCart to sys";
	sys.Add(cartBody);
	cartBody->AddForce(frc2);

	sys.Add(rightFrontWheel);
	sys.Add(rightFrontLink);

	sys.Add(leftFrontWheel);
	sys.Add(leftFrontLink);

	sys.Add(rightRearWheel);
	sys.Add(rightRearLink);

	sys.Add(leftRearWheel);
	sys.Add(leftRearLink);

	sys.Add(pendulumBeam);
	sys.Add(spherePendBodyLink);

	sys.Add(pendulumSphere);
	sys.Add(lockPendSphereLink);

}
void MyCart::updateBodyForce(double force, double time) {
	frc2->SetF_x(chrono_types::make_shared<ChFunctionConst>(force));
	cartBody->UpdateForces(time);
}

ChVector3d MyCart::getPendulumPos() {
	return pendulumSphere->GetPos();
}

ChVector3d MyCart::getPendulumVel() {
	return pendulumSphere->GetLinVel();
}

ChVector3d MyCart::getSphereAngle() {
	return pendulumSphere->GetRot().GetCardanAnglesXYZ();

}

ChVector3d MyCart::getSphereAngleDt() {
	return pendulumSphere->GetAngVelLocal();
	//return spherePendBodyLink->GetFrame1Rel().GetRot().GetAxisZ();
}

ChVector3d MyCart::getBodyPos() {
	return cartBody->GetPos();
}
ChVector3d MyCart::getBodyVel() {
	return cartBody->GetLinVel();
}

void MyCart::setMorotRotSpeed(double val) {
	motorRotSpeed = val;
}

void MyCart::fixBody() {
	pendulumSphere->SetFixed(true);
}
