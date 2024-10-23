
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
using namespace rapidjson;

//MyCart::MyCart(std::shared_ptr<Document> config) {
MyCart::MyCart(Document& config) {
	//localConfig = config;

	localConfig.CopyFrom(config, localConfig.GetAllocator());

	std::cout << "Create MyCart\n";

	if (localConfig.HasMember("CartBody")) {
		xBodySize = localConfig["CartBody"]["xSize"].GetDouble();
		std::cout << "x body size - " << xBodySize << "\n";

		yBodySize = localConfig["CartBody"]["ySize"].GetDouble();
		std::cout << "y body size - " << yBodySize << "\n";

		zBodySize = localConfig["CartBody"]["zSize"].GetDouble();
		std::cout << "z body size - " << zBodySize << "\n";

		bodyDensity = localConfig["CartBody"]["density"].GetDouble();
	}

	createBody();
	initPosition = ChVector3d(config["Position"]["x"].GetDouble(),
							config["Position"]["y"].GetDouble(), 
							config["Position"]["z"].GetDouble());
	cartBody->SetPos(initPosition);

	if (localConfig.HasMember("Wheel")) {
		rWheelSize = localConfig["Wheel"]["radius"].GetDouble();
		std::cout << "wheels radius - " << rWheelSize << "\n";

		hWheelSize = localConfig["Wheel"]["width"].GetDouble();
		std::cout << "wheels width - " << hWheelSize << "\n";

		wheelDensity = localConfig["Wheel"]["density"].GetDouble();
	}

	wheels.insert({ "rightFront", susWheel() });
	wheels.insert({ "leftFront", susWheel() });
	wheels.insert({ "rightRear", susWheel() });
	wheels.insert({ "leftRear", susWheel() });


	for (auto& item : wheels) {
		item.second.wheel = createWheel();
		connectWheel(item.second.wheel, item.first);
	}

	if (config.HasMember("Motors")) {
		if (config["Motors"].HasMember("rotTorque")) {
			motorRotTorque = config["Motors"]["rotTorque"].GetDouble();
		}
		for (auto& item : wheels) {
			auto wheelPos = item.first.c_str();
			if (config["Motors"].HasMember(wheelPos)) {
				if (config["Motors"][wheelPos].GetBool() == true) {
					item.second.motorValid = true;
					item.second.motor = attachMotor(item.second.wheel);
				}
			}
			else {
				item.second.link = attachLink(item.second.wheel);
		}
	}

	}

	if (config.HasMember("Beam")) {
		rPendulumBeam = localConfig["Beam"]["radius"].GetDouble();
		std::cout << "beam radius - " << rPendulumBeam << "\n";

		hPendulumBeam = localConfig["Beam"]["height"].GetDouble();
		std::cout << "beam height - " << hPendulumBeam << "\n";

		pendulumBeamDensity = localConfig["Beam"]["density"].GetDouble();
	}

	if (config.HasMember("Sphere")) {
		rPendulumSphere = float(config["Sphere"]["radius"].GetDouble());
		std::cout << "sphere radius - " << rPendulumSphere << "\n";

		pendulumSphereDensity = config["Sphere"]["density"].GetDouble();
	}

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
	cartBody->SetFixed(false);
}

std::shared_ptr<ChBody> MyCart::createWheel() {

	auto wheelBody = chrono_types::make_shared<ChBody>();

	double mass = wheelDensity * (CH_PI * pow(rWheelSize, 2) * hWheelSize);
	double I_axis = 0.5 * mass * pow(rWheelSize, 2);
	double I_orth = (1 / 12.0) * mass * (3 * pow(rWheelSize, 2) + pow(hWheelSize, 2));
	ChQuaternion<> rot;

	wheelBody->SetMass(mass);

	rot = QuatFromAngleY(-CH_PI_2);
	wheelBody->SetInertiaXX(ChVector3d(I_orth, I_axis, I_orth));
	wheelBody->SetRot(rot);


	wheelMat = chrono_types::make_shared<ChContactMaterialSMC>();

	auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
		GetChronoDataFile("../../sourceFiles/textures/tractor_wheel.obj"));

	auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
	vis_shape->SetMesh(trimesh);
	vis_shape->SetColor(ChColor(0.2f, 0.2f, 0.2f));
	wheelBody->AddVisualShape(vis_shape);

	auto ct_shape =
		chrono_types::make_shared<ChCollisionShapeTriangleMesh>(wheelMat, trimesh, false, false, 0.02);
	wheelBody->AddCollisionShape(ct_shape, ChFrame<>(VNULL, ChMatrix33<>(1)));
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

	pendulumBeam->AddCollisionShape(collshape, ChFrame<>(ChVector3d(0, 0, 0), rotBeam));
	pendulumBeam->AddVisualShape(visshape, ChFrame<>(ChVector3d(0, 0, 0), rotBeam));
	pendulumBeam->EnableCollision(true);

	pendulumBeam->SetPos(initPosition + ChVector3d(0, (hPendulumBeam + yBodySize) / 2 + pendDistFromBody, 0));

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

void MyCart::connectWheel(std::shared_ptr<ChBody>& wheel, std::string positoin) {
	ChVector3d deltaVect;
	int dir = 1;
	int side = 1;

	if (positoin.find("left") != std::string::npos) {
		side = -1;
	}
	if (positoin.find("Rear") != std::string::npos) {
		dir = -1;
	}

	deltaVect = ChVector3d(dir * (xBodySize / 2), 0, side * (zBodySize / 2 + hWheelSize*1.5f));
	wheel->SetPos(initPosition + deltaVect);

}

std::shared_ptr<ChLinkMotorRotationTorque> MyCart::attachMotor(std::shared_ptr<ChBody>& wheel) {

	auto linkPos = wheel->GetPos() - initPosition + ChVector3d(0, 0, hWheelSize / 2);

	auto motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
	motor->Initialize(wheel, cartBody, ChFrame<>(initPosition + linkPos, QuatFromAngleZ(CH_PI_2)));
	mfun->SetConstant(motorRotTorque);
	motor->SetTorqueFunction(mfun);

	return motor;
}

std::shared_ptr<ChLinkMateSpherical> MyCart::attachLink(std::shared_ptr<ChBody>& wheel) {

	auto linkPos = wheel->GetPos() - initPosition + ChVector3d(0, 0, hWheelSize / 2);

	auto link = chrono_types::make_shared<ChLinkMateSpherical>();
	link->Initialize(wheel, cartBody, ChFrame<>(initPosition + linkPos));
	link->SetConstrainedCoords(true, true, true, true, true, false);

	return link;
}

void MyCart::addCartToSys(ChSystemSMC& sys) {
	std::cout << "ADD MyCart to sys" << std::endl;
	sys.Add(cartBody);
	cartBody->AddForce(frc2);

	for (auto item : wheels) {
		sys.Add(item.second.wheel);
		if (item.second.motorValid) {
			sys.Add(item.second.motor);
		}
		else {
			sys.Add(item.second.link);
		}
	}

	sys.Add(pendulumBeam);
	sys.Add(spherePendBodyLink);

	sys.Add(pendulumSphere);
	sys.Add(lockPendSphereLink);

}
void MyCart::updateBodyForce(double force, double time) {
	frc2->SetF_x(chrono_types::make_shared<ChFunctionConst>(force));
	cartBody->UpdateForces(time);
}

void MyCart::updateMotorTorque(double value) {
	mfun->SetConstant(value);
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

void MyCart::setMorotRotTorque(double val) {
	motorRotTorque = val;
}

void MyCart::fixBody() {
	pendulumSphere->SetFixed(true);
}
