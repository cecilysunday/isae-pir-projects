// =============================================================================
// Use these shared functions for the Chrono Validation programs found on
// cecilysunday/chrono-validation. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChIrrApp.h"
#endif

#include "ProjShape.h"

#include <iostream>

using namespace chrono;
using namespace chrono::collision;



std::shared_ptr<ChMaterialSurfaceSMC> AddMaterialProperties(float y_modulus, float p_ratio,
	float s_frict, float k_frict, float roll_frict, float spin_frict, float cor, float ad) {

	auto mat = std::make_shared<ChMaterialSurfaceSMC>();

	mat->SetYoungModulus(y_modulus);
	mat->SetPoissonRatio(p_ratio);
	mat->SetSfriction(s_frict);
	mat->SetKfriction(k_frict);
	mat->SetRollingFriction(roll_frict);
	mat->SetSpinningFriction(spin_frict);
	mat->SetRestitution(cor);
	mat->SetAdhesion(ad);
	mat->SetAdhesionMultDMT(0.0f);
	mat->SetAdhesionSPerko(0.0f);

	return mat;
}


std::shared_ptr<ChBody> AddMovingSphere(int id, ChSystemParallelSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat,
	double radius, double mass, ChVector<> pos, ChVector<> init_v, ChVector<> init_w) {

	// Shared parameters for the falling ball
	ChQuaternion<> rot(1, 0, 0, 0);
	ChVector<> inertia(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));

	// Create a spherical body. Set body parameters and sphere collision model
	auto body = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);
	body->SetIdentifier(id);
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetPos_dt(init_v);
	body->SetWvel_par(init_w);
	body->SetInertiaXX(inertia);
	body->SetMaterialSurface(mat);
	body->SetBodyFixed(false);
	body->SetCollide(true);
	
	body->GetCollisionModel()->ClearModel();
	utils::AddSphereGeometry(body.get(), radius);
	body->GetCollisionModel()->SetEnvelope(radius);
	body->GetCollisionModel()->BuildModel();

	// Return a pointer to the sphere object so that it can be accessed in main
	msystem->AddBody(body);
	return body;
}


std::shared_ptr<ChBody> AddWall(int id, ChSystemParallelSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat, 
	ChVector<> size, double mass, ChVector<> pos, ChVector<> init_v, bool fixed) {

	// Set parameters for the containing bin
	ChQuaternion<> rot(1, 0, 0, 0);
	ChVector<> inertia((1.0 / 12.0) * mass * (pow(size.y(), 2) + pow(size.z(), 2)),
		(1.0 / 12.0) * mass * (pow(size.x(), 2) + pow(size.z(), 2)),
		(1.0 / 12.0) * mass * (pow(size.x(), 2) + pow(size.y(), 2)));

	// Create container. Set body parameters and container collision model
	auto body = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);
	body->SetIdentifier(id);
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetPos_dt(init_v);
	body->SetInertiaXX(inertia);
	body->SetMaterialSurface(mat);
	body->SetBodyFixed(fixed);
	body->SetCollide(true);

	body->GetCollisionModel()->ClearModel();
	utils::AddBoxGeometry(body.get(), size/2);
	body->GetCollisionModel()->BuildModel();

	// Attach a color to the visible container
	auto mvisual = std::make_shared<ChColorAsset>();
	mvisual->SetColor(ChColor(0.55f, 0.57f, 0.67f));
	body->AddAsset(mvisual);

	// Return a pointer to the wall object so that it can be accessed in main 
	msystem->AddBody(body);
	return body;
}


std::shared_ptr<ChBody> AddPovRayWall(int id, ChSystemParallelSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat,
	ChVector<> size, double mass, ChVector<> pos, ChQuaternion<> rot, bool vis) {

	// Create container. Set body parameters and container collision model
	auto body = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);
	body->SetIdentifier(id);
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetMaterialSurface(mat);
	body->SetBodyFixed(true);
	body->SetCollide(true);

	body->GetCollisionModel()->ClearModel();
	utils::AddBoxGeometry(body.get(), size, ChVector<>(0,0,0), ChQuaternion<>(1,0,0,0), vis);
	body->GetCollisionModel()->BuildModel();

	// Attach a color to the visible container
	auto mvisual = std::make_shared<ChColorAsset>();
	mvisual->SetColor(ChColor(0.11f, 0.11f, 0.11f));  // Purple: (0.59f, 0.0f, 0.9f), Grey: (0.55f, 0.57f, 0.67f)
	body->AddAsset(mvisual);

	// Return a pointer to the wall object so that it can be accessed in main 
	msystem->AddBody(body);
	return body;
}


std::shared_ptr<ChBody> AddCylinder(int id, ChSystemParallelSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat, 
	double radius, double height, double mass, ChVector<> pos, bool fixed) {

	// Set parameters for the containing bin
	ChQuaternion<> rot(1, 0, 0, 0);
	ChVector<> inertia((1.0 / 12.0) * mass * (3 * pow(radius, 2) + pow(height, 2)),
		0.5 * mass * pow(radius, 2),
		(1.0 / 12.0) * mass * (3 * pow(radius, 2) + pow(height, 2)));

	// Create a cylinder. Set body parameters and cylinder collision model
	auto body = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);
	body->SetIdentifier(id);
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetInertiaXX(inertia);
	body->SetMaterialSurface(mat);
	body->SetBodyFixed(fixed);
	body->SetCollide(true);
	
	body->GetCollisionModel()->ClearModel();
	utils::AddCylinderGeometry(body.get(), radius, height, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));
	body->GetCollisionModel()->BuildModel();

	// Return a pointer to the cylinder object so that it can be accessed in main 
	msystem->AddBody(body);
	return body;
}


#ifdef CHRONO_IRRLICHT
void AddPattern(std::shared_ptr<ChBody> body, std::string pattern) {
	
	auto mtexture = std::make_shared<ChTexture>();
	mtexture->SetTextureFilename(GetChronoDataFile(pattern));
	
	body->AddAsset(mtexture);
}
#endif


void AddMotor(ChSystemParallelSMC* msystem, std::shared_ptr<ChBody> rotor, std::shared_ptr<ChBody> stator, ChVector<> pos, double rot_vel) {

	auto motor = std::make_shared<ChLinkMotorRotationSpeed>();
	motor->Initialize(rotor, stator, ChFrame<>(pos));
	msystem->Add(motor);

	auto mwspeed = std::make_shared<ChFunction_Const>(rot_vel);
	motor->SetSpeedFunction(mwspeed);
	motor->SetAvoidAngleDrift(0);

}