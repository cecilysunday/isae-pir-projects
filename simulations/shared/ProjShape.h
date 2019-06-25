// =============================================================================
// Use these shared functions for the Chrono Validation programs found on
// cecilysunday/chrono-validation. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
// =============================================================================


#ifndef PROJSPHAPE_H
#define PROJSPHAPE_H

using namespace chrono;


// Function to create a set of shared material properties
std::shared_ptr<ChMaterialSurfaceSMC> AddMaterialProperties(float y_modulus, float p_ratio,
	float s_frict, float k_frict, float roll_frict, float spin_frict, float cor, float ad);

// Function to configure and add a sphere-shaped body to a simulation
std::shared_ptr<ChBody> AddMovingSphere(int id, ChSystemParallelSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat,
	double radius, double mass, ChVector<> pos, ChVector<> init_v, ChVector<> init_w);

// Function to configure and add a plate-shaped body to a simulation
std::shared_ptr<ChBody> AddWall(int id, ChSystemParallelSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat,
	ChVector<> size, double mass, ChVector<> pos, ChVector<> init_v, bool fixed);

// Function to create the box that povray sees. Not the same as the collision box used in the sim. FIXME
std::shared_ptr<ChBody> AddPovRayWall(int id, ChSystemParallelSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat,
	ChVector<> size, double mass, ChVector<> pos, ChQuaternion<> rot, bool vis);

// Function to configure and add a cylinder-shaped body to a simulation
std::shared_ptr<ChBody> AddCylinder(int id, ChSystemParallelSMC* msystem, std::shared_ptr<ChMaterialSurfaceSMC> mat,
	double radius, double height, double mass, ChVector<> pos, bool fixed);

// Function to apply a .JPEG pattern to a body from the Chrono data directory
//#ifdef CHRONO_IRRLICHT
void AddPattern(std::shared_ptr<ChBody> body, std::string pattern);
//#endif 

//Function to link a motor between two bodies
void AddMotor(ChSystemParallelSMC* msystem, std::shared_ptr<ChBody> rotor, std::shared_ptr<ChBody> stator, ChVector<> pos, double speed);


#endif 