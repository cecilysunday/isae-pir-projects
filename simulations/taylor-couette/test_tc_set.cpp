// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jules Marti
// =============================================================================
//
// Set the beads with random velocity, then wait until they are immobile. Store the positions in a given file. 
//
// =============================================================================


#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_postprocess/ChPovRay.h"
#include "chrono_postprocess/ChPovRayAssetCustom.h"

#include "ProjDataPath.h"
#include "ProjShape.h"
#include "ProjLog.h"
#include "ProjWriteData.h"

#include <ctime>        
#include <iomanip>
#include <random>

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::postprocess;

#define BUFFER_SIZE 1


// If IRRLICHT is enabled, add irrlicht headers and functions
#ifdef CHRONO_IRRLICHT

#include <irrlicht.h>
#include "chrono_irrlicht/ChIrrApp.h"
using namespace chrono::irrlicht;
using namespace irr;
using namespace irr::video;

// Function to handle irrlicht visualization parameters
ChIrrApp* SetSimVis(ChSystemParallelSMC* msystem, double time_step, bool vis, double y) {
	if (vis) {
		ChIrrApp* application = new ChIrrApp(msystem, L"Shear Cell Experiment", core::dimension2d<u32>(800, 600));

		// Add camera, lights, logo and sky in Irrlicht scene
		application->AddTypicalLogo();
		application->AddTypicalSky();
		application->AddTypicalLights();
		application->AddTypicalCamera(core::vector3df(0, y/2, y));

		// Complete asset construction: convert all assets to Irrlicht
		application->SetStepManage(true);
		application->SetTimestep(time_step);
		application->AssetBindAll();
		application->AssetUpdateAll();

		return application;
	}
	else {
		return NULL;
	}
}
#endif


void create_bead(ChSystemParallelSMC* msystem, bool isWall, bool isFixed, double rad, double mass, ChVector<> pos, ChVector<> init_v) {
	// Get start index of wall list
	std::pair<size_t, size_t> prange;
	prange.first = msystem->Get_bodylist().size();

	long long int id = msystem->Get_bodylist().at(prange.first - 1)->GetIdentifier();
	if (isWall) --id;
	else if (!isWall && id < 0) id = 0;
	else ++id;

	// Create a shared material for the particles
	auto pmat = std::make_shared<ChMaterialSurfaceSMC>();

	pmat->SetSfriction(0.4f);
	pmat->SetKfriction(0.4f);
	pmat->SetRollingFriction(0.0f);
	pmat->SetSpinningFriction(0.0f);
	pmat->SetRestitution(0.6f);
	pmat->SetAdhesion(0.0f);
	pmat->SetAdhesionMultDMT(0.0f);
	pmat->SetAdhesionSPerko(0.0f);

	// Create particle
	auto ball = AddMovingSphere(id, msystem, pmat, rad, mass, pos, init_v, ChVector<>(0, 0, 0));
	if (isFixed == true) ball->SetBodyFixed(true);

	// Create the visualization asset for the particle
	if (isWall == true) {
		auto mvisual = std::make_shared<ChColorAsset>();
		mvisual->SetColor(ChColor(0.48f, 0.71f, 0.38f));
		ball->AddAsset(mvisual);
	}
	else AddPattern(ball, "bluwhite.png");

	GetLog() << "\n" << (int)id << "\t" << msystem->Get_bodylist().at(prange.first - 1)->GetId();

	// Get the end index of the particle list and return
	prange.second = msystem->Get_bodylist().size() - 1;
}


std::pair<size_t, size_t> remplir(ChSystemParallelSMC* msystem, double r_cyl_int, double r_cyl_ext, double height_bead, double r_bead, double mass) {
	// Get start index of particle list
	std::pair<size_t, size_t> prange;
	prange.first = msystem->Get_bodylist().size();

	for (int k = 0; k < floor(((r_cyl_ext - 2 * r_bead) - (r_cyl_int + 2 * r_bead)) / (2 * r_bead)); k++) {
		for (int j = 0; j < floor(height_bead / (2 * r_bead)); j = j++) {
			for (int i = 0; i < floor((CH_C_PI * (r_cyl_int + 3 * r_bead + 2 * k * r_bead)) / r_bead); i++) {

				unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
				std::default_random_engine generator(seed);
				std::normal_distribution<double> distribution(r_bead, r_bead / 100);
				double radius = distribution(generator);

				double vx = -1 + ChRandom() * 1;
				double vy = -1 + ChRandom() * 1;
				double vz = -1 + ChRandom() * 1;
				ChVector<> vel = ChVector<>(vx, vy, vz);

				ChVector <> pos = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));

				create_bead(msystem, false, false, radius, mass, pos, vel);
			}
		}
	}

	// Get the end index of the particle list and return
	prange.second = msystem->Get_bodylist().size() - 1;
	GetLog() << "\nCHECK: num_particles = " << prange.second - prange.first + 1 << ", prange_first = " << prange.first << ", prange_second = " << prange.second;

	return prange;
}


void create_cylinder_ext(ChSystemParallelSMC* msystem, double r_cyl_ext, double height, double r_bead, double mass) {
	for (int j = 0; j < floor(height / (sqrt(3) * r_bead)); j = j + 2) {
		for (int i = 0; i < floor((CH_C_PI * (r_cyl_ext - r_bead)) / r_bead) + 1; i++) {
			ChVector<> pos = ChVector<>((r_cyl_ext - r_bead) * cos(i * (2 * atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3) * r_bead * j + r_bead, (r_cyl_ext - r_bead) * sin(i * (2 * atan(r_bead / (r_cyl_ext - r_bead)))));
			create_bead(msystem, true, true, r_bead, mass, pos, ChVector<>(0, 0, 0));

			if (j + 1 < floor(height / (sqrt(3) * r_bead))) {
				ChVector<> pos2 = ChVector<>((r_cyl_ext - r_bead) * cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3) * r_bead * (j + 1) + r_bead, (r_cyl_ext - r_bead) * sin((2 * i + 1) * (atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(msystem, true, true, r_bead, mass, pos2, ChVector<>(0, 0, 0));
			}
		}
	}
}


void create_cylinder_int(ChSystemParallelSMC* msystem, double r_cyl_int, double height, double r_bead, double mass) {
	// Create a shared material for the particles
	auto pmat = std::make_shared<ChMaterialSurfaceSMC>();

	pmat->SetSfriction(0.4f);
	pmat->SetKfriction(0.4f);
	pmat->SetRollingFriction(0.0f);
	pmat->SetSpinningFriction(0.0f);
	pmat->SetRestitution(0.6f);
	pmat->SetAdhesion(0.0f);
	pmat->SetAdhesionMultDMT(0.0f);
	pmat->SetAdhesionSPerko(0.0f);

	// Define the shared wall properties
	int id = msystem->Get_bodylist().at(msystem->Get_bodylist().size() - 1)->GetIdentifier() - 1;
	ChQuaternion<> rot = ChQuaternion<>(1, 0, 0, 0);
	ChVector<> posc = ChVector<>(0, height / 2, 0);
	ChVector<> poso = ChVector<>(0, r_bead / 6, 0);

	// Create the wall from a group of particles
	auto cylinder = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);
	cylinder->SetIdentifier(id);
	cylinder->SetMass(mass); // FIX ME
	cylinder->SetPos(posc);
	cylinder->SetRot(rot);
	cylinder->SetMaterialSurface(pmat); // FIX ME
	cylinder->SetBodyFixed(true);
	cylinder->SetCollide(true);

	cylinder->GetCollisionModel()->ClearModel();
	utils::AddCylinderGeometry(cylinder.get(), r_cyl_int, height / 2, ChVector<>(0, 0, 0), rot, true);
	for (int j = 0; j < floor(height / (sqrt(3) * r_bead)); j = j + 2) {
		for (int i = 0; i < floor(CH_C_PI * (r_cyl_int + r_bead) / r_bead) + 1; i++) {
			ChVector<> pos = ChVector<>((r_cyl_int + r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))), sqrt(3)*r_bead * j + r_bead, (r_cyl_int + r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))));
			utils::AddSphereGeometry(cylinder.get(), r_bead, pos - posc + poso, rot, true);

			if (j + 1 < floor(height / (sqrt(3) * r_bead))) {
				ChVector<> pos2 = ChVector<>((r_cyl_int + r_bead) * cos((2 * i + 1) * (atan(r_bead / (r_cyl_int + r_bead)))), sqrt(3) * r_bead * (j + 1) + r_bead, (r_cyl_int + r_bead) * sin((2 * i + 1) * (atan(r_bead / (r_cyl_int + r_bead)))));
				utils::AddSphereGeometry(cylinder.get(), r_bead, pos2 - posc + poso, rot, true);
			}
		}
	}
	cylinder->GetCollisionModel()->BuildModel();

	// Add a color to the interior wall
	auto mvisual = std::make_shared<ChColorAsset>();
	mvisual->SetColor(ChColor(0.48f, 0.71f, 0.38f));
	cylinder->AddAsset(mvisual);

	// Add the interior wall to the system and return
	msystem->AddBody(cylinder);
}


std::pair<size_t, size_t> set_up(ChSystemParallelSMC* msystem, double r_cyl_int, double r_cyl_ext, double height, double r_bead, double mass) {
	// Get start index of the wall list
	std::pair<size_t, size_t> wrange;
	wrange.first = msystem->Get_bodylist().size();

	// Shared wall properties
	int id = 0;
	ChQuaternion<> rot = ChQuaternion<>(1, 0, 0, 0);

	// Shared wall material
	auto wmat = std::make_shared<ChMaterialSurfaceSMC>();
	wmat->SetRestitution(0.1f);
	wmat->SetFriction(0.4f);
	wmat->SetAdhesion(0);

	//Création du sol
	ChVector<> box_size = ChVector<>(r_cyl_ext, 0.1, r_cyl_ext);
	ChVector<> fpos = ChVector<>(0, -box_size.y(), 0);
	ChVector<> cpos = ChVector<>(0, height, 0);

	auto floor = AddPovRayWall(--id, msystem, wmat, box_size, 10.0, fpos, rot, true);
	auto ceiling = AddPovRayWall(--id, msystem, wmat, box_size, 10.0, cpos, rot, true);

	// Add the inner and outer cylinders
	create_cylinder_int(msystem, r_cyl_int, height, r_bead, mass);
	create_cylinder_ext(msystem, r_cyl_ext, height, r_bead, mass);

	// Find and return index range of wall list 
	wrange.second = msystem->Get_bodylist().size() - 1;
	GetLog() << "\nCHECK: num_walls = " << msystem->Get_bodylist().size();

	return wrange;
}


void SetPovrayParameters(ChPovRay* pov_exporter, double x_cam, double y_cam, double z_cam) {
	// Modify the default light and camera
	pov_exporter->SetLight(ChVector<>(-3, 4, 2), ChColor(0.15f, 0.15f, 0.12f), false);
	pov_exporter->SetCamera(ChVector<>(x_cam, y_cam, z_cam), ChVector<>(0, 0, 0), 0.0, false);
	pov_exporter->SetBackground(ChColor(1.0f, 1.0f, 1.0f));

	// Create an area light for soft shadows
	pov_exporter->SetCustomPOVcommandsScript("light_source { <2, 10, -300> color rgb<1.2,1.2,1.2> area_light <4, 0, 0>, <0, 0, 4>, 8, 8 adaptive 1 jitter}");

	// Tell to the POVray exporter to convert the shapes of all items
	pov_exporter->AddAll();
	pov_exporter->ExportScript();
}


// Set simulation settings and collision detection parameters
void SetSimParameters(ChSystemParallelSMC* msystem, ChVector<> gravity, double r_bead) {
	// Set CalcContactForce properties
	msystem->Set_G_acc(gravity);

	msystem->GetSettings()->solver.min_roll_vel = 1E-5;
	msystem->GetSettings()->solver.min_spin_vel = 1E-5;

	msystem->GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz; /// Types: Hooke, Hertz, PlainCoulomb, Flores
	msystem->GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;
	msystem->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::MultiStep;

	// Set collision detection and solver properties
	msystem->ChangeCollisionSystem(CollisionSystemType::COLLSYS_PARALLEL); /// Types:: COLLSYS_PARALLEL, COLLSYS_BULLET_PARALLEL
	msystem->SetTimestepperType(ChTimestepper::Type::LEAPFROG); /// Types: LEAPFROG....

	msystem->GetSettings()->solver.max_iteration_bilateral = 100;
	msystem->GetSettings()->solver.tolerance = 1e-3;
	msystem->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
	msystem->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R; /// Types: NARROWPHASE_HYBRID_MPR, NARROWPHASE_R, NARROWPHASE_MPR

	// Change the default collision effective radius of curvature 
	ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(r_bead / 2);
}


int main(int argc, char* argv[]) {
	// Set the output data directory. dontcare = false when a timestamped directory is desired
	bool dontcare = false;
	std::string projname = "_tc_set";

	const std::string out_dir = SetDataPath(projname, dontcare);

	if (out_dir == "") {
		fprintf(stderr, "Error creating output data directory\n");
		return -1;
	}

	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	//Déclaration des paramètres
	double gy = -9.81E2;
	double r_bead = 0.2;
	double r_cyl_ext = 8;
	double r_cyl_int = 3;
	double height = 5;
	double height_bead = 4;
	double rho = 2.55;
	double mass = rho * (4 / 3)*CH_C_PI*pow(r_bead, 3);

	//Writes the settings into the file : settings.dat
	std::string datafile3 = out_dir + "/settings.dat";
	ChStreamOutAsciiFile settings(datafile3.c_str());
	settings << gy << " " << r_bead << " " << r_cyl_ext << " " << r_cyl_int << " " << height << " " << height_bead << " " << mass << " " << "\n";

	// Create a .dat file with three columns of demo data:
	std::string datafile = out_dir + "/mean_v_data.dat";
	ChStreamOutAsciiFile mean_v(datafile.c_str());
	mean_v << "time" << " " << "mean_v" << "\n";

	// Create a parallel SMC system and set the system parameters
	ChVector<> gravity(0, gy, 0);

	double time_step = 1.0E-4;
	double out_step = 2.0E-2;
	double time_sim = 3.0;

	ChSystemParallelSMC msystem;
	SetSimParameters(&msystem, gravity, r_bead);

	// Add the shear-cell structure and fill the cell with particles according to tc_set inputs
	std::pair<size_t, size_t> wlist = set_up(&msystem, r_cyl_int, r_cyl_ext, height, r_bead, mass);
	std::pair<size_t, size_t> plist = remplir(&msystem, r_cyl_int, r_cyl_ext, height_bead, r_bead, mass);

	size_t start_wlist = wlist.first;
	size_t num_walls = wlist.second - start_wlist + 1;

	size_t start_plist = wlist.second + 1;
	size_t num_particles = plist.second - start_plist + 1;

	// Print simulation parameters to log file
	GetLog() << "\n" << "SYS, time_step, " << time_step
			 << "\n" << "SYS, out_step, " << out_step
			 << "\n" << "SYS, total_sim_time, " << time_sim
			 << "\n" << "SYS, num_walls, " << (double)num_walls
			 << "\n" << "SYS, num_particles, " << (double)num_particles
			 << "\n" << "SYS, buffer_size, " << BUFFER_SIZE
			 << "\n" << "SYS, data_array_size, " << num_particles * BUFFER_SIZE * sizeof(ParticleData);

	PrintSimParameters(&msystem);

	// Create an exporter to POVray and set all associated filepaths and settings
	ChPovRay pov_exporter = ChPovRay(&msystem);
	if (SetPovrayPaths(&pov_exporter, out_dir) != 0) {
		fprintf(stderr, "Error creating povray data paths\n");
		return -1;
	}
	SetPovrayParameters(&pov_exporter, 0.0, 30.0, 0.0);

	// Create the Irrlicht visualization. Set vis to false to turn off visualization. 
	#ifdef CHRONO_IRRLICHT
		bool vis = false;
		auto application = SetSimVis(&msystem, time_step, vis, 20.0);
	#endif

	// THE SOFT-REAL-TIME CYCLE
	double time = 0.0;
	double out_time = 0.0;

	double v_avg;
	double timer_sim = 0.0;
	double timer_bcollision = 0.0;
	double timer_ncollision = 0.0;
	double timer_fcalc = 0.0;

	// Iterate through simulation and calculate resultant motion for each timestep
	while (time < time_sim) {
		#ifdef CHRONO_IRRLICHT
			if (application != NULL) {
				application->BeginScene(true, true, SColor(255, 255, 255, 255));
				application->GetDevice()->run();
				application->DrawAll();
		}
		#endif

		while (time == 0 || time < out_time) {
			msystem.DoStepDynamics(time_step);
			time += time_step;

			timer_sim += msystem.GetTimerStep();
			timer_bcollision += msystem.GetTimerCollisionBroad();
			timer_ncollision += msystem.GetTimerCollisionNarrow();
			timer_fcalc += msystem.GetTimerProcessContact();
		}

		#ifdef CHRONO_IRRLICHT
			if (application != NULL) {
				application->EndScene();
			}
		#endif

		v_avg = 0;
		for (int i = start_plist; i < start_plist + num_particles; ++i) {
			std::shared_ptr<ChBody> body = msystem.Get_bodylist().at(i);
			double v_mag = body->GetPos_dt().Length();
			v_avg += v_mag;
		}
		v_avg = v_avg / num_particles;

		mean_v << time << " " << v_avg << "\n";
		fprintf(stderr, "time : %f \tmean_v : %f\n", time, v_avg);
		pov_exporter.ExportData();

		if (v_avg < 0.001) break;
		out_time = time - time_step + out_step;
	}

	fprintf(stderr, "time : %f \tmean_v : %f\n", time, v_avg);

	std::string datafile2 = out_dir + "/position.dat";
	std::ofstream position(datafile2, std::ios::out | std::ios::trunc);

	if (position) {
		for (int i = start_plist; i < start_plist + num_particles; ++i) {
			std::shared_ptr<ChBody> body = msystem.Get_bodylist().at(i);
			position << body->GetPos().x() << " " << body->GetPos().y() << " " << body->GetPos().z() << " " << body->GetCollisionModel()->GetEnvelope() << "\n";
		}

		position << -100000000 << " " << -100000000 << " " << -100000000 << -100000000 << "\n";
		position.close();
	}

	fprintf(stderr, "Les positions ont bien ete enregistrees\n");

	// Delete dynamically allocated objects and arrays and return
	#ifdef CHRONO_IRRLICHT
		delete application;
	#endif

	// Print simulation timers to log
	chrono::GetLog() << "\n" << "SYS, timer_bcollision, " << timer_bcollision
				     << "\n" << "SYS, timer_ncollision, " << timer_ncollision
				     << "\n" << "SYS, timer_fcalc, " << timer_fcalc
				     << "\n" << "SYS, timer_sim, " << timer_sim;

	return 0;
}