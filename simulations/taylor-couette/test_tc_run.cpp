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
// Run the simulations with beads at positions specified by a given file. Write data of the surface in given files. 
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_postprocess/ChPovRay.h"
#include "chrono_postprocess/ChPovRayAssetCustom.h"

#include "ProjDataPath.h"
#include "ProjLog.h"
#include "ProjShape.h"
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
			application->AddTypicalCamera(core::vector3df(0, y, 0));

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


void create_bead(ChSystemParallelSMC* msystem, bool isWall, bool isFixed, double rad, double mass, ChVector<> pos) {
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
	ChVector<> init_v = ChVector<>(0, 0, 0);
	ChVector<> init_w = ChVector<>(0, 0, 0);
	
	auto ball = AddMovingSphere(id, msystem, pmat, rad, mass, pos, init_v, init_w);
	if (isFixed == true) ball->SetBodyFixed(true);

	// Create the visualization asset for the particle
	if (isWall == true) {
		auto mvisual = std::make_shared<ChColorAsset>();
		mvisual->SetColor(ChColor(0.48f, 0.71f, 0.38f));
		ball->AddAsset(mvisual);
	}
	else AddPattern(ball, "bluwhite.png");

	// Get the end index of the particle list and return
	prange.second = msystem->Get_bodylist().size() - 1;
}


std::pair<size_t, size_t> remplir(ChSystemParallelSMC* msystem, double mass, std::vector<ChVector<>>* p_list_pos, std::vector<double>* p_radius) {
	// Get start index of particle list
	std::pair<size_t, size_t> prange;
	prange.first = msystem->Get_bodylist().size();

	// Add particles according to the position and radius provided by tc_set
	for (int i = 0; i < p_list_pos->size(); i++) {
		ChVector <> pos = (*p_list_pos)[i];
		double radius = (*p_radius)[i];
		create_bead(msystem, false, false, radius, mass, pos);
	}

	// Get the end index of the particle list and return
	prange.second = msystem->Get_bodylist().size() - 1;
	GetLog() << "\nCHECK: num_particles = " << prange.second - prange.first + 1 << ", prange_first = " << prange.first << ", prange_second = " << prange.second;

	return prange;
}


void create_cylinder_ext(ChSystemParallelSMC* msystem, double r_cyl_ext, double height, double r_bead, double mass, int methode) {
	//Columns arrangement
	if (methode == 1) { 

		for (int i = 0; i < floor(CH_C_PI * (r_cyl_ext - r_bead) / r_bead) + 1; i++) {
			for (int j = 0; j < floor(height / (2 * r_bead)); j = j++) {
				ChVector <> pos = ChVector<>((r_cyl_ext - r_bead) * cos(i * (2 * atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(msystem, true, true, r_bead, mass, pos);
			}
		}
	}
	
	//More compact arrangement (horizontal shift from a line to an other)
	else if (methode == 2) { 
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI*(r_cyl_ext - r_bead)) / r_bead) + 1; i++) {
				ChVector<> pos = ChVector<>((r_cyl_ext - r_bead) * cos(i * (2 * atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_ext - r_bead) * sin(i * (2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(msystem, true, true, r_bead, mass, pos);

				ChVector<> pos2 = ChVector<>((r_cyl_ext - r_bead) * cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * (j + 1) + r_bead, (r_cyl_ext - r_bead)*sin((2 * i + 1) * (atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(msystem, true, true, r_bead, mass, pos2);
			}
		}
	}

	//Compact arrangement
	else if (methode == 3) {
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI * (r_cyl_ext - r_bead)) / r_bead) + 1; i++) {
				ChVector<> pos = ChVector<>((r_cyl_ext - r_bead) * cos(i * (2 * atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3) * r_bead * j + r_bead, (r_cyl_ext - r_bead) * sin(i * (2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(msystem, true, true, r_bead, mass, pos);
				
				if (j + 1 < floor(height / (2 * r_bead))) {
					ChVector<> pos2 = ChVector<>((r_cyl_ext - r_bead) * cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3) * r_bead * (j + 1) + r_bead, (r_cyl_ext - r_bead) * sin((2 * i + 1) * (atan(r_bead / (r_cyl_ext - r_bead)))));
					create_bead(msystem, true, true, r_bead, mass, pos2);
				}
			}
		}
	}

	else fprintf(stderr, "La methode rentree est incorrecte\n");
}


void create_cylinder_int(ChSystemParallelSMC* msystem, double r_cyl_int, double height, double r_bead, double mass, int methode, std::shared_ptr<chrono::ChBodyFrame> rotatingBody) {
	//Remplissage en colonne
	if (methode == 1) {
		for (int i = 0; i < floor((CH_C_PI * (r_cyl_int + r_bead)) / r_bead) + 1; i++) {
			for (int j = 0; j < floor(height / (2 * r_bead)); j++) {
				ChVector<> pos = ChVector<>((r_cyl_int + r_bead) * cos(i * (2 * atan(r_bead / (r_cyl_int + r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_int + r_bead) * sin(i * (2 * atan(r_bead / (r_cyl_int + r_bead)))));
				create_bead(msystem, true, false, r_bead, mass, pos);
				
				auto lock = std::make_shared<ChLinkMateFix>();
				lock->Initialize(msystem->Get_bodylist().back(), rotatingBody);
				msystem->AddLink(lock);
			}
		}
	}
	
	//Remplissage en décalé sans contact vertical
	else if (methode == 2) { 
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI * (r_cyl_int + r_bead)) / r_bead) + 1; i++) {
				ChVector<> pos = ChVector<>((r_cyl_int + r_bead) * cos(i * (2 * atan(r_bead / (r_cyl_int + r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_int + r_bead) * sin(i * (2 * atan(r_bead / (r_cyl_int + r_bead)))));
				create_bead(msystem, true, false, r_bead, mass, pos);

				auto lock = std::make_shared<ChLinkMateFix>();
				lock->Initialize(msystem->Get_bodylist().back(), rotatingBody);
				msystem->AddLink(lock);

				ChVector<> pos2 = ChVector<>((r_cyl_int + r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))), r_bead * 2 * (j + 1) + r_bead, (r_cyl_int + r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))));
				create_bead(msystem, true, false, r_bead, mass, pos2);

				auto lock2 = std::make_shared<ChLinkMateFix>();
				lock2->Initialize(msystem->Get_bodylist().back(), rotatingBody);
				msystem->AddLink(lock2);
			}
		}
	}

	//Remplissage en décalé avec contact vertical
	else if (methode == 3) { 
		for (int j = 0; j < floor((height - r_bead) / (sqrt(3) * r_bead)); j = j + 2) {
			for (int i = 0; i < floor( CH_C_PI * (r_cyl_int + r_bead) / r_bead) + 1; i++) {
				ChVector<> pos = ChVector<>((r_cyl_int + r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))), sqrt(3)*r_bead * j+r_bead, (r_cyl_int + r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))));
				create_bead(msystem, true, false, r_bead, mass, pos);

				auto lock = std::make_shared<ChLinkMateFix>();
				lock->Initialize(msystem->Get_bodylist().back(), rotatingBody);
				msystem->AddLink(lock);

				if (j + 1 < floor((height - r_bead) / (sqrt(3) * r_bead))) {
					ChVector<> pos2 = ChVector<>((r_cyl_int + r_bead) * cos((2 * i + 1) * (atan(r_bead / (r_cyl_int + r_bead)))), sqrt(3) * r_bead * (j + 1) + r_bead, (r_cyl_int + r_bead) * sin((2 * i + 1) * (atan(r_bead / (r_cyl_int + r_bead)))));
					create_bead(msystem, true, false, r_bead, mass, pos2);

					auto lock2 = std::make_shared<ChLinkMateFix>();
					lock2->Initialize(msystem->Get_bodylist().back(), rotatingBody);
					msystem->AddLink(lock2);
				}
			}
		}
	} 
	
	else fprintf(stderr, "La methode rentree est incorrecte\n");
}


std::pair<size_t, size_t> set_up(ChSystemParallelSMC* msystem, double r_cyl_int, double r_cyl_ext, double height, double r_bead, double mass, double rotation_speed) {
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
	ChVector<> fb_size = ChVector<>(r_cyl_ext, 0.1, r_cyl_ext); //or should it be (r_cyl_ext, 1.0, r_cyl_ext)?
	ChVector<> fb_pos = ChVector<>(ChVector<>(0, - fb_size.y(), 0));
		
	auto fixedBody = AddPovRayWall(--id, msystem, wmat, fb_size, 10.0, fb_pos, rot, true);
	
	// Add the rotating cylinder
	ChVector<> rb_pos = ChVector<>(ChVector<>(0, height/2, 0));
	
	auto rotatingBody = AddCylinder(--id, msystem, wmat, r_cyl_int, height, 10.0, rb_pos, false);
	AddPattern(rotatingBody, "bluwhite.png");

	// Add a motor between the rotating and fixed bodies
	auto motor = std::make_shared<ChLinkMotorRotationSpeed>();
	motor->Initialize(rotatingBody, fixedBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
	
	auto mfun = std::make_shared<ChFunction_Const>(rotation_speed);  // speed w=90°/s CH_C_PI / 2.0
	motor->SetSpeedFunction(mfun);
	motor->SetAvoidAngleDrift(0);
	msystem->AddLink(motor);

	// Add particles linning the inner cylyinder wall
	create_cylinder_int(msystem, r_cyl_int, height, r_bead, mass, 3, rotatingBody);
	create_cylinder_ext(msystem, r_cyl_ext, height, r_bead, mass, 3);

	// Find and return index range of wall list 
	wrange.second = msystem->Get_bodylist().size() - 1;
	GetLog() << "\nCHECK: num_walls = " << msystem->Get_bodylist().size();

	return wrange;
}


/*void detect_surface(std::vector< std::shared_ptr< ChBody > >* p_beads_list, ChVectorDynamic<std::shared_ptr<ChBody>>* p_surface, double r_bead, double time) {
	
	p_surface->Reset();
	p_surface->Resize(0);
	int nb_bille_surf = 0;
	ChVectorDynamic<std::shared_ptr<ChBody>> list_surf;
	list_surf.Resize(p_beads_list->size());
	std::shared_ptr<ChBody> test = (*p_beads_list)[0];
	
	for (int j = 0; j < p_beads_list->size(); j++) {
		if ((*p_beads_list)[j]->GetPos().y() > test->GetPos().y()) {
			test = (*p_beads_list)[j];
		}
	}

	for (int j = 0; j < p_beads_list->size(); j++) {
		if ((*p_beads_list)[j]->GetPos().y() > test->GetPos().y() - 2 * (r_bead + r_bead / 100)) {
			list_surf.SetElementN(nb_bille_surf, (*p_beads_list)[j]);
			nb_bille_surf = nb_bille_surf + 1;
		}
	}

	GetLog() << "\nTime = " << time << "\tNum Total Beads = " << p_beads_list->size() << "\tNum Surface Beads = " << nb_bille_surf;
	
	p_surface->Resize(nb_bille_surf);
	for (int i = 0; i < nb_bille_surf; i++) {
		p_surface->SetElementN(i, list_surf.GetElementN(i));
	}
	
}


double mean_vector(ChVectorDynamic<double>* p_vector) {
	double s = 0;
	for (int i = 0; i < p_vector->GetLength(); i++) {
		s = s + p_vector->GetElementN(i);
	}
	return s / p_vector->GetLength();
}


bool is_in_mouvement(std::vector< std::shared_ptr< ChBody > >* p_beads_list) {//renvoie true si les billes sont en mouvement
	ChVectorDynamic<double> tab_v = ChVectorDynamic<double>(p_beads_list->size());
	for (int i = 0; i < p_beads_list->size(); ++i) {
		double v=sqrt((*p_beads_list)[i]->GetPos_dt().x()*(*p_beads_list)[i]->GetPos_dt().x() + (*p_beads_list)[i]->GetPos_dt().z()*(*p_beads_list)[i]->GetPos_dt().z());
		tab_v.SetElementN(i, v<0.5);
		
	}
	double mean_v = mean_vector(&tab_v);
	fprintf(stderr, "mean_v : %f\n", mean_v);
	return (!(mean_v-1.0<0.0001 && mean_v-1.0>-0.0001) );
}*/


//Read the position in a given file, and stock it into the list pointed by p_list_pos
void read_pos(std::vector<ChVector<>>* p_list_pos, std::vector<double>* p_radius, const std::string set_path) {
	int i = 0;
	bool end_of_doc = false;
	double x, y, z, radius;

	std::ifstream fichier(set_path + "/position.dat");
	while (end_of_doc == false) {
		fichier >> x >> y >> z >> radius;
		if (x != -100000000 && y != -100000000 && z != -100000000 && radius != -100000000) {
			p_list_pos->push_back(ChVector<>(x, y, z));
			p_radius->push_back(radius);
		}
		else end_of_doc = true;
		i = i + 1;
	}
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
	msystem->Set_G_acc(gravity);

	msystem->GetSettings()->solver.max_iteration_bilateral = 100;
	msystem->GetSettings()->solver.tolerance = 1e-3;

	msystem->GetSettings()->solver.min_roll_vel = 1E-5;
	msystem->GetSettings()->solver.min_spin_vel = 1E-5;

	msystem->GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz; /// Types: Hooke, Hertz, PlainCoulomb, Flores
	msystem->GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;
	msystem->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::MultiStep;

	msystem->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
	msystem->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R; /// Types: NARROWPHASE_HYBRID_MPR, NARROWPHASE_R, NARROWPHASE_MPR

	msystem->ChangeCollisionSystem(CollisionSystemType::COLLSYS_PARALLEL); /// Types:: COLLSYS_PARALLEL, COLLSYS_BULLET_PARALLEL
	msystem->SetTimestepperType(ChTimestepper::Type::LEAPFROG); /// Types: LEAPFROG....

	// Change the default collision effective radius of curvature 
	ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(r_bead / 2);
}


int main(int argc, char* argv[]) {
	// Set the output data directory. dontcare = false when a timestamped directory is desired
	bool dontcare = false;
	std::string projname = "_tc_run"; 

	const std::string out_dir = SetDataPath(projname, dontcare);

	if (out_dir == "") {
		fprintf(stderr, "Error creating output data directory\n");
		return -1;
	}
	
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	//Import geometic parameters from tc_set simulation
	double gy, r_bead, r_cyl_ext, r_cyl_int, height, height_bead, mass;

	std::string path = out_dir + "/../20190621_160112_tc_set";
	//std::string path = out_dir + "/../TEMP_calmip/test_0/TEMP_tc_set";
	std::ifstream fichier(path + "/settings.dat");

	fichier >> gy >> r_bead >> r_cyl_ext >> r_cyl_int >> height >> height_bead >> mass;

	// Create a parallel SMC system and set the system parameters
	ChVector<> gravity(0, gy, 0);

	double time_step = 1.0E-4;
	double out_step = 2.0E-2;
	double time_sim = 10.0;

	ChSystemParallelSMC msystem;
	SetSimParameters(&msystem, gravity, r_bead);

	// Recreate particles from the set simulation by reading in position/radii data
	std::vector< ChVector<> > list_position;
	std::vector< ChVector<> >* p_list_position(0);
	p_list_position = &list_position;

	std::vector< double> list_radius;
	std::vector<double>* p_list_radius(0);
	p_list_radius = &list_radius;

	read_pos(p_list_position, p_list_radius, path);
	
	// Add the shear-cell structure and fill the cell with particles according to tc_set inputs
	double rotation_speed = 0.096;
	std::pair<size_t, size_t> wlist = set_up(&msystem, r_cyl_int, r_cyl_ext, height, r_bead, mass, rotation_speed);
	std::pair<size_t, size_t> plist = remplir(&msystem, mass, p_list_position, p_list_radius);

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

	// Create a 2D array of ParticleData structs to store state information throughout the simulation
	/*ParticleData **cstate_data = new ParticleData*[num_particles];
	for (size_t i = 0; i < num_particles; ++i) {
		cstate_data[i] = new ParticleData[BUFFER_SIZE];
		memset(cstate_data[i], 0, BUFFER_SIZE * sizeof(ParticleData));
	}*/

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

	/*std::vector< std::shared_ptr< ChBody > > beads_list;
	std::vector< std::shared_ptr< ChBody > >* p_beads_list(0);
	p_beads_list = &beads_list;
	
	ChVectorDynamic<std::shared_ptr<ChBody>>* p_surface(0);
	ChVectorDynamic<std::shared_ptr<ChBody>> surface = ChVectorDynamic<std::shared_ptr<ChBody>>(0);
	
	p_surface = &surface;
	detect_surface(p_beads_list,p_surface, r_bead, time);*/
	
	// create a .dat file with three columns of demo data:
	std::string datafile = out_dir + "/velocity_profile.dat";
	ChStreamOutAsciiFile velocity_profile(datafile.c_str());

	std::string datafile2 = out_dir + "/all_data_surface.dat";
	ChStreamOutAsciiFile all_data_surface(datafile2.c_str());
	
	velocity_profile << "0" << " " << "0" << " " << "id_frame" << " " << "id_part" << " " << "v_r" << " " << "v_t" << " " << "r" << "\n";
	all_data_surface << "time" << " " << "id" << " " << "x" << " " << "y" << " " << "z" << " " << "v_x" << " " << "v_y" << " " << "v_z" << " " << "\n";

	// THE SOFT-REAL-TIME CYCLE
	double time = 0.0;
	double out_time = 0.0;
	int frame = 0;

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
		}

		#ifdef CHRONO_IRRLICHT
			if (application != NULL) {
				application->EndScene();
			}
		#endif

		//detect_surface(p_beads_list, p_surface, r_bead, time);
		
		for (int j = start_plist; j < start_plist + num_particles; ++j) {
			std::shared_ptr<ChBody> body = msystem.Get_bodylist().at(j);

			int id = body->GetIdentifier();
			double v_x = body->GetPos_dt().x();
			double v_y = body->GetPos_dt().y();
			double v_z = body->GetPos_dt().z();
			double x = body->GetPos().x();
			double y = body->GetPos().y();
			double z = body->GetPos().z();
			double theta = atan2(z, x);
			double r = sqrt(x*x +z*z);
			double v_r = v_x * cos(theta) + v_z * sin(theta);
			double v_t = v_z * cos(theta) - v_x * sin(theta);
			
			velocity_profile << 0 << " " << 0 << " " << frame << " " << id << " " << v_r << " " << v_t << " " << r << "\n";
			all_data_surface << time << " " << id << " " << x << " " << y << " " << z << " " << v_x << " " << v_y << " " << v_z << " " << "\n";
		}

		pov_exporter.ExportData();
		frame = frame + 1;
		out_time = time - time_step + out_step;
	}

	// Delete dynamically allocated objects and arrays and return
	#ifdef CHRONO_IRRLICHT
		delete application;
	#endif

	return 0;
}