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
#include "chrono_parallel/physics/ChSystemParallel.h""
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/assets/ChTexture.h"
#include "chrono_postprocess/ChPovRay.h"
#include "chrono_postprocess/ChPovRayAssetCustom.h"

#include "ProjDataPath.h"

#include <ctime>        
#include <iomanip>
#include <random>


using namespace chrono;
using namespace chrono::collision;
using namespace chrono::postprocess;


#ifdef CHRONO_IRRLICHT
	#include <irrlicht.h>
	#include "chrono_irrlicht/ChIrrApp.h"

	using namespace chrono::irrlicht;
	using namespace irr;
	using namespace irr::core;
	using namespace irr::scene;
	using namespace irr::video;
	using namespace irr::io;
	using namespace irr::gui;
#endif


//Creates a bead at a specific pos with a specific mass and radius. Attributes an identifier i to the bead. Eventually fix the bead and give it a texture. Eventually set velocity of the bead. Add the bead to the list p_list.
void create_bead(double r_bead, ChSystemParallelSMC& mphysicalSystem, ChVector<> pos, double mass, bool isFixed, bool isWall, std::vector< std::shared_ptr< ChBody > >* p_list, std::vector <double>* p_ray, int i = 0, bool has_velocity = false) {
	
	//USEFUL ?
	ChQuaternion<> rot(1, 0, 0, 0);
	ChVector<> init_vel(0, 0, 0);
	
	//Creation of the surface material
	auto material = std::make_shared<ChMaterialSurfaceSMC>();
	material->SetRestitution(0.6f);
	material->SetFriction(0.4f);
	material->SetAdhesion(0);

	//Creation of the beads and setting on parameters
	auto body = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetPos_dt(init_vel);
	body->SetMaterialSurface(material);
	body->SetCollide(true);

	//Fix the bead if it is a bead of the outer cylinder
	if (isWall == true) {
		body->SetBodyFixed(true);
	}
	else {
		body->SetBodyFixed(false);
	}

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);

	std::normal_distribution<double> distribution(r_bead, r_bead/100);
	double ray = distribution(generator);
	
	//Sets the collision parameters
	body->GetCollisionModel()->ClearModel();
	body->GetCollisionModel()->AddSphere(ray);
	body->GetCollisionModel()->BuildModel();
	
	//USEFULL ?
	body->SetInertiaXX(0.4 * mass * r_bead * r_bead * ChVector<>(1, 1, 1));

	//Visualisation of the beads that are not part of the outer cylinder.
	if (isWall == false || isFixed == false) {
		auto sphere = std::make_shared<ChSphereShape>();
		sphere->GetSphereGeometry().rad = r_bead;
		body->AddAsset(sphere);

		auto text = std::make_shared<ChTexture>();
		auto mvisual = std::make_shared<ChColorAsset>();

		if (isWall == true) {
			mvisual->SetColor(ChColor(0.48f, 0.71f, 0.38f));
			body->AddAsset(mvisual);
		}
		else {
			text->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
			body->AddAsset(text);
		}
	}
	
	//Gives a random velocity to the bead
	if (has_velocity == true) {
		double vx = ChRandom() * 2;
		double vy =  ChRandom() * 2;
		double vz = ChRandom() * 2;
		ChVector<> v = ChVector<>(vx, vy, vz);
		body->SetPos_dt(v);
		body->SetIdentifier(i);
		p_ray->push_back(ray);
	}

	//Add the bead to the list and the system
	p_list->push_back(body);
	mphysicalSystem.AddBody(body);
}


//Creates the beads of the outer cylinder and makes it invisible for the visualization. Add all the beads to the list pointed by p_list
void create_cylinder_ext(ChSystemParallelSMC& mphysicalSystem, double r_bead, double r_cyl_ext,double height, int methode, double mass, std::vector< std::shared_ptr< ChBody > >* p_list, std::vector <double>* p_ray) {
	
	if (methode == 1) { //Columns arrangement
		
		for (int i = 0; i < floor(CH_C_PI*(r_cyl_ext-r_bead) / r_bead)+1 ; i++) {
			for (int j = 0; j < floor(height / (2 * r_bead)); j = j++) {
				ChVector <> pos = ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,true,true,p_list,p_ray);
			}
		}
	}

	else if (methode == 2) { //More compact arrangement (horizontal shift from a line to an other)
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI*(r_cyl_ext-r_bead)) / r_bead)+1; i++) {
				ChVector<> pos = ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,true,true, p_list,p_ray);

				ChVector<> pos2= ChVector<>((r_cyl_ext - r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * (j + 1) + r_bead, (r_cyl_ext - r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos2, mass,true,true,p_list,p_ray);
			}
		}
	}

	else if (methode == 3) { //Compact arrangement
		for (int j = 0; j < floor((height - r_bead) / (sqrt(3) * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI*(r_cyl_ext-r_bead)) / r_bead)+1  ; i++) {
				ChVector<> pos= ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3)*r_bead * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				
				create_bead(r_bead, mphysicalSystem, pos, mass,true,true,p_list,p_ray);
				if (j + 1 < floor((height - r_bead) / (sqrt(3) * r_bead))) {
					ChVector<> pos2 = ChVector<>((r_cyl_ext - r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3)*r_bead * (j + 1) + r_bead, (r_cyl_ext - r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos2, mass, true, true, p_list,p_ray);
				}
			}
		}
	}

	else fprintf(stderr, "La methode rentree est incorrecte\n");
	
}


//Creates the beads of the inner cylinder and gives it a specific texture. Add all the beads to the list pointed by p_list. Fix the beads to rotatingBody. 
void create_cylinder_int(ChSystemParallelSMC& mphysicalSystem, double r_bead, double r_cyl_int, double height, int methode, double mass, std::vector< std::shared_ptr< ChBody > >* p_list, std::vector <double>* p_ray) {
	
	if (methode == 1) { //Column arrangement
		for (int i = 0; i < floor((CH_C_PI*(r_cyl_int+r_bead)) / r_bead)+1; i++) {
			for (int j = 0; j < floor(height / (2 * r_bead)); j++) {
				
				//Calculation of positions of all the beads
				ChVector<> pos = ChVector<>((r_cyl_int + r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int+r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_int + r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int+r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,false, true, p_list,p_ray);
			}
		}
	}

	else if (methode == 2) { //More compact arrangement
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI*(r_cyl_int+r_bead)) / r_bead)+1; i++) {
				
				//Calculation of positions of all the beads
				ChVector<> pos = ChVector<>((r_cyl_int + r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_int + r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,false,true, p_list,p_ray);

				if (j + 1 < floor(height / (2 * r_bead))) {
					//Calculation of positions of all the beads
					ChVector<> pos2 = ChVector<>((r_cyl_int + r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))), r_bead * 2 * (j + 1) + r_bead, (r_cyl_int + r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos2, mass, false, true, p_list,p_ray);
				}
			}
		}
	}

	else if (methode == 3) { //Compact arrangement
		for (int j = 0; j < floor((height-r_bead) / (sqrt(3) * r_bead)); j = j + 2) {
			for (int i = 0; i < floor(CH_C_PI*(r_cyl_int+r_bead) / r_bead) +1; i++) {

				//Calculation of positions of all the beads
				ChVector<> pos = ChVector<>((r_cyl_int + r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))), sqrt(3)*r_bead * j + r_bead, (r_cyl_int + r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,false,true, p_list,p_ray);

				if (j + 1 < floor((height-r_bead) / (sqrt(3)* r_bead))) {
					//Calculation of positions of all the beads
					ChVector<> pos2 = ChVector<>((r_cyl_int + r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))), sqrt(3)*r_bead * (j + 1) + r_bead, (r_cyl_int + r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos2, mass, false, true, p_list,p_ray);

				}
			}
		}
	}

	else fprintf(stderr, "La methode rentree est incorrecte\n");
}


//Fills the shear-cell with beads that have eventually random speed. Add the bead to the list pointed by p_list. 
void remplir(ChSystemParallelSMC& mphysicalSystem,  double r_bead, double r_cyl_int, double r_cyl_ext, double mass, int methode, double height_bead, std::vector< std::shared_ptr< ChBody > >* p_list, bool has_velocity, std::vector <double>* p_ray) {
	
	int id = 0;
	if (methode == 1) {//Column arrangement
		
		for (int k = 0; k < floor(((r_cyl_ext-2*r_bead) - (r_cyl_int+2*r_bead)) / (2*r_bead)); k++) {
			for (int j = 0; j < floor(height_bead / (2*r_bead)); j = j ++) {
				for (int i = 0; i < floor((CH_C_PI*(r_cyl_int + 3 * r_bead + 2 * k*r_bead)) / r_bead) ; i++) {
					
					//Calculation of positions of all the beads

					ChVector <> pos = ChVector<>((r_cyl_int+3*r_bead+2*k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), r_bead  * 2*j + r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos, mass, false, false, p_list,p_ray,id, has_velocity);
					
					id = id + 1;
				}
			}
		}
	}

	 else if (methode == 2) {//More compact arrangement (horizontal shift from a row of beads to an other
		for (int k = 0; k < floor(((r_cyl_ext - r_bead) - (r_cyl_int + r_bead)) / (2 * r_bead)) - 1; k++) {
			for (int i = 0; i < floor((CH_C_PI*(r_cyl_int + 3 * r_bead + 2 * k*r_bead)) / r_bead); i++) {
				for (int j = 0; j < floor(height_bead / (2 * r_bead)); j = j + 2) {
					//Calculation of positions of all the beads
					ChVector <> pos = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), r_bead+2*j*r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos, mass, false, false, p_list,p_ray, id, has_velocity);
					id = id + 1;

					if (j + 1 < floor(height_bead / (2 * r_bead))) {
						//Calculation of positions of all the beads
						ChVector<> pos2 = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), r_bead+2*(j+1)*r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
						create_bead(r_bead, mphysicalSystem, pos2, mass, false, false, p_list,p_ray, id, has_velocity);
						id = id + 1;
					}
				}
			}
		}
	}

	 else if (methode == 3) { //Compact arrangement
		for (int k = 0; k < floor(((r_cyl_ext - r_bead) - (r_cyl_int + r_bead)) / (2 * r_bead)) - 1; k++) {
			for (int j = 0; j < floor((height_bead - r_bead) / (sqrt(3) * r_bead)); j = j + 2) {
				for (int i = 0; i < floor((CH_C_PI*(r_cyl_int + 3 * r_bead + 2 * k*r_bead)) / r_bead) + 1; i++) {

					//Calculation of positions of all the beads
					ChVector<> pos = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), sqrt(3)*r_bead * j + r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
					
					create_bead(r_bead, mphysicalSystem, pos, mass, false, false, p_list,p_ray,id, has_velocity);
					id = id + 1;

					if (j + 1 < floor((height_bead - r_bead) / (sqrt(3) * r_bead))) {
						//Calculation of positions of all the beads
						ChVector<> pos2 = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), sqrt(3)*r_bead * (j + 1) + r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
						create_bead(r_bead, mphysicalSystem, pos2, mass, false, false, p_list,p_ray,id,has_velocity);
						id = id + 1;
					}
				}
			}
		}
	}

	 else fprintf(stderr, "La methode demandee est incorrecte\n");
}


//Create all the experimental set-up
void create_some_falling_items(ChSystemParallelSMC& mphysicalSystem, double r_cyl_int, double r_cyl_ext, double height, double r_bead, double mass, double height_bead, std::vector< std::shared_ptr< ChBody > >* p_cylinder_ext_list, std::vector< std::shared_ptr< ChBody > >* p_cylinder_int_list, std::vector< std::shared_ptr< ChBody > >* p_beads_list, std::shared_ptr<ChLinkMotorRotationSpeed>* motor, std::vector <double>* p_ray) {

	//Creation of the surface material of the ground
	auto material = std::make_shared<ChMaterialSurfaceSMC>();
	material->SetRestitution(0.1f);
	material->SetFriction(0.4f);
	material->SetAdhesion(0);

	//Creation of the ground + seiling body
	auto fixedBody = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);
	fixedBody->SetMaterialSurface(material);
	fixedBody->SetMass(10.0);
	fixedBody->SetBodyFixed(true);
	fixedBody->SetPos(ChVector<>());
	fixedBody->SetCollide(true);

	//Sets size of the bounding boxes
	ChVector<> hsize = 0.5 * ChVector<>(2 * r_cyl_ext, 1, 2 * r_cyl_ext);

	//Creation of the ground
	auto box1 = std::make_shared<ChBoxShape>();
	box1->GetBoxGeometry().Pos = ChVector<>(0, -0.5, 0);
	box1->GetBoxGeometry().Size = hsize;
	box1->SetColor(ChColor(1, 0, 0));
	box1->SetFading(0.6f);
	fixedBody->AddAsset(box1);

	//Creation of the ceiling
	auto box2 = std::make_shared<ChBoxShape>();
	box2->GetBoxGeometry().Pos = ChVector<>(0, height + 0.5, 0);
	box2->GetBoxGeometry().Size = hsize;
	box2->SetColor(ChColor(1, 0, 0));
	box2->SetFading(0.6f);
	fixedBody->AddAsset(box2);

	//Creation of the inner cylinder
	auto box3 = std::make_shared<ChCylinderShape>();
	box3->GetCylinderGeometry().rad = r_cyl_int;
	box3->GetCylinderGeometry().p1 = ChVector<>(0, height, 0);
	box3->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
	box3->SetColor(ChColor(0, 0, 1));
	box3->SetFading(0.6f);
	fixedBody->AddAsset(box3);

	//End of settings the collision parameters of the ground + seiling body
	fixedBody->GetCollisionModel()->ClearModel();
	fixedBody->GetCollisionModel()->AddBox(hsize.x(), hsize.y(), hsize.z(), ChVector<>(0, -0.5, 0));
	fixedBody->GetCollisionModel()->AddBox(hsize.x(), hsize.y(), hsize.z(), ChVector<>(0, height + 0.5, 0));
	fixedBody->GetCollisionModel()->AddCylinder(r_cyl_int, height / 2, r_cyl_int, ChVector<>(0, 0, 0));
	fixedBody->GetCollisionModel()->BuildModel();

	mphysicalSystem.AddBody(fixedBody);

	// optional, attach a texture for better visualization
	auto mtexture = std::make_shared<ChTexture>();
	mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
	fixedBody->AddAsset(mtexture);

	//creation of all the set-up
	create_cylinder_ext(mphysicalSystem, r_bead, r_cyl_ext, height, 3, mass, p_cylinder_ext_list, p_ray);
	create_cylinder_int(mphysicalSystem, r_bead, r_cyl_int, height_bead, 3, mass, p_cylinder_int_list, p_ray);
	remplir(mphysicalSystem, r_bead, r_cyl_int, r_cyl_ext, mass, 1, height_bead, p_beads_list, true, p_ray);
}


//Fills a ChVectorDynamic with the velocity of all the beads that are in the list pointed by p_beads_list
void create_array_velocity(std::vector< std::shared_ptr< ChBody > >* p_beads_list, ChVectorDynamic<double>* p_tab_v) {
	
	p_tab_v->Reset(p_beads_list->size());

	for (int i = 0; i < p_beads_list->size(); ++i) {
		double v_x= (*p_beads_list)[i]->GetPos_dt().x();
		double v_z = (*p_beads_list)[i]->GetPos_dt().z();
		double v_y = (*p_beads_list)[i]->GetPos_dt().y();
		p_tab_v->SetElementN(i, sqrt(v_x*v_x+v_y*v_y+v_z*v_z));
	}
	
}


//calculate the mean of the vector pointed by p_vector
double mean_vector(ChVectorDynamic<double>* p_vector) {
	double s = 0;
	for (int i = 0; i < p_vector->GetLength(); i++) {
		s = s + p_vector->GetElementN(i);
	}
	return s / p_vector->GetLength();
}


void SetPovrayParameters(ChPovRay* pov_exporter, double x, double y, double z) {
	// Modify the default light and camera
	pov_exporter->SetLight(ChVector<>(-3, 4, 2), ChColor(0.15f, 0.15f, 0.12f), false);
	pov_exporter->SetCamera(ChVector<>(x,y,z), ChVector<>(0, 0, 0), 0.0, false);
	pov_exporter->SetBackground(ChColor(1.0f, 1.0f, 1.0f));

	// Create an area light for soft shadows
	pov_exporter->SetCustomPOVcommandsScript("light_source { <2, 10, -300> color rgb<1.2,1.2,1.2> area_light <4, 0, 0>, <0, 0, 4>, 8, 8 adaptive 1 jitter}");

	// Tell to the POVray exporter to convert the shapes of all items
	pov_exporter->AddAll();
	pov_exporter->ExportScript();
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
	double gravity = -9.81E2;
	double r_bead = 0.2;// 0.5, 0.2
	double r_cyl_ext = 10;//5, 100
	double r_cyl_int = 5;//2, 50
	double height = 7;//9, 5
	double height_bead = 5;//7, 4.5
	double rho = 2.55;
	double mass = rho * (4 / 3)*CH_C_PI*pow(r_bead, 3);
	
	//Paramètres de simulation
	double time_step = 1e-4;//1e-4
	double out_step = 0.02;
	double time = 0;
	double out_time = 0;

	//Writes the settings into the file : settings.dat
	std::string datafile3 = out_dir +"/settings.dat";
	ChStreamOutAsciiFile settings(datafile3.c_str());
	settings << gravity << " " << r_bead << " " << r_cyl_ext << " " << r_cyl_int << " " << height << " " << height_bead << " " << mass << " " << "\n";

	// Create a ChronoENGINE physical system
	ChSystemParallelSMC mphysicalSystem;
	mphysicalSystem.GetSettings()->solver.max_iteration_bilateral = 100;
	mphysicalSystem.GetSettings()->solver.tolerance = 1e-3;

	//msystem->GetSettings()->solver.use_material_properties = false;
	mphysicalSystem.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz; /// Types: Hooke, Hertz, PlainCoulomb, Flores
	mphysicalSystem.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;
	mphysicalSystem.GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::MultiStep;

	mphysicalSystem.GetSettings()->collision.bins_per_axis = vec3(15, 15, 15);
	mphysicalSystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R; /// Types: NARROWPHASE_HYBRID_MPR, NARROWPHASE_R, NARROWPHASE_MPR

	mphysicalSystem.ChangeCollisionSystem(CollisionSystemType::COLLSYS_PARALLEL); /// Types:: COLLSYS_PARALLEL, COLLSYS_BULLET_PARALLEL
	mphysicalSystem.SetTimestepperType(ChTimestepper::Type::LEAPFROG); /// Types: LEAPFROG....
	mphysicalSystem.Set_G_acc(ChVector<>(0, gravity, 0));

	collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(r_bead / 2);

	#ifdef CHRONO_IRRLICHT
		// Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc. etc.)
		ChIrrApp application(&mphysicalSystem, L"Collisions between objects", core::dimension2d<u32>(800, 600), false, true);

		// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
		ChIrrWizard::add_typical_Logo(application.GetDevice());
		ChIrrWizard::add_typical_Sky(application.GetDevice());
		ChIrrWizard::add_typical_Lights(application.GetDevice());
		ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(15,-2, 0)); //30
	#endif

	// Create all the rigid bodies.
	std::shared_ptr<ChLinkMotorRotationSpeed>* motor;
	auto my_motor = std::make_shared<ChLinkMotorRotationSpeed>();
	motor = &my_motor;

	std::vector< std::shared_ptr< ChBody > > cylinder_ext_list;
	std::vector< std::shared_ptr< ChBody > >* p_cylinder_ext_list(0);
	p_cylinder_ext_list = &cylinder_ext_list;
	
	std::vector< std::shared_ptr< ChBody > > cylinder_int_list;
	std::vector< std::shared_ptr< ChBody > >* p_cylinder_int_list(0);
	p_cylinder_int_list = &cylinder_int_list;

	std::vector< std::shared_ptr< ChBody > > beads_list;
	std::vector< std::shared_ptr< ChBody > >* p_beads_list(0);
	p_beads_list = &beads_list;
	
	std::vector< double> radius_list;
	std::vector< double >* p_radius_list(0);
	p_radius_list = &radius_list;
	
	create_some_falling_items(mphysicalSystem, r_cyl_int, r_cyl_ext, height, r_bead, mass, height_bead, p_cylinder_ext_list, p_cylinder_int_list,p_beads_list, motor,p_radius_list);
	//printf("nombre de billes : %i\n", p_beads_list->size());
	fprintf(stderr, "nombre de billes : %i\n", p_beads_list->size());
	ChVectorDynamic<double>* p_tab_v_cyl_int(0);
	ChVectorDynamic<double> tab_v_cyl_int = ChVectorDynamic<double>(0);
	p_tab_v_cyl_int = &tab_v_cyl_int;

	ChVectorDynamic<double>* p_tab_v(0);
	ChVectorDynamic<double> tab_v = ChVectorDynamic<double>(0);
	p_tab_v = &tab_v;
	
	// create a .dat file with three columns of demo data:
	std::string datafile = out_dir + "/mean_v_data.dat";
	ChStreamOutAsciiFile mean_v(datafile.c_str());
	mean_v << "time" << " " << "mean_v" << "\n";
	
	// Create an exporter to POVray and set all associated filepaths and settings 
	ChPovRay pov_exporter = ChPovRay(&mphysicalSystem);
	if (SetPovrayPaths(&pov_exporter, out_dir) != 0) {
		fprintf(stderr, "Error creating povray data paths\n");
		return -1;
	}
	SetPovrayParameters(&pov_exporter, 0, 0, r_cyl_ext*2.5);

	// Use this function for adding a ChIrrNodeAsset to all items and for 'converting' assets into Irrlicht meshes.
	#ifdef CHRONO_IRRLICHT
		application.AssetBindAll();
		application.AssetUpdateAll();
	#endif
	
	// THE SOFT-REAL-TIME CYCLE
	double i = 0.0;
	bool motor_launched = false;
	int id_frame = 0;
	double time_sim = 1.5;
	
	while (time<time_sim) {

		#ifdef CHRONO_IRRLICHT
			application.BeginScene(true, true, SColor(255, 255, 255, 255));
			application.GetDevice()->run();
			application.DrawAll();
		#endif

		while (time == 0 || time < out_time) {
			mphysicalSystem.DoStepDynamics(time_step);
			time += time_step;
		}

		#ifdef CHRONO_IRRLICHT
			application.EndScene();
		#endif

		create_array_velocity(p_beads_list, p_tab_v);
		mean_v << time << " " << mean_vector(p_tab_v) << "\n";
		//fprintf(stderr, "time : %f \tmean_v : %f\n", time, mean_vector(p_tab_v));
		
		pov_exporter.ExportData();

		if (mean_vector(p_tab_v) < 0.001) break;
		out_time = time - time_step + out_step;
	}

	fprintf(stderr, "time : %f \tmean_v : %f\n", time, mean_vector(p_tab_v));

	std::string datafile2 = out_dir + "/position.dat";
	std::ofstream position(datafile2, std::ios::out | std::ios::trunc);

	if (position) {
		for (int j = 0; j < p_beads_list->size(); j++) {
			position << (*p_beads_list)[j]->GetPos().x() << " " << (*p_beads_list)[j]->GetPos().y() << " " << (*p_beads_list)[j]->GetPos().z() << " " << (*p_radius_list)[j] << "\n";
			// printf("rayon : %f\n", (*p_radius_list)[j]);
		}

		position << -100000000 << " " << -100000000 << " " << -100000000 << -100000000 << "\n";
		position.close();
	}

	fprintf(stderr, "Les positions ont bien ete enregistrees\n");

	return 0;
}

