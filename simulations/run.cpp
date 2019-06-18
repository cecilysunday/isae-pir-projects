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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
//     - collisions and contacts
//     - using Irrlicht to display objects.
//
// =============================================================================

//#include "chrono/ChConfig.h"
#include "chrono/physics/ChBody.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include <irrlicht.h>
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/assets/ChTexture.h"
#include <chrono_postprocess/ChGnuPlot.h>
#include "chrono_postprocess/ChPovRay.h"
#include "chrono_postprocess/ChPovRayAssetCustom.h"
#include <ctime>        
#include <iomanip>
#include "chrono_thirdparty/filesystem/path.h"

//#include <cstdio>
//#include <vector>
//#include <cmath>

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::postprocess;
#ifdef  CHRONO_IRRLICHT
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui; 
#endif
//using namespace std;

std::string SetDataPath(std::string projname, bool archive) {
	// Create a timestamped string to use when creating new data output path
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
	auto timestamp = oss.str();

	// Create the output data path. If the directory provided by the user exists, add a timestamped folder to the specified directory
	// If the path does not exist, create a timestamped folder in the to current working directory
	std::string out_dir_temp = PROJECT_DATA_DIR;

	if (archive) {
		out_dir_temp = out_dir_temp + "/TEMP" + projname;
	}
	else if (!archive) {
		out_dir_temp = out_dir_temp + "/" + timestamp + projname;
	}

	const std::string out_dir = out_dir_temp;
	const std::string out_dir_log = out_dir + "/userlog.txt";

	chrono::GetLog() << "\IM HERE 1";
	// Create the directory according to the output data path  and write a copy of the consule log to the directory
	auto out_path = filesystem::path(out_dir);
	filesystem::create_directory(out_path);
	chrono::GetLog() << "\IM HERE 2";
	if (!out_path.exists()) {
		return "";
	}
	else {
		fflush(stdout);
		freopen(out_dir_log.c_str(), "w", stdout);
	}

	chrono::GetLog() << "\IM HERE 3";
	// Set the Chrono data and output paths so that this information can be access by other functions
	SetChronoDataPath(CHRONO_DATA_DIR);
	SetChronoOutputPath(out_dir);

	chrono::GetLog() << out_dir;
	return out_dir;
}



int SetPovrayPaths(ChPovRay* pov_exporter, const std::string out_dir) {
	// Sets some file names for in-out processes
	pov_exporter->SetTemplateFile(GetChronoDataFile("_template_POV.pov"));
	pov_exporter->SetOutputScriptFile(out_dir + "/rendering_frames.pov");

	// Save the .dat files and the .bmp files in two subdirectories
	const std::string povout_dir = out_dir + "/output";
	const std::string anim_dir = out_dir + "/anim";

	auto output = filesystem::path(povout_dir);
	auto anim = filesystem::path(anim_dir);

	filesystem::create_directory(output);
	filesystem::create_directory(anim);

	if (!output.exists() || !anim.exists()) {
		return -1;
	}

	// Sets some file names for the output povray state and image files
	pov_exporter->SetOutputDataFilebase(povout_dir + "/my_state_");
	pov_exporter->SetPictureFilebase(anim_dir + "/img_");

	return 0;
}

//Read the position in a given file, and stock it into the list pointed by p_list_pos
void read_pos(std::vector<ChVector<>>* p_list_pos,std::vector<double>* p_radius) {
	printf("On arrive avant de lire le fichier\n");
	std::ifstream fichier("C:/Users/jules/Documents/PIR/Simulations/Test5_build/bin/Release/position.dat");
	printf("On passe la lecture du fichier\n");
	int i = 0;
	bool end_of_doc = false;
	double x;
	double y;
	double z;
	double radius;
	while (end_of_doc==false){
		
		fichier >> x >> y>> z;
		printf("(%f,%f,%f)", x, y, z);
		if (x != -100000000 && y != -100000000 && z != -100000000 && radius != -100000000) {
			p_list_pos->push_back(ChVector<>(x, y, z));
			p_radius->push_back(radius);
		}
		else {
			end_of_doc = true;
		}
		printf("On rentre dans la boucle : %i\n",i);
		i = i + 1;
	}
	
}


void create_bead(double r_bead, ChSystemParallelSMC& mphysicalSystem, ChVector<> pos, double mass, bool isFixed, bool isWall, std::vector< std::shared_ptr< ChBody > >* p_list, int i=0, bool has_velocity = false) {
	ChQuaternion<> rot(1, 0, 0, 0);
	ChVector<> init_vel(0, 0, 0);
	
	auto material = std::make_shared<ChMaterialSurfaceSMC>();
	material->SetRestitution(0.1f);
	material->SetFriction(0.4f);
	material->SetAdhesion(0);

	auto body = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);
	
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetPos_dt(init_vel);
	if (isFixed == true) {
		body->SetBodyFixed(true);
	}
	else {
		body->SetBodyFixed(false);
	}
	body->SetMaterialSurface(material);
	body->SetCollide(true);


	body->GetCollisionModel()->ClearModel();
	body->GetCollisionModel()->AddSphere(r_bead);
	body->GetCollisionModel()->BuildModel();
	
	body->SetInertiaXX(0.4 * mass * r_bead * r_bead * ChVector<>(1, 1, 1));
	auto sphere = std::make_shared<ChSphereShape>();
	sphere->GetSphereGeometry().rad = r_bead;
	sphere->SetColor(ChColor(0.9f, 0.4f, 0.2f));
	body->AddAsset(sphere);

	
	auto text = std::make_shared<ChTexture>();
	if (isWall == true) {
		text->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
		
		
		
	}
	else {
		text->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
		body->SetId(i);
	}
	body->AddAsset(text);
	
	
	if (has_velocity == true) {
		double vx = ChRandom()*10;
		double vy = ChRandom() * 10;
		double vz = ChRandom() * 10;
		body->SetPos_dt(ChVector<>(vx, vy, vz));

	}
	p_list->push_back(body);
	mphysicalSystem.AddBody(body);
}

void create_cylinder_ext(ChSystemParallelSMC& mphysicalSystem, double r_bead, double r_cyl_ext,double height, int methode, double mass, std::vector< std::shared_ptr< ChBody > >* p_list) {
	
	if (methode == 1) { //Remplissage en colonne
		
		for (int i = 0; i < floor(CH_C_PI*(r_cyl_ext-r_bead) / r_bead) ; i++) {
			for (int j = 0; j < floor(height / (2 * r_bead)); j = j++) {
				ChVector <> pos = ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,true,true, p_list);
			}
		}
	}

	else if (methode == 2) { //Remplissage en décalé sans contact vertical
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI*(r_cyl_ext-r_bead)) / r_bead)+1; i++) {
				ChVector<> pos = ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,true,true, p_list);

				ChVector<> pos2= ChVector<>((r_cyl_ext - r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * (j + 1) + r_bead, (r_cyl_ext - r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos2, mass,true,true,p_list);
			}
		}
	}

	else if (methode == 3) { //Remplissage en décalé avec contact vertical
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI*(r_cyl_ext-r_bead)) / r_bead)+1  ; i++) {
				ChVector<> pos= ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3)*r_bead * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				
				create_bead(r_bead, mphysicalSystem, pos, mass,true,true,p_list);
				
				ChVector<> pos2 = ChVector<>((r_cyl_ext - r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3)*r_bead * (j + 1) + r_bead, (r_cyl_ext - r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos2, mass,true, true,p_list);
			}
		}
	}

	else {
		printf("La methode rentree est incorrecte\n");
	}
	
}

void create_cylinder_int(ChSystemParallelSMC& mphysicalSystem, double r_bead, double r_cyl_int, double height, int methode, std::shared_ptr<chrono::ChBodyFrame> rotatingBody, double mass, std::vector< std::shared_ptr< ChBody > >* p_list) {
	

	
	if (methode == 1) { //Remplissage en colonne
		for (int i = 0; i < floor((CH_C_PI*(r_cyl_int+r_bead)) / r_bead) ; i++) {
			for (int j = 0; j < floor(height / (2 * r_bead)); j++) {
				ChVector<> pos = ChVector<>((r_cyl_int + r_bead)*cos(i*(2 * atan(r_bead / r_cyl_int))), r_bead * 2 * j + r_bead, (r_cyl_int + r_bead)*sin(i*(2 * atan(r_bead / r_cyl_int))));
				create_bead(r_bead, mphysicalSystem, pos, mass,false, true, p_list);
				
				auto lock = std::make_shared<ChLinkMateFix>();
				lock->Initialize(mphysicalSystem.Get_bodylist().back(), rotatingBody);
				mphysicalSystem.AddLink(lock);
			}
		}
	}

	else if (methode == 2) { //Remplissage en décalé sans contact vertical
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI*(r_cyl_int+r_bead)) / r_bead) +1; i++) {
				ChVector<> pos = ChVector<>((r_cyl_int + r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_int + r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,false,true, p_list);
				
				auto lock = std::make_shared<ChLinkMateFix>();
				lock->Initialize(mphysicalSystem.Get_bodylist().back(), rotatingBody);
				mphysicalSystem.AddLink(lock);

				ChVector<> pos2 = ChVector<>((r_cyl_int + r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))), r_bead * 2 * (j + 1) + r_bead, (r_cyl_int + r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos2, mass,false,true, p_list);

				auto lock2 = std::make_shared<ChLinkMateFix>();
				lock2->Initialize(mphysicalSystem.Get_bodylist().back(), rotatingBody);
				mphysicalSystem.AddLink(lock2);
			}
		}
	}

	else if (methode == 3) { //Remplissage en décalé avec contact vertical
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor( CH_C_PI*(r_cyl_int+r_bead) / r_bead) +1; i++) {

				ChVector<> pos = ChVector<>((r_cyl_int + r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))), sqrt(3)*r_bead * j + r_bead, (r_cyl_int + r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,false,true, p_list);

				auto lock = std::make_shared<ChLinkMateFix>();
				lock->Initialize(mphysicalSystem.Get_bodylist().back(), rotatingBody);
				mphysicalSystem.AddLink(lock);

				if (j + 1 < floor(height / (2 * r_bead))) {
					ChVector<> pos2 = ChVector<>((r_cyl_int + r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))), sqrt(3)*r_bead * (j + 1) + r_bead, (r_cyl_int + r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos2, mass, false, true, p_list);

					auto lock2 = std::make_shared<ChLinkMateFix>();
					lock2->Initialize(mphysicalSystem.Get_bodylist().back(), rotatingBody);
					mphysicalSystem.AddLink(lock2);
				}
			}
		}
	}

	else {
		printf("La methode rentree est incorrecte\n");
	}
}

void remplir(ChSystemParallelSMC& mphysicalSystem, double r_bead, double r_cyl_int, double r_cyl_ext, double mass, int methode, double height_bead, std::vector< std::shared_ptr< ChBody > >* p_list, std::vector<ChVector<>>* p_list_pos, std::vector<double>* p_radius) {
	for (int i = 0; i < p_list_pos->size(); i++) {
		ChVector <> pos = (*p_list_pos)[i];
		double radius = (*p_radius)[i];
		create_bead(radius, mphysicalSystem, pos, mass, false, false, p_list,i);
	}

}

void create_some_falling_items(ChSystemParallelSMC& mphysicalSystem, double r_cyl_int, double r_cyl_ext, double height, double r_bead, double mass, double height_bead, std::vector< std::shared_ptr< ChBody > >* p_cylinder_ext_list, std::vector< std::shared_ptr< ChBody > >* p_cylinder_int_list, std::vector< std::shared_ptr< ChBody > >* p_beads_list, std::shared_ptr<ChLinkMotorRotationSpeed>* motor, std::vector<ChVector<>>* p_list_pos, std::vector<double>* p_radius) {

	//Création du sol
	auto material = std::make_shared<ChMaterialSurfaceSMC>();
	material->SetRestitution(0.1f);
	material->SetFriction(0.4f);
	material->SetAdhesion(0);

	auto fixedBody = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);

	fixedBody->SetMass(1.0);
	fixedBody->SetBodyFixed(true);
	fixedBody->SetPos(ChVector<>(0, -0.5, 0));
	fixedBody->SetRot(Q_from_AngY(0.0));
	fixedBody->SetCollide(true);
	fixedBody->SetMaterialSurface(material);
	fixedBody->GetCollisionModel()->ClearModel();
	fixedBody->GetCollisionModel()->AddBox(r_cyl_ext, 1, r_cyl_ext, fixedBody->GetPos(), fixedBody->GetRot());
	fixedBody->GetCollisionModel()->BuildModel();
	
	auto cylinder = std::make_shared<ChBoxShape>();
	cylinder->GetBoxGeometry().Size = ChVector<>(r_cyl_ext, 0.5, r_cyl_ext);
	cylinder->GetBoxGeometry().Pos = ChVector<>(0, 0, 0);
	cylinder->SetColor(ChColor(1, 0, 1));
	cylinder->SetFading(0.6f);
	fixedBody->AddAsset(cylinder);

	auto textcyl = std::make_shared<ChTexture>();
	textcyl->SetTextureFilename(GetChronoDataFile("blu.png"));
	fixedBody->AddAsset(textcyl);
	mphysicalSystem.AddBody(fixedBody);
	// Add the rotating mixer

	auto rotatingBody = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC); //FIXME

	rotatingBody->SetMass(10.0);
	rotatingBody->SetInertiaXX(ChVector<>(50, 50, 50));
	rotatingBody->SetPos(ChVector<>(0, -1, 0));
	rotatingBody->SetCollide(true);
	rotatingBody->SetMaterialSurface(material);

	rotatingBody->GetCollisionModel()->ClearModel();
	rotatingBody->GetCollisionModel()->AddCylinder(r_cyl_int, r_cyl_int, height, rotatingBody->GetPos(), rotatingBody->GetRot());
	rotatingBody->GetCollisionModel()->BuildModel();

	auto box = std::make_shared<ChCylinderShape>();
	box->GetCylinderGeometry().rad = r_cyl_int;
	box->GetCylinderGeometry().p1 = ChVector<>(0, height - 1, 0);
	box->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
	box->SetColor(ChColor(0, 0, 1));
	box->SetFading(0.6f);
	rotatingBody->AddAsset(box);

	mphysicalSystem.AddBody(rotatingBody);
	

	// .. a motor between mixer and truss

	//auto my_motor = std::make_shared<ChLinkMotorRotationSpeed>();
	(*motor)->Initialize(rotatingBody, fixedBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
	auto mfun = std::make_shared<ChFunction_Const>(CH_C_PI/2.0);  // speed w=90°/s CH_C_PI / 2.0
	//auto mfun = std::make_shared<ChFunction_Const>(0);
	(*motor)->SetSpeedFunction(mfun);
	//(*motor)->SetAvoidAngleDrift(0);
	mphysicalSystem.AddLink(*motor);

	// optional, attach a texture for better visualization
	auto mtexture = std::make_shared<ChTexture>();
	mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
	rotatingBody->AddAsset(mtexture);


	create_cylinder_ext(mphysicalSystem,  r_bead, r_cyl_ext, height, 3, mass, p_cylinder_ext_list);



	create_cylinder_int(mphysicalSystem, r_bead, r_cyl_int, height_bead, 3, rotatingBody, mass, p_cylinder_int_list);


	remplir(mphysicalSystem, r_bead, r_cyl_int, r_cyl_ext, mass, 3, height_bead, p_beads_list, p_list_pos,p_radius);

	


}

void create_array_velocity(std::vector< std::shared_ptr< ChBody > >* p_beads_list, ChVectorDynamic<double>* p_tab_v_r, ChVectorDynamic<double>* p_tab_v_t, ChVectorDynamic<double>* p_tab_r, ChVectorDynamic<double>* p_tab_theta, ChVectorDynamic<int>* p_tab_id,double height_bead, double r_bead) {
	
	vector<std::shared_ptr<ChBody>> surface;
	
	
	for (int i = 0; i<p_beads_list->size(); ++i) {
		
		
		bool au_dessus = false;
		double y = (*p_beads_list)[i]->GetPos().y();
		double x = (*p_beads_list)[i]->GetPos().x();
		double z = (*p_beads_list)[i]->GetPos().z();
		
		for (int j=0; j<p_beads_list->size(); ++j) {
			
			double y2 = (*p_beads_list)[j]->GetPos().y();
			double x2 = (*p_beads_list)[j]->GetPos().x();
			double z2 = (*p_beads_list)[j]->GetPos().z();
			if ((y2 > y +2* r_bead) && sqrt((x2-x)*(x2-x)+(z2-z)*(z2-z))<r_bead/2 && au_dessus==false) {
				au_dessus = true;
			}
		}
		if (au_dessus == false) {
			surface.push_back((*p_beads_list)[i]);
			
		}
	
	}
	
	
	
	p_tab_r->Reset(surface.size());
	p_tab_v_r->Reset(surface.size());
	p_tab_v_t->Reset(surface.size());
	p_tab_theta->Reset(surface.size());
	p_tab_id->Reset(surface.size());
	
	for (int i = 0; i < surface.size(); ++i) {
		double v_x= surface[i]->GetPos_dt().x();
		double v_z = surface[i]->GetPos_dt().z();
		double theta = atan2(surface[i]->GetPos().z(),surface[i]->GetPos().x());
		//printf("v : %f\n", sqrt(v_x*v_x+v_z*v_z));
		
		
		double r = sqrt(surface[i]->GetPos().x()*surface[i]->GetPos().x() + surface[i]->GetPos().z()*surface[i]->GetPos().z());
		double v_r = v_x * cos(theta) + v_z * sin(theta);
		double v_t = v_z * cos(theta) - v_x * sin(theta);
		
		p_tab_r->SetElementN(i, r);
		p_tab_v_r->SetElementN(i, v_r);
		p_tab_v_t->SetElementN(i, v_t);
		p_tab_theta->SetElementN(i, theta);
		p_tab_id->SetElementN(i, surface[i]->GetId());
		

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
	printf("mean_v : %f\n", mean_v);
	return (!(mean_v-1.0<0.0001 && mean_v-1.0>-0.0001) );
}

void vel_by_radius(ChVectorDynamic<double>* p_tab_vel_r, ChVectorDynamic<double>* p_tab_vel_t, ChVectorDynamic<double>* p_r, ChVectorDynamic<double>* p_mean__v_r, ChVectorDynamic<double>* p_mean_v_t, double r_cyl_int, double r_bead, double r_cyl_ext) {
	ChVectorDynamic<double> tab_v_r_loc;
	ChVectorDynamic<double> tab_v_t_loc;
	for (int i = 0; i < floor(r_cyl_ext - r_cyl_int / (2 * r_bead)); i++) {
		double r_considere = r_cyl_int + r_bead + 2 * i*r_bead;
		tab_v_r_loc.Reset();
		tab_v_t_loc.Reset();
		double s_v_r_loc=0;
		double s_v_t_loc=0;
		int c = 0;
		for (int j = 0; j < p_tab_vel_t->GetLength(); j++) {
			if (p_r->GetElementN(j) - r_considere - r_bead < 0.001 && p_r->GetElementN(j) - r_considere - r_bead < -0.001) {
				s_v_r_loc = s_v_r_loc + p_tab_vel_r->GetElementN(i);
				s_v_t_loc = s_v_t_loc + p_tab_vel_t->GetElementN(i);
				c = c + 1;
			}
		}
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

int main(int argc, char* argv[]) {
	// Set the output data directory. dontcare = false when a timestamped directory is desired
	bool dontcare = true;
	std::string projname = "_run";

	const std::string out_dir = SetDataPath(projname, dontcare);

	if (out_dir == "") {
		fprintf(stderr, "Error creating output data directory\n");
		return -1;
	}
	
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";



	//Déclaration des paramètres
	double gravity = -9.81;
	double r_bead = 1;
	double r_cyl_ext = 10;
	double r_cyl_int = 2;
	double height = 10;
	double height_bead = 10;
	double mass = 1;
	double rotation_speed = CH_C_PI / 2.0;

	std::ifstream fichier("C:/Users/jules/Documents/CH_C_PIR/Simulations/Test5_build/bin/Release/settings.dat");
	fichier >> gravity >> r_bead>> r_cyl_ext >> r_cyl_int >> height >> height_bead >> mass;
	//Paramètres de simulation
	double time_step = 1e-4;//1e-4
	double out_step = 0.02;
	double time = 0;
	double out_time = 0;
	double time_sim = 1.0;

	SetChronoDataPath(CHRONO_DATA_DIR);




	// Create a ChronoENGINE physical system
	ChSystemParallelSMC mphysicalSystem;
	/*mphysicalSystem.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
	//mphysicalSystem.SetContactForceModel(ChSystemParallelSMC::ContactForceModel::Hertz);
	mphysicalSystem.SetAdhesionForceModel(ChSystemParallelSMC::AdhesionForceModel::Constant);*/
	mphysicalSystem.GetSettings()->solver.max_iteration_bilateral = 100;
	mphysicalSystem.GetSettings()->solver.tolerance = 1e-3;

	//msystem->GetSettings()->solver.use_material_properties = false;
	mphysicalSystem.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz; /// Types: Hooke, Hertz, PlainCoulomb, Flores
	mphysicalSystem.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;
	mphysicalSystem.GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::MultiStep;

	mphysicalSystem.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
	mphysicalSystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R; /// Types: NARROWPHASE_HYBRID_MPR, NARROWPHASE_R, NARROWPHASE_MPR

	mphysicalSystem.ChangeCollisionSystem(CollisionSystemType::COLLSYS_PARALLEL); /// Types:: COLLSYS_PARALLEL, COLLSYS_BULLET_PARALLEL
	mphysicalSystem.SetTimestepperType(ChTimestepper::Type::LEAPFROG); /// Types: LEAPFROG....
	mphysicalSystem.Set_G_acc(ChVector<>(0, gravity, 0));

	// Create an exporter to POVray and set all associated filepaths and settings 
	ChPovRay pov_exporter = ChPovRay(&mphysicalSystem);
	if (SetPovrayPaths(&pov_exporter, out_dir) != 0) {
		fprintf(stderr, "Error creating povray data paths\n");
		return -1;
	}
	SetPovrayParameters(&pov_exporter, 0, 30, 0);
	// Create the Irrlicht visualization (open the Irrlicht device,
	// bind a simple user interface, etc. etc.)
#ifdef CHRONO_IRRLICHT
	ChIrrApp application(&mphysicalSystem, L"Collisions between objects", core::dimension2d<u32>(800, 600), false, true);
	//ChIrrApp application(&mphysicalSystem, L"ChLinkLockPlanePlane", irr::core::dimension2d<irr::u32>(800, 600), false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 30, 0));
#endif
	//application.AddTypicalCamera(irr::core::vector3df(300, 0, 300));



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

	std::vector< ChVector<> > list_position;
	std::vector< ChVector<> >* p_list_position(0);
	p_list_position = &list_position;

	std::vector< double> list_radius;
	std::vector<double>* p_list_radius(0);
	p_list_radius = &list_radius;

	read_pos(p_list_position,p_list_radius);
	create_some_falling_items(mphysicalSystem, r_cyl_int, r_cyl_ext, height, r_bead, mass, height_bead, p_cylinder_ext_list, p_cylinder_int_list, p_beads_list, motor, p_list_position,p_list_radius);

	ChVectorDynamic<double>* p_tab_vel_r(0);
	ChVectorDynamic<double> tab_vel_r = ChVectorDynamic<double>(0);
	p_tab_vel_r = &tab_vel_r;

	ChVectorDynamic<double>* p_tab_vel_t(0);
	ChVectorDynamic<double> tab_vel_t = ChVectorDynamic<double>(0);
	p_tab_vel_t = &tab_vel_t;

	ChVectorDynamic<double>* p_tab_r(0);
	ChVectorDynamic<double> tab_r = ChVectorDynamic<double>(0);
	p_tab_r = &tab_r;

	ChVectorDynamic<double>* p_tab_theta(0);
	ChVectorDynamic<double> tab_theta = ChVectorDynamic<double>(0);
	p_tab_theta = &tab_theta;

	ChVectorDynamic<int>* p_tab_id(0);
	ChVectorDynamic<int> tab_id = ChVectorDynamic<int>(0);
	p_tab_id = &tab_id;

	create_array_velocity(p_beads_list, p_tab_vel_r, p_tab_vel_t, p_tab_r, p_tab_theta, p_tab_id, height_bead, r_bead);



	std::string filename = "graphes.gpl";
	ChGnuPlot mplot(filename.c_str());
	mplot.SetGrid();

	// create a .dat file with three columns of demo data:
	std::string datafile = "test_gnuplot_data.dat";
	ChStreamOutAsciiFile mdatafile(datafile.c_str());

	collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(r_bead / 2);

	// Use this function for adding a ChIrrNodeAsset to all items
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
#ifdef CHRONO_IRRLICHT
	application.AssetBindAll();

	// Use this function for 'converting' assets into Irrlicht meshes
	application.AssetUpdateAll();
#endif
	//application.SetStepManage(true);
	//application.SetTimestep(0.02);

	//
	// THE SOFT-REAL-TIME CYCLE
	double i = 0.0;
	bool motor_launched = false;
	bool in_mouvement = true;
	int id_frame = 0;

	while (time < time_sim) {

#ifdef CHRONO_IRRLICHT
		application.BeginScene(true, true, SColor(255, 255, 255, 255));
		application.GetDevice()->run();
		application.DrawAll();
#endif
		create_array_velocity(p_beads_list, p_tab_vel_r, p_tab_vel_t, p_tab_r, p_tab_theta, p_tab_id, height_bead, r_bead);
		

		for (int j = 0; j < p_tab_vel_r->GetLength(); j++) {
			mdatafile << 0 << ", " << 0 << "," << id_frame << "," << p_tab_id->GetElementN(j) << "," << p_tab_vel_r->GetElementN(j) << ", " << p_tab_vel_t->GetElementN(j) << "," << p_tab_r->GetElementN(j) << "\n";
		}

		id_frame = id_frame + 1;
		while (time == 0 || time < out_time) {
			mphysicalSystem.DoStepDynamics(time_step);
			time += time_step;
		}

		pov_exporter.ExportData();

#ifdef CHRONO_IRRLICHT
		application.EndScene();
#endif

		out_time = time - time_step + out_step;
	}
	return 0;
}
/*#ifdef IRRLICHT
	while (application.GetDevice()->run()) {
		application.BeginScene();
		ChIrrTools::drawSegment(application.GetVideoDriver(), ChVector<>(0, -100, 0), ChVector<>(0, 100, 0), irr::video::SColor(255, 0, 0, 0), true);
		ChIrrTools::drawGrid(application.GetVideoDriver(), 0.2, 0.2, 20, 20,
			ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
			video::SColor(255, 80, 100, 100), true);
		application.DrawAll();
		
		std::string time_s = "time : " +std::to_string(time);
		char *cstr = new char[time_s.length() + 1];
		time_s.copy(cstr, time_s.length());
		cstr[time_s.length()] = '\0';
		strcpy(cstr, time_s.c_str());
		printf("time :%f\n", time);
		
		if (time > 0.5 && motor_launched==false) {
			in_mouvement = is_in_mouvement(p_beads_list);
			
		}

		if (in_mouvement==false && motor_launched==false) {
			
			auto mfun = std::make_shared<ChFunction_Const>(rotation_speed);  
			(*motor)->SetSpeedFunction(mfun);
			motor_launched = true;

		}
		//printf("On arrive ici : %i\n", mphysicalSystem.Get_bodylist().size());
		//mphysicalSystem.RemoveBody(mphysicalSystem.Get_bodylist()[mphysicalSystem.Get_bodylist().size()-1]);

		if (motor_launched == true && (time - i<0.001 && time - i>-0.001)) {
			printf("on cree les graphes\n");
			create_array_velocity(p_beads_list, p_tab_vel_r, p_tab_vel_t, p_tab_r, p_tab_theta,p_tab_id, height_bead, r_bead);
			//mplot.Plot(*p_tab_r, *p_tab_vel_t, cstr, "with points");
			
			for (int j = 0; j < p_tab_vel_r->GetLength(); j++) {
				mdatafile << 0 << ", " << 0 << "," << id_frame << "," << p_tab_id->GetElementN(j) << "," << p_tab_vel_r->GetElementN(j) << ", " << p_tab_vel_t->GetElementN(j) << "," << p_tab_r->GetElementN(j) << "\n";
			}
			id_frame = id_frame + 1;
		}

		if (time - i<0.001 && time - i>-0.001) {
			i = i + 0.5;
			printf("i : %f\n", i);
		}

		while (time < out_time) {
			mphysicalSystem.DoStepDynamics(time_step);
			time += time_step;
		}
		out_time += out_step;

		application.EndScene();
	}
#endif
	
	return 0;
}
*/
