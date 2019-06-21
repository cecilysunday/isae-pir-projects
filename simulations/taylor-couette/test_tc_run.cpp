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
#include "chrono/physics/ChBody.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
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


//Read the position in a given file, and stock it into the list pointed by p_list_pos
void read_pos(std::vector<ChVector<>>* p_list_pos,std::vector<double>* p_radius, const std::string out_dir) {
	
	//fprintf(stderr, "On arrive avant de lire le fichier\n");
	std::ifstream fichier(out_dir + "/position.dat");
	//std::ifstream fichier( out_dir + "/../TEMP_set/position.dat");
	//fprintf(stderr, "On passe la lecture du fichier\n");

	int i = 0;
	bool end_of_doc = false;
	double x;
	double y;
	double z;
	double radius;
	
	while (end_of_doc==false){
		
		fichier >> x >> y>> z >> radius;
		//fprintf(stderr, "(%f,%f,%f)", x, y, z);
		if (x != -100000000 && y != -100000000 && z != -100000000 && radius != -100000000) {
			p_list_pos->push_back(ChVector<>(x, y, z));
			p_radius->push_back(radius);
		}
		else {
			end_of_doc = true;
		}
		//fprintf(stderr, "On rentre dans la boucle : %i\n",i);
		i = i + 1;
	}
	
}


void create_bead(double r_bead, ChSystemParallelSMC& mphysicalSystem, ChVector<> pos, double mass, bool isFixed, bool isWall, std::vector< std::shared_ptr< ChBody > >* p_list, int i=0) {
	ChQuaternion<> rot(1, 0, 0, 0);
	ChVector<> init_vel(0, 0, 0);
	
	auto material = std::make_shared<ChMaterialSurfaceSMC>();
	material->SetRestitution(0.6f);
	material->SetFriction(0.4f);
	material->SetAdhesion(0);

	auto body = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);
	
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetMaterialSurface(material);
	body->SetPos_dt(init_vel);
	body->SetInertiaXX(0.4 * mass * r_bead * r_bead * ChVector<>(1, 1, 1));
	body->SetCollide(true);
	if (isFixed == true) {
		body->SetBodyFixed(true);
	}
	else {
		body->SetBodyFixed(false);
	}

	body->GetCollisionModel()->ClearModel();
	body->GetCollisionModel()->AddSphere(r_bead);
	body->GetCollisionModel()->BuildModel();
	
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
		body->SetIdentifier(i);
		body->AddAsset(text);
		//p_list->push_back(body);
	}

	p_list->push_back(body);
	mphysicalSystem.AddBody(body);
}


void create_cylinder_ext(ChSystemParallelSMC& mphysicalSystem, double r_bead, double r_cyl_ext, double height, int methode, double mass, std::vector< std::shared_ptr< ChBody > >* p_list) {

	if (methode == 1) { //Columns arrangement

		for (int i = 0; i < floor(CH_C_PI*(r_cyl_ext - r_bead) / r_bead) + 1; i++) {
			for (int j = 0; j < floor(height / (2 * r_bead)); j = j++) {
				ChVector <> pos = ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass, true, true, p_list);
			}
		}
	}

	else if (methode == 2) { //More compact arrangement (horizontal shift from a line to an other)
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI*(r_cyl_ext - r_bead)) / r_bead) + 1; i++) {
				ChVector<> pos = ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass, true, true, p_list);

				ChVector<> pos2 = ChVector<>((r_cyl_ext - r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * (j + 1) + r_bead, (r_cyl_ext - r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos2, mass, true, true, p_list);
			}
		}
	}

	else if (methode == 3) { //Compact arrangement
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((CH_C_PI*(r_cyl_ext - r_bead)) / r_bead) + 1; i++) {
				ChVector<> pos = ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3)*r_bead * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));

				create_bead(r_bead, mphysicalSystem, pos, mass, true, true, p_list);
				if (j + 1 < floor(height / (2 * r_bead))) {
					ChVector<> pos2 = ChVector<>((r_cyl_ext - r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3)*r_bead * (j + 1) + r_bead, (r_cyl_ext - r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos2, mass, true, true, p_list);
				}
			}
		}
	}

	else fprintf(stderr, "La methode rentree est incorrecte\n");
}

void create_cylinder_int(ChSystemParallelSMC& mphysicalSystem, double r_bead, double r_cyl_int, double height, int methode, std::shared_ptr<chrono::ChBodyFrame> rotatingBody, double mass, std::vector< std::shared_ptr< ChBody > >* p_list) {
	
	if (methode == 1) { //Remplissage en colonne
		for (int i = 0; i < floor((CH_C_PI*(r_cyl_int+r_bead)) / r_bead) +1; i++) {
			for (int j = 0; j < floor(height / (2 * r_bead)); j++) {
				ChVector<> pos = ChVector<>((r_cyl_int + r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int+r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_int + r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int+r_bead)))));
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
		for (int j = 0; j < floor((height-r_bead) / (sqrt(3) * r_bead)); j = j + 2) {
			for (int i = 0; i < floor( CH_C_PI*(r_cyl_int+r_bead) / r_bead) +1; i++) {

				ChVector<> pos = ChVector<>((r_cyl_int + r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))), sqrt(3)*r_bead * j+r_bead, (r_cyl_int + r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,false,true, p_list);

				auto lock = std::make_shared<ChLinkMateFix>();
				lock->Initialize(mphysicalSystem.Get_bodylist().back(), rotatingBody);
				mphysicalSystem.AddLink(lock);

				if (j + 1 < floor((height-r_bead) / (sqrt(3)* r_bead))) {
					ChVector<> pos2 = ChVector<>((r_cyl_int + r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))), sqrt(3)*r_bead * (j + 1)+r_bead, (r_cyl_int + r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_int + r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos2, mass, false, true, p_list);

					auto lock2 = std::make_shared<ChLinkMateFix>();
					lock2->Initialize(mphysicalSystem.Get_bodylist().back(), rotatingBody);
					mphysicalSystem.AddLink(lock2);
				}
			}
		}
	}

	else fprintf(stderr, "La methode rentree est incorrecte\n");
}


void remplir(ChSystemParallelSMC& mphysicalSystem, double r_bead, double r_cyl_int, double r_cyl_ext, double mass, int methode, double height_bead, std::vector< std::shared_ptr< ChBody > >* p_list, std::vector<ChVector<>>* p_list_pos, std::vector<double>* p_radius) {
	for (int i = 0; i < p_list_pos->size(); i++) {
		ChVector <> pos = (*p_list_pos)[i];
		double radius = (*p_radius)[i];
		create_bead(radius, mphysicalSystem, pos, mass, false, false, p_list,i);
	}

}


void set_up(ChSystemParallelSMC& mphysicalSystem, double r_cyl_int, double r_cyl_ext, double height, double r_bead, double mass, double height_bead, std::vector< std::shared_ptr< ChBody > >* p_cylinder_ext_list, std::vector< std::shared_ptr< ChBody > >* p_cylinder_int_list, std::vector< std::shared_ptr< ChBody > >* p_beads_list, double rotation_speed, std::vector<ChVector<>>* p_list_pos, std::vector<double>* p_radius) {

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
	rotatingBody->SetPos(ChVector<>(0, 0, 0));//0,-1,0
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
	auto my_motor = std::make_shared<ChLinkMotorRotationSpeed>();
	my_motor->Initialize(rotatingBody, fixedBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
	auto mfun = std::make_shared<ChFunction_Const>(rotation_speed);  // speed w=90°/s CH_C_PI / 2.0
	//auto mfun = std::make_shared<ChFunction_Const>(0);
	my_motor->SetSpeedFunction(mfun);
	//(*motor)->SetAvoidAngleDrift(0);
	mphysicalSystem.AddLink(my_motor);

	// optional, attach a texture for better visualization
	auto mtexture = std::make_shared<ChTexture>();
	mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
	rotatingBody->AddAsset(mtexture);

	create_cylinder_ext(mphysicalSystem,  r_bead, r_cyl_ext, height, 3, mass, p_cylinder_ext_list);
	create_cylinder_int(mphysicalSystem, r_bead, r_cyl_int, height, 3, rotatingBody, mass, p_cylinder_int_list);
	remplir(mphysicalSystem, r_bead, r_cyl_int, r_cyl_ext, mass, 3, height_bead, p_beads_list, p_list_pos,p_radius);

}


void detect_surface(std::vector< std::shared_ptr< ChBody > >* p_beads_list, ChVectorDynamic<std::shared_ptr<ChBody>>* p_surface, double r_bead, double time) {
	
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
	bool dontcare = false;
	std::string projname = "_tc_run"; 

	const std::string out_dir = SetDataPath(projname, dontcare);

	if (out_dir == "") {
		fprintf(stderr, "Error creating output data directory\n");
		return -1;
	}
	
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	//Déclaration des paramètres
	double gravity = -9.81E2;
	double r_bead =0.2;
	double r_cyl_ext = 10;//5;
	double r_cyl_int = 5;// 2.5;
	double height = 5;// 2.5;
	double height_bead = 4.5;// 2.5;
	double rho = 2.55;
	double mass = rho*(4/3)*CH_C_PI*pow(r_bead,3);
	double rotation_speed =0.096;//0.096
	
	//std::string path = out_dir + "/../TEMP_calmip/test_0/TEMP_tc_set";
	std::string path = out_dir + "/../20190621_113216_tc_set";
	std::ifstream fichier(path + "/settings.dat");
	fichier >> gravity >> r_bead>> r_cyl_ext >> r_cyl_int >> height >> height_bead >> mass;
	
	//Paramètres de simulation
	double time_step = 1e-4;//1e-4
	double out_step = 0.02;

	double time = 0;
	double out_time = 0;
	double time_sim = 2.0;

	// Create a ChronoENGINE physical system
	ChSystemParallelSMC mphysicalSystem;
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
	collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(r_bead / 2);

	// Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc. etc.)
	#ifdef CHRONO_IRRLICHT
		ChIrrApp application(&mphysicalSystem, L"Collisions between objects", core::dimension2d<u32>(800, 600), false, true);

		// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
		ChIrrWizard::add_typical_Logo(application.GetDevice());
		ChIrrWizard::add_typical_Sky(application.GetDevice());
		ChIrrWizard::add_typical_Lights(application.GetDevice());
		ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,20 , 0));
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

	std::vector< ChVector<> > list_position;
	std::vector< ChVector<> >* p_list_position(0);
	p_list_position = &list_position;

	std::vector< double> list_radius;
	std::vector<double>* p_list_radius(0);
	p_list_radius = &list_radius;
	
	read_pos(p_list_position,p_list_radius, path);
	set_up(mphysicalSystem, r_cyl_int, r_cyl_ext, height, r_bead, mass, height_bead, p_cylinder_ext_list, p_cylinder_int_list, p_beads_list, rotation_speed, p_list_position,p_list_radius);
	fprintf(stderr, "Tout a bien ete cree\n");
	
	ChVectorDynamic<std::shared_ptr<ChBody>>* p_surface(0);
	ChVectorDynamic<std::shared_ptr<ChBody>> surface = ChVectorDynamic<std::shared_ptr<ChBody>>(0);
	
	p_surface = &surface;
	detect_surface(p_beads_list,p_surface, r_bead, time);
	
	// create a .dat file with three columns of demo data:
	std::string datafile = out_dir + "/velocity_profile.dat";
	ChStreamOutAsciiFile velocity_profile(datafile.c_str());

	std::string datafile2 = out_dir + "/all_data_surface.dat";
	ChStreamOutAsciiFile all_data_surface(datafile2.c_str());
	
	velocity_profile << "0" << " " << "0" << " " << "id_frame" << " " << "id_part" << " " << "v_r" << " " << "v_t" << " " << "r" << "\n";
	all_data_surface << "time" << " " << "id" << " " << "x" << " " << "y" << " " << "z" << " " << "v_x" << " " << "v_y" << " " << "v_z" << " " << "\n";
	
	// Create an exporter to POVray and set all associated filepaths and settings 
	ChPovRay pov_exporter = ChPovRay(&mphysicalSystem);
	if (SetPovrayPaths(&pov_exporter, out_dir) != 0) {
		fprintf(stderr, "Error creating povray data paths\n");
		return -1;
	}
	SetPovrayParameters(&pov_exporter, 0, 30, 0);

	// Use this function for adding a ChIrrNodeAsset to all items
	#ifdef CHRONO_IRRLICHT
		application.AssetBindAll();
		application.AssetUpdateAll();
	#endif

	// THE SOFT-REAL-TIME CYCLE
	double i = 0.0;
	bool motor_launched = false;
	bool in_mouvement = true;
	int id_frame = 0;

	while (time < time_sim) {
		//printf("nb bille = %i \n", p_beads_list->size());
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

		detect_surface(p_beads_list, p_surface, r_bead, time);
			
		for (int j = 0; j < p_surface->GetLength(); j++) {
			double v_x = p_surface->GetElementN(j)->GetPos_dt().x();
			double v_y = p_surface->GetElementN(j)->GetPos_dt().y();
			double v_z = p_surface->GetElementN(j)->GetPos_dt().z();
			double x = p_surface->GetElementN(j)->GetPos().x();
			double y = p_surface->GetElementN(j)->GetPos().y();
			double z = p_surface->GetElementN(j)->GetPos().z();
			double theta = atan2(z, x);
			double r = sqrt(x*x +z*z);
			double v_r = v_x * cos(theta) + v_z * sin(theta);
			double v_t = v_z * cos(theta) - v_x * sin(theta);
			int id = p_surface->GetElementN(j)->GetIdentifier();
			
			velocity_profile << 0 << " " << 0 << " " << id_frame << " " << id << " " << v_r << " " << v_t << " " <<r << "\n";
			all_data_surface << time << " " << id << " " << x << " " << y << " " << z << " " << v_x << " " << v_y << " " << v_z << " " << "\n";
		}

		pov_exporter.ExportData();
		id_frame = id_frame + 1;
		out_time = time - time_step + out_step;
	}

	return 0;
}