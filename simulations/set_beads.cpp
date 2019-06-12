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

//#include <cstdio>
//#include <vector>
//#include <cmath>

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::postprocess;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
//using namespace std;



void create_bead(double r_bead, ChSystemParallelSMC& mphysicalSystem, ChVector<> pos, double mass, bool isFixed, bool isWall, std::vector< std::shared_ptr< ChBody > >* p_list, int i=0, bool has_velocity = false) {
	ChQuaternion<> rot(1, 0, 0, 0);
	ChVector<> init_vel(0, 0, 0);
	
	auto material = std::make_shared<ChMaterialSurfaceSMC>();
	material->SetRestitution(0.1f);
	material->SetFriction(0.4f);
	material->SetAdhesion(0);

	auto body = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(),ChMaterialSurface::SMC);
	
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
	if (isWall == false || isFixed == false) {
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
	}
	
	if (has_velocity == true) {
		double vx = ChRandom()*2;
		double vy = ChRandom() * 2;
		double vz = ChRandom() * 2;
		body->SetPos_dt(ChVector<>(vx, vy, vz));
		//printf("v dans create bead : %f\n", sqrt(vx*vx + vy * vy + vz * vz));
		double vx_get = body->GetPos_dt().x();
		printf("vx = vx_get : %i\n", vx==vx_get);
	}
	p_list->push_back(body);
	mphysicalSystem.AddBody(body);
}

void create_cylinder_ext(ChSystemParallelSMC& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver, double r_bead, double r_cyl_ext,double height, int methode, double mass, std::vector< std::shared_ptr< ChBody > >* p_list) {
	
	if (methode == 1) { //Remplissage en colonne
		
		for (int i = 0; i < floor(PI*(r_cyl_ext-r_bead) / r_bead) ; i++) {
			for (int j = 0; j < floor(height / (2 * r_bead)); j = j++) {
				ChVector <> pos = ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,true,true, p_list);
			}
		}
	}

	else if (methode == 2) { //Remplissage en décalé sans contact vertical
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((PI*(r_cyl_ext-r_bead)) / r_bead)+1; i++) {
				ChVector<> pos = ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass,true,true, p_list);

				ChVector<> pos2= ChVector<>((r_cyl_ext - r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), r_bead * 2 * (j + 1) + r_bead, (r_cyl_ext - r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos2, mass,true,true,p_list);
			}
		}
	}

	else if (methode == 3) { //Remplissage en décalé avec contact vertical
		for (int j = 0; j < floor(height / (2 * r_bead)); j = j + 2) {
			for (int i = 0; i < floor((PI*(r_cyl_ext-r_bead)) / r_bead)+1  ; i++) {
				ChVector<> pos= ChVector<>((r_cyl_ext - r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3)*r_bead * j + r_bead, (r_cyl_ext - r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_ext - r_bead)))));
				
				create_bead(r_bead, mphysicalSystem, pos, mass,true,true,p_list);
				if (j + 1 < floor(height / (2 * r_bead))) {
					ChVector<> pos2 = ChVector<>((r_cyl_ext - r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3)*r_bead * (j + 1) + r_bead, (r_cyl_ext - r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos2, mass, true, true, p_list);
				}
			}
		}
	}

	else {
		printf("La methode rentree est incorrecte\n");
	}
	
}

void create_cylinder_int(ChSystemParallelSMC& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver, double r_bead, double r_cyl_int, double height, int methode, std::shared_ptr<chrono::ChBodyFrame> rotatingBody, double mass, std::vector< std::shared_ptr< ChBody > >* p_list) {
	

	
	if (methode == 1) { //Remplissage en colonne
		for (int i = 0; i < floor((PI*(r_cyl_int+r_bead)) / r_bead) ; i++) {
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
			for (int i = 0; i < floor((PI*(r_cyl_int+r_bead)) / r_bead) +1; i++) {
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
			for (int i = 0; i < floor( PI*(r_cyl_int+r_bead) / r_bead) +1; i++) {

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

void remplir(ChSystemParallelSMC& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver, double r_bead, double r_cyl_int, double r_cyl_ext, double mass, int methode, double height_bead, std::vector< std::shared_ptr< ChBody > >* p_list, bool has_velocity) {
	if (methode == 1) {
		for (int k = 0; k < floor(((r_cyl_ext-r_bead) - (r_cyl_int+r_bead)) / (2*r_bead))-1; k++) {
			for (int j = 0; j < floor(height_bead / (2 * r_bead)); j = j + 2) {
				for (int i = 0; i < floor((PI*(r_cyl_int + 3 * r_bead + 2 * k*r_bead)) / r_bead) ; i++) {
					ChVector <> pos = ChVector<>((r_cyl_int+3*r_bead+2*k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), r_bead  * j + r_bead+2*height_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos, mass, false, false, p_list,i+j+k, has_velocity);
					
				}
			}
		}
	}

	 else if (methode == 2) {
		for (int k = 0; k < floor(((r_cyl_ext - r_bead) - (r_cyl_int + r_bead)) / (2 * r_bead)) - 1; k++) {
			for (int i = 0; i < floor((PI*(r_cyl_int + 3 * r_bead + 2 * k*r_bead)) / r_bead); i++) {
				ChVector <> pos = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), 2*height_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass, false, false, p_list,i+k+1, has_velocity);
			}
		}
	}

	 else if (methode == 3) { //Remplissage en décalé avec contact vertical
		int id = 0;
		for (int k = 0; k < floor(((r_cyl_ext - r_bead) - (r_cyl_int + r_bead)) / (2 * r_bead)) - 1; k++) {
			for (int j = 0; j < floor(height_bead / (2 * r_bead)); j = j + 2) {
				for (int i = 0; i < floor((PI*(r_cyl_int + 3 * r_bead + 2 * k*r_bead)) / r_bead) + 1; i++) {

					ChVector<> pos = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), sqrt(3)*r_bead * j + r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos, mass, false, false, p_list,id, has_velocity);
					id = id + 1;

					if (j + 1 < floor(height_bead / (2 * r_bead))) {
						ChVector<> pos2 = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), sqrt(3)*r_bead * (j + 1) + r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
						create_bead(r_bead, mphysicalSystem, pos2, mass, false, false, p_list,id,has_velocity);
						id = id + 1;
					}
				}
			}
		}
	}

	 else {
	 printf("La methode demandee est incorrecte");
	}

}

void create_some_falling_items(ChSystemParallelSMC& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver, double r_cyl_int, double r_cyl_ext, double height, double r_bead, double mass, double height_bead, std::vector< std::shared_ptr< ChBody > >* p_cylinder_ext_list, std::vector< std::shared_ptr< ChBody > >* p_cylinder_int_list, std::vector< std::shared_ptr< ChBody > >* p_beads_list, std::shared_ptr<ChLinkMotorRotationSpeed>* motor) {

	//Création du sol
	auto material = std::make_shared<ChMaterialSurfaceSMC>();
	material->SetRestitution(0.1f);
	material->SetFriction(0.4f);
	material->SetAdhesion(0);

	auto fixedBody = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);

	fixedBody->SetMass(1.0);
	fixedBody->SetBodyFixed(true);
	fixedBody->SetPos(ChVector<>());
	fixedBody->SetCollide(true);

	fixedBody->GetCollisionModel()->ClearModel();
	ChVector<> hsize = 0.5 * ChVector<>(2*r_cyl_ext,1,2*r_cyl_ext);
	
	auto box1 = std::make_shared<ChBoxShape>();
	box1->GetBoxGeometry().Pos = ChVector<>(0,-0.5,0);
	box1->GetBoxGeometry().Size = hsize;
	box1->SetColor(ChColor(1, 0, 0));
	box1->SetFading(0.6f);
	fixedBody->AddAsset(box1);

	fixedBody->GetCollisionModel()->AddBox(hsize.x(), hsize.y(), hsize.z(), ChVector<>(0,-0.5,0));

	auto box2 = std::make_shared<ChBoxShape>();
	box2->GetBoxGeometry().Pos = ChVector<>(0, height+0.5, 0);
	box2->GetBoxGeometry().Size = hsize;
	box2->SetColor(ChColor(1, 0, 0));
	box2->SetFading(0.6f);
	fixedBody->AddAsset(box2);

	fixedBody->GetCollisionModel()->AddBox(hsize.x(), hsize.y(), hsize.z(), ChVector<>(0, height+0.5, 0));
	fixedBody->GetCollisionModel()->BuildModel();
	/*fixedBody->SetMass(1.0);
	fixedBody->SetBodyFixed(true);
	fixedBody->SetPos(ChVector<>(0, -0.5, 0));
	fixedBody->SetRot(Q_from_AngY(0.0));
	fixedBody->SetCollide(true);
	fixedBody->SetMaterialSurface(material);
	fixedBody->GetCollisionModel()->ClearModel();
	fixedBody->GetCollisionModel()->AddBox(r_cyl_ext, 1, r_cyl_ext, fixedBody->GetPos(), fixedBody->GetRot());
	fixedBody->GetCollisionModel()->AddBox(r_cyl_ext, 1, r_cyl_ext, fixedBody->GetPos()+ChVector<>(0,height,0), fixedBody->GetRot());
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
	printf("Test1\n");
	//mphysicalSystem.AddBody(fixedBody);
	printf("Test2\n");

	//Create seiling
	/*auto seiling = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(), ChMaterialSurface::SMC);

	seiling->SetMass(1.0);
	seiling->SetBodyFixed(true);
	seiling->SetPos(ChVector<>(0, height+r_bead+0.5-1, 0));
	seiling->SetCollide(true);
	seiling->SetMaterialSurface(material);
	seiling->GetCollisionModel()->ClearModel();
	seiling->GetCollisionModel()->AddBox(r_cyl_ext, 1, r_cyl_ext, seiling->GetPos(), seiling->GetRot());
	seiling->GetCollisionModel()->BuildModel();

	auto box_seiling = std::make_shared<ChBoxShape>();
	box_seiling->GetBoxGeometry().Size = ChVector<>(r_cyl_ext, 0.5, r_cyl_ext);
	box_seiling->GetBoxGeometry().Pos = ChVector<>(0, -0.5, 0);
	box_seiling->SetColor(ChColor(1, 0, 1));
	box_seiling->SetFading(0.6f);
	fixedBody->AddAsset(box_seiling);*/

	
	//seiling->AddAsset(textcyl);
	printf("Test1\n");
	mphysicalSystem.AddBody(fixedBody);
	printf("Test2\n");
	// Add the rotating mixer

	auto rotatingBody = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>(),ChMaterialSurface::SMC); 

	rotatingBody->SetMass(10.0);
	rotatingBody->SetInertiaXX(ChVector<>(50, 50, 50));
	rotatingBody->SetPos(ChVector<>(0, 0, 0));
	rotatingBody->SetCollide(true);
	rotatingBody->SetMaterialSurface(material);

	rotatingBody->GetCollisionModel()->ClearModel();
	rotatingBody->GetCollisionModel()->AddCylinder(r_cyl_int, r_cyl_int, height, rotatingBody->GetPos(), rotatingBody->GetRot());
	rotatingBody->GetCollisionModel()->BuildModel();

	auto box = std::make_shared<ChCylinderShape>();
	box->GetCylinderGeometry().rad = r_cyl_int;
	box->GetCylinderGeometry().p1 = ChVector<>(0, height, 0);
	box->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
	box->SetColor(ChColor(0, 0, 1));
	box->SetFading(0.6f);
	rotatingBody->AddAsset(box);

	mphysicalSystem.AddBody(rotatingBody);
	
	/*
	// .. a motor between mixer and truss

	//auto my_motor = std::make_shared<ChLinkMotorRotationSpeed>();
	(*motor)->Initialize(rotatingBody, fixedBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
	auto mfun = std::make_shared<ChFunction_Const>(0.0);  // speed w=90°/s CH_C_PI / 2.0
	//auto mfun = std::make_shared<ChFunction_Const>(0);
	(*motor)->SetSpeedFunction(mfun);
	//(*motor)->SetAvoidAngleDrift(0);
	mphysicalSystem.AddLink(*motor);

	// optional, attach a texture for better visualization
	auto mtexture = std::make_shared<ChTexture>();
	mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
	rotatingBody->AddAsset(mtexture);
	*/

	create_cylinder_ext(mphysicalSystem, msceneManager, driver, r_bead, r_cyl_ext, height, 3, mass, p_cylinder_ext_list);



	create_cylinder_int(mphysicalSystem, msceneManager, driver, r_bead, r_cyl_int, height_bead, 3, rotatingBody, mass, p_cylinder_int_list);


	remplir(mphysicalSystem, msceneManager, driver, r_bead, r_cyl_int, r_cyl_ext, mass, 3, height_bead, p_beads_list, true);

	//printf("taille beads_list %i \n", p_beads_list->size());


}

void create_array_velocity(std::vector< std::shared_ptr< ChBody > >* p_beads_list, ChVectorDynamic<double>* p_tab_v) {
	
	p_tab_v->Reset(p_beads_list->size());

	for (int i = 0; i < p_beads_list->size(); ++i) {
		double v_x= (*p_beads_list)[i]->GetPos_dt().x();
		double v_z = (*p_beads_list)[i]->GetPos_dt().z();
		double v_y = (*p_beads_list)[i]->GetPos().y();
		
		//printf("v : %f\n", sqrt(v_x*v_x+v_z*v_z+v_y*v_y));
		p_tab_v->SetElementN(i, sqrt(v_x*v_x+v_y*v_y+v_z*v_z));
		
		

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


int main(int argc, char* argv[]) {
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	//Déclaration des paramètres
	double gravity = -9.81;
	double r_bead = 1;
	double r_cyl_ext =10 ;
	double r_cyl_int = 2;
	double height = 10;
	double height_bead = 8;
	double mass = 1;
	double rotation_speed = CH_C_PI / 2.0;
	//Paramètres de simulation
	double time_step = 1e-4;//1e-4
	double out_step = 0.02;
	double time = 0;
	double out_time = 0;

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

	// Create the Irrlicht visualization (open the Irrlicht device,
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Collisions between objects", core::dimension2d<u32>(800, 600), false, true);
	//ChIrrApp application(&mphysicalSystem, L"ChLinkLockPlanePlane", irr::core::dimension2d<irr::u32>(800, 600), false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(30, 0, 0));
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
	
	create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver(), r_cyl_int, r_cyl_ext, height, r_bead, mass, height_bead, p_cylinder_ext_list, p_cylinder_int_list,p_beads_list, motor);
	
	/*ChVectorDynamic<double>* p_tab_vel_r(0);
	ChVectorDynamic<double> tab_vel_r=ChVectorDynamic<double>(0);
	p_tab_vel_r = &tab_vel_r;

	ChVectorDynamic<double>* p_tab_vel_t(0);
	ChVectorDynamic<double> tab_vel_t=ChVectorDynamic<double>(0);
	p_tab_vel_t = &tab_vel_t;

	ChVectorDynamic<double>* p_tab_r(0);
	ChVectorDynamic<double> tab_r=ChVectorDynamic<double>(0);
	p_tab_r = &tab_r;

	ChVectorDynamic<double>* p_tab_theta(0);
	ChVectorDynamic<double> tab_theta = ChVectorDynamic<double>(0);
	p_tab_theta = &tab_theta;

	ChVectorDynamic<int>* p_tab_id(0);
	ChVectorDynamic<int> tab_id = ChVectorDynamic<int>(0);
	p_tab_id = &tab_id;

	ChVectorDynamic<double>* p_tab_v_y(0);
	ChVectorDynamic<double> tab_v_y = ChVectorDynamic<double>(0);
	p_tab_v_y = &tab_v_y;*/

	ChVectorDynamic<double>* p_tab_v(0);
	ChVectorDynamic<double> tab_v = ChVectorDynamic<double>(0);
	p_tab_v = &tab_v;
	create_array_velocity(p_beads_list, p_tab_v);



	/*std::string filename ="graphes.gpl";
	ChGnuPlot mplot(filename.c_str());
	mplot.SetGrid();*/
	
	// create a .dat file with three columns of demo data:
	std::string datafile = "test_gnuplot_data.dat";
	ChStreamOutAsciiFile mdatafile(datafile.c_str());
	mdatafile << 0 << ", " << "mean_v" << "\n";
	collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(r_bead/2);

	// Use this function for adding a ChIrrNodeAsset to all items
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();

	// Use this function for 'converting' assets into Irrlicht meshes
	application.AssetUpdateAll();
	//application.SetStepManage(true);
	//application.SetTimestep(0.02);

	//
	// THE SOFT-REAL-TIME CYCLE
	double i = 0.0;
	bool motor_launched = false;
	bool in_mouvement = true;
	int id_frame = 0;
	while (application.GetDevice()->run()) {
		application.BeginScene();
		ChIrrTools::drawSegment(application.GetVideoDriver(), ChVector<>(0, -100, 0), ChVector<>(0, 100, 0), irr::video::SColor(255, 0, 0, 0), true);
		ChIrrTools::drawGrid(application.GetVideoDriver(), 0.2, 0.2, 20, 20,
			ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
			video::SColor(255, 80, 100, 100), true);
		application.DrawAll();
		
		printf("time :%f\n", time);
		

		
		

	
		printf("on cree les graphes\n");
		create_array_velocity(p_beads_list, p_tab_v);
		printf("moyenne v : %f\n", mean_vector(p_tab_v));
		//mplot.Plot(*p_tab_r, *p_tab_vel_t, cstr, "with points");
			
		
		mdatafile << time << ", " << mean_vector(p_tab_v) << "\n";
		
		id_frame = id_frame + 1;
		

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
	
	
	return 0;
}
