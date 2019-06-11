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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/assets/ChTexture.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include <math.h>
#include <chrono_postprocess/ChGnuPlot.h>






// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace postprocess;
// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
using namespace std;

void create_bead(double r_bead, ChSystemSMC& mphysicalSystem, ChVector<> pos, double mass, bool isFixed, bool isWall, std::vector< std::shared_ptr< ChBody > >* p_list, int i=0) {
	ChQuaternion<> rot(1, 0, 0, 0);
	ChVector<> init_vel(0, 0, 0);
	
	auto material = std::make_shared<ChMaterialSurfaceSMC>();
	material->SetRestitution(0.1f);
	material->SetFriction(0.4f);
	material->SetAdhesion(0);

	auto body = std::make_shared<ChBody>(ChMaterialSurface::SMC);
	
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
	p_list->push_back(body);
	mphysicalSystem.AddBody(body);
}

void create_cylinder_ext(ChSystemSMC& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver, double r_bead, double r_cyl_ext,double height, int methode, double mass, std::vector< std::shared_ptr< ChBody > >* p_list) {
	
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
				
				ChVector<> pos2 = ChVector<>((r_cyl_ext - r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))), sqrt(3)*r_bead * (j + 1) + r_bead, (r_cyl_ext - r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_ext - r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos2, mass,true, true,p_list);
			}
		}
	}

	else {
		printf("La methode rentree est incorrecte\n");
	}
	
}

void create_cylinder_int(ChSystemSMC& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver, double r_bead, double r_cyl_int, double height, int methode, std::shared_ptr<chrono::ChBodyFrame> rotatingBody, double mass, std::vector< std::shared_ptr< ChBody > >* p_list) {
	

	
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

void remplir(ChSystemSMC& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver, double r_bead, double r_cyl_int, double r_cyl_ext, double mass, int methode, double height_bead, std::vector< std::shared_ptr< ChBody > >* p_list) {
	if (methode == 1) {
		for (int k = 0; k < floor(((r_cyl_ext-r_bead) - (r_cyl_int+r_bead)) / (2*r_bead))-1; k++) {
			for (int j = 0; j < floor(height_bead / (2 * r_bead)); j = j + 2) {
				for (int i = 0; i < floor((PI*(r_cyl_int + 3 * r_bead + 2 * k*r_bead)) / r_bead) ; i++) {
					ChVector <> pos = ChVector<>((r_cyl_int+3*r_bead+2*k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), r_bead  * j + r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos, mass, false, false, p_list,i+j+k);
					
				}
			}
		}
	}

	 else if (methode == 2) {
		for (int k = 0; k < floor(((r_cyl_ext - r_bead) - (r_cyl_int + r_bead)) / (2 * r_bead)) - 1; k++) {
			for (int i = 0; i < floor((PI*(r_cyl_int + 3 * r_bead + 2 * k*r_bead)) / r_bead); i++) {
				ChVector <> pos = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), 2*height_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
				create_bead(r_bead, mphysicalSystem, pos, mass, false, false, p_list,i+k+1);
			}
		}
	}

	 else if (methode == 3) { //Remplissage en décalé avec contact vertical
		int id = 0;
		for (int k = 0; k < floor(((r_cyl_ext - r_bead) - (r_cyl_int + r_bead)) / (2 * r_bead)) - 1; k++) {
			for (int j = 0; j < floor(height_bead / (2 * r_bead)); j = j + 2) {
				for (int i = 0; i < floor((PI*(r_cyl_int + 3 * r_bead + 2 * k*r_bead)) / r_bead) + 1; i++) {

					ChVector<> pos = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), sqrt(3)*r_bead * j + r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin(i*(2 * atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
					create_bead(r_bead, mphysicalSystem, pos, mass, false, false, p_list,id);
					id = id + 1;

					if (j + 1 < floor(height_bead / (2 * r_bead))) {
						ChVector<> pos2 = ChVector<>((r_cyl_int + 3 * r_bead + 2 * k*r_bead)*cos((2 * i + 1)*(atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))), sqrt(3)*r_bead * (j + 1) + r_bead, (r_cyl_int + 3 * r_bead + 2 * k*r_bead)*sin((2 * i + 1)*(atan(r_bead / (r_cyl_int + 3 * r_bead + 2 * k*r_bead)))));
						create_bead(r_bead, mphysicalSystem, pos2, mass, false, false, p_list,id);
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

void create_some_falling_items(ChSystemSMC& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver, double r_cyl_int, double r_cyl_ext, double height, double r_bead, double mass, double height_bead, std::vector< std::shared_ptr< ChBody > >* p_cylinder_ext_list, std::vector< std::shared_ptr< ChBody > >* p_cylinder_int_list, std::vector< std::shared_ptr< ChBody > >* p_beads_list, std::shared_ptr<ChLinkMotorRotationSpeed>* motor) {

	//Création du sol
	auto material = std::make_shared<ChMaterialSurfaceSMC>();
	material->SetRestitution(0.1f);
	material->SetFriction(0.4f);
	material->SetAdhesion(0);

	auto fixedBody = std::make_shared<ChBody>(ChMaterialSurface::SMC);

	fixedBody->SetMass(1.0);
	fixedBody->SetBodyFixed(true);
	fixedBody->SetPos(ChVector<>(0, -0.5, 0));
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

	auto rotatingBody = std::make_shared<ChBody>(ChMaterialSurface::SMC);

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
	auto mfun = std::make_shared<ChFunction_Const>(0.0);  // speed w=90°/s CH_C_PI / 2.0
	//auto mfun = std::make_shared<ChFunction_Const>(0);
	(*motor)->SetSpeedFunction(mfun);
	mphysicalSystem.AddLink(*motor);

	// optional, attach a texture for better visualization
	auto mtexture = std::make_shared<ChTexture>();
	mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
	rotatingBody->AddAsset(mtexture);


	create_cylinder_ext(mphysicalSystem, msceneManager, driver, r_bead, r_cyl_ext, height, 3, mass, p_cylinder_ext_list);



	create_cylinder_int(mphysicalSystem, msceneManager, driver, r_bead, r_cyl_int, height_bead, 3, rotatingBody, mass, p_cylinder_int_list);


	remplir(mphysicalSystem, msceneManager, driver, r_bead, r_cyl_int, r_cyl_ext, mass, 3, height_bead, p_beads_list);

	//printf("taille beads_list %i \n", p_beads_list->size());


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
	printf("tab_id : [");
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
		printf("%i,", p_tab_id->GetElementN(i));

	}
	printf("]\n");
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

int main(int argc, char* argv[]) {
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	//Déclaration des paramètres
	double gravity = -9.81;
	double r_bead = 1;
	double r_cyl_ext =10 ;
	double r_cyl_int = 2;
	double height = 10;
	double height_bead = 10;
	double mass = 1;
	double rotation_speed = CH_C_PI / 2.0;
	//Paramètres de simulation
	double time_step = 1e-4;//1e-4
	double out_step = 0.02;
	double time = 0;
	double out_time = 0;

	// Create a ChronoENGINE physical system
	ChSystemSMC mphysicalSystem;
	mphysicalSystem.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
	mphysicalSystem.SetAdhesionForceModel(ChSystemSMC::AdhesionForceModel::Constant);
	mphysicalSystem.Set_G_acc(ChVector<>(0, gravity, 0));

	// Create the Irrlicht visualization (open the Irrlicht device,
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Collisions between objects", core::dimension2d<u32>(800, 600), false, true);
	/*ChIrrApp application(&mphysicalSystem, L"ChLinkLockPlanePlane", irr::core::dimension2d<irr::u32>(800, 600), false, true);*/

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 30, 0));
	/*application.AddTypicalCamera(irr::core::vector3df(300, 0, 300));*/
	
	

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
	
	ChVectorDynamic<double>* p_tab_vel_r(0);
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

	create_array_velocity(p_beads_list, p_tab_vel_r, p_tab_vel_t, p_tab_r,p_tab_theta,p_tab_id, height_bead, r_bead);

	

	std::string filename ="graphes.gpl";
	ChGnuPlot mplot(filename.c_str());
	mplot.SetGrid();
	
	// create a .dat file with three columns of demo data:
	std::string datafile = "test_gnuplot_data.dat";
	ChStreamOutAsciiFile mdatafile(datafile.c_str());

	collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(r_bead);

	// Use this function for adding a ChIrrNodeAsset to all items
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();

	// Use this function for 'converting' assets into Irrlicht meshes
	application.AssetUpdateAll();
	application.SetStepManage(true);
	application.SetTimestep(0.02);

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
		
		std::string time_s = "time : " + to_string(time);
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
	

	return 0;
}