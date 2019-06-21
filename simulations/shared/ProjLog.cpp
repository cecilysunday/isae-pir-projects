// =============================================================================
// Use these shared functions for the Chrono Validation programs found on
// cecilysunday/chrono-validation. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
// =============================================================================


//#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_parallel/physics/ChSystemParallel.h"

#include "ProjLog.h"

#include <fstream>
#include <iostream> 

using namespace chrono;

// TODO: add checks to make sure everything openes and closes ok


void PrintSimParameters(ChSystemParallelSMC* msystem) {
	ChVector<> gravity = msystem->Get_G_acc();

	GetLog() << "\n" << "SYS, num_sim_threads, " << CHOMPfunctions::GetNumThreads()
		<< "\n" << "SYS, force_model, " << static_cast<std::underlying_type<ChTimestepper::Type>::type>(msystem->GetSettings()->solver.contact_force_model)
		<< "\n" << "SYS, adhesion_model, " << static_cast<std::underlying_type<ChTimestepper::Type>::type>(msystem->GetSettings()->solver.adhesion_force_model)
		<< "\n" << "SYS, tangential_displ_mode, " << static_cast<std::underlying_type<ChTimestepper::Type>::type>(msystem->GetSettings()->solver.tangential_displ_mode)
		<< "\n" << "SYS, timestepper, " << static_cast<std::underlying_type<ChTimestepper::Type>::type>(msystem->GetTimestepperType())
		<< "\n" << "SYS, gravity_x, " << gravity.x()
		<< "\n" << "SYS, gravity_y, " << gravity.y()
		<< "\n" << "SYS, gravity_z, " << gravity.z();
}


void PrintMaterialProperties(std::shared_ptr<ChBody> body) {
	GetLog() << "\n" << body->GetIdentifier() << ", mass, " << body->GetMass()
		<< "\n" << body->GetIdentifier() << ", youngs_modulus, " << body->GetMaterialSurfaceSMC()->GetYoungModulus()
		<< "\n" << body->GetIdentifier() << ", poissons_ratio, " << body->GetMaterialSurfaceSMC()->GetPoissonRatio()
		<< "\n" << body->GetIdentifier() << ", static_friction, " << body->GetMaterialSurfaceSMC()->GetSfriction()
		<< "\n" << body->GetIdentifier() << ", kinetic_friction, " << body->GetMaterialSurfaceSMC()->GetKfriction()
		<< "\n" << body->GetIdentifier() << ", rolling_friction, " << body->GetMaterialSurfaceSMC()->GetRollingFriction()
		<< "\n" << body->GetIdentifier() << ", spinning_friction, " << body->GetMaterialSurfaceSMC()->GetSpinningFriction()
		<< "\n" << body->GetIdentifier() << ", cor, " << body->GetMaterialSurfaceSMC()->GetRestitution()
		<< "\n" << body->GetIdentifier() << ", constant_adhesion, " << body->GetMaterialSurfaceSMC()->GetAdhesion()
		<< "\n" << body->GetIdentifier() << ", DMT_adhesion_multiplier, " << body->GetMaterialSurfaceSMC()->GetAdhesionMultDMT()
		<< "\n" << body->GetIdentifier() << ", perko_adhesion_multiplier, " << body->GetMaterialSurfaceSMC()->GetAdhesionSPerko();
}


void PrintInitialProperties(std::shared_ptr<ChBody> body) {
	GetLog() << "\n" << body->GetIdentifier() << ", inertia_x, " << body->GetInertiaXX().x()
		<< "\n" << body->GetIdentifier() << ", inertia_y, " << body->GetInertiaXX().y()
		<< "\n" << body->GetIdentifier() << ", inertia_z, " << body->GetInertiaXX().z()
		<< "\n" << body->GetIdentifier() << ", init_v_x, " << body->GetPos_dt().x()
		<< "\n" << body->GetIdentifier() << ", init_v_y, " << body->GetPos_dt().y()
		<< "\n" << body->GetIdentifier() << ", init_v_z, " << body->GetPos_dt().z()
		<< "\n" << body->GetIdentifier() << ", init_w_x, " << body->GetWvel_par().x()
		<< "\n" << body->GetIdentifier() << ", init_w_y, " << body->GetWvel_par().y()
		<< "\n" << body->GetIdentifier() << ", init_w_z, " << body->GetWvel_par().z();
}


void PrintSphereProperties(std::shared_ptr<ChBody> body, double radius) {
	GetLog() << "\n" << body->GetIdentifier() << ", shape, " << "sphere"
		<< "\n" << body->GetIdentifier() << ", radius, " << radius;
	PrintInitialProperties(body);
	PrintMaterialProperties(body);
}


void PrintWallProperties(std::shared_ptr<ChBody> body, ChVector<> size) {
	GetLog() << "\n" << body->GetIdentifier() << ", shape, " << "box"
		<< "\n" << body->GetIdentifier() << ", length, " << size.x()
		<< "\n" << body->GetIdentifier() << ", height, " << size.y()
		<< "\n" << body->GetIdentifier() << ", width, " << size.z();
	PrintMaterialProperties(body);
}


void PrintCylinderProperties(std::shared_ptr<ChBody> body, double radius, double height) {
	GetLog() << "\n" << body->GetIdentifier() << ", shape, " << "cylinder"
		<< "\n" << body->GetIdentifier() << ", radius, " << radius
		<< "\n" << body->GetIdentifier() << ", thickness, " << height;
	PrintInitialProperties(body);
	PrintMaterialProperties(body);
}


const std::string SetHeader1(std::ofstream* file, const std::string out_dir, uint prec, uint w) {
	const std::string filename = out_dir + "/projdatsum.txt";

	file->open(filename);

	*file << std::left << std::setw(w) << "time"
		<< "\t" << std::left << std::setw(w) << "v_rel_mag_in"
		<< "\t" << std::left << std::setw(w) << "v_rel_mag_out"
		<< "\t" << std::left << std::setw(w) << "w_rel_mag_in"
		<< "\t" << std::left << std::setw(w) << "w_rel_mag_out"
		<< "\t" << std::left << std::setw(w) << "cor_input"
		<< "\t" << std::left << std::setw(w) << "cor_output"
		<< "\t" << std::left << std::setw(w) << "cor_delta"
		<< "\t" << std::left << std::setw(w) << "collision_steps"
		<< "\t" << std::left << std::setw(w) << "time_bcollision"
		<< "\t" << std::left << std::setw(w) << "time_ncollision"
		<< "\t" << std::left << std::setw(w) << "time_fcalc"
		<< "\t" << std::left << std::setw(w) << "time_sim";

	file->close();
	file->clear();

	return filename;
}


const std::string SetHeader1B(std::ofstream* file, const std::string out_dir, uint prec, uint w) {
	const std::string filename = out_dir + "/projdatsum.txt";

	file->open(filename);

	*file << std::left << std::setw(w) << "time"
		<< "\t" << std::left << std::setw(w) << "time_bcollision"
		<< "\t" << std::left << std::setw(w) << "time_ncollision"
		<< "\t" << std::left << std::setw(w) << "time_fcalc"
		<< "\t" << std::left << std::setw(w) << "time_sim";

	file->close();
	file->clear();

	return filename;
}


const std::string SetHeader1C(std::ofstream* file, const std::string out_dir, uint prec, uint w) {
	const std::string filename = out_dir + "/projdatsum.txt";

	file->open(filename);

	*file << std::left << std::setw(w) << "time"
		<< "\t" << std::left << std::setw(w) << "impact_angle"
		<< "\t" << std::left << std::setw(w) << "collision_steps"
		<< "\t" << std::left << std::setw(w) << "v_rel_in_x"
		<< "\t" << std::left << std::setw(w) << "v_rel_in_y"
		<< "\t" << std::left << std::setw(w) << "v_rel_in_z"
		<< "\t" << std::left << std::setw(w) << "v_rel_in_mag"
		<< "\t" << std::left << std::setw(w) << "v_rel_out_x"
		<< "\t" << std::left << std::setw(w) << "v_rel_out_y"
		<< "\t" << std::left << std::setw(w) << "v_rel_out_z"
		<< "\t" << std::left << std::setw(w) << "v_rel_out_mag"
		<< "\t" << std::left << std::setw(w) << "w_rel_in_x"
		<< "\t" << std::left << std::setw(w) << "w_rel_in_y"
		<< "\t" << std::left << std::setw(w) << "w_rel_in_z"
		<< "\t" << std::left << std::setw(w) << "w_rel_in_mag"
		<< "\t" << std::left << std::setw(w) << "w_rel_out_x"
		<< "\t" << std::left << std::setw(w) << "w_rel_out_y"
		<< "\t" << std::left << std::setw(w) << "w_rel_out_z"
		<< "\t" << std::left << std::setw(w) << "w_rel_out_mag"
		<< "\t" << std::left << std::setw(w) << "cor_n_in"
		<< "\t" << std::left << std::setw(w) << "cor_n_out"
		<< "\t" << std::left << std::setw(w) << "cor_n_diff"
		<< "\t" << std::left << std::setw(w) << "cor_t_in"
		<< "\t" << std::left << std::setw(w) << "cor_t_out"
		<< "\t" << std::left << std::setw(w) << "cor_t_diff"
		<< "\t" << std::left << std::setw(w) << "cor_in"
		<< "\t" << std::left << std::setw(w) << "cor_out"
		<< "\t" << std::left << std::setw(w) << "cor_diff"
		<< "\t" << std::left << std::setw(w) << "w_out_diff"
		<< "\t" << std::left << std::setw(w) << "time_bcollision"
		<< "\t" << std::left << std::setw(w) << "time_ncollision"
		<< "\t" << std::left << std::setw(w) << "time_fcalc"
		<< "\t" << std::left << std::setw(w) << "time_sim";

	file->close();
	file->clear();

	return filename;
}


const std::string SetHeader2(std::ofstream* file, const std::string out_dir, std::string apnd, uint prec, uint w) {
	std::string temp = out_dir + "/projdat.txt";

	if (apnd != "") {
		temp = out_dir + "/projdat_" + apnd + ".txt";
	}

	const std::string filename = temp;

	file->open(filename);
	*file << std::left << std::setw(w) << "time"
		<< "\t" << std::left << std::setw(w) << "collision_steps"
		<< "\t" << std::left << std::setw(w) << "pos1_x"
		<< "\t" << std::left << std::setw(w) << "pos1_y"
		<< "\t" << std::left << std::setw(w) << "pos1_z"
		<< "\t" << std::left << std::setw(w) << "pos1_mag"
		<< "\t" << std::left << std::setw(w) << "pos2_x"
		<< "\t" << std::left << std::setw(w) << "pos2_y"
		<< "\t" << std::left << std::setw(w) << "pos2_z"
		<< "\t" << std::left << std::setw(w) << "pos2_mag"
		<< "\t" << std::left << std::setw(w) << "v1_x"
		<< "\t" << std::left << std::setw(w) << "v1_y"
		<< "\t" << std::left << std::setw(w) << "v1_z"
		<< "\t" << std::left << std::setw(w) << "v1_mag"
		<< "\t" << std::left << std::setw(w) << "v2_x"
		<< "\t" << std::left << std::setw(w) << "v2_y"
		<< "\t" << std::left << std::setw(w) << "v2_z"
		<< "\t" << std::left << std::setw(w) << "v2_mag"
		<< "\t" << std::left << std::setw(w) << "w1_x"
		<< "\t" << std::left << std::setw(w) << "w1_y"
		<< "\t" << std::left << std::setw(w) << "w1_z"
		<< "\t" << std::left << std::setw(w) << "w1_mag"
		<< "\t" << std::left << std::setw(w) << "w2_x"
		<< "\t" << std::left << std::setw(w) << "w2_y"
		<< "\t" << std::left << std::setw(w) << "w2_z"
		<< "\t" << std::left << std::setw(w) << "w2_mag"
		<< "\t" << std::left << std::setw(w) << "f1_mag"
		<< "\t" << std::left << std::setw(w) << "f2_mag"
		<< "\t" << std::left << std::setw(w) << "t1_mag"
		<< "\t" << std::left << std::setw(w) << "t2_mag";

	file->close();
	file->clear();

	return filename;
}


const std::string SetHeader2B(std::ofstream* file, const std::string out_dir, std::string apnd, uint prec, uint w) {
	std::string temp = out_dir + "/projdat.txt";

	if (apnd != "") {
		temp = out_dir + "/projdat_" + apnd + ".txt";
	}

	const std::string filename = temp;

	file->open(filename);
	*file << std::left << std::setw(w) << "time"
		<< "\t" << std::left << std::setw(w) << "collision_steps"
		<< "\t" << std::left << std::setw(w) << "pos1_x"
		<< "\t" << std::left << std::setw(w) << "pos1_y"
		<< "\t" << std::left << std::setw(w) << "pos1_z"
		<< "\t" << std::left << std::setw(w) << "pos1_mag"
		<< "\t" << std::left << std::setw(w) << "v1_x"
		<< "\t" << std::left << std::setw(w) << "v1_y"
		<< "\t" << std::left << std::setw(w) << "v1_z"
		<< "\t" << std::left << std::setw(w) << "v1_mag"
		<< "\t" << std::left << std::setw(w) << "a1_x"
		<< "\t" << std::left << std::setw(w) << "a1_y"
		<< "\t" << std::left << std::setw(w) << "a1_z"
		<< "\t" << std::left << std::setw(w) << "a1_mag"
		<< "\t" << std::left << std::setw(w) << "w1_x"
		<< "\t" << std::left << std::setw(w) << "w1_y"
		<< "\t" << std::left << std::setw(w) << "w1_z"
		<< "\t" << std::left << std::setw(w) << "w1_mag"
		<< "\t" << std::left << std::setw(w) << "rota1_x"
		<< "\t" << std::left << std::setw(w) << "rota1_y"
		<< "\t" << std::left << std::setw(w) << "rota1_z"
		<< "\t" << std::left << std::setw(w) << "rota1_mag"
		<< "\t" << std::left << std::setw(w) << "f1_mag"
		<< "\t" << std::left << std::setw(w) << "f2_mag"
		<< "\t" << std::left << std::setw(w) << "t1_mag"
		<< "\t" << std::left << std::setw(w) << "t2_mag";

	file->close();
	file->clear();

	return filename;
}