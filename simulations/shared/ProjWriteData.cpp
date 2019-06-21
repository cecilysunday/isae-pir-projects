// =============================================================================
// Use these shared functions for the Chrono Validation programs found on
// cecilysunday/chrono-validation. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
// =============================================================================


#include "chrono_parallel/physics/ChSystemParallel.h"

#include "ProjWriteData.h"

#include <fstream>
#include <iostream>
#include <experimental/filesystem>
#include <vector>

#include <string.h>
#include <assert.h>
#include <memory.h>


using namespace chrono;
using namespace std::experimental::filesystem;



int StoreData(const ChSystemParallelSMC& msystem, ParticleData** data, size_t num_particles, size_t start_list, int index) {
	size_t id = start_list;

	// Populate the data array at a given timestep (index) with state information for every particle in the system
	for (size_t i = 0; i < num_particles; ++i) {
		const std::shared_ptr<ChBody> body = msystem.Get_bodylist().at(id);
		
		data[i][index].id = body->GetIdentifier();
		data[i][index].time = msystem.GetChTime();
		data[i][index].radius = body->GetCollisionModel()->GetEnvelope();
		data[i][index].collision_state = msystem.data_manager->host_data.ct_body_map[id];
		data[i][index].pos_x = body->GetPos().x();
		data[i][index].pos_y = body->GetPos().y();
		data[i][index].pos_z = body->GetPos().z();
		data[i][index].vel_x = body->GetPos_dt().x();
		data[i][index].vel_y = body->GetPos_dt().y();
		data[i][index].vel_z = body->GetPos_dt().z();
		data[i][index].acc_x = body->GetPos_dtdt().x();
		data[i][index].acc_y = body->GetPos_dtdt().y();
		data[i][index].acc_z = body->GetPos_dtdt().z();
		data[i][index].rot_vel_x = body->GetWvel_loc().x();
		data[i][index].rot_vel_y = body->GetWvel_loc().y();
		data[i][index].rot_vel_z = body->GetWvel_loc().z();
		data[i][index].rot_acc_x = body->GetWacc_loc().x();
		data[i][index].rot_acc_y = body->GetWacc_loc().y();
		data[i][index].rot_acc_z = body->GetWacc_loc().z();
		data[i][index].force_x = msystem.GetBodyContactForce(body).x;
		data[i][index].force_y = msystem.GetBodyContactForce(body).y;
		data[i][index].force_z = msystem.GetBodyContactForce(body).z;
		data[i][index].torque_x = msystem.GetBodyContactTorque(body).x;
		data[i][index].torque_y = msystem.GetBodyContactTorque(body).y;
		data[i][index].torque_z = msystem.GetBodyContactTorque(body).z;

		++id;
	}

	return 0;
}


std::vector<std::string> GetAllBinFiles(const std::string &dirPath) {

	std::vector<std::string> listOfFiles;
	std::string ext = ".bin";

	try {
		// Check if given path exists and points to a directory
		if (exists(dirPath) && is_directory(dirPath)) {

			// Create a Recursive Directory Iterator object and points to the start and end of directory
			recursive_directory_iterator iter(dirPath);
			recursive_directory_iterator end;

			// Iterate though directory and store all .bin filetypes in listOfFiles
			while (iter != end) {
				if (iter->path().extension().string() == ext) {
					listOfFiles.push_back(iter->path().string());
				}
				// Increment the iterator to point to next entry in recursive iteration
				std::error_code ec;
				iter.increment(ec);
				if (ec) {
					std::cerr << "Error While Accessing : " << iter->path().string() << " :: " << ec.message() << '\n';
				}
			}
		}
	}
	catch (std::system_error & e) {
		std::cerr << "Exception :: " << e.what();
	}

	return listOfFiles;
}


int ReadBinary(const std::string &dirPath, ParticleData** data, size_t num_particles, int index) {

	// Get the last bin file in the given directory
	vector<std::string> listOfFiles = GetAllBinFiles(dirPath);
	std::vector<std::string>::iterator lastFile = listOfFiles.end() - 1;
	std::ifstream input_file(*lastFile, std::ios::binary | std::ios::in);
	ParticleData dataIO;

	// Count the total instances of ParticleData types in the file
	int total = 0;
	while (input_file.read((char*)&dataIO, sizeof(ParticleData))) {
		++total;
	}

	// Reset iteration flag to the beginning of the file
	input_file.clear();
	input_file.seekg(0, input_file.beg);

	// Reset iteration flag to the beginning of the file
	int it = 0;
	while (input_file.read((char*)&dataIO, sizeof(ParticleData))) {
		if (it >= total - num_particles) {
			size_t i = it % num_particles;

			data[i][index].id = dataIO.id;
			data[i][index].time = dataIO.time;
			data[i][index].radius = dataIO.radius;
			data[i][index].collision_state = dataIO.collision_state;
			data[i][index].pos_x = dataIO.pos_x;
			data[i][index].pos_y = dataIO.pos_y;
			data[i][index].pos_z = dataIO.pos_z;
			data[i][index].vel_x = dataIO.vel_x;
			data[i][index].vel_y = dataIO.vel_y;
			data[i][index].vel_z = dataIO.vel_z;
			data[i][index].acc_x = dataIO.acc_x;
			data[i][index].acc_y = dataIO.acc_y;
			data[i][index].acc_z = dataIO.acc_z;
			data[i][index].rot_vel_x = dataIO.rot_vel_x;
			data[i][index].rot_vel_y = dataIO.rot_vel_z;
			data[i][index].rot_vel_z = dataIO.rot_vel_x;
			data[i][index].rot_acc_x = dataIO.rot_acc_x;
			data[i][index].rot_acc_y = dataIO.rot_acc_y;
			data[i][index].rot_acc_z = dataIO.rot_acc_z;
			data[i][index].force_x = dataIO.force_x;
			data[i][index].force_y = dataIO.force_y;
			data[i][index].force_z = dataIO.force_z;
			data[i][index].torque_x = dataIO.torque_x;
			data[i][index].torque_y = dataIO.torque_y;
			data[i][index].torque_z = dataIO.torque_z;
		}
		++it;
	}

	return 0;
}


int WriteBinary(std::string binFileName, ParticleData** data, size_t num_particles, int index) {
	// Create a binary file for writing
	std::ofstream bin_out(binFileName, std::ios::binary | std::ios::out );

	// Iterate through the 2D data array and write the ParticleDate at each index to the binary file
	for (int j = 0; j < index; ++j) {
		for (size_t i = 0; i < num_particles; ++i) {
			bin_out.write((char*)&data[i][j], sizeof(ParticleData));
		}
	}

	return 0;
}


int WriteCsv(std::string csvFileName, ParticleData** data, size_t num_particles, int index) {
	// Create a csv file and write the header column to the file
	std::ofstream csv_out(csvFileName);
	
	csv_out << "body_id, time, radius, collision_map," 
			<< "pos_x, pos_y, pos_z," 
			<< "vel_x, vel_y, vel_z," 
			<< "acc_x, acc_y, acc_z,"
			<< "rot_vel_x, rot_vel_y, rot_vel_z," 
			<< "rot_acc_x, rot_acc_y, rot_acc_z,"
			<< "force_x, force_y, force_z,"
			<< "torque_x, torque_y, torque_z\n";

	// Iterate through the 2D data array and write the ParticleDate at each index to the csv file
	for (int j = 0; j < index; ++j) {
		for (size_t i = 0; i < num_particles; ++i) {
			csv_out << data[i][j].id << ","
					<< data[i][j].time << ","
					<< data[i][j].radius << ","
					<< data[i][j].collision_state << ","
					<< data[i][j].pos_x << ","
					<< data[i][j].pos_y << ","
					<< data[i][j].pos_z << ","
					<< data[i][j].vel_x << ","
					<< data[i][j].vel_y << ","
					<< data[i][j].vel_z << ","
					<< data[i][j].acc_x << ","
					<< data[i][j].acc_y << ","
					<< data[i][j].acc_z << ","
					<< data[i][j].rot_vel_x << ","
					<< data[i][j].rot_vel_y << ","
					<< data[i][j].rot_vel_z << ","
					<< data[i][j].rot_acc_x << ","
					<< data[i][j].rot_acc_y << ","
					<< data[i][j].rot_acc_z << ","
					<< data[i][j].force_x << "," 
					<< data[i][j].force_y << ","
					<< data[i][j].force_z << ","
					<< data[i][j].torque_x << ","
					<< data[i][j].torque_y << ","
					<< data[i][j].torque_z << "\n"; 
		}
	}

	return 0;
}


int ConvertBinaryToCsv(const std::string &dirPath) {
	// Get a list of all bin files in the given directory
	vector<std::string> listOfFiles = GetAllBinFiles(dirPath);

	for (std::vector<std::string>::iterator i = listOfFiles.begin(); i != listOfFiles.end(); ++i) {
		// Create a string for the csv file name by copying the name of the bin file and changing the extention type
		std::string binFileName = *i;
		
		std::string ext = ".bin";
		size_t ext_start = binFileName.find(ext);
		std::string temp = binFileName.erase(ext_start, ext.length());
		
		std::string csvFileName = temp + ".csv";

		// Open the csv file for writing and write the header column
		std::ofstream csv_out(csvFileName);
		csv_out << "body_id, time, radius, collision_map,"
				<< "pos_x, pos_y, pos_z,"
				<< "vel_x, vel_y, vel_z,"
				<< "acc_x, acc_y, acc_z,"
				<< "rot_vel_x, rot_vel_y, rot_vel_z,"
				<< "rot_acc_x, rot_acc_y, rot_acc_z,"
				<< "force_x, force_y, force_z,"
				<< "torque_x, torque_y, torque_z\n";

		// Open the binary file for reading
		std::ifstream input_file(*i, std::ios::binary | std::ios::in);
		
		// Read the ParticleDat structures from the bin file and write the structures to the csv file, struct by struct
		ParticleData data;
		while (input_file.read((char*)&data, sizeof(ParticleData))) {
			csv_out << data.id << ","
					<< data.time << ","
					<< data.radius << ","
					<< data.collision_state << ","
					<< data.pos_x << ","
					<< data.pos_y << ","
					<< data.pos_z << ","
					<< data.vel_x << ","
					<< data.vel_y << ","
					<< data.vel_z << ","
					<< data.acc_x << ","
					<< data.acc_y << ","
					<< data.acc_z << ","
					<< data.rot_vel_x << ","
					<< data.rot_vel_y << ","
					<< data.rot_vel_z << ","
					<< data.rot_acc_x << ","
					<< data.rot_acc_y << ","
					<< data.rot_acc_z << ","
					<< data.force_x << ","
					<< data.force_y << ","
					<< data.force_z << ","
					<< data.torque_x << ","
					<< data.torque_y << ","
					<< data.torque_z << "\n";
		}
	}

	return 0;
}


size_t CountObjectsInFile(const std::string &set_path) {
	// if (exists(set_path) && is_directory(set_path)) -->  add in error handeling!
	std::string fs_fname = set_path + "/projdat_final_state.bin";
	std::ifstream input_file(fs_fname, std::ios::binary | std::ios::in);
	ParticleData dataIO;

	// Count the total instances of ParticleData types in the file
	size_t num_objects = 0;
	while (input_file.read((char*)&dataIO, sizeof(ParticleData))) {
		++num_objects;
	}
	return num_objects;
}


int ImportFinalState(ParticleData* data, const std::string &set_path) {
	// if (exists(set_path) && is_directory(set_path)) -->  add in error handeling!
	std::string fs_fname = set_path + "/projdat_final_state.bin";
	std::ifstream input_file(fs_fname, std::ios::binary | std::ios::in);
	ParticleData dataIO;

	// Loop through final_state.bin and the store objects in a data data array
	size_t i = 0;
	while (input_file.read((char*)&dataIO, sizeof(ParticleData))) {
		data[i].id = dataIO.id;
		data[i].time = dataIO.time;
		data[i].radius = dataIO.radius;
		data[i].collision_state = dataIO.collision_state;
		data[i].pos_x = dataIO.pos_x;
		data[i].pos_y = dataIO.pos_y;
		data[i].pos_z = dataIO.pos_z;
		data[i].vel_x = dataIO.vel_x;
		data[i].vel_y = dataIO.vel_y;
		data[i].vel_z = dataIO.vel_z;
		data[i].acc_x = dataIO.acc_x;
		data[i].acc_y = dataIO.acc_y;
		data[i].acc_z = dataIO.acc_z;
		data[i].rot_vel_x = dataIO.rot_vel_x;
		data[i].rot_vel_y = dataIO.rot_vel_z;
		data[i].rot_vel_z = dataIO.rot_vel_x;
		data[i].rot_acc_x = dataIO.rot_acc_x;
		data[i].rot_acc_y = dataIO.rot_acc_y;
		data[i].rot_acc_z = dataIO.rot_acc_z;
		data[i].force_x = dataIO.force_x;
		data[i].force_y = dataIO.force_y;
		data[i].force_z = dataIO.force_z;
		data[i].torque_x = dataIO.torque_x;
		data[i].torque_y = dataIO.torque_y;
		data[i].torque_z = dataIO.torque_z;

		++i;
	}

	return 0;
}