// =============================================================================
// Use these shared functions for the Chrono Validation programs found on
// cecilysunday/chrono-validation. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
// =============================================================================

#ifndef PROJWRITEDATA_H
#define PROJWRITEDATA_H

using namespace chrono;

typedef struct {
	long long int id;
	int collision_state;
	double time;
	double radius;
	double pos_x;
	double pos_y;
	double pos_z;
	double vel_x;
	double vel_y;
	double vel_z;
	double rot_vel_x;
	double rot_vel_y;
	double rot_vel_z;
} ParticleData;



// Populates a 2D data array with state information for every particle in the system for one timestep index
int StoreData(const ChSystemParallelSMC& msystem, ParticleData** data, size_t num_particles, size_t start_list, int index);

// Searches through a given directory and returns a vector containing the names of all files with a .bin extension 
std::vector<std::string> GetAllBinFiles(const std::string &dirPath);

// Read in the last binary file in a given directory and store the file contents in a data structure
int ReadBinary(const std::string &dirPath, ParticleData** data, size_t num_particles, int index);

// Writes all contents of a 2D array of ParticleData structures to a binary file
int WriteBinary(std::string binFileName, ParticleData** data, size_t num_particles, int index);

// Convert all binary files in the given directory to csv files. Assumes that all binary files consist only of ParticleData structures
int ConvertBinaryToCsv(const std::string &dirPath);

// Writes all contents of a 2D array of ParticleData structures to a csv file
int WriteCsv(std::string csvFileName, ParticleData** data, size_t num_particles, int index);

// Read a file containing ParticleData objects and count the number of objects in the file
size_t CountObjectsInFile(const std::string &set_path);

// Import data from final state binary file and save in an array
int ImportFinalState(ParticleData* data, const std::string &set_path);

#endif