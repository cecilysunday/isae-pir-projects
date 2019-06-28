// =============================================================================
// Use these shared functions for the Chrono Validation programs found on
// cecilysunday/chrono-validation. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
// =============================================================================


#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono_parallel/ProjWriteData.h"

#include <fstream>
#include <iostream>
#include <experimental/filesystem>
#include <vector>

#include <string.h>
#include <assert.h>
#include <memory.h>



using namespace chrono;
using namespace std::experimental::filesystem;



std::vector<std::string> GetAllPovFiles(const std::string &dirPath) {
	printf("on rentre dans getallpovfiles\n");
	std::vector<std::string> listOfFiles;
	std::string ext = ".pov";

	try {
		// Check if given path exists and points to a directory
		printf("on rentre dans le try\n");
		printf("exists(dirPath) = %i, is_directory(dirPath) = %i\n ", exists(dirPath), is_directory(dirPath));
		if (exists(dirPath) && is_directory(dirPath)) {
			printf("on rentre dans le dossier\n");
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
		printf("on a pas ouvert le fichier\n");
	}

	return listOfFiles;
}

std::string changeFormatPath(std::string pathToCorrect) {
	std::vector<std::string> result;

	std::istringstream iss(pathToCorrect);
	std::string str;

	while (std::getline(iss, str, '\\')){
		result.push_back(str);
	}
	
	std::string toReturn;
	for (int i = 0; i < result.size(); i++) {
		if (i != result.size() - 1) {
			toReturn = toReturn + result[i] + "/";
		}
		else {
			toReturn = toReturn + result[i];
		}
	}
	return toReturn;
}

int changeFirstLine(const std::string &dirPath, std::string toAdd) {
	// Get a list of all bin files in the given directory
	vector<std::string> listOfFiles = GetAllPovFiles(dirPath);
	printf("nb fichiers : %i\n", listOfFiles.size());
	int j = 0;
	for (std::vector<std::string>::iterator i = listOfFiles.begin(); i != listOfFiles.end(); ++i) {
		// Create a string for the csv file name by copying the name of the bin file and changing the extention type
		std::string toAdd_loop = toAdd;
		std::string povFileName = *i;
		std::string ext = ".pov";
		size_t ext_start = povFileName.find(ext);
		std::string temp = povFileName.erase(ext_start, ext.length());
		//std::cout << "toAdd = " << toAdd << "temp" << temp <<"\n";
		/*std::string ext = "my_state_";
		
		size_t ext_start = povFileName.find(ext);

		std::string temp = povFileName.erase(0, ext_start+1);
		temp = temp + toAdd;*/

		//std::string csvFileName = temp + ".csv";

		// Open the csv file for writing and write the header column
		/*std::ofstream csv_out(csvFileName);
		csv_out << "body_id, time, radius, collision_map,"
			<< "pos_x, pos_y, pos_z,"
			<< "vel_x, vel_y, vel_z,"
			<< "acc_x, acc_y, acc_z,"
			<< "rot_vel_x, rot_vel_y, rot_vel_z,"
			<< "rot_acc_x, rot_acc_y, rot_acc_z,"
			<< "force_x, force_y, force_z,"
			<< "torque_x, torque_y, torque_z\n";
		*/
		// Open the binary file for reading
		
		//toAdd = toAdd + temp + ".dat\"";
		//toAdd_loop = toAdd_loop + ".dat\"";
		printf("on arrive avant d'écrire, %i\n",j);
		j = j + 1;
		toAdd_loop = toAdd_loop + changeFormatPath(temp) + ".dat\"";
		std::ofstream output_file(*i, std::ios::out | std::ios::in);
		output_file << toAdd_loop << "\n";
		output_file << "//";
		output_file.close();
		
	}

	return 0;
}

int main(int argc, char* argv[]) {
	std::string path = "#declare dat_file = \"";
	//ChangeFirstline("C:/Users/jules/Desktop/Test_output",path);
	changeFirstLine("D:/PIR/Simulations/data/20190622_091842_tc_run", path);
	return 0;
}
