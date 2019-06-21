// =============================================================================
// Use these shared functions for the Chrono Validation programs found on
// cecilysunday/chrono-validation. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
// =============================================================================


#ifndef PROJLOG_H
#define PROJLOG_H

#include "chrono_parallel/physics/ChSystemParallel.h"

#include <fstream>

using namespace chrono;


// Function to print simulation parameters to log
void PrintSimParameters(ChSystemParallelSMC* msystem);

// Function to print shared material properties to log
void PrintMaterialProperties(std::shared_ptr<ChBody> body);

// Function to print shared material properties to log
void PrintInitialProperties(std::shared_ptr<ChBody> body);

// Function to print all material, geometric, and simulation properties to log for sphere-shaped objects
void PrintSphereProperties(std::shared_ptr<ChBody> body, double radius);

// Function to print all material, geometric, and simulation properties to log for box-shaped objects
void PrintWallProperties(std::shared_ptr<ChBody> body, ChVector<> size);

// Function to print all material, geometric, and simulation properties to log for box-shaped objects
void PrintCylinderProperties(std::shared_ptr<ChBody> body, double radius, double height);

// Function to set the header row for the SUMMARY files that are generated from inside the project
const std::string SetHeader1(std::ofstream* file, const std::string out_dir, uint prec, uint w);

// Function to set the header row for the SUMMARY files that are generated from inside the project
const std::string SetHeader1B(std::ofstream* file, const std::string out_dir, uint prec, uint w);

// Function to set the header row for the SUMMARY files that are generated from inside the project
const std::string SetHeader1C(std::ofstream* file, const std::string out_dir, uint prec, uint w);

// Function to set the header row for the HIGH FREQUENCY DATA files that are generated from inside the project
const std::string SetHeader2(std::ofstream* file, const std::string out_dir, std::string apnd, uint prec, uint w);

// Function to set the header row for the HIGH FREQUENCY DATA files that are generated from inside the project
const std::string SetHeader2B(std::ofstream* file, const std::string out_dir, std::string apnd, uint prec, uint w);

#endif 