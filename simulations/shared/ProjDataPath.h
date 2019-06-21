// =============================================================================
// Use these shared functions for the Chrono Validation programs found on
// cecilysunday/chrono-validation. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
// =============================================================================


#ifndef PROJDATAPATH_H
#define PROJDATAPATH_H

namespace chrono {
	namespace postprocess {
		class ChPovRay;
	}
}

// Function to create and set the destination directory for saved data files
std::string SetDataPath(std::string projname, bool archive);

// Function to set directories for Povray generated files
int SetPovrayPaths(chrono::postprocess::ChPovRay* pov_exporter, const std::string out_dir);

#endif 
