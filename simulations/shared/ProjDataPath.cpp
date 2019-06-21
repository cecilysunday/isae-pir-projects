// =============================================================================
// Use these shared functions for the Chrono Validation programs found on
// cecilysunday/chrono-validation. Build the programs with the Project Chrono 
// code version available on cecilysunday/chrono.
//
// Authors: Cecily Sunday
// =============================================================================


#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_postprocess/ChPovRay.h"
#include "chrono_postprocess/ChPovRayAssetCustom.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "ProjDataPath.h"

#include <experimental/filesystem>
#include <fstream>
#include <iostream>     
#include <ctime>        
#include <iomanip>

using namespace chrono;
using namespace chrono::postprocess;
namespace fs = std::experimental::filesystem;


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

	// Create the directory according to the output data path. If the path already exists, delete the contents before continuing.
	auto out_path = filesystem::path(out_dir);
	/*if (out_path.exists()) {
		fs::recursive_directory_iterator iter(out_dir);
		fs::recursive_directory_iterator end;

		while (iter != end) {
			filesystem::path clear = filesystem::path(iter->path().string());
			clear.remove_file();

			std::error_code ec;
			iter.increment(ec);
			if (ec) {
				std::cerr << "Error While Deleting : " << iter->path().string() << " :: " << ec.message() << '\n';
			}
		}
	}
	else*/ filesystem::create_directory(out_path);

	// Redirect the consule printouts to a userlog file
	if (!out_path.exists()) return "";
	else {
		fflush(stdout);
		freopen(out_dir_log.c_str(), "w", stdout);
	}

	// Set the Chrono data and output paths so that this information can be access by other functions
	SetChronoDataPath(CHRONO_DATA_DIR);
	SetChronoOutputPath(out_dir);

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