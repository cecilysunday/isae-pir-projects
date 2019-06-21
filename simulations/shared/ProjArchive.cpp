#include <fstream>
#include <iostream>
#include <vector>

#include <string.h>
#include <assert.h>
#include <memory.h>

typedef struct {
	int id;
	bool collision_state;
	double pos_x;
	double pos_y;
	double pos_z;
	double vel_x;
	double vel_y;
	double vel_z;
	double acc_x;
	double acc_y;
	double acc_z;
	double rot_vel_x;
	double rot_vel_y;
	double rot_vel_z;
	double rot_acc_x;
	double rot_acc_y;
	double rot_acc_z;
	double force_x;
	double force_y;
	double force_z;
	double torque_x;
	double torque_y;
	double torque_z;
	double sim_total_t;
	double coll_total_t;
	double coll_broad_t;
	double coll_narrow_t;
	double update_total_t;
	double advance_total_t;
	double advance_calc_forces_t;
	double advance_setup_t;
	double advance_solve_t;
	double advance_stabailize_t;
} ParticleData;

int writeBinary(char fileName[]) {
	ParticleData data[200];
	assert(sizeof(data) / sizeof(ParticleData) == 200);
	memset(data, 0, sizeof(data));

	for (int i = 0; i < sizeof(data) / sizeof(ParticleData); ++i) {
		data[i].pos_x = i * i;
		data[i].pos_y = 0;
		data[i].pos_z = i + 2;
		data[i].vel_x = i + i * 3;
		data[i].in_collision = i % 2 == 0;
		data[i].sim_total_t = GetTimerStep();
		data[i].coll_total_t = GetTimerCollision();
		data[i].coll_broad_t = GetTimerCollisionBroad();
		data[i].coll_narrow_t = GetTimerCollisionNarrow();
		data[i].update_total_t = GetTimerUpdate();
		data[i].advance_total_t = GetTimerAdvance();
		data[i].advance_calc_forces_t = GetTimerProcessContact();
		data[i].advance_setup_t = GetTimerSetup();
		data[i].advance_solve_t = GetTimerSolver();
		data[i].advance_stabailize_t = data_manager->system_timer.GetTime("ChIterativeSolverParallel_Stab");
	}

	// Open file to write binary format
	std::ofstream data_file(fileName, std::ios::binary);
	data_file.write((char*)&data, sizeof(data));
	data_file.close();

	return 0;
}

/*
int convertBinaryToCsv(char fileName[], char csvFileName[]) {
	ParticleData data[200];

	// Open file for binary reading and read the data into the statically-allocated struct
	std::ifstream input_file(fileName, std::ios::binary);
	input_file.read((char*)&data, sizeof(data));
	input_file.close();

	// Write to CSV
	std::cout << "pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, in_collision\n" << std::endl;
	for (int i = 0; i < sizeof(data) / sizeof(ParticleData); ++i) {
		std::cout << data[i].pos_x
			<< ", " << data[i].pos_y
			<< ", " << data[i].pos_z
			<< ", " << data[i].vel_x
			<< ", " << data[i].vel_y
			<< ", " << data[i].vel_z
			<< ", " << data[i].in_collision
			<< std::endl;
	}

	return 0;
}
*/

int main(int argc, char *argv[]) {

	char fileName[] = "particle_data.bin";
	char csvFileName[] = "particle_data.csv";

	if (writeBinary(fileName) != 0) {
		fprintf(stderr, "Error writing file.\n");
		return -1;
	}

	if (convertBinaryToCsv(fileName, csvFileName) != 0) {
		fprintf(stderr, "Error converting file.\n");
		return -1;
	}

	return 0;
}