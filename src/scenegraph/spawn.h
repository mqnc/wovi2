
#pragma once

#include "stl.h"
#include "scene.h"

struct Pose {
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double qx = 0.0;
	double qy = 0.0;
	double qz = 0.0;
	double qw = 1.0;
};

void spawn(
    Scene& scene,
	const string& object_id,
	const string& description_file,
	const Pose& pose,
	const valarray<double>& joint_positions,
	const string& parent_object_id,
	const string& parent_link
);