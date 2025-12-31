
// source: https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp

#pragma once

#include "../joints.h"
#include "../trafo.h"
#include <vector>

struct UrDh {
	double d1 = 0;
	double a2 = 0;
	double a3 = 0;
	double d4 = 0;
	double d5 = 0;
	double d6 = 0;

	UrDh(double d1, double a2, double a3, double d4, double d5, double d6)
		: d1(d1), a2(a2), a3(a3), d4(d4), d5(d5), d6(d6) {}
};

enum class elbow {
	any, up, down
};

// https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
inline const UrDh UR20_DH {0.2363, -0.8620, -0.7287, 0.2010, 0.1593, 0.1543};
inline const UrDh UR30_DH {0.2363, -0.6370, -0.5037, 0.2010, 0.1593, 0.1543};
inline const UrDh UR3E_DH {0.15185, -0.24355, -0.2132, 0.13105, 0.08535, 0.0921};
inline const UrDh UR5E_DH {0.1625, -0.425, -0.3922, 0.1333, 0.0997, 0.0996};
inline const UrDh UR10E_DH {0.1807, -0.6127, -0.57155, 0.17415, 0.11985, 0.11655};
inline const UrDh UR16E_DH {0.1807, -0.4784, -0.36, 0.17415, 0.11985, 0.11655};
inline const UrDh UR3_DH {0.1519, -0.24365, -0.21325, 0.11235, 0.08535, 0.0819};
inline const UrDh UR5_DH {0.089159, -0.425, -0.39225, 0.10915, 0.09465, 0.0823};
inline const UrDh UR10_DH {0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922};

#ifdef RAW_KINEMATICS_FUNCTIONS
void fk_raw(
	const double* q,
	double* T,
	const UrDh& dh
);
unsigned int ik_raw(
	const double* T,
	double* q_sols,
	double q6_des,
	const UrDh& dh
);
#endif

Trafo fk(
	const Joints& q,
	const UrDh& dh
);

std::vector<Joints> ik(
	const Trafo& ee,
	const UrDh& dh,
	elbow force_elbow = elbow::any,
	bool full_range = false
);

bool is_elbow_up(
	const Joints& q,
	const UrDh& dh
);

int get_config_id(
	const double* q,
	const UrDh& dh
);
