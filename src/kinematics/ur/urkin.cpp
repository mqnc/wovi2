
// source: https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp

#include "urkin.h"
#include <math.h>
#include <stdexcept>

const double ZERO_THRESH = 0.00000001;
int SIGN(double x) {
	return (x > 0) - (x < 0);
}
const double PI = M_PI;


void fk_raw(
	const double* q,
	double* T,
	const UrDh& dh
) {
	const auto& [d1, a2, a3, d4, d5, d6] = dh;
	double s1 = sin(*q), c1 = cos(*q); q++;
	double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
	/*double s3 = sin(*q), c3 = cos(*q);*/ q23 += *q; q234 += *q; q++;
	double s4 = sin(*q), c4 = cos(*q); q234 += *q; q++;
	double s5 = sin(*q), c5 = cos(*q); q++;
	double s6 = sin(*q), c6 = cos(*q);
	double s23 = sin(q23), c23 = cos(q23);
	double s234 = sin(q234), c234 = cos(q234);

	*T = (c6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * s6); T++;
	*T = (-s6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * c6); T++;
	*T = -(c234 * c1 * s5 - c5 * s1); T++;
	*T = -(d6 * c234 * c1 * s5 - a3 * c23 * c1 - a2 * c1 * c2 - d6 * c5 * s1 - d5 * s234 * c1 - d4 * s1); T++;

	*T = (-c6 * (c1 * s5 - c234 * c5 * s1) - s234 * s1 * s6); T++;
	*T = (s6 * (c1 * s5 - c234 * c5 * s1) - s234 * c6 * s1); T++;
	*T = -(c1 * c5 + c234 * s1 * s5); T++;
	*T = -(d6 * (c1 * c5 + c234 * s1 * s5) + d4 * c1 - a3 * c23 * s1 - a2 * c2 * s1 - d5 * s234 * s1); T++;

	*T = -(-c234 * s6 - s234 * c5 * c6); T++;
	*T = -(s234 * c5 * s6 - c234 * c6); T++;
	*T = -s234 * s5; T++;
	*T = d1 + a3 * s23 + a2 * s2 - d5 * (c23 * c4 - s23 * s4) - d6 * s5 * (c23 * s4 + s23 * c4); T++;
}


unsigned int ik_raw(
	const double* T,
	double* q_sols,
	double q6_des,
	const UrDh& dh
) {
	const auto& [d1, a2, a3, d4, d5, d6] = dh;

	unsigned int num_sols = 0;

	double T00, T01, T02, T03;
	double T10, T11, T12, T13;
	double T20, T21, T22, T23;

	T00 = *T; T++; T01 = *T; T++; T02 = *T; T++; T03 = *T; T++;
	T10 = *T; T++; T11 = *T; T++; T12 = *T; T++; T13 = *T; T++;
	T20 = *T; T++; T21 = *T; T++; T22 = *T; T++; T23 = *T;

	////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
	double q1[2];
	{
		double A = d6 * T12 - T13;
		double B = d6 * T02 - T03;
		double R = A * A + B * B;
		if (fabs(A) < ZERO_THRESH) {
			double div;
			if (fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
				div = -SIGN(d4) * SIGN(B);
			else
				div = -d4 / B;
			double arcsin = asin(div);
			if (fabs(arcsin) < ZERO_THRESH)
				arcsin = 0.0;
			if (arcsin < 0.0)
				q1[0] = arcsin + 2.0 * PI;
			else
				q1[0] = arcsin;
			q1[1] = PI - arcsin;
		}
		else if (fabs(B) < ZERO_THRESH) {
			double div;
			if (fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
				div = SIGN(d4) * SIGN(A);
			else
				div = d4 / A;
			double arccos = acos(div);
			q1[0] = arccos;
			q1[1] = 2.0 * PI - arccos;
		}
		else if (d4 * d4 > R) {
			return num_sols;
		}
		else {
			double arccos = acos(d4 / sqrt(R));
			double arctan = atan2(-B, A);
			double pos = arccos + arctan;
			double neg = -arccos + arctan;
			if (fabs(pos) < ZERO_THRESH)
				pos = 0.0;
			if (fabs(neg) < ZERO_THRESH)
				neg = 0.0;
			if (pos >= 0.0)
				q1[0] = pos;
			else
				q1[0] = 2.0 * PI + pos;
			if (neg >= 0.0)
				q1[1] = neg;
			else
				q1[1] = 2.0 * PI + neg;
		}
	}
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////////// wrist 2 joint (q5) //////////////////////////////
	double q5[2][2];
	{
		for (int i = 0; i < 2; i++) {
			double numer = (T03 * sin(q1[i]) - T13 * cos(q1[i]) - d4);
			double div;
			if (fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
				div = SIGN(numer) * SIGN(d6);
			else
				div = numer / d6;
			double arccos = acos(div);
			q5[i][0] = arccos;
			q5[i][1] = 2.0 * PI - arccos;
		}
	}
	////////////////////////////////////////////////////////////////////////////////

	{
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 2; j++) {
				double c1 = cos(q1[i]), s1 = sin(q1[i]);
				double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
				double q6;
				////////////////////////////// wrist 3 joint (q6) //////////////////////////////
				if (fabs(s5) < ZERO_THRESH)
					q6 = q6_des;
				else {
					q6 = atan2(SIGN(s5) * -(T01 * s1 - T11 * c1),
						SIGN(s5) * (T00 * s1 - T10 * c1));
					if (fabs(q6) < ZERO_THRESH)
						q6 = 0.0;
					if (q6 < 0.0)
						q6 += 2.0 * PI;
				}
				////////////////////////////////////////////////////////////////////////////////

				double q2[2], q3[2], q4[2];
				///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
				double c6 = cos(q6), s6 = sin(q6);
				double x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
				double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
				double p13x = d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - d6 * (T02 * c1 + T12 * s1) +
					T03 * c1 + T13 * s1;
				double p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6);

				double c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
				if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
					c3 = SIGN(c3);
				else if (fabs(c3) > 1.0) {
					// TODO NO SOLUTION
					continue;
				}
				double arccos = acos(c3);
				q3[0] = arccos;
				q3[1] = 2.0 * PI - arccos;
				double denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3;
				double s3 = sin(arccos);
				double A = (a2 + a3 * c3), B = a3 * s3;
				q2[0] = atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
				q2[1] = atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);
				double c23_0 = cos(q2[0] + q3[0]);
				double s23_0 = sin(q2[0] + q3[0]);
				double c23_1 = cos(q2[1] + q3[1]);
				double s23_1 = sin(q2[1] + q3[1]);
				q4[0] = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
				q4[1] = atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1);
				////////////////////////////////////////////////////////////////////////////////
				for (int k = 0; k < 2; k++) {
					if (fabs(q2[k]) < ZERO_THRESH)
						q2[k] = 0.0;
					else if (q2[k] < 0.0) q2[k] += 2.0 * PI;
					if (fabs(q4[k]) < ZERO_THRESH)
						q4[k] = 0.0;
					else if (q4[k] < 0.0) q4[k] += 2.0 * PI;
					q_sols[num_sols * 6 + 0] = q1[i]; q_sols[num_sols * 6 + 1] = q2[k];
					q_sols[num_sols * 6 + 2] = q3[k]; q_sols[num_sols * 6 + 3] = q4[k];
					q_sols[num_sols * 6 + 4] = q5[i][j]; q_sols[num_sols * 6 + 5] = q6;
					num_sols++;
				}

			}
		}
	}
	return num_sols;
}

Trafo fk(
	const Joints& q,
	const UrDh& dh
) {

	double ee_raw[3 * 4] = {};

	fk_raw(q.data(), ee_raw, dh);

	Trafo ee;

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 4; j++) {
			ee[i][j] = ee_raw[i * 4 + j];
			if (isnan(ee[i][j])) {
				return {};
			}
		}
	}

	return ee;
}

std::vector<Joints> ik(
	const Trafo& ee,
	const UrDh& dh,
	elbow force_elbow,
	bool full_range
) {

	double ee_raw[3 * 4] = {};

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 4; j++) {
			if (isnan(ee[i][j])) {
				throw std::runtime_error("NaN in input");
			}
			ee_raw[i * 4 + j] = ee[i][j];
		}
	}

	double solutions_raw[8 * 6] = {};
	size_t n = ik_raw(ee_raw, solutions_raw, 0, dh);

	std::vector<Joints> solutions;

	for (size_t s = 0; s < n; s++) {

		Joints solution {0, 0, 0, 0, 0, 0};
		bool valid = true;
		for (size_t q = 0; q < 6; q++) {
			double val = solutions_raw[s * 6 + q];
			if (isnan(val)) {
				valid = false;
				break;
			}
			while (val > M_PI) { val -= 2.0 * M_PI; }
			while (val < -M_PI) { val += 2.0 * M_PI; }
			solution[q] = val;
		}
		if (!valid) { continue; }

		if (
			(force_elbow == elbow::up && !is_elbow_up(solution, dh))
			|| (force_elbow == elbow::down && is_elbow_up(solution, dh))
		) {
			continue;
		}

		solutions.push_back(solution);

		if (full_range) {
			// generate all possible permutations of full joint rotations
			auto tmpl = solutions.back();
			auto flipped = solutions.back();
			for (unsigned char j = 0; j < 6; j++) {
				flipped[j] = flipped[j] <= 0 ?
					flipped[j] + 2.0 * M_PI
					: flipped[j] - 2.0 * M_PI;
			}
			for (unsigned char flipmask = 1; flipmask < 64; flipmask++) {
				auto mod = tmpl;
				for (unsigned char j = 0; j < 6; j++) {
					if (flipmask & (1 << j)) {
						mod[j] = flipped[j];
					}
				}
				solutions.push_back(mod);
			}
		}
	}

	return solutions;
}


bool is_elbow_up(
	const Joints& q,
	const UrDh& dh
) {
	// we think in a vertical plane that rotates with the robot base
	// so that both lower and upper arms are always parallel to that plane
	double wrist_projection = dh.a2 * sin(q[1] - M_PI_2) + dh.a3 * sin(q[1] - M_PI_2 + q[2]);
	//                        '--------- term1 --------'   '------------ term2 ------------'
	// term1 is the projection of the upper arm onto the horizontal axis of that plane
	// term2 is the projection of the lower arm onto the horizontal axis of that plane

	// elbow upness flips either if the bending direction of the elbow changes (at q2 = 0°, 180°, 360°, ...)
	// or if the wrist moves to the other side of the base on the projection line
	return wrist_projection * sin(q[2]) >= 0;
}

int get_config_id(
	const double* q,
	const UrDh& dh
) {
	const int elbowSide = sin(q[2]) <= 0.0;

	const int wristSide = sin(q[4]) <= 0.0;

	const double upperArmInclination = -q[1];
	const double forearmInclination = upperArmInclination - q[2];
	const double wristInclination = forearmInclination - q[3] - M_PI_2;
	const double wristProjection =
		dh.a2 * cos(upperArmInclination)
		+ dh.a3 * cos(forearmInclination)
		+ dh.d4 * cos(wristInclination);
    const int overhead = wristProjection >= 0.0;

    return overhead * 4 + elbowSide * 2 + wristSide;
}
