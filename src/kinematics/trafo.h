
#pragma once

#include <array>
#include <cmath>

using Trafo = std::array<std::array<double, 4>, 3>;

inline Trafo identity(){
	return {{
		{{1, 0, 0, 0}},
		{{0, 1, 0, 0}},
		{{0, 0, 1, 0}}
	}};
}

inline Trafo dh(double theta, double a, double d, double alpha) {
	const double sin_theta = sin(theta);
	const double cos_theta = cos(theta);
	const double sin_alpha = sin(alpha);
	const double cos_alpha = cos(alpha);
	return {{
		{{cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta}},
		{{sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta}},
		{{0, sin_alpha, cos_alpha, d}}
	}};
}

inline Trafo operator*(const Trafo& a, const Trafo& b) {
	Trafo m;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 4; ++j) {
			m[i][j] = 0;
			for (int k = 0; k < 3; ++k) {
				m[i][j] += a[i][k] * b[k][j];
			}
		}
		m[i][3] += a[i][3];
	}
	return m;
}

inline Trafo inv(const Trafo& a) {
	Trafo m;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			m[i][j] = a[j][i];
		}
		m[i][3] = 0;
		for (int j = 0; j < 3; j++) {
			m[i][3] -= m[i][j] * a[j][3];
		}
	}
	return m;
}

struct Delta { double distance; double angle; };
inline Delta delta(const Trafo& d) {
	double cos_angle = (d[0][0] + d[1][1] + d[2][2] - 1.0) / 2.0;
	if (cos_angle > 1.0) { cos_angle = 1.0; }
	else if (cos_angle < -1.0) { cos_angle = -1.0; }
	return {
		sqrt(d[0][3] * d[0][3] + d[1][3] * d[1][3] + d[2][3] * d[2][3]),
		acos(cos_angle)
	};
}
inline Delta delta(const Trafo& a, const Trafo& b) {
	return delta(inv(a) * b);
}
