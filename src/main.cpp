#include "CivetServer.h"
#include <vector>
#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>
#include <iostream>

#define RAW_KINEMATICS_FUNCTIONS
#include "kinematics/ur/urkin.h"

#define MC_IMPLEM_ENABLE
#include "MC.h"

const auto kine = UR5E_DH;

std::array<std::array<double, 2>, 6> jointLimits = {{
	{-3.14, 3.14},
	{-3.14, 0.01},
	{0.01, 3.14},
	{-3.14, 3.14},
	{0.01, 3.14},
	{-3.14, 3.14}
}};

const int targetConfigId = 0;

const float n = 40;

const float xMin = -1.0, xMax = 1.0, nx = n;
const float yMin = -1.0, yMax = 1.0, ny = n;
const float zMin = -1.0 + kine.d1, zMax = 1.0 + kine.d1, nz = n;

double ikScore(double* pose) {
	// we will try to tweak this to give distance-like values around the validity boundary
	// such that the marching cubes surface is nice and smooth

	double q_sols[48];
	auto num_sols = ik_raw(pose, q_sols, 0, kine);

	for (size_t k = 0; k < num_sols; k++) {

		if (get_config_id(q_sols + k * 6, kine) != targetConfigId) {
			continue;
		}

		for (size_t j = 0; j < 6; j++) {
			double q = q_sols[k * 6 + j];

			if (
				!(
					q > jointLimits[j][0] && q < jointLimits[j][1]
					|| q - 2.0 * M_PI > jointLimits[j][0] && q - 2.0 * M_PI < jointLimits[j][1]
					|| q + 2.0 * M_PI > jointLimits[j][0] && q + 2.0 * M_PI < jointLimits[j][1]
					)
			) {
				continue;
			}
		}

		return 1;
	}

	return -1;
}


using Clock = std::chrono::steady_clock;

class WSHandler: public CivetWebSocketHandler {
public:
	bool handleConnection(CivetServer*, const mg_connection*) override {
		return true;
	}

	void handleReadyState(CivetServer*, mg_connection* conn) override {
		std::thread([conn]() {

			std::vector<MC::MC_FLOAT> field(nx * ny * nz);

			auto t0 = Clock::now();
			float phi = 0.0;

			while (true) {
				float t = std::chrono::duration<float>(Clock::now() - t0).count();

				// double pose[12] = {
				// 	1, 0, 0, 0,
				// 	0, 1, 0, 0,
				// 	0, 0, 1, 0
				// };

				const float c = cos(phi);
				const float s = sin(phi);

				double pose[12] = {
					c, 0, -s, 0,
					0, 1, 0, 0,
					s, 0, c, 0
				};

				for (int iz = 0; iz < nz; ++iz) {
					float z = zMin + iz * (zMax - zMin) / (nz - 1);
					for (int iy = 0; iy < ny; ++iy) {
						float y = yMin + iy * (yMax - yMin) / (ny - 1);
						for (int ix = 0; ix < nx; ++ix) {
							float x = xMin + ix * (xMax - xMin) / (nx - 1);

							pose[3] = x;
							pose[7] = y;
							pose[11] = z;

							double v = ikScore(pose);

							field[(iz * ny + iy) * nx + ix] = v;
						}
					}
				}

				MC::mcMesh mesh;
				MC::marching_cube(field.data(), nx, ny, nz, mesh);

				const uint32_t vertexCount = mesh.vertices.size();
				const uint32_t indexCount = mesh.indices.size();

				size_t bytes =
					4 + 4 +
					vertexCount * 3 * sizeof(float) + // positions
					vertexCount * 3 * sizeof(float) + // normals
					indexCount * sizeof(uint32_t);

				std::vector<char> packet(bytes);
				char* p = packet.data();

				memcpy(p, &vertexCount, 4); p += 4;
				memcpy(p, &indexCount, 4); p += 4;

				for (const auto& v: mesh.vertices) {
					float x = xMin + v.x * (xMax - xMin) / (nx - 1);
					float y = yMin + v.y * (yMax - yMin) / (ny - 1);
					float z = zMin + v.z * (zMax - zMin) / (nz - 1);
					float tmp[3] = {x, y, z};
					memcpy(p, tmp, sizeof(tmp));
					p += sizeof(tmp);
				}

				for (const auto& n: mesh.normals) {
					float tmp[3] = {n.x, n.y, n.z};
					memcpy(p, tmp, sizeof(tmp));
					p += sizeof(tmp);
				}

				memcpy(p, mesh.indices.data(), indexCount * sizeof(uint32_t));

				mg_websocket_write(
					conn,
					MG_WEBSOCKET_OPCODE_BINARY,
					packet.data(),
					packet.size()
				);

				phi += 0.1;
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}) .detach();
	}
};

int main() {

	const char* options[] = {
		"listening_ports", "8080",
		nullptr
	};

	CivetServer server(options);

	WSHandler ws;
	server.addWebSocketHandler("/ws", &ws);

	std::this_thread::sleep_for(std::chrono::hours(24));
}
