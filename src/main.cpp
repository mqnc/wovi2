#include "CivetServer.h"
#include <vector>
#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>
#include <iostream>
#include <sstream>
#include <atomic>
#include <mutex>

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
const std::vector<int> resSteps = {40, 80, 160, 320};

const float xMin = -1.0, xMax = 1.0;
const float yMin = -1.0, yMax = 1.0;
const float zMin = -1.0 + kine.d1, zMax = 1.0 + kine.d1;


double ikScore(double* pose) {

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


void blur3D(std::vector<MC::MC_FLOAT>& f, int nx, int ny, int nz) {
	static const float k[3][3][3] = {
		{{1, 2, 1}, {2, 4, 2}, {1, 2, 1}},
		{{2, 4, 2}, {4, 8, 4}, {2, 4, 2}},
		{{1, 2, 1}, {2, 4, 2}, {1, 2, 1}}
	};
	const float norm = 64.0f;

	std::vector<MC::MC_FLOAT> tmp = f;

	auto idx = [&](int x, int y, int z) {
		return (z * ny + y) * nx + x;
	};

	for (int z = 1; z < nz - 1; ++z)
		for (int y = 1; y < ny - 1; ++y)
			for (int x = 1; x < nx - 1; ++x) {
				float acc = 0.0f;
				for (int dz = -1; dz <= 1; ++dz)
					for (int dy = -1; dy <= 1; ++dy)
						for (int dx = -1; dx <= 1; ++dx)
							acc += k[dz + 1][dy + 1][dx + 1] * tmp[idx(x + dx, y + dy, z + dz)];

				f[idx(x, y, z)] = acc / norm;
			}
}



std::atomic<uint64_t> latestRequestId {0};
std::array<double, 12> sharedPose;
std::atomic<mg_connection*> sharedConn {nullptr};

void worker() {
	uint64_t lastHandled = 0;

	while (true) {
		uint64_t currentRequestId = latestRequestId.load();
		if (currentRequestId == lastHandled) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}

        bool firstIteration = true;

        auto cancel = [&](){
            return latestRequestId.load() != currentRequestId && !firstIteration;
        };

		auto pose = sharedPose;
		mg_connection* conn = sharedConn.load();

		for (int res: resSteps) {

			int xRes = res;
			int yRes = res;
			int zRes = res;

			std::vector<MC::MC_FLOAT> field(xRes * yRes * zRes);
			double poseRaw[12];
			memcpy(poseRaw, pose.data(), sizeof(poseRaw));

			for (int iz = 0; iz < zRes; ++iz) {
				if (cancel()) break;
				float z = zMin + iz * (zMax - zMin) / (zRes - 1);

				for (int iy = 0; iy < yRes; ++iy) {
					if (cancel()) break;
					float y = yMin + iy * (yMax - yMin) / (yRes - 1);

					for (int ix = 0; ix < xRes; ++ix) {
						if (cancel()) break;

						float x = xMin + ix * (xMax - xMin) / (xRes - 1);
						poseRaw[3] = x; poseRaw[7] = y; poseRaw[11] = z;
						field[(iz * yRes + iy) * xRes + ix] = ikScore(poseRaw);
					}
				}
			}

			if (cancel()) break;

			blur3D(field, xRes, yRes, zRes);

			MC::mcMesh mesh;
			MC::marching_cube(field.data(), xRes, yRes, zRes, mesh);

			if (cancel()) break;

			uint32_t vc = mesh.vertices.size();
			uint32_t ic = mesh.indices.size();

			size_t bytes = 8 + vc * 6 * sizeof(float) + ic * sizeof(uint32_t);
			std::vector<char> packet(bytes);
			char* p = packet.data();

			memcpy(p, &vc, 4); p += 4;
			memcpy(p, &ic, 4); p += 4;

			for (auto& v: mesh.vertices) {
				float tmp[3] = {
					xMin + v.x * (xMax - xMin) / (xRes - 1),
					yMin + v.y * (yMax - yMin) / (yRes - 1),
					zMin + v.z * (zMax - zMin) / (zRes - 1)
				};
				memcpy(p, tmp, 12); p += 12;
			}

			for (auto& n: mesh.normals) {
				float tmp[3] = {n.x, n.y, n.z};
				memcpy(p, tmp, 12); p += 12;
			}

			memcpy(p, mesh.indices.data(), ic * sizeof(uint32_t));

			mg_websocket_write(
				sharedConn,
				MG_WEBSOCKET_OPCODE_BINARY,
				packet.data(),
				packet.size()
			);

            firstIteration = false;
		}

		lastHandled = currentRequestId;
	}
}


class WSHandler: public CivetWebSocketHandler {
public:
	bool handleConnection(CivetServer*, const mg_connection*) override { return true; }

	bool handleData(CivetServer*, mg_connection* conn, int, char* data, size_t len) override {

		std::stringstream ss(std::string(data, len));
		for (int i = 0; i < 12; ++i) {
			std::string t;
			std::getline(ss, t, ',');
			sharedPose[i] = std::stod(t);
		}

		sharedConn.store(conn);
		latestRequestId.fetch_add(1);
		return true;

	}
};

int main() {
	const char* options[] = {"listening_ports", "8080", nullptr};
	CivetServer server(options);

	std::thread(worker).detach();

	WSHandler ws;
	server.addWebSocketHandler("/ws", &ws);

	std::this_thread::sleep_for(std::chrono::hours(24));
}
