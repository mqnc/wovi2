
#define MC_IMPLEM_ENABLE
#include "MC.h"

#include <vector>
#include <atomic>
#include <mutex>
#include <functional>
#include <thread>
#include <cstring>

struct Sampling {
	const float xMin = -1;
	const float xMax = 1;
	const float xRes0 = 20;
	const float yMin = -1;
	const float yMax = 1;
	const float yRes0 = 20;
	const float zMin = -1;
	const float zMax = 1;
	const float zRes0 = 20;
	const std::vector<float> resampleFactors = {1, 2, 4};
};

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

	for (int z = 1; z < nz - 1; ++z) {
		for (int y = 1; y < ny - 1; ++y) {
			for (int x = 1; x < nx - 1; ++x) {
				float acc = 0.0f;
				for (int dz = -1; dz <= 1; ++dz) {
					for (int dy = -1; dy <= 1; ++dy) {
						for (int dx = -1; dx <= 1; ++dx) {
							acc += k[dz + 1][dy + 1][dx + 1] * tmp[idx(x + dx, y + dy, z + dz)];
						}
					}
				}
				f[idx(x, y, z)] = acc / norm;
			}
		}
	}
}

void worker(
	const Sampling& sampling,
	const std::atomic<uint64_t>& latestRequestId,
	const std::array<double, 12>& sharedPose,
	std::function<float(const double[12])> score,
	std::function<void(const std::vector<char>&)> send
) {
	uint64_t lastHandled = 0;

	while (true) {
		uint64_t currentRequestId = latestRequestId.load();
		if (currentRequestId == lastHandled) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}

		bool firstIteration = true;

		auto cancel = [&]() {
			return latestRequestId.load() != currentRequestId && !firstIteration;
		};

		auto pose = sharedPose;

		for (float f: sampling.resampleFactors) {

			int xRes = sampling.xRes0 * f;
			int yRes = sampling.yRes0 * f;
			int zRes = sampling.zRes0 * f;

			std::vector<MC::MC_FLOAT> field(xRes * yRes * zRes);
			double poseRaw[12];
			memcpy(poseRaw, pose.data(), sizeof(poseRaw));

			for (int iz = 0; iz < zRes; ++iz) {
				if (cancel()) break;
				float z = sampling.zMin + iz * (sampling.zMax - sampling.zMin) / (zRes - 1);

				for (int iy = 0; iy < yRes; ++iy) {
					if (cancel()) break;
					float y = sampling.yMin + iy * (sampling.yMax - sampling.yMin) / (yRes - 1);

					for (int ix = 0; ix < xRes; ++ix) {
						if (cancel()) break;

						float x = sampling.xMin + ix * (sampling.xMax - sampling.xMin) / (xRes - 1);
						poseRaw[3] = x; poseRaw[7] = y; poseRaw[11] = z;
						field[(iz * yRes + iy) * xRes + ix] = score(poseRaw);
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
					sampling.xMin + v.x * (sampling.xMax - sampling.xMin) / (xRes - 1),
					sampling.yMin + v.y * (sampling.yMax - sampling.yMin) / (yRes - 1),
					sampling.zMin + v.z * (sampling.zMax - sampling.zMin) / (zRes - 1)
				};
				memcpy(p, tmp, 12); p += 12;
			}

			for (auto& n: mesh.normals) {
				float tmp[3] = {n.x, n.y, n.z};
				memcpy(p, tmp, 12); p += 12;
			}

			memcpy(p, mesh.indices.data(), ic * sizeof(uint32_t));

			send(packet);

			firstIteration = false;
		}

		lastHandled = currentRequestId;
	}
}
