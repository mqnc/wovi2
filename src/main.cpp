#include "CivetServer.h"
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>
#include <sstream>

#define RAW_KINEMATICS_FUNCTIONS
#include "kinematics/ur/urkin.h"

#include "workspaceworker.hpp"

#include "scenegraph/scene.h"
#include "scenegraph/spawn.h"

#include "collision/collisionchecker.h"

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

Sampling sampling {
	-1.0, 1.0, 30,
	-1.0, 1.0, 30,
	-1.0f + (float) kine.d1, 1.0f + (float) kine.d1, 30,
	{1, 2, 4, 8}
};

std::atomic<mg_connection*> sharedConn;
std::array<double, 12> sharedPose;
std::atomic<uint64_t> latestRequestId = 0;

void send(const std::vector<char>& packet) {
	mg_websocket_write(
		sharedConn,
		MG_WEBSOCKET_OPCODE_BINARY,
		packet.data(),
		packet.size()
	);
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

	Scene scene;


	string object_id = "__world__";

	string urdf = R"(
		<?xml version="1.0"?>
		<robot name="world">
			<link name="origin">
			</link>
		</robot>
	)";

	scene.robotTemplates.emplace(object_id,
		make_shared<CollisionRobotTemplate>(object_id, parseUrdf(urdf)));

	scene.robots.emplace(object_id,
		CollisionRobot(*scene.robotTemplates.at(object_id), object_id));

	spawn(
		scene,
		"robot",
		"/home/mirko/development/wovi2/models/ur5e/ur5e.urdf",
		{},
		{0, 0, 0, 0, 0, 0},
		"__world__",
		"origin"
	);

	spawn(
		scene,
		"obstacle",
		"/home/mirko/development/wovi2/models/box.urdf",
		{0.4, 0.4, 0.4},
		{},
		"__world__",
		"origin"
	);

	vector<pair<btCollisionObject*, BitMask>> objects = scene.extractBulletObjectsAndBitMasks();

	CollisionChecker checker {objects};

	auto score = [&](const double* pose) -> float {

		double q_sols[48];
		auto num_sols = ik_raw(pose, q_sols, 0, kine);
		valarray<double> joints(6);

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
				joints[j] = q;
			}

			scene.getRobot("robot").setJointPositions(joints);

            auto nCols = checker.checkCollisions(false).numCollisions;
			if (nCols == 0) {
				return 1;
			}
		}

		return -1;
	};


	const char* options[] = {"listening_ports", "8080", nullptr};
	CivetServer server(options);

	std::thread(
		worker,
		std::cref(sampling),
		std::cref(latestRequestId),
		std::cref(sharedPose),
		std::function<float(const double[12])>(score),
		std::function<void(const std::vector<char>&)>(send)
		) .detach();

	WSHandler ws;
	server.addWebSocketHandler("/ws", &ws);

	std::this_thread::sleep_for(std::chrono::hours(24));
}
