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

#include "scenevis/checkertojson.h"

const auto kine = UR5E_DH;

std::array<std::array<double, 2>, 6> jointLimits = {{
	{-3.14, 3.14},
	{-3.14, 0.01},
	{0.01, 3.14},
	{-3.14, 3.14},
	{-3.14, -0.01},
	{-3.14, 3.14}
}};

const int targetConfigId = 1;

Sampling sampling {
	-1.0, 1.0, 20,
	-1.0, 1.0, 20,
	-1.0f + (float) kine.d1, 1.0f + (float) kine.d1, 20,
	{1, 2, 4, 8}
};

std::atomic<mg_connection*> sharedConn;
std::atomic<bool> setupNeeded;
std::mutex posesMutex;
std::map<string, std::array<double, 12>> sharedPoses;
std::array<double, 12> sharedTcpPose;
std::atomic<uint64_t> latestRequestId = 0;

void sendBinary(const std::vector<char>& packet) {
	if (sharedConn.load() != 0) {
		mg_websocket_write(
			sharedConn,
			MG_WEBSOCKET_OPCODE_BINARY,
			packet.data(),
			packet.size()
		);
	}
}

void sendJson(const json& data) {
	if (sharedConn.load() != 0) {
		std::string s = data.dump();
		mg_websocket_write(
			sharedConn,
			MG_WEBSOCKET_OPCODE_TEXT,
			s.data(),
			s.size()
		);
	}
}

class WSHandler: public CivetWebSocketHandler {
public:
	bool handleConnection(CivetServer*, const mg_connection*) override {
		setupNeeded.store(true);
		return true;
	}

	bool handleData(CivetServer*, mg_connection* conn, int bits, char* data, size_t len) override {

		if ((bits & 0x0F) != MG_WEBSOCKET_OPCODE_TEXT) {
			return true;
		}

		std::string s {data, len};

		auto colon = s.find(':');
		std::string id = s.substr(0, colon);
		std::string poseData = s.substr(colon + 1);
		std::stringstream ss(poseData);

		std::array<double, 12> pose;

		for (int i = 0; i < 12; ++i) {
			std::string t;
			std::getline(ss, t, ',');
			pose[i] = std::stod(t);
		}

		{
			std::lock_guard<std::mutex> lock(posesMutex);
			sharedPoses[id] = pose;
		}

		sharedConn.store(conn);
		latestRequestId.fetch_add(1); // triggers worker
		return true;

	}

	void handleClose(CivetServer* server, const struct mg_connection* conn) override {
		sharedConn.store(0);
		latestRequestId.store(0);
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
		// "/home/mirko/development/wovi2/models/ur5e/ur5e.urdf",
		"/home/mirko/development/wovi2/models/ur/ur5e.urdf", {
			{"package://ur_description/meshes/ur5e/collision", "meshes/ur5e/collision"},
			{"package://ur_description/meshes/ur5e/visual", "models/ur/meshes/ur5e/visual"},
		},
		{0, 0, 0, 0, 0, 1, 0}, // rotated by 180° so urdf (ros convention) lines up with ik (ur convention)
		{0, 0, 0, 0, 0, 0},
		"__world__",
		"origin"
	);

	spawn(
		scene,
		"1",
		"/home/mirko/development/wovi2/models/pallet.urdf",
		{{"pallet.glb", "models/pallet.glb"}},
		{0.2, 0.0, -0.075},
		{},
		"__world__",
		"origin"
	);

	spawn(
		scene,
		"2",
		"/home/mirko/development/wovi2/models/panel.urdf",
		{{"panel.glb", "models/panel.glb"}},
		{0, -0.7, 1 - 0.075, M_SQRT1_2, 0, 0, M_SQRT1_2},
		{},
		"__world__",
		"origin"
	);

	spawn(
		scene,
		"3",
		"/home/mirko/development/wovi2/models/cocube.urdf",
		{{"cocube.glb", "models/cocube.glb"}},
		{0.6, 0.0, 0.351},
		{},
		"__world__",
		"origin"
	);

	spawn(
		scene,
		"4",
		"/home/mirko/development/wovi2/models/ball.urdf",
		{{"ball.glb", "models/ball.glb"}},
		{0, 0.6, 0},
		{},
		"__world__",
		"origin"
	);

	spawn(
		scene,
		"5",
		"/home/mirko/development/wovi2/models/rubik.urdf",
		{{"rubik.glb", "models/rubik.glb"}},
		{-0.4, 0.4, 0.4},
		{},
		"__world__",
		"origin"
	);

    scene.getRobot("robot").setActuationState(true);


	vector<string> passive;
	for (const auto& [robotId, robot] : scene.robots) {
		for (const auto& part : robot.getParts()) {
			if (!robot.isActuated) {
				passive.push_back(robotId + "." + part->linkId);
			}
		}
	}

    scene.collisionIgnoreGroupManager.createGroup("__passive__", passive);
    scene.collisionIgnoreGroupManager.generateBitMasks();

	for (auto& [robotId, robot] : scene.robots) {
		for (auto& part : robot.getMutableParts()) {
			part->collisionBitMask = scene.collisionIgnoreGroupManager.getBitMask(
				robotId + "." + part->linkId);
		}
	}

	vector<pair<btCollisionObject*, BitMask>> objects = scene.extractBulletObjectsAndBitMasks();

	CollisionChecker checker {objects};

	auto ik = [](const double* pose) -> valarray<double> {
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
				) { goto continueOuter; }
				joints[j] = q;
			}
			return joints;

		continueOuter:;
		}
		return {};
	};

	auto updatePoses = [&]() {
		std::lock_guard<std::mutex> lock(posesMutex);

		for (const auto& [id, poseMtx]: sharedPoses) {
			if (id == "tcp") { continue; }

			btMatrix3x3 basis(
				poseMtx[0], poseMtx[1], poseMtx[2],
				poseMtx[4], poseMtx[5], poseMtx[6],
				poseMtx[8], poseMtx[9], poseMtx[10]
			);

			btVector3 origin(
				poseMtx[3],
				poseMtx[7],
				poseMtx[11]
			);

			btTransform tf(basis, origin);
			scene.getRobot(id).setBaseTrafo(tf);
		}

        sharedTcpPose = sharedPoses["tcp"];
	};

	auto setupVisualization = [&](const double* pose) -> void {
		auto joints = ik(pose);
		if (joints.size() == 0) { return; }
		scene.getRobot("robot").setJointPositions(joints);
		json data = checker;
		sendJson(data);
	};

	auto updateVisualization = [&](const double* pose) -> void {
		auto joints = ik(pose);
		if (joints.size() == 0) { return; }
		scene.getRobot("robot").setJointPositions(joints);
		const auto& btObjects = checker.getCollisionWorld().getCollisionObjectArray();
		json data = json::object();
		for (int i = 0; i < btObjects.size(); i++) {
			data[btCollisionObjectToString(btObjects[i])] =
				btObjects[i]->getWorldTransform();
		}
		data["__isUpdate__"] = true;
		sendJson(data);
	};

	auto score = [&](const double* pose) -> float {
		auto joints = ik(pose);
		if (joints.size() == 0) { return -1; }
		scene.getRobot("robot").setJointPositions(joints);
		auto nCols = checker.checkCollisions(false).numCollisions;
		return nCols == 0 ? 1 : -1;
	};


	const char* options[] = {"listening_ports", "8080", nullptr};
	CivetServer server(options);

	std::thread(
		workspaceWorker,
		std::cref(sampling),
		std::ref(setupNeeded),
		std::cref(latestRequestId),
		std::cref(sharedTcpPose),
		std::function<void(void)>(updatePoses),
		std::function<void(const double[12])>(setupVisualization),
		std::function<void(const double[12])>(updateVisualization),
		std::function<float(const double[12])>(score),
		std::function<void(const std::vector<char>&)>(sendBinary)
		) .detach();

	WSHandler ws;
	server.addWebSocketHandler("/ws", &ws);

	std::this_thread::sleep_for(std::chrono::hours(24));
}
