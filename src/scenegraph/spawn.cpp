
#include "spawn.h"
#include "bullet3/src/LinearMath/btTransform.h"

btTransform btTransformFromPose(const Pose& pose) {
	size_t qnans = isnan(pose.qx) + isnan(pose.qy)
		+ isnan(pose.qz) + isnan(pose.qw);
	if (qnans != 0 && qnans != 4) {
		throw runtime_error("orientation cannot be changed partially");
	}
	return btTransform(
		btQuaternion(pose.qx, pose.qy, pose.qz, pose.qw),
		btVector3(pose.x, pose.y, pose.z)
	);
}

void spawn(
    Scene& scene,
	const string& object_id,
	const string& description_file,
	const Pose& pose,
	const valarray<double>& joint_positions,
	const string& parent_object_id,
	const string& parent_link
) {

	if (object_id.find("__", 0) == 0) {
		throw runtime_error(object_id
			+ " ids starting with '__' are reserved");
	}

	scene.assertIdFree(object_id);

	auto dir = std::filesystem::path(description_file).parent_path();
	auto urdf_source = scene.loadFile(description_file);
	auto urdf = parseUrdf(urdf_source);

	vector<string> actuatedJoints = urdf.defaultJointSelection;

	Dict<string> meshSources = {};
	for (const auto& link: urdf.links) {
		for (const auto& geom: link.collisionGeometries) {
			const auto& f = geom.filename;
			if (f != "") {
				if (std::filesystem::path(f).is_absolute()) {
					meshSources[f] = scene.loadFile(f, true);
				}
				else {
					meshSources[f] = scene.loadFile(dir / f, true);
				}
			}
		}
	}

	Dict<vector<string>> collisionIgnoreGroups = {};

	scene.robotTemplates[object_id] =
		make_shared<CollisionRobotTemplate>(object_id,
			urdf, meshSources, actuatedJoints, collisionIgnoreGroups);

	scene.robots.emplace(object_id,
		CollisionRobot(*scene.robotTemplates.at(object_id), object_id));

	auto& robot = scene.getRobot(object_id);

	auto& parent = scene.getRobot(parent_object_id);
	robot.setParent(&parent, parent_link);
	robot.setBaseTrafo(btTransformFromPose(pose));

	if (joint_positions.size() > 0) {
		robot.setJointPositions(joint_positions);
	}

	for (const auto& group_kv: robot.getCollisionIgnoreGroups()) {
		scene.collisionIgnoreGroupManager.createGroup(
			group_kv.first, group_kv.second);
	}
}
