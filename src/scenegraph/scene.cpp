
#include "scene.h"
#include "str.h"
#include "base64.hpp"

void Scene::assertIdFree(string id) {
	if (robots.count(id) > 0) {
		throw runtime_error("an object named \"" + id + "\" already exists");
	}
	if (collisionIgnoreGroupManager.getGroups().count(id) > 0) {
		throw runtime_error("a collision ignore group named \""
			+ id + "\" already exists");
	}
};

bool Scene::isRobotLink(const string& id) {
	vector<string> parts = str::split(id, '.');
	if (
		parts.size() == 2
		&& robots.count(parts[0]) > 0
		&& robots.at(parts[0]).getParts().contains(byLink, parts[1])) {

		return true;
	}
	else {
		return false;
	}
}


CollisionRobot& Scene::getRobot(string id) {
	if (robots.count(id) == 0) {
		cout << "attempted to access object with id \""
			 << id << "\" which does not exist; known objects: ";
		for (const auto& robot: robots) {
			cout << '"' << robot.first << "\" ";
		}
		cout << "\n";
		// still trigger standard exception below
	}
	return robots.at(id);
}

void Scene::cacheFile(const string& name, const string& content, bool isBase64) {
	if (isBase64) {
		fileCache[name] = base64::from_base64(content);
	}
	else {
		fileCache[name] = content;
	}
}

string Scene::loadFile(const string& name, bool binary) {
	if (!fileCache.count(name)) {
		//cout << "reading " << name << " from disk" << "\n";
		fileCache[name] = str::load(name, binary);
	}
	else {
		//cout << "reading " << name << " from cache" << "\n";
	}
	return fileCache[name];
}

bool Scene::isCached(const string& name) const {
	return fileCache.count(name) > 0;
}

const Dict<string>& Scene::getCache() const{
	return fileCache;
}

void Scene::clearCache() {
	fileCache.clear();
}

vector<pair<btCollisionObject*, BitMask>> Scene::extractBulletObjectsAndBitMasks() {
	vector<pair<btCollisionObject*, BitMask>> result;

	for (auto& [robotId, robot]: robots) {
		for (auto& part: robot.getMutableParts()) {
			result.push_back({
				&(part->bulletObject),
				part->collisionBitMask
			});
		}
	}

	return result;
}
