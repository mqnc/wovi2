
#pragma once

#include "stl.h"
#include "collisionrobot.h"

class Scene {
public:

	Dict<shared_ptr<CollisionRobotTemplate>> robotTemplates;

	Dict<CollisionRobot> robots;

	CollisionIgnoreGroupManager collisionIgnoreGroupManager;

	void assertIdFree(string id);

	bool isRobotLink(const string& id);

	CollisionRobot& getRobot(string id);

	void cacheFile(const string& name, const string& content, bool isBase64=false);

	string loadFile(const string& name, bool binary = false);

	bool isCached(const string& name) const;

	const Dict<string>& getCache() const;

	void clearCache();

	vector<pair<btCollisionObject*, BitMask>> extractBulletObjectsAndBitMasks();

private:
	Dict<string> fileCache;
};
