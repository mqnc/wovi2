
#pragma once

#include "collision/collisionchecker.h"
#include "shapestojson.h"

using nlohmann::json;

const Part* getPartFromBtObject(const btCollisionObject* obj){
	if (obj->getUserPointer() == nullptr) { throw runtime_error("btCollisionObject is not member of a Part"); }
	auto part = static_cast<Part*>(obj->getUserPointer());
	if (&(part->bulletObject) != obj) { throw runtime_error("invalid cast of btCollisionObject"); }
    return part;
}

string btCollisionObjectToString(const btCollisionObject* obj) {
    auto part = getPartFromBtObject(obj);
	return string {} + part->robotId + "." + part->linkId;
}

void to_json(json& j, const CollisionChecker& cc) {
	const auto& btObjects = cc.getCollisionWorld().getCollisionObjectArray();
	const auto& sceneInfo = cc.getSceneInfo();

	j = {};
	for (int i = 0; i < btObjects.size(); i++) {
		const btCollisionShape* btShape = btObjects[i]->getCollisionShape();
		const collision::Shape* shape = collision::Shape::castFromBullet(btShape);
        const collision::Shape* visual = getPartFromBtObject(btObjects[i])->visualShape.get();

		j.push_back(json {
			{"name", btCollisionObjectToString(btObjects[i])},
			{"shape", *shape},
            {"visual", *visual},
			{"trafo", btObjects[i]->getWorldTransform()},
			{"collisionIgnoreGroups", sceneInfo[i].second.toString()},
		});
	}
}

void to_json(json& j, const CollisionChecker::Collision& c) {
	j = {
		{"objectA", btCollisionObjectToString(c.objectA)},
		{"objectB", btCollisionObjectToString(c.objectB)},
		{"pointOnA", c.pointOnA},
		{"pointOnB", c.pointOnB}
	};
}
