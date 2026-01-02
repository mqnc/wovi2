
// The Nodes in the Scene Graph

// A node can have many child nodes and one parent node, to which it is attached
// with a sweeping transformation along/around one axis (translation or
// rotation) or an arbitrary static transformation. Collision objects are also
// attached to nodes. Please check out the devlog entry about hierarchical
// collision checking.

#pragma once

#include "stl.h"
#include "bitmask.h"
#include "fileio.h"
#include "collisionobject.h"
#include "collision/collisionchecker.h"
#include "export/checkertojson.h"
#include "utils/profiling.h"

// todo: this must go into a separate file
using CollisionIgnoreGroup = unordered_set<CollisionObject*>;

class Node {
	Node* parent = nullptr;
	vector<Node*> children;
	Dict<unique_ptr<CollisionObject>> parts;
	Transform forwardKinematicsResult;

    unique_ptr<CollisionChecker> checker;

public:
	const string name = "";

	variant<Transform, SweepingTransform> trafo;

	Node(const string& name, const Transform& t):
		name {name}, trafo {t} {}

	Node(const string& name, const SweepingTransformConfig& cfg):
		name {name}, trafo(cfg) {}

	void appendChildNode(Node* child) {
		assert(child->parent == nullptr);
		child->parent = this;
		children.push_back(child);
	}

	const Node* getParent() const { return parent; }
	const vector<Node*>& getChildren() const { return children; }

	void appendPart(const string& name, const collision::CompoundShape& compound) {
		parts.insert({name, make_unique<CollisionObject>(this->name + "." + name, compound)});
	}

	CollisionObject* getPart(const string& name) {
		return parts.at(name).get();
	}

	const CollisionObject* getPart(const string& name) const {
		return parts.at(name).get();
	}

	Dict<unique_ptr<CollisionObject>>& getParts() {
		return parts;
	}

	const Dict<unique_ptr<CollisionObject>>& getParts() const {
		return parts;
	}

	bool isSweeping() {
		return false;
	}

	// void leftMultiplyTransform(const variant<Transform, SweepingTransform>& tf) {
	// 	for (auto& child: children) {
	// 		child->leftMultiplyTransform(tf);
	// 	}
	// 	for (auto& [partName, part]: parts) {
	// 		part->leftMultiplyTransform(tf);
	// 	}
	// }

	void updateBulletTransforms(Transform parentPose = Transform {}) {
		Transform myGlobalPose = parentPose * std::get<Transform>(trafo);
		for (auto& child: children) {
			child->updateBulletTransforms(myGlobalPose);
		}
		for (auto& [partName, part]: parts) {
			part->reset();
			part->leftMultiplyTransform(myGlobalPose);
		}
	}

	void collectBulletCollisionObjects(
		vector<pair<btCollisionObject*, CollisionIgnoreGroupBitMasks>>& collection
	) {
		for (auto& [partName, part]: parts) {
			collection.push_back({
				part->getPermanentBulletObject(), {
					part->globalCollisionBitMask,
					part->temporaryCollisionBitMask
				}
			});
		}
		for (auto& child: children) {
			child->collectBulletCollisionObjects(collection);
		}
	}

	void computeForwardKinematics(Transform parentPose = Transform {}) {
		if (isSweeping()) { throw runtime_error("cannot compute forward kinematics when sweeping"); }
		if (holds_alternative<Transform>(trafo)) {
			forwardKinematicsResult = parentPose * std::get<Transform>(trafo);
		}
		else {
			forwardKinematicsResult = parentPose * std::get<SweepingTransform>(trafo).getSteps()[0];
		}
		for (const auto& child: children) {
			child->computeForwardKinematics(forwardKinematicsResult);
		}
	}

	const Transform& getForwardKinematicsResult() { return forwardKinematicsResult; }


	void generateCollisionBitMaskPerPart(int& counter) {
		BitMask mask;
		mask.set(++counter);
		for (auto& [partName, part]: parts) {
			part->temporaryCollisionBitMask = mask;
		}
		for (auto& child: children) {
			child->generateCollisionBitMaskPerPart(counter);
		}
	}


	void initChecker() {
        assert(!checker);

		int counter = 0;
		generateCollisionBitMaskPerPart(counter);

		vector<pair<btCollisionObject*, CollisionIgnoreGroupBitMasks>> objects;
		collectBulletCollisionObjects(objects);

        checker = make_unique<CollisionChecker>(objects);
	}


	inline static size_t collisionDetectionCallCounter = 0;
	bool isSubTreeCollisionFree(
		const Dict<CollisionIgnoreGroup>& globalCollisionIgnoreGroups,
		int logLevel = 0
	) {
        assert(checker);

		collisionDetectionCallCounter++;

        updateBulletTransforms();

		if (logLevel >= 2) {
			PROFILE_RUN(debug export);

			json debugScene = *checker;
			// writeFile("in_case_the_check_crashes.json", debugScene.dump(1, '\t'));

			auto report = checker->checkCollisions(true);

			json debugCollisions = report.collisions.value();
			json debugFile = {
				{"scene", debugScene},
				{"collisions", debugCollisions}
			};
			bool clear = report.numCollisions == 0;
			writeFile("../../log/scene" + zeroPad(collisionDetectionCallCounter, 4)
					+ "_" + name + (clear ? "_OK" : "_collision") + ".json",
				debugFile.dump(1, '\t'));

			PROFILE_PAUSE(debug export);

			return clear;
		}
		else {
			bool clear = checker->checkCollisions(false).numCollisions == 0;

			if (!clear && logLevel > 0) {
				PROFILE_RUN(debug export);

				json debugScene = *checker;

				auto report = checker->checkCollisions(true);

				json debugCollisions = report.collisions.value();
				json debugFile = {
					{"scene", debugScene},
					{"collisions", debugCollisions}
				};

				writeFile("../../log/scene" + zeroPad(collisionDetectionCallCounter, 4)
						+ "_" + name + "_collision.json", debugFile.dump(1, '\t'));

				PROFILE_PAUSE(debug export);
			}
			return clear;
		}
	}

	void reset() {
		forwardKinematicsResult = Transform {};
		for (auto& child: children) {
			child->reset();
		}
		for (auto& [partName, part]: parts) {
			part->reset();
		}
	}

};
