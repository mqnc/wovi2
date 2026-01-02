
#pragma once

#include "stl.h"

class CollisionIgnoreGroupManager {

	Dict<unordered_set<string>> registry;
	Dict<BitMask> bitMasks;
	bool ready = false;

public:

	template<template<class> class Cont>
	void createGroup(
		const string& name,
		const Cont<string>& memberIds
	) {
		ready = false;

		if (registry.count(name) > 0) {
			throw runtime_error(
				string("collision ignore group \"")
				+ name + "\" already exists"
			);
		}

		registry[name]={};
		for(const auto &id:memberIds){
			registry[name].insert(id);
		}
	}

	template<template<class> class Cont>
	void resetGroup(const string& name,
		const Cont<string>& memberIds
	) {
		ready = false;

		if (registry.count(name) == 0) {
			throw runtime_error(
				string("collision ignore group \"")
				+ name + "\" does not exist"
			);
		}

		registry[name]={};
		for(const auto &id:memberIds){
			registry[name].insert(id);
		}
	}

	void deleteGroup(const string& name) {
		ready = false;

		if (registry.count(name) == 0) {
			cout << "available collision ignore groups:\n";
			for(const auto& [id, _]:registry){
				cout << id << " ";
			}
			throw runtime_error(
				string("collision ignore group \"")
				+ name + "\" does not exist"
			);
		}
		registry.erase(name);
	}

	void generateBitMasks() {
		bitMasks.clear();

		int index = 0;
		for (const auto& group_kv : registry) {
			for (const auto& id : group_kv.second) {
				if (bitMasks.count(id) == 0) {
					bitMasks[id] = BitMask{};
				}
				bitMasks[id].set(index);
			}
			index++;
		}

		ready = true;
	}

	BitMask getBitMask(const string& id) {
		if (not ready) {
			throw runtime_error("bit masks must be generated first");
		}

		if (bitMasks.count(id) == 0) {
			return BitMask{};
		}
		else {
			return bitMasks[id];
		}
	}

	const Dict<unordered_set<string>>& getGroups() const {
		return registry;
	}

	Dict<unordered_set<string>>& getGroups(){
		return registry;
	}

	void clear() {
		ready = false;

		registry.clear();
		bitMasks.clear();
	}

};