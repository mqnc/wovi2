
#pragma once

#include "shape.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

namespace collision {

class ConcaveTriangleMeshShape :public Shape {
    btGImpactMeshShape bulletShape;
	shared_ptr<btTriangleMesh> mesh;
public:
	ConcaveTriangleMeshShape(shared_ptr<btTriangleMesh> mesh);
	// ConcaveTriangleMeshShape(const ConcaveTriangleMeshShape& other); // copy constructor
	ConcaveTriangleMeshShape(ConcaveTriangleMeshShape&& other) noexcept; // move constructor
	unique_ptr<Shape> clone() const override;
	btGImpactMeshShape* getBulletShape() override;
	size_t getNumTriangles();
	std::vector<btTriangleShapeEx> getTriangles();
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() const override;
};

}
