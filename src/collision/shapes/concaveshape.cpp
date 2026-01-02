
#include "concaveshape.h"

namespace collision {

ConcaveTriangleMeshShape::ConcaveTriangleMeshShape(shared_ptr<btTriangleMesh> mesh):
	bulletShape {mesh.get()},
	// might be multi-thread unsafe as the shared pointer is not copied yet
	mesh {mesh}
{
	bulletShape.setUserPointer(static_cast<Shape*>(this));
	bulletShape.updateBound();
	setSafetyMargin(0);
}

// ConcaveTriangleMeshShape::ConcaveTriangleMeshShape(const ConcaveTriangleMeshShape& other):
// 	bulletShape {other.mesh.get()},
// 	// might be multi-thread unsafe as the shared pointer is not copied yet
// 	mesh {other.mesh}
// {
// 	bulletShape.setUserPointer(static_cast<Shape*>(this));
// 	bulletShape.updateBound();
// 	setSafetyMargin(0);
// }

ConcaveTriangleMeshShape::ConcaveTriangleMeshShape(ConcaveTriangleMeshShape&& other) noexcept:
	bulletShape {other.mesh.get()},
	// might be multi-thread unsafe as the shared pointer is not copied yet
	mesh {std::move(other.mesh)}
{
	bulletShape.setUserPointer(static_cast<Shape*>(this));
	bulletShape.updateBound();
	setSafetyMargin(0);
}

unique_ptr<Shape> ConcaveTriangleMeshShape::clone() const {
	// might be multi-thread unsafe as the shared pointer is not copied yet
	auto other = make_unique<ConcaveTriangleMeshShape>(mesh);
	other->setSafetyMargin(getSafetyMargin());
	return other;
}

btGImpactMeshShape* ConcaveTriangleMeshShape::getBulletShape() {
	return &bulletShape;
}

size_t ConcaveTriangleMeshShape::getNumTriangles() {
	return mesh->getNumTriangles();
}

std::vector<btTriangleShapeEx> ConcaveTriangleMeshShape::getTriangles() {
	std::vector<btTriangleShapeEx> result;
	result.reserve(getNumTriangles());
	bulletShape.getMeshPart(0)->lockChildShapes();
	for (size_t i = 0; i < getNumTriangles(); i++) {
		btTriangleShapeEx triangle;
		bulletShape.getMeshPart(0)->getBulletTriangle(i, triangle);
		result.push_back(triangle);
	}
	bulletShape.getMeshPart(0)->unlockChildShapes();
	return result;
}

void ConcaveTriangleMeshShape::setSafetyMargin(double margin) {
	bulletShape.setMargin(margin);
	bulletShape.updateBound();
}

double ConcaveTriangleMeshShape::getSafetyMargin() const {
	return bulletShape.getMargin();
}



}
