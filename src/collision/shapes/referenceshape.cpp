
#include "referenceshape.h"

namespace collision {

ReferenceShape::ReferenceShape(string path):
	path {path}
{
    bulletShape.setUserPointer(static_cast<Shape*>(this));
}

unique_ptr<Shape> ReferenceShape::clone() const {
	auto other = make_unique<ReferenceShape>(getPath());
	return other;
}

btEmptyShape* ReferenceShape::getBulletShape() {
	return &bulletShape;
}

string ReferenceShape::getPath() const {
	return path;
}

void ReferenceShape::setSafetyMargin(double margin) {
	(void) margin;
	throw runtime_error("ReferenceShape can't have safety margin");
}

double ReferenceShape::getSafetyMargin() const {
	throw runtime_error("ReferenceShape can't have safety margin");
}

}
