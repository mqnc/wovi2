
#include "sphereshape.h"

namespace collision {

SphereShape::SphereShape(double radius):
	bulletShape(radius),
	radiusWithoutMargin {radius}
{

	setSafetyMargin(0);
}

unique_ptr<Shape> SphereShape::clone() const {
	auto other = make_unique<SphereShape>(getRadius());
	other->setSafetyMargin(getSafetyMargin());
	return other;
}

btSphereShape* SphereShape::getBulletShape() {
	return &bulletShape;
}

double SphereShape::getRadius() const {
	return radiusWithoutMargin;
}

void SphereShape::setSafetyMargin(double margin) {
	bulletShape.setUnscaledRadius(radiusWithoutMargin + margin);
}

double SphereShape::getSafetyMargin() const {
	return bulletShape.getRadius() - radiusWithoutMargin;
}

}
