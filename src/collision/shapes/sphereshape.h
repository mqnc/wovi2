
#pragma once

#include "shape.h"

namespace collision {

class SphereShape :public Shape {
	btSphereShape bulletShape;
	double radiusWithoutMargin = 0;
public:
	SphereShape(double radius);
	unique_ptr<Shape> clone() const override;
	btSphereShape* getBulletShape() override;
	double getRadius() const;
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() const override;
};

}
