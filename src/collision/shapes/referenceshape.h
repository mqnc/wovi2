
#pragma once

#include "shape.h"

namespace collision {

class ReferenceShape :public Shape {
	btEmptyShape bulletShape;
    string path;
public:
	ReferenceShape(string path);
	unique_ptr<Shape> clone() const override;
	btEmptyShape* getBulletShape() override;
    string getPath() const;
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() const override;
};

}
