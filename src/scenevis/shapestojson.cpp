
#include "shapestojson.h"

#include "collision/shapes/allshapes.h"

using nlohmann::json;

void to_json(json& j, const btTriangleShapeEx& t) {
	j = json {
		t.m_vertices1[0],
		t.m_vertices1[1],
		t.m_vertices1[2]
	};
}

namespace collision {

// This is arguably more elegant with the visitor pattern but that is not
// elegantly extensible with new shapes.

void to_json(json& j, const Shape& s) {
	if (dynamic_cast<const CompoundShape*>(&s)) {
		auto cs = const_cast<CompoundShape*>(dynamic_cast<const CompoundShape*>(&s));

		j = {
			{"kind", "Compound"},
			{"children", json::array()}
		};
		for (size_t i = 0; i < cs->getNumShapes(); i++) {
			j["children"].push_back({
				{"shape", *(cs->getShape(i))},
				{"trafo", cs->getShapeTrafo(i)}
			});
		}
	}
	else if (dynamic_cast<const LinkingCompoundShape*>(&s)) {
		auto lcs = const_cast<LinkingCompoundShape*>(dynamic_cast<const LinkingCompoundShape*>(&s));

		j = {
			{"kind", "Compound"},
			{"children", json::array()}
		};
		for (size_t i = 0; i < lcs->getNumShapes(); i++) {
			j["children"].push_back({
				{"shape", *(lcs->getShape(i))},
				{"trafo", lcs->getShapeTrafo(i)}
			});
		}
	}
	else if (dynamic_cast<const SphereShape*>(&s)) {
		auto cs = dynamic_cast<const SphereShape*>(&s);

		j = {
			{"kind", "Sphere"},
			{"radius", cs->getRadius()},
			{"margin", cs->getSafetyMargin()}
		};
	}
	else if (dynamic_cast<const BoxShape*>(&s)) {
		auto bs = dynamic_cast<const BoxShape*>(&s);

		j = {
			{"kind", "Box"},
			{"size", bs->getSize()},
			{"margin", bs->getSafetyMargin()}
		};
	}
	else if (dynamic_cast<const CylinderZShape*>(&s)) {
		auto cs = dynamic_cast<const CylinderZShape*>(&s);

		j = {
			{"kind", "CylinderZ"},
			{"radius", cs->getRadius()},
			{"height", cs->getHeight()},
			{"margin", cs->getSafetyMargin()}
		};
	}
	else if (dynamic_cast<const InflatedConvexHullShape*>(&s)) {
		auto ihs = dynamic_cast<const InflatedConvexHullShape*>(&s);

		j = {
			{"kind", "InflatedConvexHull"},
			{"radius", ihs->getRadius()},
			{"margin", ihs->getSafetyMargin()},
			{"points", json::array()}
		};
		for (size_t i = 0; i < ihs->getNumPoints(); i++) {
			j["points"].push_back(ihs->getPoint(i));
		}
	}
	else if (dynamic_cast<const VisualMeshShape*>(&s)) {
		auto ms = dynamic_cast<const VisualMeshShape*>(&s);

		j = {
			{"kind", "VisualMesh"},
			{"vertices", ms->getVertices()},
			{"faces", ms->getFaces()}
		};
	}
	else if (dynamic_cast<const ConcaveTriangleMeshShape*>(&s)) {
		auto cs = dynamic_cast<const ConcaveTriangleMeshShape*>(&s);

		j = {
			{"kind", "VisualMesh"},
			{"triangles", cs->getTriangles()},
			{"margin", cs->getSafetyMargin()},
		};
	}
	else if (dynamic_cast<const ReferenceShape*>(&s)) {
		auto rs = dynamic_cast<const ReferenceShape*>(&s);

		j = {
			{"kind", "ReferenceMesh"},
			{"path", rs->getPath()}
		};
	}
	else {
		throw runtime_error("json exporter encountered unknown shape type");
	}

}

}
