
cmake_minimum_required(VERSION 3.16)
project(wovi2)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(src)
include_directories(src/utils)
include_directories(src/thirdparty)
include_directories(src/thirdparty/bullet3)
include_directories(src/thirdparty/bullet3/src)
include_directories(src/thirdparty/civetweb/include)
include_directories(src/thirdparty/gainput/lib/include)

# Bullet

add_library(bullet
	src/thirdparty/bullet3/src/btLinearMathAll.cpp
	src/thirdparty/bullet3/src/btBulletCollisionAll.cpp
)
target_include_directories(bullet PRIVATE src/thirdparty/bullet3/src)
target_compile_definitions(bullet
	PUBLIC BT_EULER_DEFAULT_ZYX
	PUBLIC BT_USE_DOUBLE_PRECISION
)

# server

add_library(civetweb
    src/thirdparty/civetweb/src/civetweb.c
    src/thirdparty/civetweb/src/CivetServer.cpp
)
target_compile_definitions(civetweb
	PUBLIC NO_SSL
	PUBLIC NO_CGI
    PUBLIC USE_WEBSOCKET
)

# main

add_executable(main
    src/scenegraph/collisionrobot.cpp
    src/scenegraph/scene.cpp
    src/collision/collisionchecker.cpp
    src/collision/shapes/linkingcompoundshape.cpp
    src/collision/shapes/compoundshape.cpp
    src/collision/shapes/concaveshape.cpp
    src/collision/shapes/sphereshape.cpp
    src/collision/shapes/cylinderzshape.cpp
    src/collision/shapes/boxshape.cpp
    src/collision/shapes/inflatedconvexhullshape.cpp
    src/collision/shapes/shape.cpp
    src/collision/shapes/visualmeshshape.cpp
    src/kinematics/ur/urkin.cpp
    src/main.cpp
)
target_compile_definitions(main
    PUBLIC COLLISION_BETWEEN_INFLATED_BOXES_CONSIDERS_ROUNDED_EDGES
)

target_link_libraries(main bullet civetweb)
