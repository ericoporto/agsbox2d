cmake_minimum_required(VERSION 3.14)
project(agsbox2d VERSION 0.1.0 DESCRIPTION "Box2D plugin for Adventure Game Studio.")

set(CMAKE_CXX_STANDARD 14)

include_directories(../ags/Engine/)
include_directories(../ags/Common/)

include_directories(.)

add_library(agsbox2d SHARED
        agsbox2d/PointF.cpp
        agsbox2d/PointF.h
        agsbox2d/AgsBody.cpp
        agsbox2d/AgsBody.h
        agsbox2d/agsbox2d.cpp
        agsbox2d/agsbox2d.h
        agsbox2d/AgsFixture.cpp
        agsbox2d/AgsFixture.h
        agsbox2d/AgsNumberInterface.h
        agsbox2d/AgsShape.cpp
        agsbox2d/AgsShape.h
        agsbox2d/AgsShapeCircle.cpp
        agsbox2d/AgsShapeCircle.h
        agsbox2d/AgsShapeRect.cpp
        agsbox2d/AgsShapeRect.h
        agsbox2d/AgsWorld.cpp
        agsbox2d/AgsWorld.h
        agsbox2d/Book.cpp
        agsbox2d/Book.h
        agsbox2d/Box2D.cpp
        agsbox2d/Box2D.h
        agsbox2d/DebugDraw.cpp
        agsbox2d/DebugDraw.h
        agsbox2d/Scale.cpp
        agsbox2d/Scale.h
        agsbox2d/AgsJoint.h
        agsbox2d/AgsJoint.cpp
        agsbox2d/AgsJointDistance.h
        agsbox2d/AgsJointDistance.cpp
        agsbox2d/AgsJointMouse.h
        agsbox2d/AgsJointMouse.cpp
        agsbox2d/AgsJointMotor.h
        agsbox2d/AgsJointMotor.cpp
        agsbox2d/AgsJointPulley.h
        agsbox2d/AgsJointPulley.cpp
        agsbox2d/SerialHelper.cpp
        agsbox2d/SerialHelper.h)
