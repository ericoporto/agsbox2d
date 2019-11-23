/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef AGSBOX2D_BOOK_H
#define AGSBOX2D_BOOK_H

#include "Box2D.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>

class AgsWorld; // forward declaration, we need AgsWorld.h on the cpp
class AgsBody; // forward declaration, we need AgsBody.h on the cpp
class AgsFixture; // forward declaration, we need AgsFixture.h on the cpp
class AgsShape; // forward declaration, we need AgsShape.h on the cpp
class AgsJoint; // forward declaration, we need AgsShape.h on the cpp
class AgsJointDistance; // forward declaration, we need AgsShape.h on the cpp
class AgsJointMotor; // forward declaration, we need AgsShape.h on the cpp
class AgsJointMouse; // forward declaration, we need AgsShape.h on the cpp
class AgsJointPulley; // forward declaration, we need AgsShape.h on the cpp

class Book
{
private:
	Book();
	~Book(void);
	std::vector<b2World* > ListB2World;
    std::unordered_map<int32, std::unordered_map<int32, b2Body* >*> B2BodiesByID;
    std::unordered_map<int32, std::unordered_map<b2Body*, int32 >*> B2BodiesByPointer;

    std::unordered_map<int32, std::unordered_map<int32, b2Joint* >*> B2JointByID;
    std::unordered_map<int32, std::unordered_map<b2Joint*, int32 >*> B2JointByPointer;

    std::unordered_map<int32, std::unordered_map<int32, b2Fixture* >*> B2FixtureByID;
    std::unordered_map<int32, std::unordered_map<b2Fixture*, int32 >*> B2FixtureByPointer;

	std::unordered_map<int32, AgsWorld*> MapAgsWorld;
	std::unordered_map<int32, AgsBody*> MapAgsBody;
	std::unordered_map<int32, AgsFixture*> MapAgsFixture;
	std::unordered_map<int32, AgsShape*> MapAgsShape;

    std::unordered_map<int32, AgsJoint*> MapAgsJoint;
    std::unordered_map<int32, AgsJointDistance*> MapAgsJointDistance;
    std::unordered_map<int32, AgsJointMotor*> MapAgsJointMotor;
    std::unordered_map<int32, AgsJointMouse*> MapAgsJointMouse;
    std::unordered_map<int32, AgsJointPulley*> MapAgsJointPulley;

	uint32 BodyIDCount;
    uint32 JointIDCount;
    uint32 FixtureIDCount;

	static void DisposeWorldIfNeeded();

public:
    static int32 GetNewBodyID(int32 world_id);
	static bool RegisterBodyFromWorld(b2Body* body, int32 body_id, int32 world_id);
    static bool UnregisterBodyFromWorldByID(int32 body_id, int32 world_id);
    static int32 b2BodyToID(int32 world_id, b2Body* body);
    static b2Body* IDtoB2Body(int32 world_id, int32 body_id);
    static std::unordered_map<int32, b2Body* >::iterator GetBodiesBegin(int32 world_id);
	static std::unordered_map<int32, b2Body* >::iterator GetBodiesEnd(int32 world_id);
	static int32 GetBodiesCount(int32 world_id);
	static bool GetBodiesEmpty(int32 world_id);

    static int32 GetNewJointID(int32 world_id);
    static bool RegisterJointFromWorld(b2Joint* joint, int32 joint_id, int32 world_id);
    static bool UnregisterJointFromWorldByID(int32 joint_id, int32 world_id);
    static int32 b2JointToID(int32 world_id, b2Joint* joint);
    static b2Joint* IDtoB2Joint(int32 world_id, int32 joint_id);
    static std::unordered_map<int32, b2Joint* >::iterator GetJointBegin(int32 world_id);
    static std::unordered_map<int32, b2Joint* >::iterator GetJointEnd(int32 world_id);
    static int32 GetJointCount(int32 world_id);

    static int32 GetNewFixtureID(int32 world_id);
    static bool RegisterFixtureFromWorld(b2Fixture* fixture, int32 fixture_id, int32 world_id);
    static bool UnregisterFixtureFromWorldByID(int32 fixture_id, int32 world_id);
    static int32 b2FixtureToID(int32 world_id, b2Fixture* fixture);
    static b2Fixture* IDtoB2Fixture(int32 world_id, int32 fixture_id);
    static std::unordered_map<int32, b2Fixture* >::iterator GetFixtureBegin(int32 world_id);
    static std::unordered_map<int32, b2Fixture* >::iterator GetFixtureEnd(int32 world_id);
    static int32 GetFixtureCount(int32 world_id);

	static Book* i();

	static bool isAgsWorldRegisteredByID(int32 id);
	static bool RegisterAgsWorld(int32 id, AgsWorld* world);
	static bool UnregisterAgsWorldByID(int32 id);
	static AgsWorld* IDtoAgsWorld(int32 id);

	static bool isAgsBodyRegisteredByID(int32 id);
	static bool RegisterAgsBody(int32 id, AgsBody* body);
	static bool UnregisterAgsBodyByID(int32 id);
	static AgsBody* IDtoAgsBody(int32 id);
	static AgsBody* b2bodyIDtoAgsBody(int32 body_id, int32 world_id);

	static bool isAgsShapeRegisteredByID(int32 id);
	static bool RegisterAgsShape(int32 id, AgsShape* shape);
	static bool UnregisterAgsShapeByID(int32 id);
	static AgsShape* IDtoAgsShape(int32 id);

	static bool isAgsFixtureRegisteredByID(int32 id);
	static bool RegisterAgsFixture(int32 id, AgsFixture* fixture);
	static bool UnregisterAgsFixtureByID(int32 id);
	static AgsFixture* IDtoAgsFixture(int32 id);
    static AgsFixture* b2FixtureIDtoAgsFixture(int32 fixture_id, int32 world_id);

    static bool isAgsJointRegisteredByID(int32 id);
    static bool RegisterAgsJoint(int32 id, AgsJoint* joint);
    static bool UnregisterAgsJointByID(int32 id);
    static AgsJoint* IDtoAgsJoint(int32 id);

    static bool isAgsJointDistanceRegisteredByID(int32 id);
    static bool RegisterAgsJointDistance(int32 id, AgsJointDistance* joint);
    static bool UnregisterAgsJointDistanceByID(int32 id);
    static AgsJointDistance* IDtoAgsJointDistance(int32 id);

    static bool isAgsJointMotorRegisteredByID(int32 id);
    static bool RegisterAgsJointMotor(int32 id, AgsJointMotor* joint);
    static bool UnregisterAgsJointMotorByID(int32 id);
    static AgsJointMotor* IDtoAgsJointMotor(int32 id);

    static bool isAgsJointMouseRegisteredByID(int32 id);
    static bool RegisterAgsJointMouse(int32 id, AgsJointMouse* joint);
    static bool UnregisterAgsJointMouseByID(int32 id);
    static AgsJointMouse* IDtoAgsJointMouse(int32 id);

    static bool isAgsJointPulleyRegisteredByID(int32 id);
    static bool RegisterAgsJointPulley(int32 id, AgsJointPulley* joint);
    static bool UnregisterAgsJointPulleyByID(int32 id);
    static AgsJointPulley* IDtoAgsJointPulley(int32 id);
};


#endif /* AGSBOX2D_BOOK_H */
