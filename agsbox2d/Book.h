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

class Book
{
private:
	Book();
	~Book(void);
	std::vector<b2World* > ListB2World;
    std::unordered_map<int32, std::unordered_map<int32, b2Body* >*> B2BodiesByID;
    std::unordered_map<int32, std::unordered_map<b2Body*, int32 >*> B2BodiesByPointer;

	std::unordered_map<int32, AgsWorld*> MapAgsWorld;
	std::unordered_map<int32, AgsBody*> MapAgsBody;
	std::unordered_map<int32, AgsFixture*> MapAgsFixture;
	std::unordered_map<int32, AgsShape*> MapAgsShape;
    std::unordered_map<int32, AgsJoint*> MapAgsJoint;

	uint32 BodyIDCount;

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

    static bool isAgsJointRegisteredByID(int32 id);
    static bool RegisterAgsJoint(int32 id, AgsJoint* joint);
    static bool UnregisterAgsJointByID(int32 id);
    static AgsJoint* IDtoAgsJoint(int32 id);
};


#endif /* AGSBOX2D_BOOK_H */
