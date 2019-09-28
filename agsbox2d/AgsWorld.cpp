/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsWorld.h"
#include "AgsBody.h"
#include "Scale.h"
#include "Book.h"

AgsWorld::AgsWorld(float32 gravityX, float32 gravityY) {
	B2AgsWorld = new b2World(Scale::ScaleDown(b2Vec2(gravityX, gravityY)));
}


AgsBody* AgsWorld::NewBody(float32 x, float32 y, b2BodyType bodytype) {
	AgsBody* body = new AgsBody(this, x, y, bodytype);
	//AgsBodyList.push_back(body);
	return body;
}

void AgsWorld::DestroyBody(AgsBody* body) {
	if (body == nullptr && body->GetIsDestroyed()) {
		return;
	}

	B2AgsWorld->DestroyBody(body->B2AgsBody);
	body->SetIsDestroyed();
}

void AgsWorld::Step(float32 dt, int32 velocityIterations, int32 positionIterations) {
	B2AgsWorld->Step(dt, velocityIterations, positionIterations);
	//printf("step of world id %d of dt %f and v %d and p %d\n", ID, dt, velocityIterations, positionIterations );
}

AgsWorld::~AgsWorld(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsWorldInterface AgsWorld_Interface;
AgsWorldReader AgsWorld_Reader;

const char* AgsWorldInterface::name = "World";

//------------------------------------------------------------------------------

#include "SerialHelper.h"
using namespace SerialHelper;

int AgsWorldInterface::Dispose(const char* address, bool force)
{
	Book::UnregisterAgsWorldByID(((AgsWorld*)address)->ID);

	//still need to figure an strategy for when to dispose of the Box2D World
	//printf("dispose of world %d\n", ((AgsWorld*)address)->ID);
	delete ((AgsWorld*)address);
	return (1);
}

//------------------------------------------------------------------------------

int AgsWorldInterface::Serialize(const char* address, char* buffer, int bufsize)
{
	AgsWorld* world = (AgsWorld*)address;
	char* ptr = buffer;
	char* end = buffer + bufsize;

	ptr = FloatToChar(Scale::GetMeter(), ptr, end);

	ptr = b2Vec2ToChar(world->B2AgsWorld->GetGravity(), ptr, end);

    int32 bodycount = Book::GetBodiesCount(world->ID);
	ptr = IntToChar(bodycount, ptr, end);

	if (bodycount > 0) {
		for (std::unordered_map<int32, b2Body* >::iterator itr = Book::GetBodiesBegin(world->ID);
			itr != Book::GetBodiesEnd(world->ID); ++itr) {
            ptr = IntToChar(itr->first, ptr, end);
			ptr = b2BodyToChar(itr->second, ptr, end);
		}
	}

	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsWorldReader::Unserialize(int key, const char* serializedData, int dataSize)
{
	AgsWorld* world;
	char* ptr = (char*)serializedData;
	int world_id = key;
	float32 meter;
	ptr = CharToFloat(meter, ptr);
	Scale::SetMeter(meter);

	b2Vec2 gravity;
	ptr = CharTob2Vec2(gravity, ptr);

	if (Book::isAgsWorldRegisteredByID(world_id)) {
		world = Book::IDtoAgsWorld(world_id);
		//printf("   --->>  got world already existed %d\n", world_id);
	}
	else {
		//printf("   --->>  created world %d\n", world_id);
		world = new AgsWorld(0, 0);
		world-> ID = world_id;
		Book::RegisterAgsWorld(world_id, world);
	}

	world->B2AgsWorld->SetGravity(gravity);

    int32 bodycount;
	ptr = CharToInt(bodycount, ptr);

	
	if (bodycount > 0) {
		for (int i = 0; i < bodycount; i++) {
			b2Body * body;
			b2BodyDef bodydef;
			int32 body_id;
            ptr = CharToInt(body_id, ptr);

			ptr = CharTob2Body(bodydef, &body, world->B2AgsWorld, ptr);
			Book::RegisterBodyFromWorld(body, body_id, world->ID);
		}
	}

	engine->RegisterUnserializedObject(key, world, &AgsWorld_Interface);
}
//..............................................................................
