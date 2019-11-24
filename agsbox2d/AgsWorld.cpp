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
#include "AgsJoint.h"
#include "Scale.h"
#include "Book.h"
#include <vector>

AgsWorld::AgsWorld(float32 gravityX, float32 gravityY) {
	B2AgsWorld = new b2World(Scale::ScaleDown(b2Vec2(gravityX, gravityY)));
    b2BodyDef def;
    B2GroundBody = B2AgsWorld->CreateBody(&def);
}


AgsBody* AgsWorld::NewBody(float32 x, float32 y, b2BodyType bodytype) {
	AgsBody* body = new AgsBody(this, x, y, bodytype);
	//AgsBodyList.push_back(body);
	return body;
}

b2Body* AgsWorld::GetGroundB2Body(){
    return B2GroundBody;
}

void AgsWorld::DestroyBody(AgsBody* body) {
	if (body == nullptr || body->GetIsDestroyed()) {
		return;
	}

    Book::UnregisterBodyFromWorldByID(Book::b2BodyToID(this->ID,body->GetB2AgsBody()),this->ID);
	B2AgsWorld->DestroyBody(body->GetB2AgsBody());

	body->SetIsDestroyed();
}

void AgsWorld::DestroyJoint(AgsJoint* joint) {
    if (joint == nullptr || !joint->isValid()) {
        return;
    }

    Book::UnregisterJointFromWorldByID(Book::b2JointToID(this->ID,joint->GetB2AgsJoint()),this->ID);
    B2AgsWorld->DestroyJoint(joint->GetB2AgsJoint());
}

void AgsWorld::Step(float32 dt, int32 velocityIterations, int32 positionIterations) {
	B2AgsWorld->Step(dt, velocityIterations, positionIterations);
	//printf("step of world id %d of dt %f and v %d and p %d\n", ID, dt, velocityIterations, positionIterations );
}


// -- functions for AABB query
std::vector<b2Fixture* > _fixtureQueryList;

class QueryFixturesCallback : public b2QueryCallback
{
public:
    bool ReportFixture(b2Fixture* fixture)
    {
        _fixtureQueryList.push_back(fixture);
        return true;
    }
};

int32 AgsWorld::BoundingBoxQuery(float32 lx, float32 ly, float32 ux, float32 uy) {
    b2AABB box;
    box.lowerBound = Scale::ScaleDown(b2Vec2(lx, ly));
    box.upperBound = Scale::ScaleDown(b2Vec2(ux, uy));
    _fixtureQueryList.clear();
    QueryFixturesCallback query;
    B2AgsWorld->QueryAABB(&query, box);
    return _fixtureQueryList.size();
}

int32 AgsWorld::BoundingBoxQueryFixtureCount() {
    return _fixtureQueryList.size();
}

b2Fixture* AgsWorld::BoundingBoxQueryFixture(int32 i) {
    if(i >= _fixtureQueryList.size() || i<0)
        return nullptr;

    return _fixtureQueryList[i];
}

void AgsWorld::BoundingBoxQueryReset() {
    _fixtureQueryList.clear();
}

// -- end of functions for AABB query

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

	// Serialize the b2bodies
    int32 bodycount = Book::GetBodiesCount(world->ID);
	ptr = IntToChar(bodycount, ptr, end);

	if (bodycount > 0) {
		for (std::unordered_map<int32, b2Body* >::iterator itr = Book::GetBodiesBegin(world->ID);
			itr != Book::GetBodiesEnd(world->ID); ++itr) {
            ptr = IntToChar(itr->first, ptr, end);
			ptr = b2BodyToChar(itr->second, ptr, end);
		}
	}

    // Serialize the b2fixtures
	int32 fixturecount = Book::GetFixtureCount(world->ID);
    ptr = IntToChar(fixturecount, ptr, end);

    if (fixturecount > 0) {
        for (std::unordered_map<int32, b2Fixture* >::iterator itr = Book::GetFixtureBegin(world->ID);
             itr != Book::GetFixtureEnd(world->ID); ++itr) {
            ptr = IntToChar(itr->first, ptr, end);
            int b2body_id = Book::b2BodyToID(world->ID, itr->second->GetBody());
            ptr = IntToChar(b2body_id, ptr, end);
        }
    }

    // Serialize the b2joints
    int32 jointcount = Book::GetJointCount(world->ID);
    ptr = IntToChar(jointcount, ptr, end);

    if (jointcount > 0) {
        for (std::unordered_map<int32, b2Joint* >::iterator itr = Book::GetJointBegin(world->ID);
             itr != Book::GetJointEnd(world->ID); ++itr) {
            int joint_id = itr->first;
            b2Joint* b2joint =  itr->second;
            int body_a_id = Book::b2BodyToID(world->ID, b2joint->GetBodyA());
            int body_b_id = Book::b2BodyToID(world->ID, b2joint->GetBodyB());
            ptr = IntToChar(joint_id, ptr, end);
            ptr = IntToChar(body_a_id, ptr, end);
            ptr = IntToChar(body_b_id, ptr, end);
            ptr = b2JointToChar(b2joint, ptr, end);
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
		world->ID = world_id;
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

    int32 fixturecount;
    ptr = CharToInt(fixturecount, ptr);

    int32 temp_prev_body_id=-1;
    b2Fixture* temp_f;
    if (fixturecount > 0) {
        for (int i = 0; i < fixturecount; i++) {
            int32 fixture_id;
            int32 body_id;
            ptr = CharToInt(fixture_id, ptr);
            ptr = CharToInt(body_id, ptr);
            b2Body* b2body = Book::IDtoB2Body( world->ID, body_id);
            if(temp_prev_body_id != body_id)
                temp_f = b2body->GetFixtureList();

            Book::RegisterFixtureFromWorld(temp_f, fixture_id, world->ID);

            temp_prev_body_id = body_id;
            temp_f->GetNext();
        }
    }

    int32 jointcount;
    ptr = CharToInt(jointcount, ptr);
    if (jointcount > 0) {
        for (int i = 0; i < jointcount; i++) {
            int joint_id;
            int body_a_id;
            int body_b_id;
            b2JointDef* b2jointdef;
            ptr = CharToInt(joint_id, ptr);
            ptr = CharToInt(body_a_id, ptr);
            ptr = CharToInt(body_b_id, ptr);
            ptr = CharTob2JointDef(b2jointdef, ptr);

            b2jointdef->bodyA = Book::IDtoB2Body(world->ID, body_a_id);
            b2jointdef->bodyB = Book::IDtoB2Body(world->ID, body_b_id);

            if(b2jointdef->type == b2JointType::e_mouseJoint) {
                b2jointdef->bodyA = world->GetGroundB2Body();
                b2jointdef->bodyB = Book::IDtoB2Body(world->ID, body_a_id);
            }

            b2Joint* b2joint = world->B2AgsWorld->CreateJoint(b2jointdef);

            Book::RegisterJointFromWorld(b2joint, joint_id, world->ID);
        }
    }

	engine->RegisterUnserializedObject(key, world, &AgsWorld_Interface);
}
//..............................................................................
