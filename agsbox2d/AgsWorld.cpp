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
#include "AgsFixtureArray.h"
#include "AgsRaycastResult.h"
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
    joint->EraseB2AgsJoint();
}

void AgsWorld::Step(float32 dt, int32 velocityIterations, int32 positionIterations) {
	B2AgsWorld->Step(dt, velocityIterations, positionIterations);
	//printf("step of world id %d of dt %f and v %d and p %d\n", ID, dt, velocityIterations, positionIterations );
}

AgsFixtureArray* AgsWorld::BoundingBoxQuery(float32 lx, float32 ly, float32 ux, float32 uy) {

    class QueryFixturesCallback : public b2QueryCallback
    {
        int32 WorldID;
        b2World* B2AGSWorld;
    public:
        QueryFixturesCallback(int32 world_id, b2World* b2world)
        {
            WorldID = world_id;
            B2AGSWorld = b2world;
        }
        AgsFixtureArray* FixtureQueryList = new AgsFixtureArray();
        bool ReportFixture(b2Fixture* fixture)
        {
            AgsFixtureArrayData fad;
            fad.WorldID = WorldID;
            fad.B2World = B2AGSWorld;
            fad.B2Fixture = fixture;
            FixtureQueryList->push_fad(fad);
            return true;
        }
    };

    b2AABB box;
    box.lowerBound = Scale::ScaleDown(b2Vec2(lx, ly));
    box.upperBound = Scale::ScaleDown(b2Vec2(ux, uy));
    QueryFixturesCallback query(ID, B2AgsWorld);
    B2AgsWorld->QueryAABB(&query, box);
    return query.FixtureQueryList;
}


// Raycast x0 y0 x1 y1 (StopAtFirstFixture, StopAtSpecificFixture, StopOnFixtureFromArray, Passthrough)
AgsRaycastResult* AgsWorld::RaycastQuery(float32 x0, float32 y0, float32 x1, float32 y1, RaycastType raycastType, AgsFixtureArray* agsFixtureArray) {
    class AgsRayCastCallback : public b2RayCastCallback
    {
        int32 WorldID;
        b2World* B2AGSWorld;
        AgsFixtureArray* TargetFixtures;
    public:
        AgsRayCastCallback(int32 world_id, b2World* b2world, RaycastType raycastType)
        {
            WorldID = world_id;
            B2AGSWorld = b2world;
            m_raycast_type = raycastType;
        }
        AgsRaycastResult* RaycastResult = new AgsRaycastResult();

        float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                              const b2Vec2& normal, float32 fraction)
        {

            RaycastResultItem rritem;

            rritem.FixtureData.WorldID = WorldID;
            rritem.FixtureData.B2FixtureID = Book::b2FixtureToID(WorldID, fixture);
            rritem.PointX = point.x;
            rritem.PointY = point.y;
            rritem.NormalX = normal.x;
            rritem.NormalY = normal.y;
            rritem.Fraction = fraction;

            RaycastResult->RaycastResultList.push_back(rritem);

            if(m_raycast_type == eRaycastUntilHit) {
                if(TargetFixtures == nullptr || TargetFixtures->size() == 0) return fraction; // stop at first hit
                else {
                    std::vector<AgsFixtureArrayData>::iterator it;
                    for (it = TargetFixtures->data.begin(); it != TargetFixtures->data.end(); ++it) {
                        AgsFixtureArrayData fad = (*it);
                        if(fixture == fad.B2Fixture) return fraction; // stop, we hit a target fixture
                    }
                    return  1.0; // we didn't hit anything, so we go on.
                }
            } else  {
                return  1.0; // passthrough
            }
        }

        RaycastType m_raycast_type;
    };


    b2Vec2 v1 = Scale::ScaleDown(b2Vec2(x0, y0));
    b2Vec2 v2 = Scale::ScaleDown(b2Vec2(x1, y1));

    AgsRayCastCallback query(ID, B2AgsWorld, raycastType);
    B2AgsWorld->RayCast(&query, v1, v2);
    return  query.RaycastResult;
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
    b2Fixture* temp_f = nullptr;
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
            b2JointDef* b2jointdef = nullptr;
            ptr = CharToInt(joint_id, ptr);
            ptr = CharToInt(body_a_id, ptr);
            ptr = CharToInt(body_b_id, ptr);
            ptr = CharTob2JointDef(&b2jointdef, ptr);

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
