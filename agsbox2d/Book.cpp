/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "Book.h"

#include "AgsWorld.h"
#include "AgsBody.h"
#include "AgsShape.h"
#include "AgsFixture.h"
#include "AgsJoint.h"

// -- PRIVATE --

Book::Book()
{
    BodyIDCount = 0;
}

Book::~Book()
{
}


void Book::DisposeWorldIfNeeded(){
	if (i()->MapAgsWorld.empty() &&
			i()->MapAgsBody.empty() ) {

		while (!i()->ListB2World.empty()) {
			b2World* world = *i()->ListB2World.rbegin();

			delete world;

			i()->ListB2World.pop_back();
		}
		//
		if(!(i()->B2BodiesByID.empty())){
		
			std::unordered_map<int32, std::unordered_map<int32, b2Body* >*>::iterator it;
		
		 	for (it=i()->B2BodiesByID.begin(); it != i()->B2BodiesByID.end(); ++it){
		 		if(it->second != nullptr){
		 			delete it->second;
		 		}
		 	}
		//
		 	i()->B2BodiesByID.clear();
		}

        if(!(i()->B2BodiesByPointer.empty())){
            i()->B2BodiesByID.clear();
        }
	}
}

// -- PUBLIC --


int32 Book::b2BodyToID(int32 world_id, b2Body* body){
    if (i()->B2BodiesByPointer[world_id]->count(body) == 0) {
        return -1;
    }

    return (*i()->B2BodiesByPointer[world_id])[body];
}

b2Body* Book::IDtoB2Body(int32 world_id, int32 body_id){
    if (i()->B2BodiesByID[world_id]->count(body_id) == 0) {
        return nullptr;
    }

    return (*i()->B2BodiesByID[world_id])[body_id];
}

int32 Book::GetNewBodyID(int32 world_id){
    if (i()->B2BodiesByID.count(world_id) == 0) {
        i()->B2BodiesByID[world_id] = new std::unordered_map<int32, b2Body* >;
        i()->B2BodiesByPointer[world_id] = new std::unordered_map<b2Body*, int32 >;
    }

    if(i()->B2BodiesByID[world_id] != nullptr) {
        while( (*i()->B2BodiesByID[world_id]).count(i()->BodyIDCount) != 0){
            i()->BodyIDCount++;
        }
    }

    return i()->BodyIDCount;
}

bool Book::RegisterBodyFromWorld(b2Body* body,  int32 body_id, int32 world_id) {
	if (i()->B2BodiesByID.count(world_id) == 0) {
		i()->B2BodiesByID[world_id] = new std::unordered_map<int32, b2Body* >;
        i()->B2BodiesByPointer[world_id] = new std::unordered_map<b2Body*, int32 >;
	}

	if(body != nullptr && i()->B2BodiesByID[world_id] != nullptr) {
        if (i()->B2BodiesByPointer[world_id]->count(body) == 0 &&
            i()->B2BodiesByID[world_id]->count(body_id) == 0) {

            (*i()->B2BodiesByID[world_id])[body_id] = body;
            (*i()->B2BodiesByPointer[world_id])[body] = body_id;

            return true;
        }
    }
    return false;
}

bool Book::UnregisterBodyFromWorldByID(int32 body_id, int32 world_id) {
    if (i()->B2BodiesByID[world_id]->count(body_id) == 0) {
        return false;
    }

    i()->B2BodiesByPointer[world_id]->erase((*i()->B2BodiesByID[world_id])[body_id]);
    i()->B2BodiesByID[world_id]->erase(body_id);
    return true;
}

std::unordered_map<int32, b2Body* >::iterator  Book::GetBodiesBegin(int32 world_id){
	return i()->B2BodiesByID[world_id]->begin();
}

std::unordered_map<int32, b2Body* >::iterator  Book::GetBodiesEnd(int32 world_id){
	return i()->B2BodiesByID[world_id]->end();
}

int32 Book::GetBodiesCount(int32 world_id){
	if(i()->B2BodiesByID[world_id]==nullptr) return 0;

  return i()->B2BodiesByID[world_id]->size();
}

bool Book::GetBodiesEmpty(int32 world_id){
	if(i()->B2BodiesByID[world_id]==nullptr) return true;

  return i()->B2BodiesByID[world_id]->empty();
}

// Joint related functions

int32 Book::b2JointToID(int32 world_id, b2Joint* joint){
    if (i()->B2JointByPointer[world_id]->count(joint) == 0) {
        return -1;
    }

    return (*i()->B2JointByPointer[world_id])[joint];
}

b2Joint* Book::IDtoB2Joint(int32 world_id, int32 joint_id){
    if (i()->B2JointByID[world_id]->count(joint_id) == 0) {
        return nullptr;
    }

    return (*i()->B2JointByID[world_id])[joint_id];
}

int32 Book::GetNewJointID(int32 world_id){
    if (i()->B2JointByID.count(world_id) == 0) {
        i()->B2JointByID[world_id] = new std::unordered_map<int32, b2Joint* >;
        i()->B2JointByPointer[world_id] = new std::unordered_map<b2Joint*, int32 >;
    }

    if(i()->B2JointByID[world_id] != nullptr) {
        while( (*i()->B2JointByID[world_id]).count(i()->JointIDCount) != 0){
            i()->JointIDCount++;
        }
    }

    return i()->JointIDCount;
}

bool Book::RegisterJointFromWorld(b2Joint* joint, int32 joint_id, int32 world_id) {
    if (i()->B2JointByID.count(world_id) == 0) {
        i()->B2JointByID[world_id] = new std::unordered_map<int32, b2Joint* >;
        i()->B2JointByPointer[world_id] = new std::unordered_map<b2Joint*, int32 >;
    }

    if(joint != nullptr && i()->B2BodiesByID[world_id] != nullptr) {
        if (i()->B2JointByPointer[world_id]->count(joint) == 0 &&
            i()->B2JointByID[world_id]->count(joint_id) == 0) {

            (*i()->B2JointByID[world_id])[joint_id] = joint;
            (*i()->B2JointByPointer[world_id])[joint] = joint_id;

            return true;
        }
    }
    return false;
}

bool Book::UnregisterJointFromWorldByID(int32 joint_id, int32 world_id) {
    if (i()->B2JointByID[world_id]->count(joint_id) == 0) {
        return false;
    }

    i()->B2JointByPointer[world_id]->erase((*i()->B2JointByID[world_id])[joint_id]);
    i()->B2JointByID[world_id]->erase(joint_id);
    return true;
}

std::unordered_map<int32, b2Joint* >::iterator  Book::GetJointBegin(int32 world_id){
    return i()->B2JointByID[world_id]->begin();
}

std::unordered_map<int32, b2Joint* >::iterator  Book::GetJointEnd(int32 world_id){
    return i()->B2JointByID[world_id]->end();
}

int32 Book::GetJointCount(int32 world_id){
    if(i()->B2JointByID[world_id]==nullptr) return 0;

    return i()->B2JointByID[world_id]->size();
}

// -- Fixture related functions


int32 Book::b2FixtureToID(int32 world_id, b2Fixture* fixture){
    if (i()->B2FixtureByPointer[world_id]->count(fixture) == 0) {
        return -1;
    }

    return (*i()->B2FixtureByPointer[world_id])[fixture];
}

b2Fixture* Book::IDtoB2Fixture(int32 world_id, int32 fixture_id){
    if (i()->B2FixtureByID[world_id]->count(fixture_id) == 0) {
        return nullptr;
    }

    return (*i()->B2FixtureByID[world_id])[fixture_id];
}

int32 Book::GetNewFixtureID(int32 world_id){
    if (i()->B2FixtureByID.count(world_id) == 0) {
        i()->B2FixtureByID[world_id] = new std::unordered_map<int32, b2Fixture* >;
        i()->B2FixtureByPointer[world_id] = new std::unordered_map<b2Fixture*, int32 >;
    }

    if(i()->B2FixtureByID[world_id] != nullptr) {
        while( (*i()->B2FixtureByID[world_id]).count(i()->FixtureIDCount) != 0){
            i()->FixtureIDCount++;
        }
    }

    return i()->FixtureIDCount;
}

bool Book::RegisterFixtureFromWorld(b2Fixture* fixture, int32 fixture_id, int32 world_id) {
    if (i()->B2FixtureByID.count(world_id) == 0) {
        i()->B2FixtureByID[world_id] = new std::unordered_map<int32, b2Fixture* >;
        i()->B2FixtureByPointer[world_id] = new std::unordered_map<b2Fixture*, int32 >;
    }

    if(fixture != nullptr && i()->B2BodiesByID[world_id] != nullptr) {
        if (i()->B2FixtureByPointer[world_id]->count(fixture) == 0 &&
            i()->B2FixtureByID[world_id]->count(fixture_id) == 0) {

            (*i()->B2FixtureByID[world_id])[fixture_id] = fixture;
            (*i()->B2FixtureByPointer[world_id])[fixture] = fixture_id;

            return true;
        }
    }
    return false;
}

bool Book::UnregisterFixtureFromWorldByID(int32 fixture_id, int32 world_id) {
    if (i()->B2FixtureByID[world_id]->count(fixture_id) == 0) {
        return false;
    }

    i()->B2FixtureByPointer[world_id]->erase((*i()->B2FixtureByID[world_id])[fixture_id]);
    i()->B2FixtureByID[world_id]->erase(fixture_id);
    return true;
}

std::unordered_map<int32, b2Fixture* >::iterator  Book::GetFixtureBegin(int32 world_id){
    return i()->B2FixtureByID[world_id]->begin();
}

std::unordered_map<int32, b2Fixture* >::iterator  Book::GetFixtureEnd(int32 world_id){
    return i()->B2FixtureByID[world_id]->end();
}

int32 Book::GetFixtureCount(int32 world_id){
    if(i()->B2FixtureByID[world_id]==nullptr) return 0;

    return i()->B2FixtureByID[world_id]->size();
}

// -- End of Fixture related functions

Book* Book::i()
{
	static Book instance;

	return &instance;
}

// -- AgsWorld Bookkeeping --
bool Book::isAgsWorldRegisteredByID(int32 id) {
    return i()->MapAgsWorld.count(id) != 0;
}

bool Book::RegisterAgsWorld(int32 id, AgsWorld* world) {
	if (i()->MapAgsWorld.count(id) == 0) {
		i()->MapAgsWorld[id] = world;
		i()->ListB2World.push_back(world->B2AgsWorld);
		return true;
	}
	return false;
}

bool Book::UnregisterAgsWorldByID(int32 id) {
	if (i()->MapAgsWorld.count(id) == 0) {
		return false;
	}
	i()->MapAgsWorld.erase(id);
	DisposeWorldIfNeeded();
	return true;
}

AgsWorld* Book::IDtoAgsWorld(int32 id) {
	if (i()->MapAgsWorld.count(id) == 0) {
		return nullptr;
	}
	return i()->MapAgsWorld[id];
}
// -- End of AgsWorld Bookkeeping --

// -- AgsBody Bookkeeping --
bool Book::isAgsBodyRegisteredByID(int32 id) {
    return i()->MapAgsBody.count(id) != 0;
}

bool Book::RegisterAgsBody(int32 id, AgsBody* body) {
	if (i()->MapAgsBody.count(id) == 0) {
		i()->MapAgsBody[id] = body;
		return true;
	}
	return false;
}

bool Book::UnregisterAgsBodyByID(int32 id) {
	if (i()->MapAgsBody.count(id) == 0) {
		return false;
	}

	i()->MapAgsBody.erase(id);
	DisposeWorldIfNeeded();
	return true;
}

AgsBody* Book::IDtoAgsBody(int32 id) {
	if (i()->MapAgsBody.count(id) == 0) {
		return nullptr;
	}
	return i()->MapAgsBody[id];
}

AgsBody* Book::b2bodyIDtoAgsBody(int32 body_id, int32 world_id){
    b2Body* b2body = Book::IDtoB2Body(world_id, body_id);

    if (i()->MapAgsBody.empty())
        return nullptr;

    std::unordered_map<int32, AgsBody*>::iterator it;

    for (it=i()->MapAgsBody.begin(); it != i()->MapAgsBody.end(); ++it){
        if(it->second->GetB2AgsBody() == b2body){
            return it->second;
        }
    }
    return nullptr;
}


// -- End of AgsBody Bookkeeping --

// -- AgsShape Bookkeeping --
bool Book::isAgsShapeRegisteredByID(int32 id) {
    return i()->MapAgsShape.count(id) != 0;
}

bool Book::RegisterAgsShape(int32 id, AgsShape* shape) {
	if (i()->MapAgsShape.count(id) == 0) {
		i()->MapAgsShape[id] = shape;
		return true;
	}
	return false;
}

bool Book::UnregisterAgsShapeByID(int32 id) {
	if (i()->MapAgsShape.count(id) == 0) {
		return false;
	}
	i()->MapAgsShape.erase(id);
	return true;
}

AgsShape* Book::IDtoAgsShape(int32 id) {
	if (i()->MapAgsShape.count(id) == 0) {
		return nullptr;
	}
	return i()->MapAgsShape[id];
}
// -- End of AgsShape Bookkeeping --

// -- AgsFixture Bookkeeping --
bool Book::isAgsFixtureRegisteredByID(int32 id) {
    return i()->MapAgsFixture.count(id) != 0;
}

bool Book::RegisterAgsFixture(int32 id, AgsFixture* fixture) {
	if (i()->MapAgsFixture.count(id) == 0) {
		i()->MapAgsFixture[id] = fixture;
		return true;
	}
	return false;
}

bool Book::UnregisterAgsFixtureByID(int32 id) {
	if (i()->MapAgsFixture.count(id) == 0) {
		return false;
	}
	i()->MapAgsFixture.erase(id);
	return true;
}

AgsFixture* Book::IDtoAgsFixture(int32 id) {
	if (i()->MapAgsFixture.count(id) == 0) {
		return nullptr;
	}
	return i()->MapAgsFixture[id];
}

AgsFixture* Book::b2FixtureIDtoAgsFixture(int32 fixture_id, int32 world_id){
    b2Fixture* b2fixture = Book::IDtoB2Fixture(world_id, fixture_id);

    if (i()->MapAgsFixture.empty())
        return nullptr;

    std::unordered_map<int32, AgsFixture*>::iterator it;

    for (it=i()->MapAgsFixture.begin(); it != i()->MapAgsFixture.end(); ++it){
        if(it->second->GetB2AgsFixture() == b2fixture){
            return it->second;
        }
    }
    return nullptr;
}

// -- End of AgsFixture Bookkeeping --

// -- AgsJoint Bookkeeping --
bool Book::isAgsJointRegisteredByID(int32 id) {
    return i()->MapAgsJoint.count(id) != 0;
}

bool Book::RegisterAgsJoint(int32 id, AgsJoint* joint) {
    if (i()->MapAgsJoint.count(id) == 0) {
        i()->MapAgsJoint[id] = joint;
        return true;
    }
    return false;
}

bool Book::UnregisterAgsJointByID(int32 id) {
    if (i()->MapAgsJoint.count(id) == 0) {
        return false;
    }
    i()->MapAgsJoint.erase(id);
    return true;
}

AgsJoint* Book::IDtoAgsJoint(int32 id) {
    if (i()->MapAgsJoint.count(id) == 0) {
        return nullptr;
    }
    return i()->MapAgsJoint[id];
}
// -- End of AgsJoint Bookkeeping --

// -- AgsJointDistance Bookkeeping --
bool Book::isAgsJointDistanceRegisteredByID(int32 id) {
    return i()->MapAgsJointDistance.count(id) != 0;
}

bool Book::RegisterAgsJointDistance(int32 id, AgsJointDistance* joint) {
    if (i()->MapAgsJointDistance.count(id) == 0) {
        i()->MapAgsJointDistance[id] = joint;
        return true;
    }
    return false;
}

bool Book::UnregisterAgsJointDistanceByID(int32 id) {
    if (i()->MapAgsJointDistance.count(id) == 0) {
        return false;
    }
    i()->MapAgsJointDistance.erase(id);
    return true;
}

AgsJointDistance* Book::IDtoAgsJointDistance(int32 id) {
    if (i()->MapAgsJointDistance.count(id) == 0) {
        return nullptr;
    }
    return i()->MapAgsJointDistance[id];
}
// -- End of AgsJointDistance Bookkeeping --

// -- AgsJointMotor Bookkeeping --
bool Book::isAgsJointMotorRegisteredByID(int32 id) {
    return i()->MapAgsJointMotor.count(id) != 0;
}

bool Book::RegisterAgsJointMotor(int32 id, AgsJointMotor* joint) {
    if (i()->MapAgsJointMotor.count(id) == 0) {
        i()->MapAgsJointMotor[id] = joint;
        return true;
    }
    return false;
}

bool Book::UnregisterAgsJointMotorByID(int32 id) {
    if (i()->MapAgsJointMotor.count(id) == 0) {
        return false;
    }
    i()->MapAgsJointMotor.erase(id);
    return true;
}

AgsJointMotor* Book::IDtoAgsJointMotor(int32 id) {
    if (i()->MapAgsJointMotor.count(id) == 0) {
        return nullptr;
    }
    return i()->MapAgsJointMotor[id];
}
// -- End of AgsJointMotor Bookkeeping --

// -- AgsJointMouse Bookkeeping --
bool Book::isAgsJointMouseRegisteredByID(int32 id) {
    return i()->MapAgsJointMouse.count(id) != 0;
}

bool Book::RegisterAgsJointMouse(int32 id, AgsJointMouse* joint) {
    if (i()->MapAgsJointMouse.count(id) == 0) {
        i()->MapAgsJointMouse[id] = joint;
        return true;
    }
    return false;
}

bool Book::UnregisterAgsJointMouseByID(int32 id) {
    if (i()->MapAgsJointMouse.count(id) == 0) {
        return false;
    }
    i()->MapAgsJointMouse.erase(id);
    return true;
}

AgsJointMouse* Book::IDtoAgsJointMouse(int32 id) {
    if (i()->MapAgsJointMouse.count(id) == 0) {
        return nullptr;
    }
    return i()->MapAgsJointMouse[id];
}
// -- End of AgsJointMouse Bookkeeping --

// -- AgsJointPulley Bookkeeping --
bool Book::isAgsJointPulleyRegisteredByID(int32 id) {
    return i()->MapAgsJointPulley.count(id) != 0;
}

bool Book::RegisterAgsJointPulley(int32 id, AgsJointPulley* joint) {
    if (i()->MapAgsJointPulley.count(id) == 0) {
        i()->MapAgsJointPulley[id] = joint;
        return true;
    }
    return false;
}

bool Book::UnregisterAgsJointPulleyByID(int32 id) {
    if (i()->MapAgsJointPulley.count(id) == 0) {
        return false;
    }
    i()->MapAgsJointPulley.erase(id);
    return true;
}

AgsJointPulley* Book::IDtoAgsJointPulley(int32 id) {
    if (i()->MapAgsJointPulley.count(id) == 0) {
        return nullptr;
    }
    return i()->MapAgsJointPulley[id];
}
// -- End of AgsJointPulley Bookkeeping --