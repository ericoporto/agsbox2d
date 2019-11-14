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
// -- End of AgsFixture Bookkeeping --
