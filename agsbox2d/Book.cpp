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
		if(!(i()->ListB2Bodies.empty())){
		
			std::unordered_map<int32, std::unordered_set<b2Body* >*>::iterator it;
		
		 	for (it=i()->ListB2Bodies.begin(); it != i()->ListB2Bodies.end(); ++it){
		 		if(it->second != nullptr){
		 			delete it->second;
		 		}
		 	}
		//
		 	i()->ListB2Bodies.clear();
		}
	}
}

// -- PUBLIC --

void Book::NoteBodyAndWorld(b2Body* body, int32 world_id) {
	if (i()->ListB2Bodies.count(world_id) == 0) {
		i()->ListB2Bodies[world_id] = new std::unordered_set<b2Body* >;
	}

	if(body != nullptr && i()->ListB2Bodies[world_id] != nullptr) {
        i()->ListB2Bodies[world_id]->insert(body);
    }
}

std::unordered_set<b2Body* >::iterator  Book::GetBodiesSetBegin(int32 world_id){
	return i()->ListB2Bodies[world_id]->begin();
}

std::unordered_set<b2Body* >::iterator  Book::GetBodiesSetEnd(int32 world_id){
	return i()->ListB2Bodies[world_id]->end();
}

int32 Book::GetBodiesCount(int32 world_id){
	if(i()->ListB2Bodies[world_id]==nullptr) return 0;

  return i()->ListB2Bodies[world_id]->size();
}

bool Book::GetBodiesEmpty(int32 world_id){
	if(i()->ListB2Bodies[world_id]==nullptr) return true;

  return i()->ListB2Bodies[world_id]->empty();
}

Book* Book::i()
{
	static Book instance;

	return &instance;
}

// -- AgsWorld Bookkeeping --
bool Book::isAgsWorldRegisteredByID(int32 id) {
	if(i()->MapAgsWorld.count(id) == 0) {
		return false;
	}
	return true;
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
	if(i()->MapAgsBody.count(id) == 0) {
		return false;
	}
	return true;
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

	if(!(i()->MapAgsBody[id]->GetIsDestroyed())) {
		NoteBodyAndWorld( i()->MapAgsBody[id]->B2AgsBody,
		 				  i()->MapAgsBody[id]->World->ID );
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
	if (i()->MapAgsShape.count(id) == 0) {
		return false;
	}
	return true;
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
	if(i()->MapAgsFixture.count(id) == 0) {
		return false;
	}
	return true;
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
