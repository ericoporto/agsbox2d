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
		
			std::unordered_map<int, std::unordered_set<b2Body* >*>::iterator it;
		
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

void Book::NoteBodyAndWorld(b2Body* body, int world_id) {
	if (i()->ListB2Bodies.count(world_id) == 0) {
		i()->ListB2Bodies[world_id] = new std::unordered_set<b2Body* >;
	}

	i()->ListB2Bodies[world_id]->insert(body);
}

std::unordered_set<b2Body* >::iterator  Book::GetBodiesSetBegin(int world_id){
	return i()->ListB2Bodies[world_id]->begin();
}

std::unordered_set<b2Body* >::iterator  Book::GetBodiesSetEnd(int world_id){
	return i()->ListB2Bodies[world_id]->end();
}

int Book::GetBodiesCount(int world_id){
	if(i()->ListB2Bodies[world_id]==nullptr) return 0;

  return i()->ListB2Bodies[world_id]->size();
}

bool Book::GetBodiesEmpty(int world_id){
	if(i()->ListB2Bodies[world_id]==nullptr) return true;

  return i()->ListB2Bodies[world_id]->empty();
}

Book* Book::i()
{
	static Book instance;

	return &instance;
}

// -- AgsWorld Bookkeeping --
bool Book::isAgsWorldRegisteredByID(int id) {
	if(i()->MapAgsWorld.count(id) == 0) {
		return false;
	}
	return true;
}

bool Book::RegisterAgsWorld(int id, AgsWorld* world) {
	if (i()->MapAgsWorld.count(id) == 0) {
		i()->MapAgsWorld[id] = world;
		i()->ListB2World.push_back(world->B2AgsWorld);
		return true;
	}
	return false;
}

bool Book::UnregisterAgsWorldByID(int id) {
	if (i()->MapAgsWorld.count(id) == 0) {
		return false;
	}
	i()->MapAgsWorld.erase(id);
	DisposeWorldIfNeeded();
	return true;
}

AgsWorld* Book::IDtoAgsWorld(int id) {
	if (i()->MapAgsWorld.count(id) == 0) {
		return nullptr;
	}
	return i()->MapAgsWorld[id];
}
// -- End of AgsWorld Bookkeeping --

// -- AgsBody Bookkeeping --
bool Book::isAgsBodyRegisteredByID(int id) {
	if(i()->MapAgsBody.count(id) == 0) {
		return false;
	}
	return true;
}

bool Book::RegisterAgsBody(int id, AgsBody* body) {
	if (i()->MapAgsBody.count(id) == 0) {
		i()->MapAgsBody[id] = body;
		return true;
	}
	return false;
}

bool Book::UnregisterAgsBodyByID(int id) {
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

AgsBody* Book::IDtoAgsBody(int id) {
	if (i()->MapAgsBody.count(id) == 0) {
		return nullptr;
	}
	return i()->MapAgsBody[id];
}
// -- End of AgsBody Bookkeeping --

// -- AgsShape Bookkeeping --
bool Book::isAgsShapeRegisteredByID(int id) {
	if (i()->MapAgsShape.count(id) == 0) {
		return false;
	}
	return true;
}

bool Book::RegisterAgsShape(int id, AgsShape* shape) {
	if (i()->MapAgsShape.count(id) == 0) {
		i()->MapAgsShape[id] = shape;
		return true;
	}
	return false;
}

bool Book::UnregisterAgsShapeByID(int id) {
	if (i()->MapAgsShape.count(id) == 0) {
		return false;
	}
	i()->MapAgsShape.erase(id);
	return true;
}

AgsShape* Book::IDtoAgsShape(int id) {
	if (i()->MapAgsShape.count(id) == 0) {
		return nullptr;
	}
	return i()->MapAgsShape[id];
}
// -- End of AgsShape Bookkeeping --

// -- AgsFixture Bookkeeping --
bool Book::isAgsFixtureRegisteredByID(int id) {
	if(i()->MapAgsFixture.count(id) == 0) {
		return false;
	}
	return true;
}

bool Book::RegisterAgsFixture(int id, AgsFixture* fixture) {
	if (i()->MapAgsFixture.count(id) == 0) {
		i()->MapAgsFixture[id] = fixture;
		return true;
	}
	return false;
}

bool Book::UnregisterAgsFixtureByID(int id) {
	if (i()->MapAgsFixture.count(id) == 0) {
		return false;
	}
	i()->MapAgsFixture.erase(id);
	return true;
}

AgsFixture* Book::IDtoAgsFixture(int id) {
	if (i()->MapAgsFixture.count(id) == 0) {
		return nullptr;
	}
	return i()->MapAgsFixture[id];
}
// -- End of AgsFixture Bookkeeping --
