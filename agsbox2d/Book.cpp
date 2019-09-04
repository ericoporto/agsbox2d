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

// -- PUBLIC --

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
		return true;
	}
	return false;
}

bool Book::UnregisterAgsWorldByID(int id) {
	if (i()->MapAgsWorld.count(id) == 0) {
		return false;
	}
	i()->MapAgsWorld.erase(id);
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
    i()->MapAgsBody.erase(id);
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