#pragma once

#ifndef AGSBOX2D_BOOK_H
#define AGSBOX2D_BOOK_H

#include "Box2D.h"
#include <vector>
#include <unordered_map>

class AgsWorld; // forward declaration, we need AgsWorld.h on the cpp
class AgsBody; // forward declaration, we need AgsBody.h on the cpp
class AgsFixture; // forward declaration, we need AgsFixture.h on the cpp
class AgsShape; // forward declaration, we need AgsShape.h on the cpp

class Book
{
private:
	Book();
	~Book(void);
	std::vector<b2World* > ListB2World;
	std::unordered_map<int, AgsWorld*> MapAgsWorld;
	std::unordered_map<int, AgsBody*> MapAgsBody;
	std::unordered_map<int, AgsFixture*> MapAgsFixture;
	std::unordered_map<int, AgsShape*> MapAgsShape;

	static void DisposeWorldIfNeeded();

public:
	static Book* i();

	static bool isAgsWorldRegisteredByID(int id);
	static bool RegisterAgsWorld(int id, AgsWorld* world);
	static bool UnregisterAgsWorldByID(int id);
	static AgsWorld* IDtoAgsWorld(int id);

	static bool isAgsBodyRegisteredByID(int id);
	static bool RegisterAgsBody(int id, AgsBody* body);
	static bool UnregisterAgsBodyByID(int id);
	static AgsBody* IDtoAgsBody(int id);

	static bool isAgsShapeRegisteredByID(int id);
	static bool RegisterAgsShape(int id, AgsShape* shape);
	static bool UnregisterAgsShapeByID(int id);
	static AgsShape* IDtoAgsShape(int id);

	static bool isAgsFixtureRegisteredByID(int id);
	static bool RegisterAgsFixture(int id, AgsFixture* fixture);
	static bool UnregisterAgsFixtureByID(int id);
	static AgsFixture* IDtoAgsFixture(int id);
};


#endif /* AGSBOX2D_BOOK_H */
