/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_WORLD_H
#define _AGS_WORLD_H

#include "Box2D.h"
#include "plugin/agsplugin.h"
#include <vector>

class AgsBody; // forward declaration, we need AgsBody.h on the cpp

class AgsWorld
{
    b2Body* B2GroundBody;
public:
	AgsWorld(float32 gravityX, float32 gravityY);
	~AgsWorld(void);
	b2World* B2AgsWorld;
    b2Body* GetGroundB2Body();
	AgsBody* NewBody(float32 x, float32 y, b2BodyType bodytype = b2_dynamicBody);
    AgsBody* findObject(b2Body* b2body);
	void DestroyBody(AgsBody* body);
	void Step(float32 dt, int velocityIterations, int positionIterations);
	int32 ID;
	//std::vector<AgsBody*> AgsBodyList;
};



//------------------------------------------------------------------------------
// AGS interface instances

class AgsWorldInterface : public IAGSScriptManagedObject
{
public:
	static const char* name;

	AgsWorldInterface() {};

	virtual int Dispose(const char* address, bool force);
	virtual const char* GetType() { return (name); }
	virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsWorldReader : public IAGSManagedObjectReader
{
public:

	AgsWorldReader() {}

	virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsWorldInterface AgsWorld_Interface;
extern AgsWorldReader AgsWorld_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_WORLD_H */

//..............................................................................