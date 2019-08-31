#pragma once

#ifndef _AGS_WORLD_H
#define _AGS_WORLD_H

#include "Box2D.h"
#include "plugin/agsplugin.h"

class AgsBody; // forward declaration, we need AgsBody.h on the cpp

class AgsWorld
{
public:
	AgsWorld(float32 gravityX, float32 gravityY);
	~AgsWorld(void);
	b2World* B2AgsWorld;
	AgsBody* NewBody(float32 x, float32 y);
	int32 ID;
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