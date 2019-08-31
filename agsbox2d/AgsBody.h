#pragma once

#ifndef _AGS_BODY_H
#define _AGS_BODY_H

#include "Box2D.h"
#include "plugin/agsplugin.h"

class AgsWorld; // forward declaration, we need AgsWorld.h on the cpp

class AgsBody
{
public:
	AgsBody(AgsWorld* world, float32 x, float32 y, b2BodyType bodytype = b2_dynamicBody);
	~AgsBody(void);
	AgsWorld* World;
	b2Body* B2AgsBody;
	b2BodyDef B2AgsBodyDef;
	float32 GetPosX();
	float32 GetPosY();
	void SetPosX(float32 x);
	void SetPosY(float32 y);
	void SetPos(float32 x, float32 y);

	void ApplyForce(float32 force_x, float32 force_y);
	int32 ID;
};



//------------------------------------------------------------------------------
// AGS interface instances

class AgsBodyInterface : public IAGSScriptManagedObject
{
public:
	static const char* name;

	AgsBodyInterface() {};

	virtual int Dispose(const char* address, bool force);
	virtual const char* GetType() { return (name); }
	virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsBodyReader : public IAGSManagedObjectReader
{
public:

	AgsBodyReader() {}

	virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsBodyInterface AgsBody_Interface;
extern AgsBodyReader AgsBody_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_BODY_H */

//..............................................................................