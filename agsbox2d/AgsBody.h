#pragma once

#ifndef _AGS_BODY_H
#define _AGS_BODY_H

#include "Box2D.h"
#include "plugin/agsplugin.h"

class AgsBody
{
public:
	AgsBody(b2World* world, float32 x, float32 y, b2BodyType bodytype = b2_dynamicBody);
	~AgsBody(void);
	b2World* B2AgsWorld;
	b2Body* B2AgsBody;
	b2BodyDef B2AgsBodyDef;
	float32 GetPosX();
	float32 GetPosY();
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

extern AgsBodyInterface AgsBodyInterface_Interface;
extern AgsBodyReader AgsBodyReader_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_BODY_H */

//..............................................................................