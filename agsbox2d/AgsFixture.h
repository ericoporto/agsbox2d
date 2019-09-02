#pragma once

#ifndef _AGS_FIXTURE_H
#define _AGS_FIXTURE_H

#include "Box2D.h"
#include "plugin/agsplugin.h"
#include "AgsBody.h"
#include "AgsShape.h"

class AgsFixture
{
public:
	AgsFixture(AgsBody* agsBody, AgsShape* agsShape, float32 density);
	~AgsFixture(void);
	AgsBody* Body;
	AgsShape* Shape;
	b2Fixture* B2AgsFixture;
	b2FixtureDef B2AgsFixtureDef;

	float32 GetDensity();
	void SetDensity(float32 density);

	float32 GetFriction();
	void SetFriction(float32 friction);

	float32 GetRestitution();
	void SetRestitution(float32 restitution);

	int32 ID;
};



//------------------------------------------------------------------------------
// AGS interface instances

class AgsFixtureInterface : public IAGSScriptManagedObject
{
public:
	static const char* name;

	AgsFixtureInterface() {};

	virtual int Dispose(const char* address, bool force);
	virtual const char* GetType() { return (name); }
	virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsFixtureReader : public IAGSManagedObjectReader
{
public:

	AgsFixtureReader() {}

	virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsFixtureInterface AgsFixture_Interface;
extern AgsFixtureReader AgsFixture_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_FIXTURE_H */

//..............................................................................