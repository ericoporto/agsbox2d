#pragma once

#ifndef _AGS_BODY_H
#define _AGS_BODY_H

#include "Box2D.h"
#include "plugin/agsplugin.h"

class AgsWorld; // forward declaration, we need AgsWorld.h on the cpp

class AgsBody
{
	bool IsDestroyed;
public:
	AgsBody();
	AgsBody(AgsWorld* world, float32 x, float32 y, b2BodyType bodytype = b2_dynamicBody);
	~AgsBody(void);
	AgsWorld* World;
	b2Body* B2AgsBody;
	b2BodyDef B2AgsBodyDef;

	bool IsTouching(AgsBody* body);

	bool GetIsDestroyed();
	void SetIsDestroyed();

	float32 GetPosX();
	float32 GetPosY();
	void SetPosX(float32 x);
	void SetPosY(float32 y);
	void SetPos(float32 x, float32 y);

	void SetLinearVelocity(float32 vel_x, float32 vel_y);
	float32 GetLinearVelocityX();
	float32 GetLinearVelocityY();

	bool GetFixedRotation();
	void SetFixedRotation(bool fixed);

	bool GetIsBullet();
	void SetIsBullet(bool bullet);

	float32 GetAngle();
	void SetAngle(float32 angle);

	float32 GetLinearDamping();
	void SetLinearDamping(float32 ldamping);

	float32 GetAngularDamping();
	void SetAngularDamping(float32 adamping);

	float32 GetAngularVelocity();
	void SetAngularVelocity(float32 avel);

	float32 GetInertia();
	void SetInertia(float32 inertia);

	void ApplyForce(float32 force_x, float32 force_y);
	void ApplyAngularImpulse(float32 impulse);
	void ApplyLinearImpulse(float32 intensity_x, float32 intensity_y);
	void ApplyTorque(float32 torque);

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
