/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsBody.h"
#include "AgsWorld.h"
#include "Scale.h"
#include "Book.h"

AgsBody::AgsBody(AgsWorld* world, float32 x, float32 y, b2BodyType bodytype) {
	IsDestroyed = false;
	B2AgsBodyDef.position.Set(Scale::ScaleDown(x), Scale::ScaleDown(y));
	B2AgsBodyDef.type = bodytype;
	B2AgsBodyDef.fixedRotation = true;
	B2AgsBody = world->B2AgsWorld->CreateBody(&B2AgsBodyDef);
	World = world;
}

AgsBody::AgsBody(bool destroyed) {
	IsDestroyed = destroyed;
}

void AgsBody::InitializeIfNeeded(){
    if(B2AgsBody == nullptr) B2AgsBody = Book::IDtoB2Body(World->ID,B2BodyID);
}

b2Body* AgsBody::GetB2AgsBody(){
    InitializeIfNeeded();
    return B2AgsBody;
}

bool AgsBody::IsTouching(AgsBody* body) {
    InitializeIfNeeded();

	if (body == nullptr) {
		return false;
	}

	const b2ContactEdge *ce = B2AgsBody->GetContactList();
	b2Body *otherbody = body->B2AgsBody;

	while (ce != nullptr)
	{
		if (ce->other == otherbody &&
			ce->contact != nullptr &&
			ce->contact->IsTouching())
			return true;

		ce = ce->next;
	}

	return false;
}

bool AgsBody::GetIsDestroyed() {
	return IsDestroyed;
}

void AgsBody::SetIsDestroyed() {
	IsDestroyed = true;
}

void AgsBody::ApplyForce(float32 force_x, float32 force_y) {
    InitializeIfNeeded();
	B2AgsBody->ApplyForceToCenter(Scale::ScaleDown(b2Vec2(force_x, force_y)), true);
}

void AgsBody::SetLinearVelocity(float32 vel_x, float32 vel_y) {
    InitializeIfNeeded();
	B2AgsBody->SetLinearVelocity(Scale::ScaleDown(b2Vec2(vel_x, vel_y)));
}

void AgsBody::ApplyAngularImpulse(float32 impulse) {
    InitializeIfNeeded();
	B2AgsBody->ApplyAngularImpulse(Scale::ScaleDown(impulse), true);
}

float32 AgsBody::GetLinearVelocityX() {
    InitializeIfNeeded();
	return Scale::ScaleUp(B2AgsBody->GetLinearVelocity().x);
}

float32 AgsBody::GetLinearVelocityY() {
    InitializeIfNeeded();
	return Scale::ScaleUp(B2AgsBody->GetLinearVelocity().y);
}

float32 AgsBody::GetPosX() {
    InitializeIfNeeded();
	return Scale::ScaleUp(B2AgsBody->GetPosition().x);
}

float32 AgsBody::GetPosY() {
    InitializeIfNeeded();
	return Scale::ScaleUp(B2AgsBody->GetPosition().y);
}

void AgsBody::SetPosX(float32 x) {
    InitializeIfNeeded();
	B2AgsBody->SetTransform(b2Vec2(Scale::ScaleDown(x), B2AgsBody->GetPosition().y), B2AgsBody->GetAngle());
}

void AgsBody::SetPosY(float32 y) {
    InitializeIfNeeded();
	B2AgsBody->SetTransform(b2Vec2(B2AgsBody->GetPosition().x, Scale::ScaleDown(y)), B2AgsBody->GetAngle());
}

void AgsBody::SetPos(float32 x, float32 y) {
    InitializeIfNeeded();
	B2AgsBody->SetTransform(Scale::ScaleDown(b2Vec2(x,y)), B2AgsBody->GetAngle());
}

bool AgsBody::GetFixedRotation() {
    InitializeIfNeeded();
	return B2AgsBody->IsFixedRotation();
}

void AgsBody::SetFixedRotation(bool fixed) {
    InitializeIfNeeded();
	B2AgsBody->SetFixedRotation(fixed);
}

bool AgsBody::GetIsBullet() {
    InitializeIfNeeded();
	return B2AgsBody->IsBullet();
}

void AgsBody::SetIsBullet(bool bullet) {
    InitializeIfNeeded();
	B2AgsBody->SetBullet(bullet);
}

float32 AgsBody::GetAngle() {
    InitializeIfNeeded();
	return B2AgsBody->GetAngle();
}

void AgsBody::SetAngle(float32 angle) {
    InitializeIfNeeded();
	B2AgsBody->SetTransform(B2AgsBody->GetPosition(), angle);
}

float32 AgsBody::GetLinearDamping() {
    InitializeIfNeeded();
	return B2AgsBody->GetLinearDamping();
}

void AgsBody::SetLinearDamping(float32 ldamping) {
    InitializeIfNeeded();
	B2AgsBody->SetLinearDamping(ldamping);
}

float32 AgsBody::GetAngularDamping() {
    InitializeIfNeeded();
	return B2AgsBody->GetAngularDamping();
}

void AgsBody::SetAngularDamping(float32 adamping) {
    InitializeIfNeeded();
	B2AgsBody->SetAngularDamping(adamping);
}

float32 AgsBody::GetAngularVelocity() {
    InitializeIfNeeded();
	return B2AgsBody->GetAngularVelocity();
}

void AgsBody::SetAngularVelocity(float32 avel) {
    InitializeIfNeeded();
	B2AgsBody->SetAngularVelocity(avel);
}

float32 AgsBody::GetInertia() {
    InitializeIfNeeded();
	return Scale::ScaleUp(B2AgsBody->GetInertia());
}

void AgsBody::SetInertia(float32 inertia) {
    InitializeIfNeeded();
	b2MassData massData;
	massData.center = B2AgsBody->GetLocalCenter();
	massData.mass = B2AgsBody->GetMass();
	massData.I = Scale::ScaleDown(Scale::ScaleDown(inertia));
	B2AgsBody->SetMassData(&massData);
}

void AgsBody::ApplyLinearImpulse(float32 intensity_x, float32 intensity_y) {
    InitializeIfNeeded();
	B2AgsBody->ApplyLinearImpulse(Scale::ScaleDown(b2Vec2(intensity_x, intensity_y)), B2AgsBody->GetWorldCenter(), true);
}

void AgsBody::ApplyTorque(float32 torque) {
    InitializeIfNeeded();
	B2AgsBody->ApplyTorque(Scale::ScaleDown(torque), true);
}


AgsBody::~AgsBody(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsBodyInterface AgsBody_Interface;
AgsBodyReader AgsBody_Reader;

const char* AgsBodyInterface::name = "Body";

//------------------------------------------------------------------------------

#include "SerialHelper.h"
using namespace SerialHelper;

int AgsBodyInterface::Dispose(const char* address, bool force)
{
	Book::UnregisterAgsBodyByID(((AgsBody*)address)->ID);
	delete ((AgsBody*)address);
	AgsBody* body = ((AgsBody*)address);
    body = nullptr;
	return (1);
}

//------------------------------------------------------------------------------

int AgsBodyInterface::Serialize(const char* address, char* buffer, int bufsize)
{
	AgsBody* body = (AgsBody*)address;
	char* ptr = buffer;
	char* end = buffer + bufsize;

	//printf("--- serializing AgsBody %d --------->>>\n", body->ID);

	if (body->GetIsDestroyed()) {
		ptr = BoolToChar(true, ptr, end);
		return (ptr - buffer);
	}
	ptr = BoolToChar(false, ptr, end);
	ptr = IntToChar(body->World->ID, ptr, end);
    ptr = IntToChar(Book::b2BodyToID(body->World->ID, body->GetB2AgsBody()), ptr, end);

	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsBodyReader::Unserialize(int key, const char* serializedData, int dataSize)
{
	int body_id = key;
	AgsBody* body;
	char* ptr = (char*)serializedData;

	//printf("--- deserializing AgsBody %d ---------<<<\n", body_id);

	bool isdestroyed;
	int32 world_id;
	ptr = CharToBool(isdestroyed, ptr);

	if (isdestroyed) {
		body = new AgsBody();
		engine->RegisterUnserializedObject(key, body, &AgsBody_Interface);
		return;
	}

	ptr = CharToInt(world_id, ptr);

	AgsWorld * world;
	if (Book::isAgsWorldRegisteredByID(world_id)) {
		world = Book::IDtoAgsWorld(world_id);
	}
	else {
		world = new AgsWorld(0, 0);
		Book::RegisterAgsWorld(world_id, world);
	}

	if (Book::isAgsBodyRegisteredByID(body_id)) {
		body = Book::IDtoAgsBody(body_id);
		body->World = world;
	}
	else {
        int32 b2body_id;
        ptr = CharToInt(b2body_id, ptr);

        body = new AgsBody();
		body->ID = body_id;
		body->B2BodyID = b2body_id;
		body->World = world;

		Book::RegisterAgsBody(body_id, body);
	}

	engine->RegisterUnserializedObject(key, body, &AgsBody_Interface);
}

//..............................................................................
