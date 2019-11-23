/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsFixture.h"
#include "Book.h"
#include "AgsWorld.h"

AgsFixture::AgsFixture(AgsBody* agsBody, AgsShape* agsShape, float32 density) {

    AgsWorld* agsWorld = agsBody->World;
    WorldID = agsWorld->ID;
    b2BodyID = Book::b2BodyToID(WorldID, agsBody->GetB2AgsBody());

	B2AgsFixtureDef.density = density;

	if (agsShape->ShapeCircle != NULL) {
		B2AgsFixtureDef.shape = agsShape->ShapeCircle->B2AgsShapeCircle;
	}
	else if (agsShape->ShapeRect != NULL) {
		B2AgsFixtureDef.shape = agsShape->ShapeRect->B2AgsShapeRect;
	}
	else {
		B2AgsFixtureDef.shape = agsShape->B2AgsShape;
	}

	B2AgsFixture = agsBody->GetB2AgsBody()->CreateFixture(&B2AgsFixtureDef);
}

AgsFixture::AgsFixture(int32 world_id, int32 b2body_id, int32 fixture_id) {
    WorldID = world_id;
    b2BodyID = b2body_id;
    b2FixtureID = fixture_id;
}

void AgsFixture::InitializeIfNeeded(){
    if(B2AgsFixture == nullptr) {
        B2AgsFixture = Book::IDtoB2Fixture(WorldID, b2FixtureID);
    }
}

b2Fixture*  AgsFixture::GetB2AgsFixture(){
    InitializeIfNeeded();
    return B2AgsFixture;
}

float32 AgsFixture::GetDensity() {
    InitializeIfNeeded();
	return B2AgsFixture->GetDensity();
}

void AgsFixture::SetDensity(float32 density) {
    InitializeIfNeeded();
	B2AgsFixture->SetDensity(density);
}

float32 AgsFixture::GetFriction() {
    InitializeIfNeeded();
	return B2AgsFixture->GetFriction();
}

void AgsFixture::SetFriction(float32 friction) {
    InitializeIfNeeded();
	B2AgsFixture->SetFriction(friction);
}

float32 AgsFixture::GetRestitution() {
    InitializeIfNeeded();
	return B2AgsFixture->GetRestitution();
}

void AgsFixture::SetRestitution(float32 restitution) {
    InitializeIfNeeded();
	B2AgsFixture->SetRestitution(restitution);
}

b2Body* AgsFixture::GetB2Body() {
    InitializeIfNeeded();
    return B2AgsFixture->GetBody();
}

AgsFixture::~AgsFixture(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsFixtureInterface AgsFixture_Interface;
AgsFixtureReader AgsFixture_Reader;

const char* AgsFixtureInterface::name = "Fixture";

//------------------------------------------------------------------------------
#include "SerialHelper.h"
using namespace SerialHelper;

int AgsFixtureInterface::Dispose(const char* address, bool force)
{
	//if (((AgsFixture*)address)->Body != NULL && 
	//	(((AgsFixture*)address)->Body->B2AgsBody != NULL) && 
	//	(((AgsFixture*)address)->B2AgsFixture != NULL)){

	//	((AgsFixture*)address)->Body->B2AgsBody->DestroyFixture(((AgsFixture*)address)->B2AgsFixture);
	//}

	Book::UnregisterAgsFixtureByID(((AgsFixture*)address)->ID);
	delete ((AgsFixture*)address);
	return (1);
}

//------------------------------------------------------------------------------

int AgsFixtureInterface::Serialize(const char* address, char* buffer, int bufsize)
{
	AgsFixture* agsFixture = (AgsFixture*)address;
	char* ptr = buffer;
    char* end = buffer + bufsize;

    int32 world_id = agsFixture->WorldID;
    int32 b2body_id = agsFixture->b2BodyID;
    int32 fixture_id = agsFixture->b2FixtureID;

    ptr = IntToChar(world_id, ptr, end);
    ptr = IntToChar(b2body_id, ptr, end);
    ptr = IntToChar(fixture_id, ptr, end);
	
	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsFixtureReader::Unserialize(int key, const char* serializedData, int dataSize)
{
    int32 world_id;
    int32 b2body_id;
    int32 fixture_id;

    char* ptr = (char*)serializedData;

    ptr = CharToInt(world_id, ptr);
    ptr = CharToInt(b2body_id, ptr);
    ptr = CharToInt(fixture_id, ptr);

    AgsFixture* agsFixture = new AgsFixture(world_id, b2body_id, fixture_id);

	engine->RegisterUnserializedObject(key, agsFixture, &AgsFixture_Interface);
}

//..............................................................................
