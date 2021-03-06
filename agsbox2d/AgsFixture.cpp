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
#include "Scale.h"

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

AgsFixture::AgsFixture(int32 world_id, int32 b2body_id, int32 b2fixture_id) {
    WorldID = world_id;
    b2BodyID = b2body_id;
    b2FixtureID = b2fixture_id;
}

void AgsFixture::InitializeIfNeeded(){
    if(b2FixtureID <0) {
        B2AgsFixture = nullptr;
    }

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

void AgsFixture::SetGroupIndex(int32 index) {
    InitializeIfNeeded();
    if(index < -32768 || index > 32767) return;

    b2Filter f = B2AgsFixture->GetFilterData();
    f.groupIndex = (uint16)index;
    B2AgsFixture->SetFilterData(f);
}

int32 AgsFixture::GetGroupIndex() {
    InitializeIfNeeded();
    return B2AgsFixture->GetFilterData().groupIndex;
}

int32 AgsFixture::GetCategoryBits(){
    InitializeIfNeeded();
    return B2AgsFixture->GetFilterData().categoryBits;
}

void AgsFixture::SetCategoryBits(int32 category){
    InitializeIfNeeded();
    if(category < 1 || category > 65535) return;

    b2Filter f = B2AgsFixture->GetFilterData();
    f.categoryBits = (uint16)category;
    B2AgsFixture->SetFilterData(f);
}

int32 AgsFixture::GetMaskBits(){
    InitializeIfNeeded();
    return B2AgsFixture->GetFilterData().maskBits;
}

void AgsFixture::SetMaskBits(int32 mask){
    InitializeIfNeeded();
    if(mask < 0 || mask > 65535) return;

    b2Filter f = B2AgsFixture->GetFilterData();
    f.maskBits = (uint16)mask;
    B2AgsFixture->SetFilterData(f);
}

bool AgsFixture::TestPoint(float32 x, float32 y){
    InitializeIfNeeded();
    return B2AgsFixture->TestPoint(Scale::ScaleDown(b2Vec2(x, y)));
}

void AgsFixture::SetIsSensor(bool sensor){
    InitializeIfNeeded();
    return B2AgsFixture->SetSensor(sensor);
}

bool AgsFixture::GetIsSensor(){
    InitializeIfNeeded();
    return B2AgsFixture->IsSensor();
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
    AgsFixture* fixture = ((AgsFixture*)address);
	if((AgsFixture*)address == nullptr)
        return (1);

	Book::UnregisterAgsFixtureByID(((AgsFixture*)address)->ID);
	delete ((AgsFixture*)address);
    fixture = nullptr;
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
    int32 b2fixture_id = agsFixture->b2FixtureID;// Book::b2FixtureToID(world_id, agsFixture->GetB2AgsFixture());

    ptr = IntToChar(world_id, ptr, end);
    ptr = IntToChar(b2body_id, ptr, end);
    ptr = IntToChar(b2fixture_id, ptr, end);
	
	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsFixtureReader::Unserialize(int key, const char* serializedData, int dataSize)
{
    int32 world_id;
    int32 b2body_id;
    int32 b2fixture_id;

    char* ptr = (char*)serializedData;

    ptr = CharToInt(world_id, ptr);
    ptr = CharToInt(b2body_id, ptr);
    ptr = CharToInt(b2fixture_id, ptr);

    AgsFixture* agsFixture = nullptr;

    AgsWorld * world;
    if (Book::isAgsWorldRegisteredByID(world_id)) {
        world = Book::IDtoAgsWorld(world_id);
        if(world == nullptr) throw;
    }
    else {
        world = new AgsWorld(0, 0);
        world->ID = world_id;
        Book::RegisterAgsWorld(world_id, world);
        //printf("AgsFixtureReader::Unserialize - world_id=%d\n",world_id);
    }

    if (Book::isAgsFixtureArrayRegisteredByID(key)) {
        agsFixture = Book::IDtoAgsFixture(key);
        agsFixture->WorldID = world_id;
        agsFixture->b2FixtureID = Book::b2FixtureToID(world_id, agsFixture->GetB2AgsFixture());
    }

    if(agsFixture == nullptr) {
        agsFixture = new AgsFixture(world_id, b2body_id, b2fixture_id);
        agsFixture->ID = key;
        Book::RegisterAgsFixture(agsFixture->ID, agsFixture);
        agsFixture->InitializeIfNeeded();
    }
    engine->RegisterUnserializedObject(key, agsFixture, &AgsFixture_Interface);
}

//..............................................................................
