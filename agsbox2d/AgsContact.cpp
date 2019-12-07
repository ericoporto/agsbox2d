/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsContact.h"
#include "Book.h"
#include "Scale.h"
#include "PointF.h"
#include "AgsWorld.h"

AgsContact::AgsContact(b2Contact* contact, int32 world_id) {
    WorldID = world_id;
    B2AgsContact = contact;
}

AgsContact::AgsContact(int32 world_id, int32 contact_index, bool valid){
    WorldID = world_id;
    B2AgsContact = nullptr;
    if(contact_index>=0 && valid) {
        AgsWorld* world = Book::IDtoAgsWorld(WorldID);
        b2Contact* contact = world->B2AgsWorld->GetContactList();
        int index = 0;

        do
        {
            if (!contact) {
                contact = nullptr;
                break;
            }
            if(index == contact_index) break;

            index++;
        }
        while ((contact = contact->GetNext()));

        B2AgsContact = contact;
    }
}

AgsContact::~AgsContact(void)
{
}

void AgsContact::Invalidate() {
    B2AgsContact = nullptr;
}

bool AgsContact::IsValid() {
    return  B2AgsContact != nullptr;
}

PointF* AgsContact::GetNormal() {
  if(B2AgsContact == nullptr) return nullptr;

    b2WorldManifold manifold;
    B2AgsContact->GetWorldManifold(&manifold);
    return new PointF(manifold.normal.x, manifold.normal.y);
}

int32 AgsContact::GetPositionsCount() {
    if(B2AgsContact == nullptr) return 0;

    return B2AgsContact->GetManifold()->pointCount;
}

PointF* AgsContact::GetPosition(int32 i) {
    if(B2AgsContact == nullptr) return nullptr;
    if(i<0 || i>=B2AgsContact->GetManifold()->pointCount) return nullptr;

    b2WorldManifold manifold;
    B2AgsContact->GetWorldManifold(&manifold);
    b2Vec2 position = Scale::ScaleUp(manifold.points[i]);
    return new PointF(position.x,position.y);
}

b2Fixture* AgsContact::GetB2FixtureA()
{
    return B2AgsContact->GetFixtureA();
}

b2Fixture* AgsContact::GetB2FixtureB()
{
    return B2AgsContact->GetFixtureB();
}




bool AgsContact::IsEnabled()
{
    return B2AgsContact->IsEnabled();
}

bool AgsContact::IsTouching()
{
    return B2AgsContact->IsTouching();
}

void AgsContact::SetFriction(float32 friction)
{
    B2AgsContact->SetFriction(friction);
}

float32 AgsContact::GetFriction()
{
    return B2AgsContact->GetFriction();
}

void AgsContact::SetRestitution(float32 restitution)
{
    B2AgsContact->SetRestitution(restitution);
}

float32 AgsContact::GetRestitution()
{
    return B2AgsContact->GetRestitution();
}

void AgsContact::SetEnabled(bool enabled)
{
    B2AgsContact->SetEnabled(enabled);
}

void AgsContact::ResetFriction()
{
    B2AgsContact->ResetFriction();
}

void AgsContact::ResetRestitution()
{
    B2AgsContact->ResetRestitution();
}

void AgsContact::SetTangentSpeed(float32 speed)
{
    B2AgsContact->SetTangentSpeed(speed);
}

float32 AgsContact::GetTangentSpeed()
{
    return B2AgsContact->GetTangentSpeed();
}

b2Contact* AgsContact::GetB2AgsContact(){
    return B2AgsContact;
}

//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsContactInterface AgsContact_Interface;
AgsContactReader AgsContact_Reader;

const char* AgsContactInterface::name = "Contact";

//------------------------------------------------------------------------------
#include "SerialHelper.h"
#include "AgsWorld.h"

using namespace SerialHelper;

int AgsContactInterface::Dispose(const char* address, bool force)
{
//	Book::UnregisterAgsContactByID(((AgsContact*)address)->ID);
    delete ((AgsContact*)address);
    return (1);
}

//------------------------------------------------------------------------------

int AgsContactInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    AgsContact* agsContact = (AgsContact*)address;
    char* ptr = buffer;
    char* end = buffer + bufsize;

    AgsWorld* world = Book::IDtoAgsWorld(agsContact->WorldID);
    b2Contact* contact = world->B2AgsWorld->GetContactList();
    int index = 0;

    do
    {
        if (!contact) {
            index = -1;
            break;
        }
        if(contact == agsContact->GetB2AgsContact()) break;

        index++;
    }
    while ((contact = contact->GetNext()));


    ptr = IntToChar(index, ptr, end);
    ptr = IntToChar(agsContact->WorldID, ptr, end);
    ptr = BoolToChar(agsContact->IsValid(), ptr, end);

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsContactReader::Unserialize(int key, const char* serializedData, int dataSize)
{
    int32 b2contact_index;
    int32 world_id;
    bool is_valid;
    char* ptr = (char*)serializedData;

    ptr = CharToInt(b2contact_index, ptr);
    ptr = CharToInt(world_id, ptr);
    ptr = CharToBool(is_valid, ptr);

    AgsContact* agsContact = new AgsContact(world_id, b2contact_index, is_valid);

    engine->RegisterUnserializedObject(key, agsContact, &AgsContact_Interface);
}

//..............................................................................
