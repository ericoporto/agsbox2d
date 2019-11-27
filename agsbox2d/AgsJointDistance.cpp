/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsJointDistance.h"
#include "Book.h"
#include "Scale.h"
#include "AgsWorld.h"
#include "AgsJoint.h"

AgsJointDistance::AgsJointDistance(AgsWorld* agsworld,
        AgsBody* agsbody_a, AgsBody* agsbody_b,
        float32 x1, float32 y1, float32 x2, float32 y2,
        bool collide_connected) {
    if (agsbody_a->World->B2AgsWorld != agsworld->B2AgsWorld ||
        agsbody_b->World->B2AgsWorld != agsworld->B2AgsWorld)
        return;

    b2DistanceJointDef def;
    def.Initialize(agsbody_a->GetB2AgsBody(), agsbody_b->GetB2AgsBody(),
            Scale::ScaleDown(b2Vec2(x1,y1)), Scale::ScaleDown(b2Vec2(x2,y2)));
    def.collideConnected = collide_connected;
    B2AgsJointDistance = dynamic_cast<b2DistanceJoint *>(agsworld->B2AgsWorld->CreateJoint(&def));
    WorldID = agsworld->ID;
    B2bodyA_ID = agsbody_a->B2BodyID;
    B2bodyB_ID = agsbody_b->B2BodyID;
}

AgsJointDistance::AgsJointDistance(int32 world_id, b2DistanceJoint* distancejoint){
    B2AgsJointDistance = distancejoint;
    WorldID = world_id;
    B2bodyA_ID = Book::b2BodyToID(WorldID, distancejoint->GetBodyA());
    B2bodyB_ID = Book::b2BodyToID(WorldID, distancejoint->GetBodyB());
    b2Joint_ID = Book::b2JointToID(WorldID, dynamic_cast<b2Joint*>(B2AgsJointDistance));
}

AgsJointDistance::~AgsJointDistance(void)
{
}

void AgsJointDistance::InitializeIfNeeded(){
    if(B2AgsJointDistance == nullptr) {
        B2AgsJointDistance = dynamic_cast<b2DistanceJoint *>(Book::IDtoB2Joint(WorldID, b2Joint_ID));
    }
}

b2DistanceJoint* AgsJointDistance::GetB2AgsJointDistance(){
    InitializeIfNeeded();
    return  B2AgsJointDistance;
}

void AgsJointDistance::SetLength(float32 length) {
    InitializeIfNeeded();
    B2AgsJointDistance->SetLength(Scale::ScaleDown(length));
}

float32 AgsJointDistance::GetLength(){
    InitializeIfNeeded();
    return  Scale::ScaleUp( B2AgsJointDistance->GetLength());
}

void AgsJointDistance::SetFrequency(float32 hz) {
    InitializeIfNeeded();
    B2AgsJointDistance->SetFrequency(hz);
}

float32 AgsJointDistance::GetFrequency(){
    InitializeIfNeeded();
    return B2AgsJointDistance->GetFrequency();
}

void AgsJointDistance::SetDampingRatio(float32 dratio) {
    InitializeIfNeeded();
    B2AgsJointDistance->SetDampingRatio(dratio);
}

float32 AgsJointDistance::GetDampingRatio(){
    InitializeIfNeeded();
    return   B2AgsJointDistance->GetDampingRatio();
}

b2Body* AgsJointDistance::GetBodyA() {
    InitializeIfNeeded();
    return B2AgsJointDistance->GetBodyB();
}

b2Body* AgsJointDistance::GetBodyB() {
    InitializeIfNeeded();
    return B2AgsJointDistance->GetBodyB();
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsJointDistanceInterface AgsJointDistance_Interface;
AgsJointDistanceReader AgsJointDistance_Reader;

const char* AgsJointDistanceInterface::name = "JointDistance";

//------------------------------------------------------------------------------
#include "SerialHelper.h"
using namespace SerialHelper;

int AgsJointDistanceInterface::Dispose(const char* address, bool force)
{
    Book::UnregisterAgsJointDistanceByID(((AgsJointDistance*)address)->ID);
    delete ((AgsJointDistance*)address);
    AgsJointDistance* agsJointDistance = ((AgsJointDistance*)address);
    agsJointDistance = nullptr;
    return (1);
}

//------------------------------------------------------------------------------

int AgsJointDistanceInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    AgsJointDistance* agsJointDistance = (AgsJointDistance*)address;
    char* ptr = buffer;
    char* end = buffer + bufsize;

    b2Joint* b2joint = dynamic_cast<b2Joint*>( agsJointDistance->GetB2AgsJointDistance());
    int32 world_id = agsJointDistance->WorldID;
    int32 b2joint_id = Book::b2JointToID(world_id, b2joint);

    ptr = IntToChar(world_id, ptr, end);
    ptr = IntToChar(b2joint_id, ptr, end);

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsJointDistanceReader::Unserialize(int key, const char* serializedData, int dataSize)
{
    char* ptr = (char*) serializedData;

    int32 agsjointdistance_id = key;
    int32 world_id;
    int32 b2joint_id;

    ptr = CharToInt(world_id, ptr);
    ptr = CharToInt(b2joint_id, ptr);

    AgsWorld * world;
    if (Book::isAgsWorldRegisteredByID(world_id)) {
        world = Book::IDtoAgsWorld(world_id);
        if(world == nullptr) throw;
    }
    else {
        world = new AgsWorld(0, 0);
        Book::RegisterAgsWorld(world_id, world);
    }

    b2Joint * b2joint = Book::IDtoB2Joint(world_id, b2joint_id);

    AgsJointDistance* agsjoint = new AgsJointDistance(world_id, dynamic_cast<b2DistanceJoint*>(b2joint));
    agsjoint->ID = agsjointdistance_id;
    agsjoint->b2Joint_ID = b2joint_id;
    Book::RegisterAgsJointDistance(agsjointdistance_id, agsjoint);

	engine->RegisterUnserializedObject(key, agsjoint, &AgsJointDistance_Interface);
}

//..............................................................................
