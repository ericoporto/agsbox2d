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

AgsJointDistance::AgsJointDistance(b2DistanceJoint* distancejoint){
    B2AgsJointDistance = distancejoint;
}


void AgsJointDistance::SetLength(float32 length) {
    B2AgsJointDistance->SetLength(Scale::ScaleDown(length));
}

float32 AgsJointDistance::GetLength(){
    return  Scale::ScaleUp( B2AgsJointDistance->GetLength());
}

void AgsJointDistance::SetFrequency(float32 hz) {
    B2AgsJointDistance->SetFrequency(hz);
}

float32 AgsJointDistance::GetFrequency(){
    return B2AgsJointDistance->GetFrequency();
}

void AgsJointDistance::SetDampingRation(float32 dratio) {
    B2AgsJointDistance->SetDampingRatio(dratio);
}

float32 AgsJointDistance::GetDampingRatio(){
    return   B2AgsJointDistance->GetDampingRatio();
}

AgsBody* AgsJointDistance::GetBodyA() {
    return AgsJoint::GetBody(WorldID, B2AgsJointDistance->GetBodyA());
}

AgsBody* AgsJointDistance::GetBodyB() {
    return AgsJoint::GetBody(WorldID, B2AgsJointDistance->GetBodyB());
}

//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsJointDistanceInterface AgsJointDistance_Interface;
AgsJointDistanceReader AgsJointDistance_Reader;

const char* AgsJointDistanceInterface::name = "JointDistance";

//------------------------------------------------------------------------------

int AgsJointDistanceInterface::Dispose(const char* address, bool force)
{
    //delete ((AgsShapeCircle*)address);
    return (1);
}

//------------------------------------------------------------------------------

int AgsJointDistanceInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    AgsJointDistance* arr = (AgsJointDistance*)address;
    char* ptr = buffer;

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsJointDistanceReader::Unserialize(int key, const char* serializedData, int dataSize)
{
//	AgsShapeCircle* arr = new AgsShapeCircle(0,0);

//	const char* ptr = serializedData;

//	engine->RegisterUnserializedObject(key, arr, &AgsShapeCircle_Interface);
}

//..............................................................................
