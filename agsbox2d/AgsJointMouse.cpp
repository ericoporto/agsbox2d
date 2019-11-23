/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsJointMouse.h"
#include "Book.h"
#include "Scale.h"
#include "AgsWorld.h"
#include "AgsJoint.h"

AgsJointMouse::AgsJointMouse(AgsWorld* agsworld, AgsBody* agsbody_a, float32 x, float32 y) {
    if (agsbody_a->World->B2AgsWorld != agsworld->B2AgsWorld )
        return;

    if (agsbody_a->GetB2AgsBody()->GetType() == b2BodyType::b2_kinematicBody)
        throw std::invalid_argument("Cannot attach a MouseJoint to a kinematic body");


    WorldID = agsworld->ID;
    B2bodyA_ID = agsbody_a->B2BodyID;
    B2bodyB_ID = -1;

    b2MouseJointDef def;

    def.bodyA = agsworld->GetGroundB2Body();
    def.bodyB = agsbody_a->GetB2AgsBody();
    def.maxForce = 1000.0f * agsbody_a->GetB2AgsBody()->GetMass();
    def.target = Scale::ScaleDown(b2Vec2(x,y));

    B2AgsJointMouse = dynamic_cast<b2MouseJoint *>(agsworld->B2AgsWorld->CreateJoint(&def));
}

AgsJointMouse::AgsJointMouse(b2MouseJoint* Mousejoint){
    B2AgsJointMouse = Mousejoint;
}


AgsJointMouse::~AgsJointMouse(void)
{
}

void AgsJointMouse::InitializeIfNeeded(){
    if(B2AgsJointMouse == nullptr) {
        B2AgsJointMouse = dynamic_cast<b2MouseJoint *>(Book::IDtoB2Joint(WorldID, b2Joint_ID));
    }
}

b2MouseJoint* AgsJointMouse::GetB2AgsJointMouse(){
    InitializeIfNeeded();
    return  B2AgsJointMouse;
}

void AgsJointMouse::SetTarget(float32 x, float32 y)
{
    InitializeIfNeeded();
    B2AgsJointMouse->SetTarget(Scale::ScaleDown(b2Vec2(x, y)));
}

float32 AgsJointMouse::GetTargetX()
{
    InitializeIfNeeded();
    return Scale::ScaleUp(B2AgsJointMouse->GetTarget().x);
}

float32 AgsJointMouse::GetTargetY()
{
    InitializeIfNeeded();
    return Scale::ScaleUp(B2AgsJointMouse->GetTarget().x);
}

void AgsJointMouse::SetMaxForce(float32 force)
{
    InitializeIfNeeded();
    B2AgsJointMouse->SetMaxForce(Scale::ScaleDown(force));
}

float32 AgsJointMouse::GetMaxForce()
{
    InitializeIfNeeded();
    return Scale::ScaleUp(B2AgsJointMouse->GetMaxForce());
}

void AgsJointMouse::SetFrequency(float32 hz)
{
    InitializeIfNeeded();
    // This is kind of a crappy check. The frequency is used in an internal
    // box2d calculation whose result must be > FLT_EPSILON, but other variables
    // go into that calculation...
    if(hz <= FLT_EPSILON * 2)
        throw std::invalid_argument("MouseJoint frequency must be a positive number.");

    B2AgsJointMouse->SetFrequency(hz);
}

float32 AgsJointMouse::GetFrequency()
{
    InitializeIfNeeded();
    return B2AgsJointMouse->GetFrequency();
}

void AgsJointMouse::SetDampingRatio(float32 d)
{
    InitializeIfNeeded();
    B2AgsJointMouse->SetDampingRatio(d);
}

float32 AgsJointMouse::GetDampingRatio()
{
    InitializeIfNeeded();
    return B2AgsJointMouse->GetDampingRatio();
}

b2Body* AgsJointMouse::GetBodyA() {
    InitializeIfNeeded();
    return  B2AgsJointMouse->GetBodyB();
}

b2Body *AgsJointMouse::GetBodyB()
{
    InitializeIfNeeded();
    return nullptr;
}

//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsJointMouseInterface AgsJointMouse_Interface;
AgsJointMouseReader AgsJointMouse_Reader;

const char* AgsJointMouseInterface::name = "JointMouse";

//------------------------------------------------------------------------------

int AgsJointMouseInterface::Dispose(const char* address, bool force)
{
    Book::UnregisterAgsJointMouseByID(((AgsJointMouse*)address)->ID);
    //delete ((AgsJointMouse*)address);
    return (1);
}

//------------------------------------------------------------------------------

int AgsJointMouseInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    AgsJointMouse* arr = (AgsJointMouse*)address;
    char* ptr = buffer;

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsJointMouseReader::Unserialize(int key, const char* serializedData, int dataSize)
{
//	AgsShapeCircle* arr = new AgsShapeCircle(0,0);

//	const char* ptr = serializedData;

//	engine->RegisterUnserializedObject(key, arr, &AgsShapeCircle_Interface);
}

//..............................................................................
