/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsJointMotor.h"
#include "Book.h"
#include "Scale.h"
#include "AgsWorld.h"
#include "AgsJoint.h"

AgsJointMotor::AgsJointMotor(AgsWorld* agsworld,
        AgsBody* agsbody_a, AgsBody* agsbody_b) {

    b2MotorJointDef def;

    def.Initialize(agsbody_a->GetB2AgsBody(), agsbody_b->GetB2AgsBody());

    B2AgsJointMotor = dynamic_cast<b2MotorJoint *>(agsworld->B2AgsWorld->CreateJoint(&def));
    WorldID = agsworld->ID;
    B2bodyA_ID = agsbody_a->B2BodyID;
    B2bodyB_ID = agsbody_b->B2BodyID;
}

AgsJointMotor::AgsJointMotor(AgsWorld* agsworld,
        AgsBody* agsbody_a, AgsBody* agsbody_b,
        float32 correction_factor, bool collide_connected) {

    if (agsbody_a->World->B2AgsWorld != agsworld->B2AgsWorld ||
        agsbody_b->World->B2AgsWorld != agsworld->B2AgsWorld)
        return;


    b2MotorJointDef def;

    def.Initialize(agsbody_a->GetB2AgsBody(), agsbody_b->GetB2AgsBody());
    def.correctionFactor = correction_factor;
    def.collideConnected = collide_connected;

    B2AgsJointMotor = dynamic_cast<b2MotorJoint *>(agsworld->B2AgsWorld->CreateJoint(&def));
    WorldID = agsworld->ID;
    B2bodyA_ID = agsbody_a->B2BodyID;
    B2bodyB_ID = agsbody_b->B2BodyID;

}

AgsJointMotor::AgsJointMotor(b2MotorJoint* Motorjoint){
    B2AgsJointMotor = Motorjoint;
}


AgsJointMotor::~AgsJointMotor(void)
{
}

void AgsJointMotor::SetLinearOffset(float32 x, float32 y)
{
    B2AgsJointMotor->SetLinearOffset(Scale::ScaleDown(b2Vec2(x, y)));
}

void AgsJointMotor::SetLinearOffsetX(float32 x)
{
    B2AgsJointMotor->SetLinearOffset(b2Vec2(Scale::ScaleDown(x), B2AgsJointMotor->GetLinearOffset().y));
}

void AgsJointMotor::SetLinearOffsetY(float32 y)
{
    B2AgsJointMotor->SetLinearOffset(b2Vec2( B2AgsJointMotor->GetLinearOffset().x, Scale::ScaleDown(y)));
}

float32 AgsJointMotor::GetLinearOffsetX()
{
    return Scale::ScaleUp(B2AgsJointMotor->GetLinearOffset().x);
}

float32 AgsJointMotor::GetLinearOffsetY()
{
    return Scale::ScaleUp(B2AgsJointMotor->GetLinearOffset().y);
}

void AgsJointMotor::SetAngularOffset(float32 angularOffset)
{
    B2AgsJointMotor->SetAngularOffset(angularOffset);
}

float32 AgsJointMotor::GetAngularOffset()
{
    return B2AgsJointMotor->GetAngularOffset();
}

void AgsJointMotor::SetMaxForce(float32 force)
{
    B2AgsJointMotor->SetMaxForce(Scale::ScaleDown(force));
}

float32 AgsJointMotor::GetMaxForce()
{
    return Scale::ScaleUp(B2AgsJointMotor->GetMaxForce());
}

void AgsJointMotor::SetMaxTorque(float32 torque)
{
    B2AgsJointMotor->SetMaxTorque(Scale::ScaleDown(Scale::ScaleDown(torque)));
}

float32 AgsJointMotor::GetMaxTorque()
{
    return Scale::ScaleUp(Scale::ScaleUp(B2AgsJointMotor->GetMaxTorque()));
}

void AgsJointMotor::SetCorrectionFactor(float32 factor)
{
    B2AgsJointMotor->SetCorrectionFactor(factor);
}

float32 AgsJointMotor::GetCorrectionFactor()
{
    return B2AgsJointMotor->GetCorrectionFactor();
}

AgsBody* AgsJointMotor::GetBodyA() {
    return AgsJoint::GetBody(WorldID, B2AgsJointMotor->GetBodyA());
}

AgsBody* AgsJointMotor::GetBodyB() {
    return AgsJoint::GetBody(WorldID, B2AgsJointMotor->GetBodyB());
}

//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsJointMotorInterface AgsJointMotor_Interface;
AgsJointMotorReader AgsJointMotor_Reader;

const char* AgsJointMotorInterface::name = "JointMotor";

//------------------------------------------------------------------------------

int AgsJointMotorInterface::Dispose(const char* address, bool force)
{
    Book::UnregisterAgsJointMotorByID(((AgsJointMotor*)address)->ID);
    //delete ((AgsJointMotor*)address);
    return (1);
}

//------------------------------------------------------------------------------

int AgsJointMotorInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    AgsJointMotor* arr = (AgsJointMotor*)address;
    char* ptr = buffer;

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsJointMotorReader::Unserialize(int key, const char* serializedData, int dataSize)
{
//	AgsShapeCircle* arr = new AgsShapeCircle(0,0);

//	const char* ptr = serializedData;

//	engine->RegisterUnserializedObject(key, arr, &AgsShapeCircle_Interface);
}

//..............................................................................
