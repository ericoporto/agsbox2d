/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsJointMotor.h"
#include "Scale.h"

AgsJointMotor::AgsJointMotor(AgsBody* agsbody_a, AgsBody* agsbody_b) {

}

AgsJointMotor::AgsJointMotor(b2MotorJoint* Motorjoint){
    B2AgsJointMotor = Motorjoint;
}


AgsJointMotor::~AgsJointMotor(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsJointMotorInterface AgsJointMotor_Interface;
AgsJointMotorReader AgsJointMotor_Reader;

const char* AgsJointMotorInterface::name = "JointMotor";

//------------------------------------------------------------------------------

int AgsJointMotorInterface::Dispose(const char* address, bool force)
{
    //delete ((AgsShapeCircle*)address);
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
