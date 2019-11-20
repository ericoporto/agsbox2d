/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsJointMouse.h"
#include "Scale.h"

AgsJointMouse::AgsJointMouse(AgsBody* agsbody_a, AgsBody* agsbody_b) {

}

AgsJointMouse::AgsJointMouse(b2MouseJoint* Mousejoint){
    B2AgsJointMouse = Mousejoint;
}


AgsJointMouse::~AgsJointMouse(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsJointMouseInterface AgsJointMouse_Interface;
AgsJointMouseReader AgsJointMouse_Reader;

const char* AgsJointMouseInterface::name = "JointMouse";

//------------------------------------------------------------------------------

int AgsJointMouseInterface::Dispose(const char* address, bool force)
{
    //delete ((AgsShapeCircle*)address);
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
