/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsJointDistance.h"
#include "Scale.h"

AgsJointDistance::AgsJointDistance(AgsBody* agsbody_a, AgsBody* agsbody_b) {

}

AgsJointDistance::AgsJointDistance(b2DistanceJoint* distancejoint){
    B2AgsJointDistance = distancejoint;
}


AgsJointDistance::~AgsJointDistance(void)
{
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
