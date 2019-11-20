/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsJointPulley.h"
#include "Scale.h"

AgsJointPulley::AgsJointPulley(AgsBody* agsbody_a, AgsBody* agsbody_b) {

}

AgsJointPulley::AgsJointPulley(b2PulleyJoint* Pulleyjoint){
    B2AgsJointPulley = Pulleyjoint;
}


AgsJointPulley::~AgsJointPulley(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsJointPulleyInterface AgsJointPulley_Interface;
AgsJointPulleyReader AgsJointPulley_Reader;

const char* AgsJointPulleyInterface::name = "JointPulley";

//------------------------------------------------------------------------------

int AgsJointPulleyInterface::Dispose(const char* address, bool force)
{
    //delete ((AgsShapeCircle*)address);
    return (1);
}

//------------------------------------------------------------------------------

int AgsJointPulleyInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    AgsJointPulley* arr = (AgsJointPulley*)address;
    char* ptr = buffer;

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsJointPulleyReader::Unserialize(int key, const char* serializedData, int dataSize)
{
//	AgsShapeCircle* arr = new AgsShapeCircle(0,0);

//	const char* ptr = serializedData;

//	engine->RegisterUnserializedObject(key, arr, &AgsShapeCircle_Interface);
}

//..............................................................................