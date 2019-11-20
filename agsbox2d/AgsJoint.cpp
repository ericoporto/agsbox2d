/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsJoint.h"
#include "Book.h"


AgsJoint::AgsJoint(b2Joint* b2joint) {

}

AgsJoint::AgsJoint(AgsBody* agsbody_a){

}

AgsJoint::AgsJoint(AgsBody* agsbody_a, AgsBody* agsbody_b){

}

AgsJoint::~AgsJoint(void)
{
}

int32 AgsJoint::GetType() {
    return  B2AgsJoint->GetType();
}

bool AgsJoint::isValid(){
    return  B2AgsJoint!=nullptr;
}

bool AgsJoint::isActive(){
    return  B2AgsJoint->IsActive();
}

AgsBody* AgsJoint::GetBodyA() {
    b2Body *b2body = B2AgsJoint->GetBodyA();
    if (b2body == nullptr)
        return nullptr;


}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsJointInterface AgsJoint_Interface;
AgsJointReader AgsJoint_Reader;

const char* AgsJointInterface::name = "Joint";

//------------------------------------------------------------------------------
#include "SerialHelper.h"
using namespace SerialHelper;

int AgsJointInterface::Dispose(const char* address, bool force)
{
   // Book::UnregisterAgsAgsJointByID(((AgsJoint*)address)->ID);
    //((AgsShape*)address)->B2AgsShape->~b2Shape();
   // delete ((AgsJoint*)address)->B2AgsJoint;
    delete ((AgsJoint*)address);
    return (1);
}

//------------------------------------------------------------------------------

int AgsJointInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    AgsJoint* agsjoint = (AgsJoint*)address;
    char* ptr = buffer;
    char* end = buffer + bufsize;

    //ptr = b2ShapeToChar(shape->B2AgsShape, ptr, end);

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsJointReader::Unserialize(int key, const char* serializedData, int dataSize)
{
    char* ptr = (char*) serializedData;
    int joint_id = key;

    b2Joint * joint = nullptr;

   // ptr = CharTob2Shape(&shape, ptr);
    AgsJoint* agsjoint = new AgsJoint(joint);
    agsjoint->ID = joint_id;
    //Book::RegisterAgsShape(joint_id, agsjoint);

    engine->RegisterUnserializedObject(key, agsjoint, &AgsJoint_Interface);
}

//..............................................................................
