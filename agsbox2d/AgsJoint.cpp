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
#include "AgsWorld.h"


AgsJoint::AgsJoint(b2Joint* b2joint) {

}

AgsJoint::AgsJoint(AgsWorld* agsworld, AgsBody* agsbody_a){
    if (agsbody_a->World->B2AgsWorld != agsworld->B2AgsWorld )
        return;

    WorldID = agsworld->ID;
    B2bodyA_ID = agsbody_a->B2BodyID;
    B2bodyB_ID = -1;
}

AgsJoint::AgsJoint(AgsWorld* agsworld, AgsBody* agsbody_a, AgsBody* agsbody_b){
    if (agsbody_a->World->B2AgsWorld != agsworld->B2AgsWorld ||
        agsbody_b->World->B2AgsWorld != agsworld->B2AgsWorld)
        return;

    WorldID = agsworld->ID;
    B2bodyA_ID = agsbody_a->B2BodyID;
    B2bodyB_ID = agsbody_b->B2BodyID;
}

AgsJoint::~AgsJoint(void)
{
}

int32 AgsJoint::GetType() {
    if(B2AgsJoint->GetType() == e_unknownJoint) {
        return eJointUnknown;
    }
    if(B2AgsJoint->GetType() == e_revoluteJoint) {
        return eJointRevolute;
    }
    if(B2AgsJoint->GetType() == e_prismaticJoint) {
        return eJointPrismatic;
    }
    if(B2AgsJoint->GetType() == e_distanceJoint) {
        return eJointDistance;
    }
    if(B2AgsJoint->GetType() == e_pulleyJoint) {
        return eJointPulley;
    }
    if(B2AgsJoint->GetType() == e_mouseJoint) {
        return eJointMouse;
    }
    if(B2AgsJoint->GetType() == e_gearJoint) {
        return eJointGear;
    }
    if(B2AgsJoint->GetType() == e_wheelJoint) {
        return eJointWheel;
    }
    if(B2AgsJoint->GetType() == e_weldJoint) {
        return eJointWeld;
    }
    if(B2AgsJoint->GetType() == e_frictionJoint) {
        return eJointFriction;
    }
    if(B2AgsJoint->GetType() == e_ropeJoint) {
        return eJointRope;
    }
    if(B2AgsJoint->GetType() == e_motorJoint) {
        return eJointMotor;
    }
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

    AgsWorld* agsworld = Book::IDtoAgsWorld(WorldID);
    AgsBody* agsbody = agsworld->findObject(b2body);

    if(agsbody!= nullptr)
        return  agsbody;


    agsbody = new AgsBody(false);

    agsbody->B2BodyID = Book::b2BodyToID(WorldID, b2body);
    agsbody->World = agsworld;
    agsbody->ID = -1;

    return  agsbody;
}

AgsBody* AgsJoint::GetBodyB() {
    b2Body *b2body = B2AgsJoint->GetBodyB();
    if (b2body == nullptr)
        return nullptr;

    AgsWorld* agsworld = Book::IDtoAgsWorld(WorldID);
    AgsBody* agsbody = agsworld->findObject(b2body);

    if(agsbody!= nullptr)
        return  agsbody;


    agsbody = new AgsBody(false);

    agsbody->B2BodyID = Book::b2BodyToID(WorldID, b2body);
    agsbody->World = agsworld;
    agsbody->ID = -1;

    return  agsbody;
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
