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

AgsJoint::AgsJoint(AgsJointDistance* ags_joint_distance) {
    JointDistance = ags_joint_distance;
    JointMotor = nullptr;
    JointMouse = nullptr;
    JointPulley = nullptr;
    WorldID = ags_joint_distance->WorldID;
    B2bodyA_ID = ags_joint_distance->B2bodyA_ID;
    B2bodyB_ID = ags_joint_distance->B2bodyB_ID;

    B2AgsJoint = dynamic_cast<b2Joint *>(ags_joint_distance->GetB2AgsJointDistance());
}

AgsJoint::AgsJoint(AgsJointMotor* ags_joint_motor) {
    JointMotor = ags_joint_motor;
    JointDistance = nullptr;
    JointMouse = nullptr;
    JointPulley = nullptr;
    WorldID = ags_joint_motor->WorldID;
    B2bodyA_ID = ags_joint_motor->B2bodyA_ID;
    B2bodyB_ID = ags_joint_motor->B2bodyB_ID;

    B2AgsJoint = dynamic_cast<b2Joint *>(ags_joint_motor->GetB2AgsJointMotor());
}

AgsJoint::AgsJoint(AgsJointMouse* ags_joint_mouse) {
    JointMouse = ags_joint_mouse;
    JointDistance = nullptr;
    JointMotor = nullptr;
    JointPulley = nullptr;
    WorldID = ags_joint_mouse->WorldID;
    B2bodyA_ID = ags_joint_mouse->B2bodyA_ID;
    B2bodyB_ID = ags_joint_mouse->B2bodyB_ID;

    B2AgsJoint = dynamic_cast<b2Joint *>(ags_joint_mouse->GetB2AgsJointMouse());
}

AgsJoint::AgsJoint(AgsJointPulley* ags_joint_pulley) {
    JointPulley = ags_joint_pulley;
    JointDistance = nullptr;
    JointMotor = nullptr;
    JointMouse = nullptr;
    WorldID = ags_joint_pulley->WorldID;
    B2bodyA_ID = ags_joint_pulley->B2bodyA_ID;
    B2bodyB_ID = ags_joint_pulley->B2bodyB_ID;

    B2AgsJoint = dynamic_cast<b2Joint *>(ags_joint_pulley->GetB2AgsJointPulley());
}

AgsJoint::~AgsJoint(void)
{
}

void AgsJoint::InitializeIfNeeded(){
    if(B2AgsJoint == nullptr) {
        B2AgsJoint = Book::IDtoB2Joint(WorldID, b2Joint_ID);
    }
}

int32 AgsJoint::GetType() {
    InitializeIfNeeded();
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
    InitializeIfNeeded();
    return  B2AgsJoint!=nullptr;
}

bool AgsJoint::isActive(){
    InitializeIfNeeded();
    return  B2AgsJoint->IsActive();
}

b2Joint* AgsJoint::GetB2AgsJoint(){
    InitializeIfNeeded();
    return  B2AgsJoint;
}

b2Body* AgsJoint::GetBodyA() {
    InitializeIfNeeded();
    return B2AgsJoint->GetBodyA();
}

b2Body* AgsJoint::GetBodyB() {
    InitializeIfNeeded();
    return B2AgsJoint->GetBodyB();
}


AgsWorld* AgsJoint::GetAgsWorld() {
    InitializeIfNeeded();
    return Book::IDtoAgsWorld(WorldID);
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
    Book::UnregisterAgsJointByID(((AgsJoint*)address)->ID);
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
