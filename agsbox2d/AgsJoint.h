/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_JOINT_H
#define _AGS_JOINT_H

#include "Box2D.h"
#include "plugin/agsplugin.h"
#include "AgsBody.h"
#include "AgsJointDistance.h"
#include "AgsJointMotor.h"
#include "AgsJointMouse.h"
#include "AgsJointPulley.h"

enum AgsJointType {
    eJointUnknown = 0,
    eJointRevolute = 1,
    eJointPrismatic = 2,
    eJointDistance = 3,
    eJointPulley = 4,
    eJointMouse = 5,
    eJointGear = 6,
    eJointWheel = 7,
    eJointWeld = 8,
    eJointFriction = 9,
    eJointRope = 10,
    eJointMotor = 11,
};

class AgsJoint
{
    int32 WorldID;
    int32 B2bodyA_ID;
    int32 B2bodyB_ID;
public:
    static AgsBody*  GetBody(int world_id, b2Body* body);

    AgsJoint(b2Joint* b2joint);
    AgsJoint(AgsWorld* agsworld, AgsBody* agsbody_a);
    AgsJoint(AgsWorld* agsworld, AgsBody* agsbody_a, AgsBody* agsbody_b);
    ~AgsJoint(void);
    b2Joint* B2AgsJoint;
    int32 ID;

    int32 GetType();
    bool isValid();
    bool isActive();

    AgsBody* GetBodyA();
    AgsBody* GetBodyB();
};



//------------------------------------------------------------------------------
// AGS interface instances

class AgsJointInterface : public IAGSScriptManagedObject
{
public:
    static const char* name;

    AgsJointInterface() {};

    virtual int Dispose(const char* address, bool force);
    virtual const char* GetType() { return (name); }
    virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsJointReader : public IAGSManagedObjectReader
{
public:

    AgsJointReader() {}

    virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsJointInterface AgsJoint_Interface;
extern AgsJointReader AgsJoint_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_JOINT_H */

//..............................................................................