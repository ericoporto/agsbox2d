/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_JOINT_MOUSE_H
#define _AGS_JOINT_MOUSE_H

#include "Box2D.h"
#include "plugin/agsplugin.h"
#include "AgsBody.h"

class AgsJointMouse
{
private:
    int32 WorldID;
    int32 B2bodyA_ID;
    int32 B2bodyB_ID;
public:
    AgsJointMouse(AgsWorld* agsworld, AgsBody* agsbody_a, float x, float y);
    AgsJointMouse(b2MouseJoint* Mousejoint);
    ~AgsJointMouse(void);
    b2MouseJoint* B2AgsJointMouse;

    AgsBody* GetBodyA();
    AgsBody* GetBodyB();

    void SetTarget(float32 x, float32 y);
    float32 GetTargetX();
    float32 GetTargetY();

    void SetMaxForce(float32 force);
    float32 GetMaxForce();

    void SetFrequency(float32 hz);
    float32 GetFrequency();

    void SetDampingRatio(float32 d);
    float32 GetDampingRatio();

    int32 ID;
};



//------------------------------------------------------------------------------
// AGS interface instances

class AgsJointMouseInterface : public IAGSScriptManagedObject
{
public:
    static const char* name;

    AgsJointMouseInterface() {};

    virtual int Dispose(const char* address, bool force);
    virtual const char* GetType() { return (name); }
    virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsJointMouseReader : public IAGSManagedObjectReader
{
public:

    AgsJointMouseReader() {}

    virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsJointMouseInterface AgsJointMouse_Interface;
extern AgsJointMouseReader AgsJointMouse_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_JOINT_MOUSE_H */

//..............................................................................