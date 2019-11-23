/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_JOINT_PULLEY_H
#define _AGS_JOINT_PULLEY_H

#include "Box2D.h"
#include "plugin/agsplugin.h"
#include "AgsBody.h"

class AgsJointPulley
{
private:
    b2PulleyJoint* B2AgsJointPulley;
public:
    AgsJointPulley(AgsWorld* agsworld, AgsBody* agsbody_a, AgsBody* agsbody_b,
                   float32 ground_anchor_a_x, float32 ground_anchor_a_y, float32 ground_anchor_b_x, float32 ground_anchor_b_y,
                   float32 anchor_a_x, float32 anchor_a_y, float32 anchor_b_x, float32 anchor_b_y,
                   float32 ratio, bool collide_connected
    );
    AgsJointPulley(b2PulleyJoint* Pulleyjoint);
    ~AgsJointPulley(void);

    void InitializeIfNeeded();
    b2PulleyJoint* GetB2AgsJointPulley();

    float32 GetLengthA();
    float32 GetLengthB();
    float32 GetRatio();

    b2Body* GetBodyA();
    b2Body* GetBodyB();

    int32 ID;
    int32 b2Joint_ID;

    int32 WorldID;
    int32 B2bodyA_ID;
    int32 B2bodyB_ID;
};



//------------------------------------------------------------------------------
// AGS interface instances

class AgsJointPulleyInterface : public IAGSScriptManagedObject
{
public:
    static const char* name;

    AgsJointPulleyInterface() {};

    virtual int Dispose(const char* address, bool force);
    virtual const char* GetType() { return (name); }
    virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsJointPulleyReader : public IAGSManagedObjectReader
{
public:

    AgsJointPulleyReader() {}

    virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsJointPulleyInterface AgsJointPulley_Interface;
extern AgsJointPulleyReader AgsJointPulley_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_JOINT_PULLEY_H */

//..............................................................................