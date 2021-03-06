/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_JOINT_DISTANCE_H
#define _AGS_JOINT_DISTANCE_H

#include "Box2D.h"
#include "plugin/agsplugin.h"
#include "AgsBody.h"

class AgsJointDistance
{
private:
    b2DistanceJoint* B2AgsJointDistance = nullptr;
public:
    AgsJointDistance(AgsWorld* agsworld, AgsBody* agsbody_a, AgsBody* agsbody_b, float32 x1, float32 y1, float32 x2, float32 y2, bool collideConnected);
    AgsJointDistance(int32 world_id, b2DistanceJoint* distancejoint);
    ~AgsJointDistance(void);

    void InitializeIfNeeded();
    b2DistanceJoint* GetB2AgsJointDistance();

    void SetLength(float32 length);
    float32  GetLength();

    void SetFrequency(float32 hz);
    float32  GetFrequency();

    void SetDampingRatio(float32 dratio);
    float32  GetDampingRatio();


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

class AgsJointDistanceInterface : public IAGSScriptManagedObject
{
public:
    static const char* name;

    AgsJointDistanceInterface() {};

    virtual int Dispose(const char* address, bool force);
    virtual const char* GetType() { return (name); }
    virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsJointDistanceReader : public IAGSManagedObjectReader
{
public:

    AgsJointDistanceReader() {}

    virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsJointDistanceInterface AgsJointDistance_Interface;
extern AgsJointDistanceReader AgsJointDistance_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_JOINT_DISTANCE_H */

//..............................................................................