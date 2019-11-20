/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_JOINT_MOTOR_H
#define _AGS_JOINT_MOTOR_H

#include "Box2D.h"
#include "plugin/agsplugin.h"
#include "AgsBody.h"

class AgsJointMotor
{
public:
    AgsJointMotor(AgsWorld* agsworld, AgsBody* agsbody_a, AgsBody* agsbody_b, float32 correction_factor, bool collide_connected);
    AgsJointMotor(AgsWorld* agsworld, AgsBody* agsbody_a, AgsBody* agsbody_b);
    AgsJointMotor(b2MotorJoint* Motorjoint);
    ~AgsJointMotor(void);
    b2MotorJoint* B2AgsJointMotor;

    void SetLinearOffset(float x, float y);
    float32 GetLinearOffsetX();
    float32 GetLinearOffsetY();

    void SetAngularOffset(float angularOffset);
    float32 GetAngularOffset();

    void SetMaxForce(float force);
    float32 GetMaxForce();

    void SetMaxTorque(float torque);
    float32 GetMaxTorque();

    void SetCorrectionFactor(float factor);
    float32 GetCorrectionFactor();
};



//------------------------------------------------------------------------------
// AGS interface instances

class AgsJointMotorInterface : public IAGSScriptManagedObject
{
public:
    static const char* name;

    AgsJointMotorInterface() {};

    virtual int Dispose(const char* address, bool force);
    virtual const char* GetType() { return (name); }
    virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsJointMotorReader : public IAGSManagedObjectReader
{
public:

    AgsJointMotorReader() {}

    virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsJointMotorInterface AgsJointMotor_Interface;
extern AgsJointMotorReader AgsJointMotor_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_JOINT_MOTOR_H */

//..............................................................................