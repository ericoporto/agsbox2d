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
public:
    AgsJointPulley(AgsBody* agsbody_a, AgsBody* agsbody_b);
    AgsJointPulley(b2PulleyJoint* Pulleyjoint);
    ~AgsJointPulley(void);
    b2PulleyJoint* B2AgsJointPulley;
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