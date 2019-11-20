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
#include "AgsShapeCircle.h"
#include "AgsShapeRect.h"

class AgsJoint
{
public:
    AgsJoint(b2Joint* b2joint);
    ~AgsJoint(void);
    b2Joint* B2AgsJoint;
    int32 ID;
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