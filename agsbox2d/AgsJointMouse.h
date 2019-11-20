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
public:
    AgsJointMouse(AgsBody* agsbody_a, AgsBody* agsbody_b);
    AgsJointMouse(b2MouseJoint* Mousejoint);
    ~AgsJointMouse(void);
    b2MouseJoint* B2AgsJointMouse;
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