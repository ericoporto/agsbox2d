/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_CONTACT_H
#define _AGS_CONTACT_H

#include "Box2D.h"
#include "plugin/agsplugin.h"

class PointF; // forward declaration, we need PointF.h on the cpp

class AgsContact
{
    b2Contact* B2AgsContact = nullptr;
public:
    AgsContact(b2Contact* contact, int32 world_id);
    AgsContact(int32 world_id, int32 contact_index = -1, bool valid = true);
    ~AgsContact(void);

    void Invalidate();
    bool IsValid();
    PointF* GetNormal();
    int32 GetPositionsCount();
    PointF* GetPosition(int32 i);
    b2Fixture* GetB2FixtureA();
    b2Fixture* GetB2FixtureB();
    b2Contact* GetB2AgsContact();

    float32 GetFriction();
    float32 GetRestitution();
    bool IsEnabled();
    bool IsTouching();
    void SetFriction(float32 friction);
    void SetRestitution(float32 restitution);
    void SetEnabled(bool enabled);
    void ResetFriction();
    void ResetRestitution();
    void SetTangentSpeed(float32 speed);
    float32 GetTangentSpeed();

    int32 ID;
    int32 WorldID;
};



//------------------------------------------------------------------------------
// AGS interface instances

class AgsContactInterface : public IAGSScriptManagedObject
{
public:
    static const char* name;

    AgsContactInterface() {};

    virtual int Dispose(const char* address, bool force);
    virtual const char* GetType() { return (name); }
    virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsContactReader : public IAGSManagedObjectReader
{
public:

    AgsContactReader() {}

    virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsContactInterface AgsContact_Interface;
extern AgsContactReader AgsContact_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_CONTACT_H */

//..............................................................................