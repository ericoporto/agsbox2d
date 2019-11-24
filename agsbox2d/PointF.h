/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _POINTF_H
#define _POINTF_H

#include "Box2D.h"
#include "plugin/agsplugin.h"

class PointF
{
    float32 _x = 0.0;
    float32 _y = 0.0;
public:
    PointF(float32 x, float32 y);
    ~PointF(void);
    void SetX(float32 x);
    float32 GetX();
    void SetY(float32 y);
    float32 GetY();
};



//------------------------------------------------------------------------------
// AGS interface instances

class PointFInterface : public IAGSScriptManagedObject
{
public:
    static const char* name;

    PointFInterface() {};

    virtual int Dispose(const char* address, bool force);
    virtual const char* GetType() { return (name); }
    virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class PointFReader : public IAGSManagedObjectReader
{
public:

    PointFReader() {}

    virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern PointFInterface PointF_Interface;
extern PointFReader PointF_Reader;

//------------------------------------------------------------------------------

#endif /* _POINTF_H */

//............