/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_RAYCAST_RESULT_H
#define _AGS_RAYCAST_RESULT_H

#include "Box2D.h"
#include "plugin/agsplugin.h"
#include <vector>

class AgsFixture;

struct AgsFixtureData {
    int32 WorldID = -1;
    int32 B2FixtureID = -1;
};

struct RaycastResultItem {
    float32 PointX = 0.0;
    float32 PointY = 0.0;
    float32 NormalX = 0.0;
    float32 NormalY = 0.0;
    float32 Fraction = 0.0;
    AgsFixtureData FixtureData;
};

class AgsRaycastResult
{
public:
    std::vector<RaycastResultItem> RaycastResultList;
    AgsRaycastResult();
    ~AgsRaycastResult(void);

    int32 GetLength();
    AgsFixtureData GetFixtureData(int i);
    float32 GetNormalX(int i);
    float32 GetNormalY(int i);
    float32 GetPointX(int i);
    float32 GetPointY(int i);
    float32 GetFraction(int i);
};




//------------------------------------------------------------------------------
// AGS interface instances

class AgsRaycastResultInterface : public IAGSScriptManagedObject
{
public:
    static const char* name;

    AgsRaycastResultInterface() {};

    virtual int Dispose(const char* address, bool force);
    virtual const char* GetType() { return (name); }
    virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class AgsRaycastResultReader : public IAGSManagedObjectReader
{
public:

    AgsRaycastResultReader() {}

    virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsRaycastResultInterface AgsRaycastResult_Interface;
extern AgsRaycastResultReader AgsRaycastResult_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_RAYCAST_RESULT_H */

//............