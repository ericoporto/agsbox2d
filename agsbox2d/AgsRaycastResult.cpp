/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsRaycastResult.h"
#include "AgsFixture.h"
#include "Book.h"
#include "Scale.h"

AgsRaycastResult::AgsRaycastResult() {
}

AgsRaycastResult::~AgsRaycastResult(void)
{
}


int32 AgsRaycastResult::GetLength(){
    return RaycastResultList.size();
}

AgsFixtureData AgsRaycastResult::GetFixtureData(int i){
    if(i<0 || i>= RaycastResultList.size())
        return AgsFixtureData();

    return RaycastResultList[i].FixtureData;
}

float32 AgsRaycastResult::GetNormalX(int i){
    if(i<0 || i>= RaycastResultList.size())
        return 0.0;

    return RaycastResultList[i].NormalX;
}

float32 AgsRaycastResult::GetNormalY(int i){
    if(i<0 || i>= RaycastResultList.size())
        return 0.0;

    return RaycastResultList[i].NormalY;
}

float32 AgsRaycastResult::GetPointX(int i){
    if(i<0 || i>= RaycastResultList.size())
        return 0.0;

    return Scale::ScaleUp( RaycastResultList[i].PointX);
}

float32 AgsRaycastResult::GetPointY(int i){
    if(i<0 || i>= RaycastResultList.size())
        return 0.0;

    return Scale::ScaleUp( RaycastResultList[i].PointY);
}

float32 AgsRaycastResult::GetFraction(int i){
    if(i<0 || i>= RaycastResultList.size())
        return 0.0;

    return RaycastResultList[i].Fraction;
}

//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsRaycastResultInterface AgsRaycastResult_Interface;
AgsRaycastResultReader AgsRaycastResult_Reader;

const char* AgsRaycastResultInterface::name = "AgsRaycastResult";

//------------------------------------------------------------------------------
#include "SerialHelper.h"
#include "AgsWorld.h"

using namespace SerialHelper;

int AgsRaycastResultInterface::Dispose(const char* address, bool force)
{
    delete ((AgsRaycastResult*)address);
    return (1);
}

//------------------------------------------------------------------------------

int AgsRaycastResultInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    AgsRaycastResult* arr = (AgsRaycastResult*)address;
    char* ptr = buffer;
    char* end = buffer + bufsize;

    ptr = IntToChar(arr->GetLength(), ptr, end);

    std::vector<RaycastResultItem>::iterator it;
    for (it = arr->RaycastResultList.begin(); it != arr->RaycastResultList.end(); ++it) {

        ptr = FloatToChar(it->PointX, ptr, end);
        ptr = FloatToChar(it->PointY, ptr, end);
        ptr = FloatToChar(it->NormalX, ptr, end);
        ptr = FloatToChar(it->NormalY, ptr, end);
        ptr = FloatToChar(it->Fraction, ptr, end);

        AgsFixtureData fad = it->FixtureData;

        int32 world_id = fad.WorldID;
        int32 fixture_id = fad.B2FixtureID;

        ptr = IntToChar(world_id, ptr, end);
        ptr = IntToChar(fixture_id, ptr, end);
    }

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsRaycastResultReader::Unserialize(int key, const char* serializedData, int dataSize)
{
    char* ptr = (char*) serializedData;
    int arr_id = key;
    int32 arr_length;

    AgsRaycastResult* arr = new AgsRaycastResult();

    ptr = CharToInt(arr_length, ptr);
    arr->RaycastResultList.reserve(arr_length);
    int i=0;

    while (ptr - serializedData < dataSize && i < arr_length) {

        float32 pointx;
        float32 pointy;
        float32 normalx;
        float32 normaly;
        float32 fraction;
        int32 world_id;
        int32 b2body_id;
        int32 fixture_id;

        ptr = CharToFloat(pointx, ptr);
        ptr = CharToFloat(pointy, ptr);
        ptr = CharToFloat(normalx, ptr);
        ptr = CharToFloat(normaly, ptr);
        ptr = CharToFloat(fraction, ptr);

        ptr = CharToInt(world_id, ptr);
        ptr = CharToInt(fixture_id, ptr);

        RaycastResultItem rritem;
        rritem.PointX = pointx;
        rritem.PointY = pointy;
        rritem.NormalX = normalx;
        rritem.NormalY = normaly;
        rritem.Fraction = fraction;

        rritem.FixtureData.WorldID = world_id;
        rritem.FixtureData.B2FixtureID = fixture_id;

        i++;
    }

    engine->RegisterUnserializedObject(arr_id, arr, &AgsRaycastResult_Interface);
}

//..............................................................................
