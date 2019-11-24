/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "PointF.h"

PointF::PointF(float32 x, float32 y) {
    _x = x;
    _y = y;
}

PointF::~PointF(void)
{
}


void PointF::SetX(float32 x){
    _x = x;
}

float32 PointF::GetX(){
    return _x;
}

void PointF::SetY(float32 y){
    _y = y;
}

float32 PointF::GetY(){
    return _y;
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

PointFInterface PointF_Interface;
PointFReader PointF_Reader;

const char* PointFInterface::name = "PointF";

//------------------------------------------------------------------------------
#include "SerialHelper.h"
using namespace SerialHelper;

int PointFInterface::Dispose(const char* address, bool force)
{
    delete ((PointF*)address);
    return (1);
}

//------------------------------------------------------------------------------

int PointFInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    PointF* point_f = (PointF*)address;
    char* ptr = buffer;
    char* end = buffer + bufsize;

    ptr = FloatToChar(point_f->GetX(), ptr, end);
    ptr = FloatToChar(point_f->GetY(), ptr, end);

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void PointFReader::Unserialize(int key, const char* serializedData, int dataSize)
{
    char* ptr = (char*) serializedData;
    int point_f_id = key;


    float val_x;
    float val_y;

    ptr = CharToFloat( val_x, ptr);
    ptr = CharToFloat( val_y, ptr);

    PointF* point_f = new PointF(val_x, val_y);

    engine->RegisterUnserializedObject(key, point_f, &PointF_Interface);
}

//..............................................................................
