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

PointF* PointF::Rotate(float32 angle, float32 pivot_x, float32 pivot_y){
    float32 s = sin(angle);
    float32 c = cos(angle);

    // translate point back to origin:
    float32 p_x = _x - pivot_x;
    float32 p_y = _y - pivot_y;

    // rotate point
    float32 xnew = p_x * c - p_y * s;
    float32 ynew = p_x * s + p_y * c;

    // translate point back:
    p_x = xnew + pivot_x;
    p_y = ynew + pivot_y;
    return new PointF(p_x, p_y);
}


PointF* PointF::Scale(float32 scale){
    return new PointF(_x * scale, _y * scale);
}

float32 PointF::Length(){
    return sqrt(_x * _x + _y * _y);
}

float32 PointF::SquaredLength(){
    return (_x * _x + _y * _y);
}

PointF* PointF::Add(PointF* pointF) {
    if(pointF == nullptr) return new PointF(_x , _y );

    return new PointF(_x + pointF->GetX(), _y + pointF->GetY());
}

PointF* PointF::Sub(PointF* pointF) {
    if(pointF == nullptr) return new PointF(_x , _y );

    return new PointF(_x - pointF->GetX(), _y - pointF->GetY());
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
