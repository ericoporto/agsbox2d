/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_FIXTURE_ARRAY_H
#define _AGS_FIXTURE_ARRAY_H

#include <vector>

#include "AgsFixture.h"
#include "plugin/agsplugin.h"

//------------------------------------------------------------------------------

// int32 is defined in agsplugin.h as long

struct AgsFixtureArrayData {
    b2World* B2World = nullptr;
    int32 WorldID = -1;
    b2Fixture* B2Fixture = nullptr;
};

class AgsFixtureArray
{
public:
    int32 ID = -1;
    int32 capacity = 0;

    //b2world pointer, world_id, b2Fixture pointer
    std::vector<AgsFixtureArrayData> data;

    // call it everytime the array length changes
    void _c() { capacity = data.capacity(); }

    AgsFixtureArray() : data() { _c(); }
    AgsFixtureArray(AgsFixtureArray* a) : data(a->data) { _c(); }

    void clear() { data.clear(); _c(); }

    int32 empty() const { return data.empty(); }

    void erase(int32 pos) { data.erase(data.begin() + pos); _c(); }

    void erase(int32 first, int32 last) {
        data.erase(data.begin() + first,
                   data.begin() + last);
        _c();
    }

    void set_item(int32 pos, AgsFixture* ags_fixture) {
        AgsFixtureArrayData fad;
        fad.WorldID = ags_fixture->WorldID;
        fad.B2World = ags_fixture->GetB2Body()->GetWorld();
        fad.B2Fixture = ags_fixture->GetB2AgsFixture();
        data[pos] = fad;
    }

    void insert(int32 pos, AgsFixture* ags_fixture) {
        AgsFixtureArrayData fad;
        fad.WorldID = ags_fixture->WorldID;
        fad.B2World = ags_fixture->GetB2Body()->GetWorld();
        fad.B2Fixture = ags_fixture->GetB2AgsFixture();
        data.insert(data.begin() + pos, fad);
        _c();
    }

    void insert(int32 pos, const AgsFixtureArray* src) {
        data.insert(data.begin() + pos,
                    src->data.begin(),
                    src->data.end());
        _c();
    }

    AgsFixtureArrayData operator[](int32 pos) { return data[pos]; _c(); }

    AgsFixtureArrayData pop()
    {
        AgsFixtureArrayData fad = *data.rbegin();
        data.pop_back();
        _c();
        return fad;
    }

    void push(AgsFixture* ags_fixture) {
        AgsFixtureArrayData fad;
        fad.WorldID = ags_fixture->WorldID;
        fad.B2World = ags_fixture->GetB2Body()->GetWorld();
        fad.B2Fixture = ags_fixture->GetB2AgsFixture();
        data.push_back(fad);
        _c();
    }

    void push_fad(AgsFixtureArrayData fad) {
        data.push_back(fad);
        _c();
    }

    int32 size() const { return data.size(); }

    void resize(int32 num) { data.resize(num); _c(); }

    void reserve(int32 num) { data.reserve(num); _c(); }

    // Excuse to use static
    static void swap(AgsFixtureArray* a, AgsFixtureArray* b)
    {
        a->data.swap(b->data);
        a->_c();
        b->_c();
    }
};

//------------------------------------------------------------------------------
// AGS interface instances

class AgsFixtureArrayInterface : public IAGSScriptManagedObject
{
public:
    static const char* name;

    AgsFixtureArrayInterface() {};

    virtual int Dispose(const char* address, bool force);
    virtual const char* GetType() { return (name); }
    virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class AgsFixtureArrayReader : public IAGSManagedObjectReader
{
public:

    AgsFixtureArrayReader() {}

    virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsFixtureArrayInterface AgsFixtureArray_Interface;
extern AgsFixtureArrayReader AgsFixtureArray_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_FIXTURE_ARRAY_H */

//..............................................................................