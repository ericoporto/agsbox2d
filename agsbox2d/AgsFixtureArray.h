#pragma once
/*
  code based on  AGS Array plugin -- copyright (c) 2011 Ferry Timmers.  All rights reserved.
  the AGS Array plugin is provided under the Artistic License 2.0
*/


#ifndef _AGS_FIXTURE_ARRAY_H
#define _AGS_FIXTURE_ARRAY_H

#include <vector>
#include <string>

#include "AgsFixture.h"
#include "plugin/agsplugin.h"

//------------------------------------------------------------------------------

// int32 is defined in agsplugin.h as long

struct AgsFixtureArray
{
    int32 capacity;
    std::vector<AgsFixture*> data;

    // call it everytime the array length changes
    void _c() { capacity = data.capacity(); }

    AgsFixtureArray() : data() { _c(); }
    AgsFixtureArray(AgsFixtureArray* a) : data(a->data) { _c(); }
    AgsFixtureArray(int32 num, AgsFixture* ags_fixture) : data(num, ags_fixture) { _c(); }

    void clear() { data.clear(); _c(); }

    int32 empty() const { return data.empty(); }

    void erase(int32 pos) { data.erase(data.begin() + pos); _c(); }

    void erase(int32 first, int32 last) {
        data.erase(data.begin() + first,
                   data.begin() + last);
        _c();
    }

    void insert(int32 pos, AgsFixture* ags_fixture) {
        data.insert(data.begin() + pos,
                    ags_fixture); _c();
    }

    void insert(int32 pos, const AgsFixtureArray* src) {
        data.insert(data.begin() + pos,
                    src->data.begin(),
                    src->data.end());
        _c();
    }

    AgsFixture operator[](int32 pos) { return *data[pos]; _c(); }

    AgsFixture* pop()
    {
        AgsFixture* ags_fixture = *data.rbegin();
        data.pop_back();
        _c();
        return ags_fixture;
    }

    void push(AgsFixture* ags_fixture) { data.push_back(ags_fixture); _c(); }

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