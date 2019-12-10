/*
  code based on  AGS Array plugin -- copyright (c) 2011 Ferry Timmers.  All rights reserved.
  the AGS Array plugin is provided under the Artistic License 2.0
*/

/***************************************************************
 * STL vector wrapper                                          *
 ***************************************************************/

#include "AgsFixtureArray.h"
#include "Book.h"
#include "AgsWorld.h"

//------------------------------------------------------------------------------

extern IAGSEngine* engine;

AgsFixtureArrayInterface AgsFixtureArray_Interface;
AgsFixtureArrayReader AgsFixtureArray_Reader;

const char* AgsFixtureArrayInterface::name = "FixtureArray";

//------------------------------------------------------------------------------
#include "SerialHelper.h"
using namespace SerialHelper;

int AgsFixtureArrayInterface::Dispose(const char* address, bool force)
{
    Book::UnregisterAgsFixtureByID(((AgsFixtureArray*)address)->ID);
    delete ((AgsFixtureArray*)address);
    AgsFixtureArray* agsFixtureArray = ((AgsFixtureArray*)address);
    agsFixtureArray = nullptr;
    return (1);
}

//------------------------------------------------------------------------------

int AgsFixtureArrayInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    AgsFixtureArray* agsFixtureArray = (AgsFixtureArray*)address;
    char* ptr = buffer;
    char* end = buffer + bufsize;
    int32 arr_length = agsFixtureArray->size();

    ptr = IntToChar( arr_length,ptr, end);

    std::vector<AgsFixtureArrayData>::iterator it;
    for (it = agsFixtureArray->data.begin(); it != agsFixtureArray->data.end(); ++it)
    {
        AgsFixtureArrayData fad = (*it);

        int32 world_id = fad.WorldID;
        int32 fixture_id = -1;
        int32 b2body_id = -1;
        if(fad.B2Fixture != nullptr && fad.B2Fixture->GetBody() != nullptr){
            fixture_id = Book::b2FixtureToID(world_id, fad.B2Fixture);
            b2body_id = Book::b2BodyToID(world_id, fad.B2Fixture->GetBody());
        }

        ptr = IntToChar(world_id, ptr, end);
        ptr = IntToChar(b2body_id, ptr, end);
        ptr = IntToChar(fixture_id, ptr, end);
    }

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsFixtureArrayReader::Unserialize(int key, const char* serializedData, int dataSize)
{

    char* ptr = (char*)serializedData;
    AgsFixtureArray* agsFixtureArray = nullptr;

    if(Book::isAgsFixtureArrayRegisteredByID(key)){
        agsFixtureArray = Book::IDtoAgsFixtureArray(key);
    } else {
        agsFixtureArray = new AgsFixtureArray();

        int arr_length;
        ptr = CharToInt(arr_length, ptr);

        int i = 0;

        while (ptr - serializedData < dataSize && i < arr_length) {

            int32 world_id;
            int32 b2body_id;
            int32 fixture_id;

            ptr = CharToInt(world_id, ptr);
            ptr = CharToInt(b2body_id, ptr);
            ptr = CharToInt(fixture_id, ptr);

            AgsWorld * world;
            if (Book::isAgsWorldRegisteredByID(world_id)) {
                world = Book::IDtoAgsWorld(world_id);
                if(world == nullptr) throw;
            }
            else {
                world = new AgsWorld(0, 0);
                Book::RegisterAgsWorld(world_id, world);
            }

            AgsFixture *agsFixture = new AgsFixture(world_id, b2body_id, fixture_id);

            agsFixtureArray->push(agsFixture);

            i++;
        }
        agsFixtureArray->ID = key;
        Book::RegisterAgsFixtureArray(key, agsFixtureArray);
    }


    engine->RegisterUnserializedObject(key, agsFixtureArray, &AgsFixtureArray_Interface);
}

//..............................................................................

