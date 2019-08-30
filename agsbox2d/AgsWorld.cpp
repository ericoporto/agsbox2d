#include "AgsWorld.h"

AgsWorld::AgsWorld(float32 gravityX, float32 gravityY) {
	B2AgsWorld = new b2World(b2Vec2(gravityX, gravityY));
}


AgsBody* AgsWorld::NewBody(float32 x, float32 y) {
	return new AgsBody(B2AgsWorld, x, y);
}

AgsWorld::~AgsWorld(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsWorldInterface AgsWorld_Interface;
AgsWorldReader AgsWorld_Reader;

const char* AgsWorldInterface::name = "AgsWorld";

//------------------------------------------------------------------------------

int AgsWorldInterface::Dispose(const char* address, bool force)
{
	delete ((AgsWorld*)address);
	return (1);
}

//------------------------------------------------------------------------------

int AgsWorldInterface::Serialize(const char* address, char* buffer, int bufsize)
{
	AgsWorld* arr = (AgsWorld*)address;
	char* ptr = buffer;
	
	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsWorldReader::Unserialize(int key, const char* serializedData, int dataSize)
{
	AgsWorld* arr = new AgsWorld(0,0);

	const char* ptr = serializedData;

	engine->RegisterUnserializedObject(key, arr, &AgsWorld_Interface);
}

//..............................................................................
