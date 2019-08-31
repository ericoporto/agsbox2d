#include "AgsWorld.h"
#include "AgsBody.h"
#include "Scale.h"


AgsWorld::AgsWorld(float32 gravityX, float32 gravityY) {
	B2AgsWorld = new b2World(Scale::ScaleDown(b2Vec2(gravityX, gravityY)));
}


AgsBody* AgsWorld::NewBody(float32 x, float32 y, b2BodyType bodytype) {
	return new AgsBody(this, x, y, bodytype);
}

void AgsWorld::Step(float32 dt, int32 velocityIterations, int32 positionIterations) {
	B2AgsWorld->Step(dt, velocityIterations, positionIterations);
}

AgsWorld::~AgsWorld(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsWorldInterface AgsWorld_Interface;
AgsWorldReader AgsWorld_Reader;

const char* AgsWorldInterface::name = "World";

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
