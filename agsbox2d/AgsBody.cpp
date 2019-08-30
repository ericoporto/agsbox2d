#include "AgsBody.h"

AgsBody::AgsBody(b2World* world, float32 x, float32 y, b2BodyType bodytype) {
	B2AgsBodyDef.position.Set(x, y);
	B2AgsBodyDef.type = bodytype;
	B2AgsBodyDef.fixedRotation = true;
	B2AgsBody = world->CreateBody(&B2AgsBodyDef);
	B2AgsWorld = world;
}

void AgsBody::ApplyForce(float32 force_x, float32 force_y) {
	B2AgsBody->ApplyForce(b2Vec2(force_x, force_y),B2AgsBody->GetPosition(), true);
}

float32 AgsBody::GetPosX() {
	return B2AgsBody->GetPosition().x;
}

float32 AgsBody::GetPosY() {
	return B2AgsBody->GetPosition().y;
}

AgsBody::~AgsBody(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsBodyInterface AgsBody_Interface;
AgsBodyReader AgsBody_Reader;

const char* AgsBodyInterface::name = "AgsBody";

//------------------------------------------------------------------------------

int AgsBodyInterface::Dispose(const char* address, bool force)
{
	delete ((AgsBody*)address);
	return (1);
}

//------------------------------------------------------------------------------

int AgsBodyInterface::Serialize(const char* address, char* buffer, int bufsize)
{
	AgsBody* arr = (AgsBody*)address;
	char* ptr = buffer;
	
	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsBodyReader::Unserialize(int key, const char* serializedData, int dataSize)
{
	//AgsBody* arr = new AgsBody();

	//const char* ptr = serializedData;

	//engine->RegisterUnserializedObject(key, arr, &AgsBody_Interface);
}

//..............................................................................
