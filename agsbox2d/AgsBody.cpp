#include "AgsBody.h"
#include "AgsWorld.h"
#include "Scale.h"


AgsBody::AgsBody(AgsWorld* world, float32 x, float32 y, b2BodyType bodytype) {
	B2AgsBodyDef.position.Set(Scale::ScaleDown(x), Scale::ScaleDown(y));
	B2AgsBodyDef.type = bodytype;
	B2AgsBodyDef.fixedRotation = true;
	B2AgsBody = world->B2AgsWorld->CreateBody(&B2AgsBodyDef);
	World = world;
}

void AgsBody::ApplyForce(float32 force_x, float32 force_y) {
	B2AgsBody->ApplyForce(Scale::ScaleDown(b2Vec2(force_x, force_y)),B2AgsBody->GetPosition(), true);
}

float32 AgsBody::GetPosX() {
	return Scale::ScaleUp(B2AgsBody->GetPosition().x);
}

float32 AgsBody::GetPosY() {
	return Scale::ScaleUp(B2AgsBody->GetPosition().y);
}

void AgsBody::SetPosX(float32 x) {
	B2AgsBody->SetTransform(b2Vec2(Scale::ScaleDown(x), B2AgsBody->GetPosition().y), B2AgsBody->GetAngle());
}

void AgsBody::SetPosY(float32 y) {
	B2AgsBody->SetTransform(b2Vec2(B2AgsBody->GetPosition().x, Scale::ScaleDown(y)), B2AgsBody->GetAngle());
}

void AgsBody::SetPos(float32 x, float32 y) {
	B2AgsBody->SetTransform(Scale::ScaleDown(b2Vec2(x,y)), B2AgsBody->GetAngle());
}

AgsBody::~AgsBody(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsBodyInterface AgsBody_Interface;
AgsBodyReader AgsBody_Reader;

const char* AgsBodyInterface::name = "Body";

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
