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
	B2AgsBody->ApplyForceToCenter(Scale::ScaleDown(b2Vec2(force_x, force_y)), true);
}

void AgsBody::SetLinearVelocity(float32 vel_x, float32 vel_y) {
	B2AgsBody->SetLinearVelocity(Scale::ScaleDown(b2Vec2(vel_x, vel_y)));
}

void AgsBody::ApplyAngularImpulse(float32 impulse) {
	B2AgsBody->ApplyAngularImpulse(Scale::ScaleDown(impulse), true);
}

float32 AgsBody::GetLinearVelocityX() {
	return Scale::ScaleUp(B2AgsBody->GetLinearVelocity().x);
}

float32 AgsBody::GetLinearVelocityY() {
	return Scale::ScaleUp(B2AgsBody->GetLinearVelocity().y);
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

bool AgsBody::GetFixedRotation() {
	return B2AgsBody->IsFixedRotation();
}

void AgsBody::SetFixedRotation(bool fixed) {
	B2AgsBody->SetFixedRotation(fixed);
}

float32 AgsBody::GetAngle() {
	return B2AgsBody->GetAngle();
}

void AgsBody::SetAngle(float32 angle) {
	B2AgsBody->SetTransform(B2AgsBody->GetPosition(), angle);
}

float32 AgsBody::GetLinearDamping() {
	return B2AgsBody->GetLinearDamping();
}

void AgsBody::SetLinearDamping(float32 ldamping) {
	B2AgsBody->SetLinearDamping(ldamping);
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
	//if (((AgsBody*)address)->World != NULL && 
	//	((AgsBody*)address)->World->B2AgsWorld != NULL && 
	//	((AgsBody*)address)->B2AgsBody != NULL &&
	//	((AgsBody*)address)->World->B2AgsWorld->GetBodyCount() != 0) {

	//	for (b2Body* body = ((AgsBody*)address)->World->B2AgsWorld->GetBodyList(); 
	//		body; 
	//		body = body->GetNext())
	//	{

	//		if (body == ((AgsBody*)address)->B2AgsBody) {
	//			((AgsBody*)address)->World->B2AgsWorld->DestroyBody(((AgsBody*)address)->B2AgsBody);
	//			((AgsBody*)address)->B2AgsBody = NULL;
	//			break;
	//		}

	//	}

	//}
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
