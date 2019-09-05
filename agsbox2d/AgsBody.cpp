#include "AgsBody.h"
#include "AgsWorld.h"
#include "Scale.h"
#include "Book.h"

AgsBody::AgsBody(AgsWorld* world, float32 x, float32 y, b2BodyType bodytype) {
	IsDestroyed = false;
	B2AgsBodyDef.position.Set(Scale::ScaleDown(x), Scale::ScaleDown(y));
	B2AgsBodyDef.type = bodytype;
	B2AgsBodyDef.fixedRotation = true;
	B2AgsBody = world->B2AgsWorld->CreateBody(&B2AgsBodyDef);
	World = world;
}

AgsBody::AgsBody() {
	IsDestroyed = true;
}

bool AgsBody::IsTouching(AgsBody* body) {
	if (body == nullptr) {
		return false;
	}

	const b2ContactEdge *ce = B2AgsBody->GetContactList();
	b2Body *otherbody = body->B2AgsBody;

	while (ce != nullptr)
	{
		if (ce->other == otherbody &&
			ce->contact != nullptr && 
			ce->contact->IsTouching())
			return true;

		ce = ce->next;
	}

	return false;
}

bool AgsBody::GetIsDestroyed() {
	return IsDestroyed;
}

void AgsBody::SetIsDestroyed() {
	IsDestroyed = true;
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

bool AgsBody::GetIsBullet() {
	return B2AgsBody->IsBullet();
}

void AgsBody::SetIsBullet(bool bullet) {
	B2AgsBody->SetBullet(bullet);
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

float32 AgsBody::GetAngularDamping() {
	return B2AgsBody->GetAngularDamping();
}

void AgsBody::SetAngularDamping(float32 adamping) {
	B2AgsBody->SetAngularDamping(adamping);
}

float32 AgsBody::GetAngularVelocity() {
	return B2AgsBody->GetAngularVelocity();
}

void AgsBody::SetAngularVelocity(float32 avel) {
	B2AgsBody->SetAngularVelocity(avel);
}

float32 AgsBody::GetInertia() {
	return Scale::ScaleUp(B2AgsBody->GetInertia());
}

void AgsBody::SetInertia(float32 inertia) {
	b2MassData massData;
	massData.center = B2AgsBody->GetLocalCenter();
	massData.mass = B2AgsBody->GetMass();
	massData.I = Scale::ScaleDown(Scale::ScaleDown(inertia));
	B2AgsBody->SetMassData(&massData);
}

void AgsBody::ApplyLinearImpulse(float32 intensity_x, float32 intensity_y) {
	B2AgsBody->ApplyLinearImpulse(Scale::ScaleDown(b2Vec2(intensity_x, intensity_y)), B2AgsBody->GetWorldCenter(), true);
}

void AgsBody::ApplyTorque(float32 torque) {
	B2AgsBody->ApplyTorque(Scale::ScaleDown(torque), true);
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

#include "SerialHelper.h"
using namespace SerialHelper;

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

	Book::UnregisterAgsBodyByID(((AgsBody*)address)->ID);
	delete ((AgsBody*)address);
	return (1);
}

//------------------------------------------------------------------------------

int AgsBodyInterface::Serialize(const char* address, char* buffer, int bufsize)
{
	AgsBody* body = (AgsBody*)address;
	char* ptr = buffer;
	char* end = buffer + bufsize;

	if (body->GetIsDestroyed()) {
		ptr = BoolToChar(true, ptr, end);
		return (ptr - buffer);
	}
	ptr = BoolToChar(false, ptr, end);
	ptr = IntToChar(body->World->ID, ptr, end);

	ptr = b2BodyToChar(body->B2AgsBody, ptr, end);

	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsBodyReader::Unserialize(int key, const char* serializedData, int dataSize)
{
	int body_id = key;
	AgsBody* body;
	char* ptr = (char*)serializedData;

	bool isdestroyed;
	int32 world_id;
	ptr = CharToBool(isdestroyed, ptr);
	
	if (isdestroyed) {
		body = new AgsBody();
		engine->RegisterUnserializedObject(key, body, &AgsBody_Interface);
		return;
	}

	ptr = CharToInt(world_id, ptr);

	AgsWorld * world;
	if (Book::isAgsWorldRegisteredByID(world_id)) {
		world = Book::IDtoAgsWorld(world_id);
	}
	else {
		world = new AgsWorld(0, 0);
		Book::RegisterAgsWorld(world_id, world);
	}

	if (Book::isAgsBodyRegisteredByID(body_id)) {
		body = Book::IDtoAgsBody(body_id);
		body->World = world;
	}
	else {
		body = new AgsBody(world, 0.0, 0.0, b2_staticBody);
		body->ID = body_id;
		Book::RegisterAgsBody(body_id, body);
	}

	ptr = CharTob2Body(body->B2AgsBody, ptr);

	engine->RegisterUnserializedObject(key, body, &AgsBody_Interface);
}

//..............................................................................
