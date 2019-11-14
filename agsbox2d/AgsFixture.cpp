/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsFixture.h"
#include "Book.h"

AgsFixture::AgsFixture(AgsBody* agsBody, AgsShape* agsShape, float32 density) {
	Body = agsBody;
	B2AgsFixtureDef.density = density;

	if (agsShape->ShapeCircle != NULL) {
		B2AgsFixtureDef.shape = agsShape->ShapeCircle->B2AgsShapeCircle;
	}
	else if (agsShape->ShapeRect != NULL) {
		B2AgsFixtureDef.shape = agsShape->ShapeRect->B2AgsShapeRect;
	}
	else {
		B2AgsFixtureDef.shape = agsShape->B2AgsShape;
	}

	B2AgsFixture = Body->GetB2AgsBody()->CreateFixture(&B2AgsFixtureDef);
}


float32 AgsFixture::GetDensity() {
	return B2AgsFixture->GetDensity();
}

void AgsFixture::SetDensity(float32 density) {
	B2AgsFixture->SetDensity(density);
}

float32 AgsFixture::GetFriction() {
	return B2AgsFixture->GetFriction();
}

void AgsFixture::SetFriction(float32 friction) {
	B2AgsFixture->SetFriction(friction);
}

float32 AgsFixture::GetRestitution() {
	return B2AgsFixture->GetRestitution();
}

void AgsFixture::SetRestitution(float32 restitution) {
	B2AgsFixture->SetRestitution(restitution);
}


AgsFixture::~AgsFixture(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsFixtureInterface AgsFixture_Interface;
AgsFixtureReader AgsFixture_Reader;

const char* AgsFixtureInterface::name = "Fixture";

//------------------------------------------------------------------------------

int AgsFixtureInterface::Dispose(const char* address, bool force)
{
	//if (((AgsFixture*)address)->Body != NULL && 
	//	(((AgsFixture*)address)->Body->B2AgsBody != NULL) && 
	//	(((AgsFixture*)address)->B2AgsFixture != NULL)){

	//	((AgsFixture*)address)->Body->B2AgsBody->DestroyFixture(((AgsFixture*)address)->B2AgsFixture);
	//}

	Book::UnregisterAgsFixtureByID(((AgsFixture*)address)->ID);
	delete ((AgsFixture*)address);
	return (1);
}

//------------------------------------------------------------------------------

int AgsFixtureInterface::Serialize(const char* address, char* buffer, int bufsize)
{
	AgsFixture* arr = (AgsFixture*)address;
	char* ptr = buffer;
	
	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsFixtureReader::Unserialize(int key, const char* serializedData, int dataSize)
{
	//AgsFixture* arr = new AgsFixture();

	//const char* ptr = serializedData;

	//engine->RegisterUnserializedObject(key, arr, &AgsFixture_Interface);
}

//..............................................................................
