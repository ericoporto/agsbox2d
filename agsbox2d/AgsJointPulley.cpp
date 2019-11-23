/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "AgsJointPulley.h"
#include "Book.h"
#include "Scale.h"
#include "AgsWorld.h"
#include "AgsJoint.h"

AgsJointPulley::AgsJointPulley(AgsWorld* agsworld, AgsBody* agsbody_a, AgsBody* agsbody_b,
        float32 ground_anchor_a_x, float32 ground_anchor_a_y, float32 ground_anchor_b_x, float32 ground_anchor_b_y,
        float32 anchor_a_x, float32 anchor_a_y, float32 anchor_b_x, float32 anchor_b_y,
        float32 ratio, bool collide_connected
    ) {

    if (agsbody_a->World->B2AgsWorld != agsworld->B2AgsWorld ||
        agsbody_b->World->B2AgsWorld != agsworld->B2AgsWorld)
        return;

    b2Vec2 groundAnchorA = b2Vec2(ground_anchor_a_x, ground_anchor_a_y);
    b2Vec2 groundAnchorB = b2Vec2(ground_anchor_b_x, ground_anchor_b_y);
    b2Vec2 anchorA = b2Vec2(anchor_a_x, anchor_a_y);
    b2Vec2 anchorB = b2Vec2(anchor_b_x, anchor_b_y);

    b2PulleyJointDef def;
    def.Initialize(agsbody_a->GetB2AgsBody() , agsbody_b->GetB2AgsBody(),
        Scale::ScaleDown(groundAnchorA), Scale::ScaleDown(groundAnchorB),
        Scale::ScaleDown(anchorA),  Scale::ScaleDown(anchorB),
        ratio);
    def.collideConnected = collide_connected;

    B2AgsJointPulley = dynamic_cast<b2PulleyJoint *>(agsworld->B2AgsWorld->CreateJoint(&def));
    WorldID = agsworld->ID;
    B2bodyA_ID = agsbody_a->B2BodyID;
    B2bodyB_ID = agsbody_b->B2BodyID;


}

AgsJointPulley::AgsJointPulley(b2PulleyJoint* Pulleyjoint){
    B2AgsJointPulley = Pulleyjoint;

}


AgsJointPulley::~AgsJointPulley(void)
{
}

float32 AgsJointPulley::GetLengthA()
{
    return Scale::ScaleUp(B2AgsJointPulley->GetLengthA());
}

float32 AgsJointPulley::GetLengthB()
{
    return Scale::ScaleUp(B2AgsJointPulley->GetLengthB());
}

float32 AgsJointPulley::GetRatio()
{
    return B2AgsJointPulley->GetRatio();
}

AgsBody* AgsJointPulley::GetBodyA() {
    return AgsJoint::GetBody(WorldID, B2AgsJointPulley->GetBodyA());
}

AgsBody* AgsJointPulley::GetBodyB() {
    return AgsJoint::GetBody(WorldID, B2AgsJointPulley->GetBodyB());
}

//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsJointPulleyInterface AgsJointPulley_Interface;
AgsJointPulleyReader AgsJointPulley_Reader;

const char* AgsJointPulleyInterface::name = "JointPulley";

//------------------------------------------------------------------------------

int AgsJointPulleyInterface::Dispose(const char* address, bool force)
{
    Book::UnregisterAgsJointPulleyByID(((AgsJointPulley*)address)->ID);
    //delete ((AgsJointPulley*)address);
    return (1);
}

//------------------------------------------------------------------------------

int AgsJointPulleyInterface::Serialize(const char* address, char* buffer, int bufsize)
{
    AgsJointPulley* arr = (AgsJointPulley*)address;
    char* ptr = buffer;

    return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsJointPulleyReader::Unserialize(int key, const char* serializedData, int dataSize)
{
//	AgsShapeCircle* arr = new AgsShapeCircle(0,0);

//	const char* ptr = serializedData;

//	engine->RegisterUnserializedObject(key, arr, &AgsShapeCircle_Interface);
}

//..............................................................................