/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef AGSBOX2D_SERIALHELPER_H
#define AGSBOX2D_SERIALHELPER_H

#include "Box2D.h"

namespace SerialHelper {
	
	union u_cf
	{
		float32 f;
		char s[sizeof(float32)];
	};

	union u_ci
	{
		int32 i;
		char s[sizeof(int32)];
	};

	char* IntToChar(int32 i, char * buf, char* end);
	char* FloatToChar(float32 f, char * buf, char* end);
	char* b2Vec2ToChar(b2Vec2 vec, char * buf, char* end);
	char* BoolToChar(bool b, char* buf, char* end);
	char* b2ShapeToChar(b2Shape* b2shape, char* buf, char* end);
	char* b2FixtureToChar(b2Fixture* b2fixture, char* buf, char* end);
	char* b2BodyToChar(b2Body* b2body, char* buf, char* end);
    char* b2JointToChar(b2Joint* b2joint, char* buf, char* end);

	char* CharToBool(bool &b, char* buf);
	char* CharToInt(int32 &i, char * buf);
	char* CharToFloat(float32 &f, char * buf);
	char* CharTob2Vec2(b2Vec2 &vec, char * buf);
	char* CharTob2Shape(b2Shape** b2shape, char * buf);
	char* CharTob2FixtureDef(b2FixtureDef* b2fixturedef, char* buf);
	char* CharTob2Body(b2BodyDef &b2bodydef, b2Body** pb2body, b2World* world, char* buf);
    char* CharTob2JointDef(b2JointDef** b2jointdef, char* buf);
}

#endif /* AGSBOX2D_SERIALHELPER_H */