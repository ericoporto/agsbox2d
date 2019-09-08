/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "Scale.h"

float32 Meter = 32.0f;

Scale* Scale::i()
{
	static Scale instance;

	return &instance;
}

Scale::Scale()
{
}

Scale::~Scale()
{
}

void Scale::SetMeter(float32 scale)
{
	if (scale < 1) Meter = 1.0f;
	Meter = scale;
}

float32 Scale::GetMeter()
{
	return Meter;
}

void Scale::ScaleDown(float32 &x, float32 &y)
{
	x /= Meter;
	y /= Meter;
}

void Scale::ScaleUp(float32 &x, float32 &y)
{
	x *= Meter;
	y *= Meter;
}

float32 Scale::ScaleDown(float32 f)
{
	return f / Meter;
}

float32 Scale::ScaleUp(float32 f)
{
	return f * Meter;
}

b2Vec2 Scale::ScaleDown(const b2Vec2 &v)
{
	b2Vec2 t = v;
	ScaleDown(t.x, t.y);
	return t;
}

b2Vec2 Scale::ScaleUp(const b2Vec2 &v)
{
	b2Vec2 t = v;
	ScaleUp(t.x, t.y);
	return t;
}

b2AABB Scale::ScaleDown(const b2AABB &aabb)
{
	b2AABB t;
	t.lowerBound = ScaleDown(aabb.lowerBound);
	t.upperBound = ScaleDown(aabb.upperBound);
	return t;
}
