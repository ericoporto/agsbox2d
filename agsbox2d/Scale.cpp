#include "Scale.h"

Scale::Scale()
{
}

Scale::~Scale()
{
}

void Scale::SetMeter(float scale)
{
	if (scale < 1) Meter = 1.0f;
	Meter = scale;
}

float Scale::GetMeter()
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

float Scale::ScaleDown(float32 f)
{
	return f / Meter;
}

float Scale::ScaleUp(float32 f)
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
