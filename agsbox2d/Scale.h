#pragma once

#ifndef AGSBOX2D_SCALE_H
#define AGSBOX2D_SCALE_H

#include "Box2D.h"

class Scale
{
public:
	Scale();
	~Scale();

	void SetMeter(float32 scale);
	float32 GetMeter();

	float ScaleDown(float32 f);
	float ScaleUp(float32 f);
	void ScaleDown(float32 &x, float32 &y);
	void ScaleUp(float32 &x, float32 &y);
	b2Vec2 ScaleDown(const b2Vec2 &v);
	b2Vec2 ScaleUp(const b2Vec2 &v);
	b2AABB ScaleDown(const b2AABB &aabb);

private:
	float32 Meter;
};

#endif /* AGSBOX2D_SCALE_H */