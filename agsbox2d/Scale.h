#pragma once

#ifndef AGSBOX2D_SCALE_H
#define AGSBOX2D_SCALE_H

#include "Box2D.h"

class Scale
{
private:
	Scale();
	~Scale();

public:
	static Scale* i();

	static void SetMeter(float32 scale);
	static float32 GetMeter();

	static float32 ScaleDown(float32 f);
	static float32 ScaleUp(float32 f);
	static void ScaleDown(float32 &x, float32 &y);
	static void ScaleUp(float32 &x, float32 &y);
	static b2Vec2 ScaleDown(const b2Vec2 &v);
	static b2Vec2 ScaleUp(const b2Vec2 &v);
	static b2AABB ScaleDown(const b2AABB &aabb);

};

#endif /* AGSBOX2D_SCALE_H */