#pragma once

#ifndef AGSBOX2D_SCALE_H
#define AGSBOX2D_SCALE_H

#include "Box2D.h"

class Scale
{
public:
	Scale();
	~Scale();

	void SetScale(int meter);
	void GetScale(int meter);

	int ScaleDownInt(float coordinate);
	int ScaleUpInt(float coordinate);

private:
	float32 _Meter;
};

#endif /* AGSBOX2D_SCALE_H */