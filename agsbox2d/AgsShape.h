/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#pragma once

#ifndef _AGS_SHAPE_H
#define _AGS_SHAPE_H

#include "Box2D.h"
#include "plugin/agsplugin.h"
#include "AgsShapeCircle.h"
#include "AgsShapeRect.h"

class AgsShape
{
public:
	AgsShape(AgsShapeCircle* shapeCircle);
	AgsShape(AgsShapeRect* shapeRect);
	AgsShape(b2Shape* b2shape);
	~AgsShape(void);
	b2Shape* B2AgsShape = nullptr;
	AgsShapeCircle* ShapeCircle = nullptr;
	AgsShapeRect* ShapeRect = nullptr;
	int32 ID;
};



//------------------------------------------------------------------------------
// AGS interface instances

class AgsShapeInterface : public IAGSScriptManagedObject
{
public:
	static const char* name;

	AgsShapeInterface() {};

	virtual int Dispose(const char* address, bool force);
	virtual const char* GetType() { return (name); }
	virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsShapeReader : public IAGSManagedObjectReader
{
public:

	AgsShapeReader() {}

	virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsShapeInterface AgsShape_Interface;
extern AgsShapeReader AgsShape_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_SHAPE_H */

//..............................................................................