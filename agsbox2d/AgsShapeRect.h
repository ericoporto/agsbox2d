#pragma once

#ifndef _AGS_SHAPE_RECT_H
#define _AGS_SHAPE_RECT_H

#include "Box2D.h"
#include "plugin/agsplugin.h"

class AgsShapeRect
{
public:
	AgsShapeRect(float32 w, float32 h, float32 x, float32 y);
	~AgsShapeRect(void);
	b2PolygonShape  * B2AgsShapeRect;
};



//------------------------------------------------------------------------------
// AGS interface instances

class AgsShapeRectInterface : public IAGSScriptManagedObject
{
public:
	static const char* name;

	AgsShapeRectInterface() {};

	virtual int Dispose(const char* address, bool force);
	virtual const char* GetType() { return (name); }
	virtual int Serialize(const char* address, char* buffer, int bufsize);

};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class AgsShapeRectReader : public IAGSManagedObjectReader
{
public:

	AgsShapeRectReader() {}

	virtual void Unserialize(int key, const char* serializedData, int dataSize);

};

//------------------------------------------------------------------------------

extern AgsShapeRectInterface AgsShapeRect_Interface;
extern AgsShapeRectReader AgsShapeRect_Reader;

//------------------------------------------------------------------------------

#endif /* _AGS_SHAPE_RECT_H */

//..............................................................................