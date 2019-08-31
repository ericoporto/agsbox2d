#include "AgsShape.h"
#include "AgsShapeCircle.h"
#include "AgsShapeRect.h"

AgsShape::AgsShape(AgsShapeCircle* shapeCircle) {
	ShapeCircle = shapeCircle;
}

AgsShape::AgsShape(AgsShapeRect* shapeRect) {
	ShapeRect = shapeRect;
}

AgsShape::~AgsShape(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsShapeInterface AgsShape_Interface;
AgsShapeReader AgsShape_Reader;

const char* AgsShapeInterface::name = "AgsShape";

//------------------------------------------------------------------------------

int AgsShapeInterface::Dispose(const char* address, bool force)
{
	delete ((AgsShape*)address);
	return (1);
}

//------------------------------------------------------------------------------

int AgsShapeInterface::Serialize(const char* address, char* buffer, int bufsize)
{
	AgsShape* arr = (AgsShape*)address;
	char* ptr = buffer;
	
	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsShapeReader::Unserialize(int key, const char* serializedData, int dataSize)
{
//	AgsShape* arr = new AgsShape(0,0);

//	const char* ptr = serializedData;

//	engine->RegisterUnserializedObject(key, arr, &AgsShape_Interface);
}

//..............................................................................
