#include "AgsShapeRect.h"

AgsShapeRect::AgsShapeRect(float32 hx, float32 hy) {
	B2AgsShapeRect = new b2PolygonShape();
	B2AgsShapeRect->SetAsBox(hx, hy);
}

AgsShapeRect::~AgsShapeRect(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsShapeRectInterface AgsShapeRect_Interface;
AgsShapeRectReader AgsShapeRect_Reader;

const char* AgsShapeRectInterface::name = "AgsShapeRect";

//------------------------------------------------------------------------------

int AgsShapeRectInterface::Dispose(const char* address, bool force)
{
	delete ((AgsShapeRect*)address);
	return (1);
}

//------------------------------------------------------------------------------

int AgsShapeRectInterface::Serialize(const char* address, char* buffer, int bufsize)
{
	AgsShapeRect* arr = (AgsShapeRect*)address;
	char* ptr = buffer;
	
	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsShapeRectReader::Unserialize(int key, const char* serializedData, int dataSize)
{
	AgsShapeRect* arr = new AgsShapeRect(0,0);

	const char* ptr = serializedData;

	engine->RegisterUnserializedObject(key, arr, &AgsShapeRect_Interface);
}

//..............................................................................
