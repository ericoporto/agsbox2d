#include "AgsShape.h"
#include "AgsShapeCircle.h"
#include "AgsShapeRect.h"
#include "Book.h"

AgsShape::AgsShape(AgsShapeCircle* shapeCircle) {
	ShapeCircle = shapeCircle;
	ShapeRect = NULL;
}

AgsShape::AgsShape(AgsShapeRect* shapeRect) {
	ShapeRect = shapeRect;
	ShapeCircle = NULL;
}

AgsShape::~AgsShape(void)
{
}


//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsShapeInterface AgsShape_Interface;
AgsShapeReader AgsShape_Reader;

const char* AgsShapeInterface::name = "Shape";

//------------------------------------------------------------------------------

int AgsShapeInterface::Dispose(const char* address, bool force)
{
	Book::UnregisterAgsShapeByID(((AgsShape*)address)->ID);
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
/*
	AgsShape* shape;

	int shape_id = key;

	if (Book::isAgsShapeRegisteredByID(shape_id)) {
		shape = Book::IDtoAgsShape(shape_id);
	}
	else {
		shape = new AgsShape();
	}

	const char* ptr = serializedData;

	engine->RegisterUnserializedObject(key, shape, &AgsShape_Interface);*/
}

//..............................................................................
