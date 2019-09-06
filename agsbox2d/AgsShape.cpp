#include "AgsShape.h"
#include "AgsShapeCircle.h"
#include "AgsShapeRect.h"
#include "Book.h"

AgsShape::AgsShape(AgsShapeCircle* shapeCircle) {
	ShapeCircle = shapeCircle;
	ShapeRect = nullptr;
	B2AgsShape = shapeCircle->B2AgsShapeCircle;
}

AgsShape::AgsShape(AgsShapeRect* shapeRect) {
	ShapeRect = shapeRect;
	ShapeCircle = nullptr;
	B2AgsShape = shapeRect->B2AgsShapeRect;
}

AgsShape::AgsShape(b2Shape* b2shape) {
	if (b2shape->GetType() == b2Shape::e_circle) {
		ShapeCircle = new AgsShapeCircle((b2CircleShape*) b2shape);
		ShapeRect = nullptr;
	} else {
		ShapeRect = new AgsShapeRect((b2PolygonShape*) b2shape);
		ShapeCircle = nullptr;
	}
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
#include "SerialHelper.h"
using namespace SerialHelper;

int AgsShapeInterface::Dispose(const char* address, bool force)
{
	Book::UnregisterAgsShapeByID(((AgsShape*)address)->ID);
	delete ((AgsShape*)address);
	return (1);
}

//------------------------------------------------------------------------------

int AgsShapeInterface::Serialize(const char* address, char* buffer, int bufsize)
{
	AgsShape* shape = (AgsShape*)address;
	char* ptr = buffer;
	char* end = buffer + bufsize;

	//printf("--- serializing AgsShape %d --------->>>\n", shape->ID);

	ptr = b2ShapeToChar(shape->B2AgsShape, ptr, end);

	return (ptr - buffer);
}

//------------------------------------------------------------------------------

void AgsShapeReader::Unserialize(int key, const char* serializedData, int dataSize)
{
	char* ptr = (char*) serializedData;
	int shape_id = key;

	b2Shape * shape = nullptr;

	//printf("--- deserializing AgsShape %d ---------<<<\n", key);

	ptr = CharTob2Shape(&shape, ptr);
	AgsShape* agsshape = new AgsShape(shape);
	agsshape->ID = shape_id;
	Book::RegisterAgsShape(shape_id, agsshape);

	engine->RegisterUnserializedObject(key, agsshape, &AgsShape_Interface);
}

//..............................................................................
