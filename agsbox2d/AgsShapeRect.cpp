#include "AgsShapeRect.h"
#include "Scale.h"

AgsShapeRect::AgsShapeRect(float32 w, float32 h, float32 x, float32 y) {
	B2AgsShapeRect = new b2PolygonShape();

	B2AgsShapeRect->SetAsBox(
		Scale::ScaleDown(w / 2.0f), Scale::ScaleDown(h / 2.0f),
		Scale::ScaleDown(b2Vec2(x, y)), 0.0);

}

AgsShapeRect::AgsShapeRect(b2PolygonShape* shape) {
	B2AgsShapeRect = shape;
}


AgsShapeRect::~AgsShapeRect(void)
{
}

void AgsShapeRect::SetWidthF(float32 wf) {
	Width = wf;
	B2AgsShapeRect->SetAsBox(
		Scale::ScaleDown(wf / 2.0f), B2AgsShapeRect->m_vertices[2].y,
		B2AgsShapeRect->m_centroid, 0.0);
}

float32 AgsShapeRect::GetWidthF() {
	return Scale::ScaleUp(2.0f * B2AgsShapeRect->m_vertices[2].x);
}

float32 AgsShapeRect::GetHeightF() {
	return Scale::ScaleUp(2.0f * B2AgsShapeRect->m_vertices[2].y);
}

void AgsShapeRect::SetHeightF(float32 hf) {
	Height = hf;
	B2AgsShapeRect->SetAsBox(
		B2AgsShapeRect->m_vertices[2].x, Scale::ScaleDown(hf / 2.0f),
		B2AgsShapeRect->m_centroid, 0.0);
}


int32 AgsShapeRect::GetWidth() {
	return (int32) GetWidthF();
}

void AgsShapeRect::SetWidth(int32 w) {
	SetWidthF((float32)w);
}


int32 AgsShapeRect::GetHeight() {
	return (int32)GetHeightF();
}

void AgsShapeRect::SetHeight(int32 h) {
	SetHeightF((float32)h);
}

float32 AgsShapeRect::GetPointX(int32 i) {
	return Scale::ScaleUp(B2AgsShapeRect->m_vertices[i].x);
}

float32 AgsShapeRect::GetPointY(int32 i) {
	return Scale::ScaleUp(B2AgsShapeRect->m_vertices[i].y);
}

//------------------------------------------------------------------------------


extern IAGSEngine* engine;

AgsShapeRectInterface AgsShapeRect_Interface;
AgsShapeRectReader AgsShapeRect_Reader;

const char* AgsShapeRectInterface::name = "ShapeRectangle";

//------------------------------------------------------------------------------

int AgsShapeRectInterface::Dispose(const char* address, bool force)
{
	//delete (AgsShapeRect*)address;
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
	AgsShapeRect* arr = new AgsShapeRect(0, 0, 0, 0);

	const char* ptr = serializedData;

	engine->RegisterUnserializedObject(key, arr, &AgsShapeRect_Interface);
}

//..............................................................................
