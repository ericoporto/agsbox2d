#include "DebugDraw.h"
#include "Scale.h"

#if !AGS_PLATFORM_OS_WINDOWS
#define min(x,y) (((x) < (y)) ? (x) : (y))
#define max(x,y) (((x) > (y)) ? (x) : (y))
#endif

#pragma region Color_Functions

#define DEFAULT_RGB_R_SHIFT_32  16
#define DEFAULT_RGB_G_SHIFT_32  8
#define DEFAULT_RGB_B_SHIFT_32  0
#define DEFAULT_RGB_A_SHIFT_32  24

int getr32(int c)
{
	return ((c >> DEFAULT_RGB_R_SHIFT_32) & 0xFF);
}


int getg32(int c)
{
	return ((c >> DEFAULT_RGB_G_SHIFT_32) & 0xFF);
}


int getb32(int c)
{
	return ((c >> DEFAULT_RGB_B_SHIFT_32) & 0xFF);
}


int geta32(int c)
{
	return ((c >> DEFAULT_RGB_A_SHIFT_32) & 0xFF);
}


int makeacol32(int r, int g, int b, int a)
{
	return ((r << DEFAULT_RGB_R_SHIFT_32) |
		(g << DEFAULT_RGB_G_SHIFT_32) |
		(b << DEFAULT_RGB_B_SHIFT_32) |
		(a << DEFAULT_RGB_A_SHIFT_32));
}

#pragma endregion

#pragma region Pixel32_Definition

struct Pixel32 {

public:
	Pixel32(int r = 0, int g = 0, int b = 0, int alpha = 0);
	Pixel32(b2Color color);
	~Pixel32() = default;
	int GetColorAsInt();
	int Red;
	int Green;
	int Blue;
	int Alpha;

};

Pixel32::Pixel32(int r, int g, int b, int alpha) {
	Red = r;
	Blue = g;
	Green = b;
	Alpha = alpha;
}

Pixel32::Pixel32(b2Color color) {
	Red = (int) color.r*255.0f;
	Blue = (int) color.g*255.0f;
	Green = (int) color.b*255.0f;
	Alpha = (int) 255.0f;
}

int Pixel32::GetColorAsInt() {
	return makeacol32(Red, Green, Blue, Alpha);
}

#pragma endregion

void _DrawPixel(unsigned int **longbufferBitmap, int x, int y, int agsColor, int width, int height) {
	if (x > 0 && x < width && y > 0 && y < height) {
		longbufferBitmap[y][x] = agsColor;
	}
}

void _DrawCircle(
	unsigned int **lbb,
	unsigned int x0,
	unsigned int y0,
	unsigned int radius,
	int agsColor, int width, int height)
{
	int f = 1 - radius;
	int ddF_x = 0;
	int ddF_y = -2 * radius;
	int x = 0;
	int y = radius;

	_DrawPixel(lbb, x0, y0 + radius, agsColor, width, height);
	_DrawPixel(lbb, x0, y0 - radius, agsColor, width, height);
	_DrawPixel(lbb, x0 + radius, y0, agsColor, width, height);
	_DrawPixel(lbb, x0 - radius, y0, agsColor, width, height);

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x + 1;
		_DrawPixel(lbb, x0 + x, y0 + y, agsColor, width, height);
		_DrawPixel(lbb, x0 - x, y0 + y, agsColor, width, height);
		_DrawPixel(lbb, x0 + x, y0 - y, agsColor, width, height);
		_DrawPixel(lbb, x0 - x, y0 - y, agsColor, width, height);
		_DrawPixel(lbb, x0 + y, y0 + x, agsColor, width, height);
		_DrawPixel(lbb, x0 - y, y0 + x, agsColor, width, height);
		_DrawPixel(lbb, x0 + y, y0 - x, agsColor, width, height);
		_DrawPixel(lbb, x0 - y, y0 - x, agsColor, width, height);
	}
}

void _DrawLine(unsigned int **longbufferBitmap, int x0, int y0, int x1, int y1, int agsColor, int width, int height) {

	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = (dx > dy ? dx : -dy) / 2, e2;

	for (;;) {
		_DrawPixel(longbufferBitmap, x0, y0, agsColor, width, height);
		if (x0 == x1 && y0 == y1) break;
		e2 = err;
		if (e2 > -dx) { err -= dy; x0 += sx; }
		if (e2 < dy) { err += dx; y0 += sy; }
	}
}

void AgsDebugDraw::InitializeAgsDebugDraw(IAGSEngine* engine, int screenWidth, int screenHeight, int colDepth) {
	Engine = engine;
	ScreenWidth = screenWidth;
	ScreenHeight = screenHeight;
	SetFlags(b2Draw::e_shapeBit || b2Draw::e_centerOfMassBit );
}

int AgsDebugDraw::GetDebugSprite() {
	Engine->NotifySpriteUpdated(SpriteId);
	return SpriteId;
}

void AgsDebugDraw::GetSurfaceForDebugDraw() {
	if (SpriteId < 0)  SpriteId = Engine->CreateDynamicSprite(32, ScreenWidth, ScreenHeight);

	BITMAP *engineSprite = Engine->GetSpriteGraphic(SpriteId);
	unsigned char **charbuffer = Engine->GetRawBitmapSurface(engineSprite);
	Longbuffer = (unsigned int**)charbuffer;
}

void AgsDebugDraw::ReleaseSurfaceForDebugDraw() {
	BITMAP *engineSprite = Engine->GetSpriteGraphic(SpriteId);
	Engine->ReleaseBitmapSurface(engineSprite);
}


void AgsDebugDraw::ClearSprite() {
	if (SpriteId < 0) return;

	BITMAP *engineSprite = Engine->GetSpriteGraphic(SpriteId);
	unsigned char **charbuffer = Engine->GetRawBitmapSurface(engineSprite);
	unsigned int **longbuffer = (unsigned int**)charbuffer;

	int srcWidth, srcHeight;
	Engine->GetBitmapDimensions(engineSprite, &srcWidth, &srcHeight, nullptr);
	
	for (int y = 0; y < srcHeight; y++) {
		for (int x = 0; x < srcWidth; x++) {
			_DrawPixel(longbuffer, x, y, 0, srcWidth, srcHeight);
		}
	}

	Engine->ReleaseBitmapSurface(engineSprite);
}


void AgsDebugDraw::DrawPoint(const b2Vec2& p, float32 size, const b2Color& color) {
	if (SpriteId < 0) return;
	
	Pixel32 pix = Pixel32(color);
	int agscolor = pix.GetColorAsInt();

	_DrawPixel(Longbuffer, p.x*Scale::GetMeter(), p.y*Scale::GetMeter(), 0, ScreenWidth, ScreenHeight);
}

void AgsDebugDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
	if (SpriteId < 0) return;
	
	Pixel32 pix = Pixel32(color);
	int agscolor = pix.GetColorAsInt();

	for (int i = 0; i < vertexCount; i++) {
		if (i == 0) {
			_DrawPixel(Longbuffer, vertices[i].x*Scale::GetMeter(), vertices[i].y*Scale::GetMeter(), agscolor, ScreenWidth, ScreenHeight);
		}
		else {
			_DrawLine(Longbuffer, 
				vertices[i-1].x*Scale::GetMeter(), vertices[i-1].y*Scale::GetMeter(),
				vertices[i].x*Scale::GetMeter(), vertices[i].y*Scale::GetMeter(),
				agscolor, ScreenWidth, ScreenHeight);
		}
	}

}

void AgsDebugDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) {
	if (SpriteId < 0) return;
	
	DrawPolygon(vertices, vertexCount, color);
}

void AgsDebugDraw::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color) {
	if (SpriteId < 0) return;

	Pixel32 pix = Pixel32(color);
	int agscolor = pix.GetColorAsInt();

	_DrawCircle(Longbuffer, center.x*Scale::GetMeter(), center.y*Scale::GetMeter(), radius*Scale::GetMeter(), agscolor, ScreenWidth, ScreenHeight);
}

void AgsDebugDraw::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) {
	if (SpriteId < 0) return;

	DrawCircle(center, radius, color);
}

void AgsDebugDraw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) {
	if (SpriteId < 0) return;

	Pixel32 pix = Pixel32(color);
	int agscolor = pix.GetColorAsInt();

	_DrawLine(Longbuffer,
		p1.x*Scale::GetMeter(), p1.y*Scale::GetMeter(),
		p2.x*Scale::GetMeter(), p2.y*Scale::GetMeter(),
		agscolor, ScreenWidth, ScreenHeight);
}

void AgsDebugDraw::DrawTransform(const b2Transform& xf) {
	if (SpriteId < 0) return;

}