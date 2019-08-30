#include "Scale.h"

Scale::Scale()
{
}

Scale::~Scale()
{
}

void Scale::SetScale(int meter) {

}

void Scale::GetScale(int meter) {

}

int Scale::ScaleDownInt(float coordinate) {
	return static_cast<int>(coordinate / _Meter);
}

int Scale::ScaleUpInt(float coordinate) {
	return static_cast<int>(coordinate * _Meter);
}