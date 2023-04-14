#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include "Vector2.h"
#include "Vector3.h"
#include "Vector4.h"
#include "Matrix4x4.h"
#include <Novice.h>

//Vetor3
//加算
inline Vector3 Add(const Vector3& v1, const Vector3& v2) {
	Vector3 tmp;
	tmp.x = v1.x + v2.x;
	tmp.y = v1.y + v2.y;
	tmp.z = v1.z + v2.z;
	return tmp;
}
inline Vector3 operator +(const Vector3& v1, const Vector3& v2){
	return{v1.x + v2.x,v1.y + v2.y,v1.z + v2.z };
}
//減算
inline Vector3 Subtract(const Vector3& v1, const Vector3& v2) {
	Vector3 tmp;
	tmp.x = v1.x - v2.x;
	tmp.y = v1.y - v2.y;
	tmp.z = v1.z - v2.z;
	return tmp;
}
inline Vector3 operator -(const Vector3& v1, const Vector3& v2) {
	return{ v1.x - v2.x,v1.y - v2.y,v1.z - v2.z };
}
//スカラー倍
inline Vector3 Multiply(float scalar, const Vector3& v) {
	Vector3 tmp;
	tmp.x = v.x * scalar;
	tmp.y = v.y * scalar;
	tmp.z = v.z * scalar;
	return tmp;
}
inline Vector3 operator *(const Vector3& v, const float& scalar) {
	return{ v.x * scalar,v.y * scalar,v.z * scalar };
}
//内積
inline float Dot(const Vector3& v1, const Vector3& v2) {
	return{ v1.x * v2.x + v1.y * v2.y + v1.z * v2.z };
}
inline Vector3 operator *(const Vector3& v1, const Vector3& v2) {
	return{ v1.x * v2.x + v1.y * v2.y + v1.z * v2.z };
}
//長さ
inline float Length(const Vector3& v) {
	float tmp = v.x * v.x + v.y * v.y + v.z * v.z;
	tmp = sqrtf(tmp);
	return tmp;
}
//正規化
inline Vector3 Normalize(const Vector3& v) {
	float tmp = v.x * v.x + v.y * v.y + v.z * v.z;
	tmp = sqrtf(tmp);
	return { v.x / tmp, v.y / tmp, v.z / tmp };
}

static const int kColumnWidth = 60;
static const int kRowHeight = 20;

void VectorScreenPrintf(int x, int y, const Vector3& vector, const char* label) {
	Novice::ScreenPrintf(x, y, "%.02f", vector.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", vector.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", vector.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%s", label);
}
