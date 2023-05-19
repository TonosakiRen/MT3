#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include "Vector2.h"
#include "Vector3.h"
#include "Vector4.h"
#include "Matrix4x4.h"
#include <Novice.h>
#include <assert.h>

#pragma region Vector3
//Vetor3
//加算
inline Vector3 Add(const Vector3& v1, const Vector3& v2) {
	Vector3 tmp;
	tmp.x = v1.x + v2.x;
	tmp.y = v1.y + v2.y;
	tmp.z = v1.z + v2.z;
	return tmp;
}
inline Vector3 operator +(const Vector3& v1, const Vector3& v2) {
	return{ v1.x + v2.x,v1.y + v2.y,v1.z + v2.z };
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
inline Vector3& operator+=(Vector3& v1, const Vector3& v2) {
	v1.x += v2.x;
	v1.y += v2.y;
	v1.z += v2.z;
	return v1;
}
inline Vector3& operator-=(Vector3& v1, const Vector3& v2) {
	v1.x -= v2.x;
	v1.y -= v2.y;
	v1.z -= v2.z;
	return v1;
}

inline Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	return { (v1.y * v2.z - v1.z * v2.y),(v1.z * v2.x - v1.x * v2.z),(v1.x * v2.y - v1.y * v2.x)};
}

inline Vector3 operator *(const Vector3& v, const Matrix4x4& m) {

	Vector3 result;
	result.x = v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0] + 1.0f * m.m[3][0];
	result.y = v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1] + 1.0f * m.m[3][1];
	result.z = v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2] + 1.0f * m.m[3][2];
	float w = v.x * m.m[0][3] + v.y * m.m[1][3] + v.z * m.m[2][3] + 1.0f * m.m[3][3];

	assert(w != 0.0f);
	result.x /= w;
	result.y /= w;
	result.z /= w;

	return result;
}

#pragma endregion
#pragma region Matrix4x4
Matrix4x4 Add(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 tmp;
	tmp.m[0][0] = m1.m[0][0] + m2.m[0][0];
	tmp.m[0][1] = m1.m[0][1] + m2.m[0][1];
	tmp.m[0][2] = m1.m[0][2] + m2.m[0][2];
	tmp.m[0][3] = m1.m[0][3] + m2.m[0][3];
	tmp.m[1][0] = m1.m[1][0] + m2.m[1][0];
	tmp.m[1][1] = m1.m[1][1] + m2.m[1][1];
	tmp.m[1][2] = m1.m[1][2] + m2.m[1][2];
	tmp.m[1][3] = m1.m[1][3] + m2.m[1][3];
	tmp.m[2][0] = m1.m[2][0] + m2.m[2][0];
	tmp.m[2][1] = m1.m[2][1] + m2.m[2][1];
	tmp.m[2][2] = m1.m[2][2] + m2.m[2][2];
	tmp.m[2][3] = m1.m[2][3] + m2.m[2][3];
	tmp.m[3][0] = m1.m[3][0] + m2.m[3][0];
	tmp.m[3][1] = m1.m[3][1] + m2.m[3][1];
	tmp.m[3][2] = m1.m[3][2] + m2.m[3][2];
	tmp.m[3][3] = m1.m[3][3] + m2.m[3][3];
	return tmp;

}
Matrix4x4 operator +(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 tmp;
	tmp.m[0][0] = m1.m[0][0] + m2.m[0][0];
	tmp.m[0][1] = m1.m[0][1] + m2.m[0][1];
	tmp.m[0][2] = m1.m[0][2] + m2.m[0][2];
	tmp.m[0][3] = m1.m[0][3] + m2.m[0][3];
	tmp.m[1][0] = m1.m[1][0] + m2.m[1][0];
	tmp.m[1][1] = m1.m[1][1] + m2.m[1][1];
	tmp.m[1][2] = m1.m[1][2] + m2.m[1][2];
	tmp.m[1][3] = m1.m[1][3] + m2.m[1][3];
	tmp.m[2][0] = m1.m[2][0] + m2.m[2][0];
	tmp.m[2][1] = m1.m[2][1] + m2.m[2][1];
	tmp.m[2][2] = m1.m[2][2] + m2.m[2][2];
	tmp.m[2][3] = m1.m[2][3] + m2.m[2][3];
	tmp.m[3][0] = m1.m[3][0] + m2.m[3][0];
	tmp.m[3][1] = m1.m[3][1] + m2.m[3][1];
	tmp.m[3][2] = m1.m[3][2] + m2.m[3][2];
	tmp.m[3][3] = m1.m[3][3] + m2.m[3][3];
	return tmp;
}
Matrix4x4 Subtract(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 tmp;
	tmp.m[0][0] = m1.m[0][0] - m2.m[0][0];
	tmp.m[0][1] = m1.m[0][1] - m2.m[0][1];
	tmp.m[0][2] = m1.m[0][2] - m2.m[0][2];
	tmp.m[0][3] = m1.m[0][3] - m2.m[0][3];
	tmp.m[1][0] = m1.m[1][0] - m2.m[1][0];
	tmp.m[1][1] = m1.m[1][1] - m2.m[1][1];
	tmp.m[1][2] = m1.m[1][2] - m2.m[1][2];
	tmp.m[1][3] = m1.m[1][3] - m2.m[1][3];
	tmp.m[2][0] = m1.m[2][0] - m2.m[2][0];
	tmp.m[2][1] = m1.m[2][1] - m2.m[2][1];
	tmp.m[2][2] = m1.m[2][2] - m2.m[2][2];
	tmp.m[2][3] = m1.m[2][3] - m2.m[2][3];
	tmp.m[3][0] = m1.m[3][0] - m2.m[3][0];
	tmp.m[3][1] = m1.m[3][1] - m2.m[3][1];
	tmp.m[3][2] = m1.m[3][2] - m2.m[3][2];
	tmp.m[3][3] = m1.m[3][3] - m2.m[3][3];
	return tmp;
}
Matrix4x4 operator -(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 tmp;
	tmp.m[0][0] = m1.m[0][0] - m2.m[0][0];
	tmp.m[0][1] = m1.m[0][1] - m2.m[0][1];
	tmp.m[0][2] = m1.m[0][2] - m2.m[0][2];
	tmp.m[0][3] = m1.m[0][3] - m2.m[0][3];
	tmp.m[1][0] = m1.m[1][0] - m2.m[1][0];
	tmp.m[1][1] = m1.m[1][1] - m2.m[1][1];
	tmp.m[1][2] = m1.m[1][2] - m2.m[1][2];
	tmp.m[1][3] = m1.m[1][3] - m2.m[1][3];
	tmp.m[2][0] = m1.m[2][0] - m2.m[2][0];
	tmp.m[2][1] = m1.m[2][1] - m2.m[2][1];
	tmp.m[2][2] = m1.m[2][2] - m2.m[2][2];
	tmp.m[2][3] = m1.m[2][3] - m2.m[2][3];
	tmp.m[3][0] = m1.m[3][0] - m2.m[3][0];
	tmp.m[3][1] = m1.m[3][1] - m2.m[3][1];
	tmp.m[3][2] = m1.m[3][2] - m2.m[3][2];
	tmp.m[3][3] = m1.m[3][3] - m2.m[3][3];
	return tmp;
}
Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 tmp;
	tmp.m[0][0] = m1.m[0][0] * m2.m[0][0] + m1.m[0][1] * m2.m[1][0] + m1.m[0][2] * m2.m[2][0] + m1.m[0][3] * m2.m[3][0];
	tmp.m[0][1] = m1.m[0][0] * m2.m[0][1] + m1.m[0][1] * m2.m[1][1] + m1.m[0][2] * m2.m[2][1] + m1.m[0][3] * m2.m[3][1];
	tmp.m[0][2] = m1.m[0][0] * m2.m[0][2] + m1.m[0][1] * m2.m[1][2] + m1.m[0][2] * m2.m[2][2] + m1.m[0][3] * m2.m[3][2];
	tmp.m[0][3] = m1.m[0][0] * m2.m[0][3] + m1.m[0][1] * m2.m[1][3] + m1.m[0][2] * m2.m[2][3] + m1.m[0][3] * m2.m[3][3];

	tmp.m[1][0] = m1.m[1][0] * m2.m[0][0] + m1.m[1][1] * m2.m[1][0] + m1.m[1][2] * m2.m[2][0] + m1.m[1][3] * m2.m[3][0];
	tmp.m[1][1] = m1.m[1][0] * m2.m[0][1] + m1.m[1][1] * m2.m[1][1] + m1.m[1][2] * m2.m[2][1] + m1.m[1][3] * m2.m[3][1];
	tmp.m[1][2] = m1.m[1][0] * m2.m[0][2] + m1.m[1][1] * m2.m[1][2] + m1.m[1][2] * m2.m[2][2] + m1.m[1][3] * m2.m[3][2];
	tmp.m[1][3] = m1.m[1][0] * m2.m[0][3] + m1.m[1][1] * m2.m[1][3] + m1.m[1][2] * m2.m[2][3] + m1.m[1][3] * m2.m[3][3];

	tmp.m[2][0] = m1.m[2][0] * m2.m[0][0] + m1.m[2][1] * m2.m[1][0] + m1.m[2][2] * m2.m[2][0] + m1.m[2][3] * m2.m[3][0];
	tmp.m[2][1] = m1.m[2][0] * m2.m[0][1] + m1.m[2][1] * m2.m[1][1] + m1.m[2][2] * m2.m[2][1] + m1.m[2][3] * m2.m[3][1];
	tmp.m[2][2] = m1.m[2][0] * m2.m[0][2] + m1.m[2][1] * m2.m[1][2] + m1.m[2][2] * m2.m[2][2] + m1.m[2][3] * m2.m[3][2];
	tmp.m[2][3] = m1.m[2][0] * m2.m[0][3] + m1.m[2][1] * m2.m[1][3] + m1.m[2][2] * m2.m[2][3] + m1.m[2][3] * m2.m[3][3];

	tmp.m[3][0] = m1.m[3][0] * m2.m[0][0] + m1.m[3][1] * m2.m[1][0] + m1.m[3][2] * m2.m[2][0] + m1.m[3][3] * m2.m[3][0];
	tmp.m[3][1] = m1.m[3][0] * m2.m[0][1] + m1.m[3][1] * m2.m[1][1] + m1.m[3][2] * m2.m[2][1] + m1.m[3][3] * m2.m[3][1];
	tmp.m[3][2] = m1.m[3][0] * m2.m[0][2] + m1.m[3][1] * m2.m[1][2] + m1.m[3][2] * m2.m[2][2] + m1.m[3][3] * m2.m[3][2];
	tmp.m[3][3] = m1.m[3][0] * m2.m[0][3] + m1.m[3][1] * m2.m[1][3] + m1.m[3][2] * m2.m[2][3] + m1.m[3][3] * m2.m[3][3];
	return tmp;
}
Matrix4x4 operator *(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 tmp;
	tmp.m[0][0] = m1.m[0][0] * m2.m[0][0] + m1.m[0][1] * m2.m[1][0] + m1.m[0][2] * m2.m[2][0] + m1.m[0][3] * m2.m[3][0];
	tmp.m[0][1] = m1.m[0][0] * m2.m[0][1] + m1.m[0][1] * m2.m[1][1] + m1.m[0][2] * m2.m[2][1] + m1.m[0][3] * m2.m[3][1];
	tmp.m[0][2] = m1.m[0][0] * m2.m[0][2] + m1.m[0][1] * m2.m[1][2] + m1.m[0][2] * m2.m[2][2] + m1.m[0][3] * m2.m[3][2];
	tmp.m[0][3] = m1.m[0][0] * m2.m[0][3] + m1.m[0][1] * m2.m[1][3] + m1.m[0][2] * m2.m[2][3] + m1.m[0][3] * m2.m[3][3];

	tmp.m[1][0] = m1.m[1][0] * m2.m[0][0] + m1.m[1][1] * m2.m[1][0] + m1.m[1][2] * m2.m[2][0] + m1.m[1][3] * m2.m[3][0];
	tmp.m[1][1] = m1.m[1][0] * m2.m[0][1] + m1.m[1][1] * m2.m[1][1] + m1.m[1][2] * m2.m[2][1] + m1.m[1][3] * m2.m[3][1];
	tmp.m[1][2] = m1.m[1][0] * m2.m[0][2] + m1.m[1][1] * m2.m[1][2] + m1.m[1][2] * m2.m[2][2] + m1.m[1][3] * m2.m[3][2];
	tmp.m[1][3] = m1.m[1][0] * m2.m[0][3] + m1.m[1][1] * m2.m[1][3] + m1.m[1][2] * m2.m[2][3] + m1.m[1][3] * m2.m[3][3];

	tmp.m[2][0] = m1.m[2][0] * m2.m[0][0] + m1.m[2][1] * m2.m[1][0] + m1.m[2][2] * m2.m[2][0] + m1.m[2][3] * m2.m[3][0];
	tmp.m[2][1] = m1.m[2][0] * m2.m[0][1] + m1.m[2][1] * m2.m[1][1] + m1.m[2][2] * m2.m[2][1] + m1.m[2][3] * m2.m[3][1];
	tmp.m[2][2] = m1.m[2][0] * m2.m[0][2] + m1.m[2][1] * m2.m[1][2] + m1.m[2][2] * m2.m[2][2] + m1.m[2][3] * m2.m[3][2];
	tmp.m[2][3] = m1.m[2][0] * m2.m[0][3] + m1.m[2][1] * m2.m[1][3] + m1.m[2][2] * m2.m[2][3] + m1.m[2][3] * m2.m[3][3];

	tmp.m[3][0] = m1.m[3][0] * m2.m[0][0] + m1.m[3][1] * m2.m[1][0] + m1.m[3][2] * m2.m[2][0] + m1.m[3][3] * m2.m[3][0];
	tmp.m[3][1] = m1.m[3][0] * m2.m[0][1] + m1.m[3][1] * m2.m[1][1] + m1.m[3][2] * m2.m[2][1] + m1.m[3][3] * m2.m[3][1];
	tmp.m[3][2] = m1.m[3][0] * m2.m[0][2] + m1.m[3][1] * m2.m[1][2] + m1.m[3][2] * m2.m[2][2] + m1.m[3][3] * m2.m[3][2];
	tmp.m[3][3] = m1.m[3][0] * m2.m[0][3] + m1.m[3][1] * m2.m[1][3] + m1.m[3][2] * m2.m[2][3] + m1.m[3][3] * m2.m[3][3];
	return tmp;
}
Matrix4x4 Inverse(const Matrix4x4& m) {
	float lal = m.m[0][0] * m.m[1][1] * m.m[2][2] * m.m[3][3]
		+ m.m[0][0] * m.m[1][2] * m.m[2][3] * m.m[3][1]
		+ m.m[0][0] * m.m[1][3] * m.m[2][1] * m.m[3][2]

		- m.m[0][0] * m.m[1][3] * m.m[2][2] * m.m[3][1]
		- m.m[0][0] * m.m[1][2] * m.m[2][1] * m.m[3][3]
		- m.m[0][0] * m.m[1][1] * m.m[2][3] * m.m[3][2]

		- m.m[0][1] * m.m[1][0] * m.m[2][2] * m.m[3][3]
		- m.m[0][2] * m.m[1][0] * m.m[2][3] * m.m[3][1]
		- m.m[0][3] * m.m[1][0] * m.m[2][1] * m.m[3][2]

		+ m.m[0][3] * m.m[1][0] * m.m[2][2] * m.m[3][1]
		+ m.m[0][2] * m.m[1][0] * m.m[2][1] * m.m[3][3]
		+ m.m[0][1] * m.m[1][0] * m.m[2][3] * m.m[3][2]

		+ m.m[0][1] * m.m[1][2] * m.m[2][0] * m.m[3][3]
		+ m.m[0][2] * m.m[1][3] * m.m[2][0] * m.m[3][1]
		+ m.m[0][3] * m.m[1][1] * m.m[2][0] * m.m[3][2]

		- m.m[0][3] * m.m[1][2] * m.m[2][0] * m.m[3][1]
		- m.m[0][2] * m.m[1][1] * m.m[2][0] * m.m[3][3]
		- m.m[0][1] * m.m[1][3] * m.m[2][0] * m.m[3][2]

		- m.m[0][1] * m.m[1][2] * m.m[2][3] * m.m[3][0]
		- m.m[0][2] * m.m[1][3] * m.m[2][1] * m.m[3][0]
		- m.m[0][3] * m.m[1][1] * m.m[2][2] * m.m[3][0]

		+ m.m[0][3] * m.m[1][2] * m.m[2][1] * m.m[3][0]
		+ m.m[0][2] * m.m[1][1] * m.m[2][3] * m.m[3][0]
		+ m.m[0][1] * m.m[1][3] * m.m[2][2] * m.m[3][0];

	Matrix4x4 tmp;
	tmp.m[0][0] = (m.m[1][1] * m.m[2][2] * m.m[3][3] + m.m[1][2] * m.m[2][3] * m.m[3][1] + m.m[1][3] * m.m[2][1] * m.m[3][2]
		- m.m[1][3] * m.m[2][2] * m.m[3][1] - m.m[1][2] * m.m[2][1] * m.m[3][3] - m.m[1][1] * m.m[2][3] * m.m[3][2]) / lal;

	tmp.m[0][1] = (-m.m[0][1] * m.m[2][2] * m.m[3][3] - m.m[0][2] * m.m[2][3] * m.m[3][1] - m.m[0][3] * m.m[2][1] * m.m[3][2]
		+ m.m[0][3] * m.m[2][2] * m.m[3][1] + m.m[0][2] * m.m[2][1] * m.m[3][3] + m.m[0][1] * m.m[2][3] * m.m[3][2]) / lal;

	tmp.m[0][2] = (m.m[0][1] * m.m[1][2] * m.m[3][3] + m.m[0][2] * m.m[1][3] * m.m[3][1] + m.m[0][3] * m.m[1][1] * m.m[3][2]
		- m.m[0][3] * m.m[1][2] * m.m[3][1] - m.m[0][2] * m.m[1][1] * m.m[3][3] - m.m[0][1] * m.m[1][3] * m.m[3][2]) / lal;

	tmp.m[0][3] = (-m.m[0][1] * m.m[1][2] * m.m[2][3] - m.m[0][2] * m.m[1][3] * m.m[2][1] - m.m[0][3] * m.m[1][1] * m.m[2][2]
		+ m.m[0][3] * m.m[1][2] * m.m[2][1] + m.m[0][2] * m.m[1][1] * m.m[2][3] + m.m[0][1] * m.m[1][3] * m.m[2][2]) / lal;


	tmp.m[1][0] = (-m.m[1][0] * m.m[2][2] * m.m[3][3] - m.m[1][2] * m.m[2][3] * m.m[3][0] - m.m[1][3] * m.m[2][0] * m.m[3][2]
		+ m.m[1][3] * m.m[2][2] * m.m[3][0] + m.m[1][2] * m.m[2][0] * m.m[3][3] + m.m[1][0] * m.m[2][3] * m.m[3][2]) / lal;

	tmp.m[1][1] = (m.m[0][0] * m.m[2][2] * m.m[3][3] + m.m[0][2] * m.m[2][3] * m.m[3][0] + m.m[0][3] * m.m[2][0] * m.m[3][2]
		- m.m[0][3] * m.m[2][2] * m.m[3][0] - m.m[0][2] * m.m[2][0] * m.m[3][3] - m.m[0][0] * m.m[2][3] * m.m[3][2]) / lal;

	tmp.m[1][2] = (-m.m[0][0] * m.m[1][2] * m.m[3][3] - m.m[0][2] * m.m[1][3] * m.m[3][0] - m.m[0][3] * m.m[1][0] * m.m[3][2]
		+ m.m[0][3] * m.m[1][2] * m.m[3][0] + m.m[0][2] * m.m[1][0] * m.m[3][3] + m.m[0][0] * m.m[1][3] * m.m[3][2]) / lal;

	tmp.m[1][3] = (m.m[0][0] * m.m[1][2] * m.m[2][3] + m.m[0][2] * m.m[1][3] * m.m[2][0] + m.m[0][3] * m.m[1][0] * m.m[2][2]
		- m.m[0][3] * m.m[1][2] * m.m[2][0] - m.m[0][2] * m.m[1][0] * m.m[2][3] - m.m[0][0] * m.m[1][3] * m.m[2][2]) / lal;


	tmp.m[2][0] = (m.m[1][0] * m.m[2][1] * m.m[3][3] + m.m[1][1] * m.m[2][3] * m.m[3][0] + m.m[1][3] * m.m[2][0] * m.m[3][1]
		- m.m[1][3] * m.m[2][1] * m.m[3][0] - m.m[1][1] * m.m[2][0] * m.m[3][3] - m.m[1][0] * m.m[2][3] * m.m[3][1]) / lal;

	tmp.m[2][1] = (-m.m[0][0] * m.m[2][1] * m.m[3][3] - m.m[0][1] * m.m[2][3] * m.m[3][0] - m.m[0][3] * m.m[2][0] * m.m[3][1]
		+ m.m[0][3] * m.m[2][1] * m.m[3][0] + m.m[0][1] * m.m[2][0] * m.m[3][3] + m.m[0][0] * m.m[2][3] * m.m[3][1]) / lal;

	tmp.m[2][2] = (m.m[0][0] * m.m[1][1] * m.m[3][3] + m.m[0][1] * m.m[1][3] * m.m[3][0] + m.m[0][3] * m.m[1][0] * m.m[3][1]
		- m.m[0][3] * m.m[1][1] * m.m[3][0] - m.m[0][1] * m.m[1][0] * m.m[3][3] - m.m[0][0] * m.m[1][3] * m.m[3][1]) / lal;

	tmp.m[2][3] = (-m.m[0][0] * m.m[1][1] * m.m[2][3] - m.m[0][1] * m.m[1][3] * m.m[2][0] - m.m[0][3] * m.m[1][0] * m.m[2][1]
		+ m.m[0][3] * m.m[1][1] * m.m[2][0] + m.m[0][1] * m.m[1][0] * m.m[2][3] + m.m[0][0] * m.m[1][3] * m.m[2][1]) / lal;


	tmp.m[3][0] = (-m.m[1][0] * m.m[2][1] * m.m[3][2] - m.m[1][1] * m.m[2][2] * m.m[3][0] - m.m[1][2] * m.m[2][0] * m.m[3][1]
		+ m.m[1][2] * m.m[2][1] * m.m[3][0] + m.m[1][1] * m.m[2][0] * m.m[3][2] + m.m[1][0] * m.m[2][2] * m.m[3][1]) / lal;

	tmp.m[3][1] = (m.m[0][0] * m.m[2][1] * m.m[3][2] + m.m[0][1] * m.m[2][2] * m.m[3][0] + m.m[0][2] * m.m[2][0] * m.m[3][1]
		- m.m[0][2] * m.m[2][1] * m.m[3][0] - m.m[0][1] * m.m[2][0] * m.m[3][2] - m.m[0][0] * m.m[2][2] * m.m[3][1]) / lal;

	tmp.m[3][2] = (-m.m[0][0] * m.m[1][1] * m.m[3][2] - m.m[0][1] * m.m[1][2] * m.m[3][0] - m.m[0][2] * m.m[1][0] * m.m[3][1]
		+ m.m[0][2] * m.m[1][1] * m.m[3][0] + m.m[0][1] * m.m[1][0] * m.m[3][2] + m.m[0][0] * m.m[1][2] * m.m[3][1]) / lal;

	tmp.m[3][3] = (m.m[0][0] * m.m[1][1] * m.m[2][2] + m.m[0][1] * m.m[1][2] * m.m[2][0] + m.m[0][2] * m.m[1][0] * m.m[2][1]
		- m.m[0][2] * m.m[1][1] * m.m[2][0] - m.m[0][1] * m.m[1][0] * m.m[2][2] - m.m[0][0] * m.m[1][2] * m.m[2][1]) / lal;

	return tmp;
}
Matrix4x4 Transpose(const Matrix4x4& m) {
	Matrix4x4 tmp;
	tmp.m[0][0] = m.m[0][0];
	tmp.m[0][1] = m.m[1][0];
	tmp.m[0][2] = m.m[2][0];
	tmp.m[0][3] = m.m[3][0];

	tmp.m[1][0] = m.m[0][1];
	tmp.m[1][1] = m.m[1][1];
	tmp.m[1][2] = m.m[2][1];
	tmp.m[1][3] = m.m[3][1];

	tmp.m[2][0] = m.m[0][2];
	tmp.m[2][1] = m.m[1][2];
	tmp.m[2][2] = m.m[2][2];
	tmp.m[2][3] = m.m[3][2];

	tmp.m[3][0] = m.m[0][3];
	tmp.m[3][1] = m.m[1][3];
	tmp.m[3][2] = m.m[2][3];
	tmp.m[3][3] = m.m[3][3];

	return tmp;
}
Matrix4x4 MakeIdentity4x4() {
	Matrix4x4 tmp;
	tmp.m[0][0] = 1.0f;
	tmp.m[0][1] = 0.0f;
	tmp.m[0][2] = 0.0f;
	tmp.m[0][3] = 0.0f;

	tmp.m[1][0] = 0.0f;
	tmp.m[1][1] = 1.0f;
	tmp.m[1][2] = 0.0f;
	tmp.m[1][3] = 0.0f;

	tmp.m[2][0] = 0.0f;
	tmp.m[2][1] = 0.0f;
	tmp.m[2][2] = 1.0f;
	tmp.m[2][3] = 0.0f;

	tmp.m[3][0] = 0.0f;
	tmp.m[3][1] = 0.0f;
	tmp.m[3][2] = 0.0f;
	tmp.m[3][3] = 1.0f;

	return tmp;
}
void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* message) {
	Novice::ScreenPrintf(x, y, message);
	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(x + column * kColumnWidth, y + row * kRowHeight + kRowHeight, "%6.02f", matrix.m[row][column]);
		}
	}
}
Matrix4x4 MakeTranslateMatrix(const Vector3& translate) {
	Matrix4x4 tmp;
	tmp.m[0][0] = 1;
	tmp.m[0][1] = 0;
	tmp.m[0][2] = 0;
	tmp.m[0][3] = 0;

	tmp.m[1][0] = 0;
	tmp.m[1][1] = 1;
	tmp.m[1][2] = 0;
	tmp.m[1][3] = 0;

	tmp.m[2][0] = 0;
	tmp.m[2][1] = 0;
	tmp.m[2][2] = 1;
	tmp.m[2][3] = 0;

	tmp.m[3][0] = translate.x;
	tmp.m[3][1] = translate.y;
	tmp.m[3][2] = translate.z;
	tmp.m[3][3] = 1;

	return tmp;

}
Matrix4x4 MakeScaleMatrix(const Vector3& scale) {
	Matrix4x4 tmp;
	tmp.m[0][0] = scale.x;
	tmp.m[0][1] = 0;
	tmp.m[0][2] = 0;
	tmp.m[0][3] = 0;

	tmp.m[1][0] = 0;
	tmp.m[1][1] = scale.y;
	tmp.m[1][2] = 0;
	tmp.m[1][3] = 0;

	tmp.m[2][0] = 0;
	tmp.m[2][1] = 0;
	tmp.m[2][2] = scale.z;
	tmp.m[2][3] = 0;

	tmp.m[3][0] = 0;
	tmp.m[3][1] = 0;
	tmp.m[3][2] = 0;
	tmp.m[3][3] = 1;

	return tmp;
}
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix) {
	Vector3 result;
	result.x = vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + 1.0f * matrix.m[3][0];
	result.y = vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + 1.0f * matrix.m[3][1];
	result.z = vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + 1.0f * matrix.m[3][2];
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + 1.0f * matrix.m[3][3];

	assert(w != 0.0f);
	result.x /= w;
	result.y /= w;
	result.z /= w;

	return result;

}



Matrix4x4 MakeRotateXMatrix(float radian) {

	Matrix4x4 tmp;

	tmp.m[0][0] = 1;
	tmp.m[0][1] = 0;
	tmp.m[0][2] = 0;
	tmp.m[0][3] = 0;

	tmp.m[1][0] = 0;
	tmp.m[1][1] = std::cos(radian);
	tmp.m[1][2] = std::sin(radian);
	tmp.m[1][3] = 0;

	tmp.m[2][0] = 0;
	tmp.m[2][1] = -std::sin(radian);
	tmp.m[2][2] = std::cos(radian);
	tmp.m[2][3] = 0;

	tmp.m[3][0] = 0;
	tmp.m[3][1] = 0;
	tmp.m[3][2] = 0;
	tmp.m[3][3] = 1;

	return tmp;
}
Matrix4x4 MakeRotateYMatrix(float radian) {

	Matrix4x4 tmp;

	tmp.m[0][0] = std::cos(radian);
	tmp.m[0][1] = 0;
	tmp.m[0][2] = -std::sin(radian);
	tmp.m[0][3] = 0;

	tmp.m[1][0] = 0;
	tmp.m[1][1] = 1;
	tmp.m[1][2] = 0;
	tmp.m[1][3] = 0;

	tmp.m[2][0] = std::sin(radian);
	tmp.m[2][1] = 0;
	tmp.m[2][2] = std::cos(radian);
	tmp.m[2][3] = 0;

	tmp.m[3][0] = 0;
	tmp.m[3][1] = 0;
	tmp.m[3][2] = 0;
	tmp.m[3][3] = 1;

	return tmp;
}
Matrix4x4 MakeRotateZMatrix(float radian) {

	Matrix4x4 tmp;

	tmp.m[0][0] = std::cos(radian);
	tmp.m[0][1] = std::sin(radian);
	tmp.m[0][2] = 0;
	tmp.m[0][3] = 0;

	tmp.m[1][0] = -sinf(radian);
	tmp.m[1][1] = std::cos(radian);
	tmp.m[1][2] = 0;
	tmp.m[1][3] = 0;

	tmp.m[2][0] = 0;
	tmp.m[2][1] = 0;
	tmp.m[2][2] = 1;
	tmp.m[2][3] = 0;

	tmp.m[3][0] = 0;
	tmp.m[3][1] = 0;
	tmp.m[3][2] = 0;
	tmp.m[3][3] = 1;

	return tmp;
}
Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate) {
	Matrix4x4 scaleMatrix = MakeScaleMatrix(scale);
	Matrix4x4 rotateXMatrix = MakeRotateXMatrix(rotate.x);
	Matrix4x4 rotateYMatrix = MakeRotateYMatrix(rotate.y);
	Matrix4x4 rotateZMatrix = MakeRotateZMatrix(rotate.z);
	Matrix4x4 translateMatrix = MakeTranslateMatrix(translate);

	Matrix4x4 tmp = scaleMatrix * rotateXMatrix * rotateYMatrix * rotateZMatrix * translateMatrix;

	return tmp;
}
inline Matrix4x4& operator*=(Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 tmp;
	tmp.m[0][0] = m1.m[0][0] * m2.m[0][0] + m1.m[0][1] * m2.m[1][0] + m1.m[0][2] * m2.m[2][0] +
		m1.m[0][3] * m2.m[3][0];
	tmp.m[0][1] = m1.m[0][0] * m2.m[0][1] + m1.m[0][1] * m2.m[1][1] + m1.m[0][2] * m2.m[2][1] +
		m1.m[0][3] * m2.m[3][1];
	tmp.m[0][2] = m1.m[0][0] * m2.m[0][2] + m1.m[0][1] * m2.m[1][2] + m1.m[0][2] * m2.m[2][2] +
		m1.m[0][3] * m2.m[3][2];
	tmp.m[0][3] = m1.m[0][0] * m2.m[0][3] + m1.m[0][1] * m2.m[1][3] + m1.m[0][2] * m2.m[2][3] +
		m1.m[0][3] * m2.m[3][3];

	tmp.m[1][0] = m1.m[1][0] * m2.m[0][0] + m1.m[1][1] * m2.m[1][0] + m1.m[1][2] * m2.m[2][0] +
		m1.m[1][3] * m2.m[3][0];
	tmp.m[1][1] = m1.m[1][0] * m2.m[0][1] + m1.m[1][1] * m2.m[1][1] + m1.m[1][2] * m2.m[2][1] +
		m1.m[1][3] * m2.m[3][1];
	tmp.m[1][2] = m1.m[1][0] * m2.m[0][2] + m1.m[1][1] * m2.m[1][2] + m1.m[1][2] * m2.m[2][2] +
		m1.m[1][3] * m2.m[3][2];
	tmp.m[1][3] = m1.m[1][0] * m2.m[0][3] + m1.m[1][1] * m2.m[1][3] + m1.m[1][2] * m2.m[2][3] +
		m1.m[1][3] * m2.m[3][3];

	tmp.m[2][0] = m1.m[2][0] * m2.m[0][0] + m1.m[2][1] * m2.m[1][0] + m1.m[2][2] * m2.m[2][0] +
		m1.m[2][3] * m2.m[3][0];
	tmp.m[2][1] = m1.m[2][0] * m2.m[0][1] + m1.m[2][1] * m2.m[1][1] + m1.m[2][2] * m2.m[2][1] +
		m1.m[2][3] * m2.m[3][1];
	tmp.m[2][2] = m1.m[2][0] * m2.m[0][2] + m1.m[2][1] * m2.m[1][2] + m1.m[2][2] * m2.m[2][2] +
		m1.m[2][3] * m2.m[3][2];
	tmp.m[2][3] = m1.m[2][0] * m2.m[0][3] + m1.m[2][1] * m2.m[1][3] + m1.m[2][2] * m2.m[2][3] +
		m1.m[2][3] * m2.m[3][3];

	tmp.m[3][0] = m1.m[3][0] * m2.m[0][0] + m1.m[3][1] * m2.m[1][0] + m1.m[3][2] * m2.m[2][0] +
		m1.m[3][3] * m2.m[3][0];
	tmp.m[3][1] = m1.m[3][0] * m2.m[0][1] + m1.m[3][1] * m2.m[1][1] + m1.m[3][2] * m2.m[2][1] +
		m1.m[3][3] * m2.m[3][1];
	tmp.m[3][2] = m1.m[3][0] * m2.m[0][2] + m1.m[3][1] * m2.m[1][2] + m1.m[3][2] * m2.m[2][2] +
		m1.m[3][3] * m2.m[3][2];
	tmp.m[3][3] = m1.m[3][0] * m2.m[0][3] + m1.m[3][1] * m2.m[1][3] + m1.m[3][2] * m2.m[2][3] +
		m1.m[3][3] * m2.m[3][3];
	m1 = tmp;
	return m1;

	
}

// 透視投影行列
inline Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip) {
	Matrix4x4 tmp;
	float cot = 1.0f / std::tan(fovY * 0.5f);
	tmp.m[0][0] = 1.0f / aspectRatio * cot;
	tmp.m[0][1] = 0;
	tmp.m[0][2] = 0;
	tmp.m[0][3] = 0;
	tmp.m[1][0] = 0;
	tmp.m[1][1] = cot;
	tmp.m[1][2] = 0;
	tmp.m[1][3] = 0;
	tmp.m[2][0] = 0;
	tmp.m[2][1] = 0;
	tmp.m[2][2] = farClip / (farClip - nearClip);
	tmp.m[2][3] = 1.0f;
	tmp.m[3][0] = 0;
	tmp.m[3][1] = 0;
	tmp.m[3][2] = -nearClip * farClip / (farClip - nearClip);
	tmp.m[3][3] = 0;

	return tmp;
}
//正射影行列
inline Matrix4x4 MakeOrthograohicmatrix(float left, float top, float right, float bottom,float nearClip,float farClip) {
	Matrix4x4 tmp;
	tmp.m[0][0] = 2.0f / (right - left);
	tmp.m[0][1] = 0;
	tmp.m[0][2] = 0;
	tmp.m[0][3] = 0;
	tmp.m[1][0] = 0;
	tmp.m[1][1] = 2.0f / (top - bottom);
	tmp.m[1][2] = 0;
	tmp.m[1][3] = 0;
	tmp.m[2][0] = 0;
	tmp.m[2][1] = 0;
	tmp.m[2][2] = 1.0f / (farClip - nearClip);
	tmp.m[2][3] = 0;
	tmp.m[3][0] = (left + right) / (left - right);
	tmp.m[3][1] = (top + bottom) / (bottom - top);
	tmp.m[3][2] = nearClip / (nearClip - farClip);
	tmp.m[3][3] = 1.0f;

	return tmp;

}
//ビューポート変換行列
inline Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height,float minDepth , float maxDepth) {
	Matrix4x4 tmp;
	tmp.m[0][0] = width / 2.0f;
	tmp.m[0][1] = 0;
	tmp.m[0][2] = 0;
	tmp.m[0][3] = 0;
	tmp.m[1][0] = 0;
	tmp.m[1][1] = -height / 2.0f;
	tmp.m[1][2] = 0;
	tmp.m[1][3] = 0;
	tmp.m[2][0] = 0;
	tmp.m[2][1] = 0;
	tmp.m[2][2] = maxDepth - minDepth;
	tmp.m[2][3] = 0;
	tmp.m[3][0] = left + (width / 2.0f);
	tmp.m[3][1] = top  + (height / 2.0f);
	tmp.m[3][2] = minDepth;
	tmp.m[3][3] = 1;
	return tmp;
}

#pragma endregion

