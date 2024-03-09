#define _USE_MATH_DEFINES
#include <Novice.h>
#include "Mymath.h"
#include "Key.h"
#include <cmath>
#include <imgui.h>
#include <Input.h>
#include <algorithm>
#include <numbers>
const char kWindowTitle[] = "学籍番号";
const int kWindowWidth = 1280;
const int kWindowHeight = 720;


struct Plane {
	Vector3 normal;//法線
	float distance;//距離
};
struct Sphere {
	Vector3 center; // 中心点
	float radius; //半径
};
struct Triangle {
	Vector3 vertices[3];
};
struct AABB {
	Vector3 min;//最小点
	Vector3 max;//最大点
};
struct OBB {
	Vector3 center; //中心点
	Vector3 orientations[3];//座標軸、正規化、直交座標
	Vector3 size;//座標軸方向の長さの半分。中心から面までの距離
};

struct Pendulum {
	Vector3 anchor; //アンカーポイント。固定された端の位置
	float length;	//紐の長さ
	float angle;	//現在の角度
	float angularVelocity;	//各深度
	float angularAcceleration;//各加速度
};

struct ConicalPendulum {
	Vector3 anchor;			//アンカーポイント
	float length;			//紐の長さ
	float halfApexAngle;	//円錐の頂点の半分
	float angle;			//現在の角度
	float angularVelocity;	//角速度w
};

Vector3 Reflect(const Vector3& input, const Vector3 normal) {
	Vector3 result = input - 2.0f * Dot(input, normal) * normal;
	return result;
}

bool IsCollision(const AABB& aabb1, const AABB& aabb2) {
	if ((aabb1.min.x <= aabb2.max.x && aabb1.max.x >= aabb2.min.x) &&
		(aabb1.min.y <= aabb2.max.y && aabb1.max.y >= aabb2.min.y) &&
		(aabb1.min.z <= aabb2.max.z && aabb1.max.z >= aabb2.min.z)){
		return true;
	}
	return false;
}

bool IsCollision(const AABB& aabb, const  Sphere& sphere) {
	//最近接点を求める
	Vector3 closestPoint
	{ std::clamp(sphere.center.x,aabb.min.x,aabb.max.x),
	 std::clamp(sphere.center.y,aabb.min.y,aabb.max.y),
	 std::clamp(sphere.center.z,aabb.min.z,aabb.max.z)
	};
	//最近接点と球の中心との距離を求める
	float distance = Length(closestPoint - sphere.center);
	//距離が半径よりも小さければ衝突
	if (distance <= sphere.radius) {
		return true;
	}
	return false;
}

bool IsCollision(const OBB& obb, Sphere& sphere) {
	Matrix4x4 obbWorldMatrix = { obb.orientations[0].x,obb.orientations[0].y,obb.orientations[0].z , 0.0f,
								obb.orientations[1].x,obb.orientations[1].y,obb.orientations[1].z , 0.0f,
								obb.orientations[2].x,obb.orientations[2].y,obb.orientations[2].z , 0.0f,
								obb.center.x,obb.center.y, obb.center.z,1.0f };
	Vector3 centerInOBBLocalSpace = Transform(sphere.center, Inverse(obbWorldMatrix));
	AABB aabbOBBLocal{ .min = -obb.size,.max = obb.size };
	Sphere sphereOBBLocal{ centerInOBBLocalSpace,
	sphere.radius };
	//ローカル空間で衝突判定
	return IsCollision(aabbOBBLocal, sphereOBBLocal);
}

bool IsCollision(const OBB& obb1, const OBB& obb2) {
	Vector3 separationAxisCandidate[15];
	//各軸
	separationAxisCandidate[0] = obb1.orientations[0];
	separationAxisCandidate[1] = obb1.orientations[1];
	separationAxisCandidate[2] = obb1.orientations[2];
	separationAxisCandidate[3] = obb2.orientations[0];
	separationAxisCandidate[4] = obb2.orientations[1];
	separationAxisCandidate[5] = obb2.orientations[2];
	//各辺のクロス積
	separationAxisCandidate[6] = Cross(obb1.orientations[0], obb2.orientations[0]);
	separationAxisCandidate[7] = Cross(obb1.orientations[0], obb2.orientations[1]);
	separationAxisCandidate[8] = Cross(obb1.orientations[0], obb2.orientations[2]);
	separationAxisCandidate[9] = Cross(obb1.orientations[1], obb2.orientations[0]);
	separationAxisCandidate[10] = Cross(obb1.orientations[1], obb2.orientations[1]);
	separationAxisCandidate[11] = Cross(obb1.orientations[1], obb2.orientations[2]);
	separationAxisCandidate[12] = Cross(obb1.orientations[2], obb2.orientations[0]);
	separationAxisCandidate[13] = Cross(obb1.orientations[2], obb2.orientations[1]);
	separationAxisCandidate[14] = Cross(obb1.orientations[2], obb2.orientations[2]);
	//頂点
	Matrix4x4 obb1WorldMatrix = { obb1.orientations[0].x,obb1.orientations[0].y,obb1.orientations[0].z , 0.0f,
								obb1.orientations[1].x,obb1.orientations[1].y,obb1.orientations[1].z , 0.0f,
								obb1.orientations[2].x,obb1.orientations[2].y,obb1.orientations[2].z , 0.0f,
								obb1.center.x,obb1.center.y, obb1.center.z,1.0f };
	Matrix4x4 obb2WorldMatrix = { obb2.orientations[0].x,obb2.orientations[0].y,obb2.orientations[0].z , 0.0f,
								obb2.orientations[1].x,obb2.orientations[1].y,obb2.orientations[1].z , 0.0f,
								obb2.orientations[2].x,obb2.orientations[2].y,obb2.orientations[2].z , 0.0f,
								obb2.center.x,obb2.center.y, obb2.center.z,1.0f };

	Vector3 vertices1[] = { -obb1.size,
		{obb1.size.x,-obb1.size.y,-obb1.size.z},
		{obb1.size.x,-obb1.size.y,obb1.size.z},
		{-obb1.size.x,-obb1.size.y,obb1.size.z},
		{-obb1.size.x,obb1.size.y,-obb1.size.z},
		{obb1.size.x,obb1.size.y,-obb1.size.z},
		obb1.size,
		{-obb1.size.x,obb1.size.y,obb1.size.z }
	};

	Vector3 vertices2[] = { -obb2.size,
		{obb2.size.x,-obb2.size.y,-obb2.size.z},
		{obb2.size.x,-obb2.size.y,obb2.size.z},
		{-obb2.size.x,-obb2.size.y,obb2.size.z},
		{-obb2.size.x,obb2.size.y,-obb2.size.z},
		{obb2.size.x,obb2.size.y,-obb2.size.z},
		obb2.size,
		{-obb2.size.x,obb2.size.y,obb2.size.z }
	};

	for (int i = 0; i < 8; i++) {
		vertices1[i] = vertices1[i] * obb1WorldMatrix;
		vertices2[i] = vertices2[i] * obb2WorldMatrix;
	}

	//各軸
	for (int i = 0; i < 15; i++) {
		//影の長さの合計
		float sumSpan;
		//2つの影の両端の差分
		float longSpan;
		//射影した最大値最小値
		float max1, min1, max2, min2;
		//差分が形状を分離軸に射影した長さ
		float L1, L2;
		//すべての頂点を射影した値
		float Dot1[8];
		float Dot2[8];
		//各頂点
		for (int j = 0; j < 8; j++) {
			Dot1[j] = Dot(separationAxisCandidate[i], vertices1[j]);
		}
		for (int k = 0; k < 8; k++) {
			Dot2[k] = Dot(separationAxisCandidate[i], vertices2[k]);
		}
		max1 = (std::max)({ Dot1[0], Dot1[1], Dot1[3], Dot1[4], Dot1[5], Dot1[6], Dot1[7] });
		min1 = (std::min)({ Dot1[0], Dot1[1], Dot1[3], Dot1[4], Dot1[5], Dot1[6], Dot1[7] });
		L1 = max1 - min1;
		max2 = (std::max)({ Dot2[0], Dot2[1], Dot2[3], Dot2[4], Dot2[5], Dot2[6], Dot2[7] });
		min2 = (std::min)({ Dot2[0], Dot2[1], Dot2[3], Dot2[4], Dot2[5], Dot2[6], Dot2[7] });
		L2 = max2 - min2;
		
		sumSpan = L1 + L2;
		longSpan = (std::max)(max1, max2) - (std::min)(min1, min2);
		if (sumSpan < longSpan) {
			return false;
		}
	}
	return true;
};

bool IsCollision(const Segment& segment, const Plane& plane , Vector3& interSectionPoint) {
	//まず垂直判定を行うために、法線と線の内積を求める
	float dot = Dot(plane.normal, segment.diff);
	//垂直 = 並行で絵あるので、衝突しているはずがない
	if (dot == 0.0f) {
		return false;
	}
	//tを求める
	float t = (plane.distance - Dot(segment.origin, plane.normal)) / dot;
	if (t >= 0.0f && t <= 1.0f) {
		interSectionPoint = segment.origin + t * segment.diff;
		return true;
	}
	return false;
}

bool IsCollision(const Ray& ray, const Plane& plane, Vector3& interSectionPoint) {
	//まず垂直判定を行うために、法線と線の内積を求める
	float dot = Dot(plane.normal, ray.diff);
	//垂直 = 並行で絵あるので、衝突しているはずがない
	if (dot == 0.0f) {
		return false;
	}
	//tを求める
	float t = (plane.distance - Dot(ray.origin, plane.normal)) / dot;
	if (t <= 0.0f) {
		interSectionPoint = ray.origin + t * ray.diff;
		return true;
	}
	return false;
}

bool IsCollision(const Line& line, const Plane& plane, Vector3& interSectionPoint) {
	//まず垂直判定を行うために、法線と線の内積を求める
	float dot = Dot(plane.normal, line.diff);
	//垂直 = 並行で絵あるので、衝突しているはずがない
	if (dot == 0.0f) {
		return false;
	}
	//tを求める
	float t = (plane.distance - Dot(line.origin, plane.normal)) / dot;
	interSectionPoint = line.origin + t * line.diff;
		
	return true;
}

Vector3 Normal(const Vector3& a,const Vector3& b, const Vector3 c) {
	return Normalize(Cross(b - a, c - b));
}

bool IsCollision(const Triangle& triangle, const Segment& segment) {

	Vector3 normal = Normal(triangle.vertices[0], triangle.vertices[1], triangle.vertices[2]);

	//まず垂直判定を行うために、法線と線の内積を求める
	float dot = Dot(normal, segment.diff);
	//垂直 = 並行で絵あるので、衝突しているはずがない
	if (dot == 0.0f) {
		return false;
	}
	float distance = Dot(triangle.vertices[0], normal);
	//tを求める
	float t = (distance - Dot(segment.origin, normal)) / dot;
	//衝突点を求める
	Vector3 point = segment.origin + segment.diff * t;
	//各辺を結んだベクトルと、頂点と衝突店ｐを結んだベクトルのクロス積をとる
	Vector3 cross01 = Cross(triangle.vertices[1] - triangle.vertices[0], point- triangle.vertices[1]);
	Vector3 cross12 = Cross(triangle.vertices[2] - triangle.vertices[1], point - triangle.vertices[2]);
	Vector3 cross20 = Cross(triangle.vertices[0] - triangle.vertices[2], point - triangle.vertices[0]);
	//すべての三角形のクロス積と法線が同じ方向を向いていたら衝突	
	if (Dot(cross01, normal) >= 0.0f &&
		Dot(cross12, normal) >= 0.0f &&
		Dot(cross20, normal) >= 0.0f) {
		return true;
	}
	return false;
}

bool IsCollision(const Sphere& s1, const Sphere& s2) {
	float length = Length(s1.center - s2.center);
	if (s1.radius + s2.radius >= length) {
		return true;
	}
	return false;
}


bool IsCollision(const Sphere& sphere, const Plane& plane) {
	float k = std::abs(Dot(plane.normal,sphere.center) - plane.distance );
	if (k <= sphere.radius) {
		return true;
	}
	return false;
}


bool IsCollision(const AABB& aabb,const Segment& segment) {
	float txMin = (aabb.min.x - segment.origin.x) / segment.diff.x;
	float txMax = (aabb.max.x - segment.origin.x) / segment.diff.x;

	float tyMin = (aabb.min.y - segment.origin.y) / segment.diff.y;
	float tyMax = (aabb.max.y - segment.origin.y) / segment.diff.y;

	float tzMin = (aabb.min.z - segment.origin.z) / segment.diff.z;
	float tzMax = (aabb.max.z - segment.origin.z) / segment.diff.z;

	float tNearX = (std::min)(txMin, txMax);
	float tFarX = (std::max)(txMin, txMax);

	float tNearY = (std::min)(tyMin, tyMax);
	float tFarY = (std::max)(tyMin, tyMax);

	float tNearZ = (std::min)(tzMin, tzMax);
	float tFarZ = (std::max)(tzMin, tzMax);

	float tmin = (std::max)((std::max)(tNearX, tNearY), tNearZ);
	float tmax = (std::min)((std::min)(tFarX, tFarY), tFarZ);

	if (tmin > tmax) {
		return false;
	}
	if (tmin < 0.0f && tmax < 0.0f || tmin > 1.0f && tmax > 1.0f) {
		return false;
	}
	
	return true;
}

bool IsCollision(const OBB& obb, const Segment& segment) {
	Matrix4x4 obbWorldMatrix = { obb.orientations[0].x,obb.orientations[0].y,obb.orientations[0].z , 0.0f,
								obb.orientations[1].x,obb.orientations[1].y,obb.orientations[1].z , 0.0f,
								obb.orientations[2].x,obb.orientations[2].y,obb.orientations[2].z , 0.0f,
								obb.center.x,obb.center.y, obb.center.z,1.0f };
	Matrix4x4 obbInverseWorldMatrix = Inverse(obbWorldMatrix);
	Vector3 obbLocalsegmentOrigin = Transform(segment.origin, obbInverseWorldMatrix);
	Vector3 obbLocalsegmentLast = Transform(segment.origin + segment.diff, obbInverseWorldMatrix);
	Vector3 obbLocalsegmentDiff = obbLocalsegmentLast - obbLocalsegmentOrigin;
	AABB aabbOBBLocal{ .min = -obb.size,.max = obb.size };
	Segment segmentOBBLocal{ .origin{obbLocalsegmentOrigin},.diff{ obbLocalsegmentDiff} };
	//ローカル空間で衝突判定
	return IsCollision(aabbOBBLocal, segmentOBBLocal);
}

Vector3 Perpendicular(const Vector3& vector) {
	if (vector.x != 0.0f || vector.y != 0.0f) {
		return { -vector.y,vector.x,0.0f };
	}
	return { 0.0f,-vector.z,vector.y };
}

void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 center = Multiply(plane.distance, plane.normal);
	Vector3 perpendiculars[4];
	perpendiculars[0] = Normalize(Perpendicular(plane.normal));
	perpendiculars[1] = Cross(plane.normal, perpendiculars[0]);
	perpendiculars[2] = -perpendiculars[0];
	perpendiculars[3] = -perpendiculars[1];

	Vector3 points[4];
	for (int32_t index = 0; index < 4; ++index) {
		Vector3 extend = Multiply(2.0f,perpendiculars[index]);
		Vector3 point = Add(center, extend);
		points[index] = Transform(Transform(point, viewProjectionMatrix),viewportMatrix);
	}
	Novice::DrawLine(static_cast<int>(points[0].x), static_cast<int>(points[0].y), static_cast<int>(points[1].x), static_cast<int>(points[1].y), color);
	Novice::DrawLine(static_cast<int>(points[1].x), static_cast<int>(points[1].y), static_cast<int>(points[2].x), static_cast<int>(points[2].y), color);
	Novice::DrawLine(static_cast<int>(points[2].x), static_cast<int>(points[2].y), static_cast<int>(points[3].x), static_cast<int>(points[3].y), color);
	Novice::DrawLine(static_cast<int>(points[3].x), static_cast<int>(points[3].y), static_cast<int>(points[0].x), static_cast<int>(points[0].y), color);
}

void CameraMove(Vector3& cameracenter , Vector3& cameraRotate) {
	Input* input = Input::GetInstance();

	auto mouseMove = input->GetMouseMove();
	auto wheel = input->GetWheel();

	if (input->IsPressMouse(1)) {
		float rot = static_cast<float>(M_PI / 180.0f);
		cameraRotate.x += rot * mouseMove.lY * 0.1f;
		cameraRotate.y += rot * mouseMove.lX * 0.1f;
	}
	else if (input->IsPressMouse(2)) {
		Matrix4x4 rotMat = MakeRotateXYZMatrix(cameraRotate);
		Vector3 cameraX = GetXAxis(rotMat) * static_cast<float>(-mouseMove.lX) * 0.01f;
		Vector3 cameraY = GetYAxis(rotMat) * static_cast<float>(mouseMove.lY) * 0.01f;
		cameracenter += cameraX + cameraY;
	}
	else if (wheel != 0) {
		Matrix4x4 rotMat = MakeRotateXYZMatrix(cameraRotate);
		Vector3 cameraZ = GetZAxis(rotMat) * (static_cast<float>(wheel / 120) * 0.5f);
		cameracenter += cameraZ;
	}
	ImGui::Begin("Camera");
	ImGui::DragFloat3("Camera centerition", &cameracenter.x, 0.01f);
	ImGui::DragFloat3("Camera rotate", &cameraRotate.x, 0.01f);
	ImGui::End();
}

void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalWidth = 2.0f;//Grid1の半分の幅
	const uint32_t kSubdivision = 10;//分割数
	const float kGirdEvery = (kGridHalWidth * 2.0f) / float(kSubdivision);//一つ分の長さ
	//奥から手前への線を順々に引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		//上の情報を使ってワールド座標上の始点と終点を求める
		Vector3 start = { kGridHalWidth - kGirdEvery * xIndex,0.0f , kGridHalWidth };
		Vector3 end = { kGridHalWidth - kGirdEvery * xIndex ,0.0f ,-kGridHalWidth };
		//スクリーン座標系まで変換をかける
		start = start * viewProjectionMatrix;
		start = start * viewportMatrix;
		end = end * viewProjectionMatrix;
		end = end * viewportMatrix;
		//変換した座標を使って表示。色は薄い灰色(0xAAAAAAFF)
		if (kSubdivision / 2 == xIndex) {
			Novice::DrawLine(int(start.x), int(start.y), int(end.x), int(end.y), 0x000000FF);
		}
		else {
			Novice::DrawLine(int(start.x), int(start.y), int(end.x), int(end.y), 0xAAAAAAFF);
		}
	}
	//左から右と同じように順々に引いていく
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		//上の情報を使ってワールド座標上の始点と終点を求める
		Vector3 start = { kGridHalWidth ,0.0f ,kGridHalWidth - kGirdEvery * zIndex };
		Vector3 end = { -kGridHalWidth ,0.0f ,kGridHalWidth - kGirdEvery * zIndex };
		//スクリーン座標系まで変換をかける
		start = start * viewProjectionMatrix * viewportMatrix;
		end = end * viewProjectionMatrix * viewportMatrix;
		//変換した座標を使って表示。色は薄い灰色(0xAAAAAAFF)
		if (kSubdivision / 2  == zIndex) {
			Novice::DrawLine(int(start.x), int(start.y), int(end.x), int(end.y), 0x000000FF);
		}
		else {
			Novice::DrawLine(int(start.x), int(start.y), int(end.x), int(end.y), 0xAAAAAAFF);
		}
	}
}
void DrawSphere(const Sphere& sphere,const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix,uint32_t color) {
	const uint32_t kSubdivision = 15; //分割数
	const float kLonEvery = float (2.0f * M_PI / kSubdivision) ;//経度分割一つ分の角度
	const float kLatEvery = float (M_PI / kSubdivision );//緯度分割一つ分の角度
	//緯度の方向に分割 -π/2 ～	π/2
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {
		float lat = float ( - M_PI / 2.0f + kLatEvery * latIndex );//現在の緯度
		//軽度の方向に分割 0 ～2π
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
			float lon = lonIndex * kLonEvery;//現在の経度
			//world座標系でのa,b,cを求める
			Vector3 a, b, c;
			a = {std::cos(lat) * std::cos(lon),std::sin(lat),std::cos(lat) * std::sin(lon)};
			b = { std::cos(lat + kLatEvery) * std::cos(lon),std::sin(lat + kLatEvery),std::cos(lat + kLatEvery) * std::sin(lon) };
			c = { std::cos(lat) * std::cos(lon + kLonEvery),std::sin(lat),std::cos(lat) * std::sin(lon + kLonEvery) };
			a = a * sphere.radius + sphere.center;
			b = b * sphere.radius + sphere.center;
			c = c * sphere.radius + sphere.center;
			// a,b,cをscreen座標系まで変換
			a = a * viewProjectionMatrix * viewportMatrix;
			b = b * viewProjectionMatrix * viewportMatrix;
			c = c * viewProjectionMatrix * viewportMatrix;

			Novice::DrawLine(int(a.x), int(a.y), int(b.x), int(b.y), color);
			Novice::DrawLine(int(a.x), int(a.y), int(c.x), int(c.y), color);
		}
	}

}

void DrawLine(const Vector3& start, const Vector3& end, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix,unsigned int color) {
	Vector3 a = start * viewProjectionMatrix * viewportMatrix;
	Vector3 b = end * viewProjectionMatrix * viewportMatrix;

	Novice::DrawLine(int(a.x), int(a.y), int(b.x), int(b.y), color);
}

void DrawAABB(const AABB& aabb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, unsigned int color){
	Vector3 vertices[] = { aabb.min,
		{aabb.max.x,aabb.min.y,aabb.min.z},
		{aabb.max.x,aabb.min.y,aabb.max.z},
		{aabb.min.x,aabb.min.y,aabb.max.z},
		{aabb.min.x,aabb.max.y,aabb.min.z},
		{aabb.max.x,aabb.max.y,aabb.min.z},
		aabb.max,
		{aabb.min.x,aabb.max.y,aabb.max.z } };

	for (int i = 0; i < 4; i++) {
		int j = (i + 1) % 4;
		DrawLine(vertices[i], vertices[j],viewProjectionMatrix,viewportMatrix, color);
		DrawLine(vertices[i], vertices[i + 4], viewProjectionMatrix, viewportMatrix, color);
		DrawLine(vertices[i + 4], vertices[j + 4], viewProjectionMatrix, viewportMatrix, color);

	}
}

void DrawOBB(const OBB& obb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Matrix4x4 obbWorldMatrix = { obb.orientations[0].x,obb.orientations[0].y,obb.orientations[0].z , 0.0f,
								obb.orientations[1].x,obb.orientations[1].y,obb.orientations[1].z , 0.0f,
								obb.orientations[2].x,obb.orientations[2].y,obb.orientations[2].z , 0.0f,
								obb.center.x,obb.center.y, obb.center.z,1.0f };

	Vector3 vertices[] = { -obb.size,
		{obb.size.x,-obb.size.y,-obb.size.z},
		{obb.size.x,-obb.size.y,obb.size.z},
		{-obb.size.x,-obb.size.y,obb.size.z},
		{-obb.size.x,obb.size.y,-obb.size.z},
		{obb.size.x,obb.size.y,-obb.size.z},
		obb.size,
		{-obb.size.x,obb.size.y,obb.size.z } };

	for (int i = 0; i < 8; i++) {
		vertices[i] = vertices[i] * obbWorldMatrix;
	}

	for (int i = 0; i < 4; i++) {
		int j = (i + 1) % 4;
		DrawLine(vertices[i], vertices[j], viewProjectionMatrix, viewportMatrix, color);
		DrawLine(vertices[i], vertices[i + 4], viewProjectionMatrix, viewportMatrix, color);
		DrawLine(vertices[i + 4], vertices[j + 4], viewProjectionMatrix, viewportMatrix, color);
	}

}

void DrawFrustum(const Frustum& frustum, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	for (int i = 0; i < 6; i++) {
		DrawPlane(frustum.plane[i], viewProjectionMatrix, viewportMatrix, color);
	}
}


void DrawTriangle(const Triangle& triangle, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, unsigned int color) {
	Triangle tmp;
	for (int i = 0; i < 3; i++) {
		tmp.vertices[i] = triangle.vertices[i] * viewProjectionMatrix * viewportMatrix;
	}
	Novice::DrawLine(int(tmp.vertices[0].x), int(tmp.vertices[0].y), int(tmp.vertices[1].x), int(tmp.vertices[1].y), color);
	Novice::DrawLine(int(tmp.vertices[1].x), int(tmp.vertices[1].y), int(tmp.vertices[2].x), int(tmp.vertices[2].y), color);
	Novice::DrawLine(int(tmp.vertices[2].x), int(tmp.vertices[2].y), int(tmp.vertices[0].x), int(tmp.vertices[0].y), color);
}

float easing(float t, float start, float end) {

	return((1.0f - t) * start + t * end);
}

Vector3 Leap(float t, const Vector3& v1, const Vector3& v2) {
	Vector3 result;
	result.x = easing(t, v1.x, v2.x);
	result.y = easing(t, v1.y, v2.y);
	result.z = easing(t, v1.z, v2.z);
	return result;
}

Vector3 Bezier(const Vector3& v1, const Vector3& v2, const Vector3& v3, float t) {
	Vector3 tmp0 = Leap(t, v1, v2);
	Vector3 tmp1 = Leap(t, v2, v3);
	Vector3 tmp = Leap(t, tmp0, tmp1);
	return tmp;
}

const int DivisionNum = 200;

void DrawBezier(const Vector3& controllPoint0, const Vector3& controllPoint1, const Vector3& controllPoint2, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	for (int index = 0; index < DivisionNum; index++) {
		float t0 = index / float(DivisionNum);
		float t1 = (index + 1) / float(DivisionNum);
		Vector3 start = Bezier(controllPoint0, controllPoint1, controllPoint2, t0);
		Vector3 end = Bezier(controllPoint0, controllPoint1, controllPoint2, t1);
		DrawLine(start, end, viewProjectionMatrix, viewportMatrix, color);
	}
}

Vector3 CatmullRomEasing(const std::vector<Vector3>& controlPoints, float t) {
	//制御点の数
	int numPoints = static_cast<int>(controlPoints.size());
	//tの範囲を制限
	t = std::clamp(t, 0.0f, 1.0f);
	//tを使用してどのセグメントに属するかを計算する
	float segment = t * (numPoints - 1);
	//セグメントの整数部分と小数部分を分離する
	int segmentIndex = static_cast<int>(std::floor(segment));
	float segmentFraction = segment - segmentIndex;

	//制御点のインデックスを計算する
	int p0 = (segmentIndex - 1 + numPoints) % numPoints;
	if (segmentIndex == 0) {
		p0 = 0;
	}
	int p1 = (segmentIndex + numPoints) % numPoints;
	int p2 = (segmentIndex + 1 + numPoints) % numPoints;
	int p3 = (segmentIndex + 2 + numPoints) % numPoints;
	if (segmentIndex == numPoints - 2 || segmentIndex == numPoints - 1) {
		p3 = numPoints - 1;
	}

	//セグメントにおける補間の重みを計算する

	float segmentFraction2 = segmentFraction * segmentFraction;
	float segmentFraction3 = segmentFraction2 * segmentFraction;


	float t0 = -segmentFraction3 + 2.0f * segmentFraction2 - segmentFraction;
	float t1 = 3.0f * segmentFraction3 - 5.0f * segmentFraction2 + 2.0f; 
	float t2 = -3.0f * segmentFraction3 + 4.0f * segmentFraction2 + segmentFraction;
	float t3 = segmentFraction3 - segmentFraction2;

	 // Catmull-Romスプラインの線形補間を計算する
	Vector3 interpolatedPoint;
	interpolatedPoint.x = (t0 * controlPoints[p0].x + t1 * controlPoints[p1].x + t2 * controlPoints[p2].x + t3 * controlPoints[p3].x)  * 0.5f;
	interpolatedPoint.y = (t0 * controlPoints[p0].y + t1 * controlPoints[p1].y + t2 * controlPoints[p2].y + t3 * controlPoints[p3].y)  * 0.5f;
	interpolatedPoint.z = (t0 * controlPoints[p0].z + t1 * controlPoints[p1].z + t2 * controlPoints[p2].z + t3 * controlPoints[p3].z)  * 0.5f;

	return interpolatedPoint;
}

void DrawCatmullRomSpline(const std::vector<Vector3>& controlPoints, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	std::vector<Vector3> points_;
	for (size_t i = 0; i < DivisionNum + 1; i++) {
		float t = 1.0f / DivisionNum * i;
		Vector3 pos = CatmullRomEasing(controlPoints, t);
		// 描画用頂点リストに追加
		points_.push_back(pos);
	}
	for (int i = 0; i < DivisionNum; i++) {
		DrawLine(points_[i], points_[i + 1], viewProjectionMatrix, viewportMatrix, color);
	}
}

struct Object {
	Vector3 scale;
	Vector3 rotate;
	Vector3 transform;
	Matrix4x4 worldTransform;
	Matrix4x4* parent = nullptr;
	Sphere localModel = {
		.center = {0.0f,0.0f,0.0f},
		.radius = 0.1f
	};
	Sphere model = {
		.center = {0.0f,0.0f,0.0f},
		.radius = 0.1f
	};
	void UpdateWorldTransform() {
		worldTransform = MakeAffineMatrix(scale, rotate, transform);
		if (parent != nullptr) {
			worldTransform *= *parent;
		}
		model.center = localModel.center * worldTransform;
	}
};

struct Spring {
	//アンカー
	Vector3 anchor;		//アンカー。固定された端の位置
	float naturalLength;//自然頂
	float stiffness;	//剛性。ばね定数k
	float dampingCoefficient;//減衰係数
};

struct Ball {
	Vector3 position;//ボールの位置
	Vector3 velocity;//ボールの速度
	Vector3 accelertion;//ボールの加速度
	float mass;//ボールの質量
	float radius;//ボールの半径
	unsigned int color;//ボールの色
};

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);
	Vector3 cameracenterition = { 0.0f,1.9f,-6.49f };
	Vector3 cameraRotate = { 0.26f,0.0f,0.0f };

	Matrix4x4 cameraMatrix = MakeAffineMatrix({1.0f,1.0f,1.0f}, cameraRotate, cameracenterition);
	Matrix4x4 viewMatrix = Inverse(cameraMatrix);
	Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
	Matrix4x4 viewProjectionMatrix = viewMatrix * projectionMatrix;

	//Viewportmatrixを作る
	Matrix4x4 viewportMatrix = MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

	float deltTime = 1.0f / 60.0f;

	
	
	Ball ball{};
	ball.position = { 0.8f,1.2f,0.3f };
	ball.mass = 2.0f;
	ball.radius = 0.05f;
	ball.color = WHITE;
	const Vector3 kGravity{ 0.0f,-9.8f,0.0f };
	ball.accelertion = kGravity;

	struct Capsule {
		Segment segment;
		float radius;
	};

	Capsule capsule;
	capsule.radius = ball.radius;
	capsule.segment = {0.0f,0.0f,0.0f};

	Plane plane;
	plane.normal = Normalize({-0.2f,0.9f,-0.3f});
	plane.distance = 0.0f;


	float e = 0.8f;

	bool start = false;


	
	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		Key::Input();

		///
		/// ↓更新処理ここから
		///
		
		//カメラ処理
		CameraMove(cameracenterition, cameraRotate);
		cameraMatrix = MakeAffineMatrix({1.0f,1.0f,1.0f}, cameraRotate, cameracenterition);
		viewMatrix = Inverse(cameraMatrix);
		viewProjectionMatrix = viewMatrix * projectionMatrix;
		//
		//ゲームの処理

		if (start) {
			Vector3 prePosition = ball.position;
			ball.velocity += ball.accelertion * deltTime;
			ball.position += ball.velocity * deltTime;

			capsule = { { prePosition,ball.position - prePosition }, ball.radius };
			Segment closestSegemnt = { capsule.segment.origin - plane.normal * capsule.radius,capsule.segment.diff };

			Vector3 intersectPoint{};
			if (IsCollision( closestSegemnt, plane, intersectPoint)) {
				ball.position = intersectPoint + plane.normal * (capsule.radius + 0.0001f);
				ball.velocity = Reflect(ball.velocity, plane.normal) * e;
			}

		}
			
			

		//
		//ImGui
		ImGui::Begin("window");
		if (ImGui::Button("start")) {
			start = true;
		}
		ImGui::End();
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		
		DrawGrid(viewProjectionMatrix, viewportMatrix);
		DrawPlane(plane, viewProjectionMatrix, viewportMatrix, WHITE);
		DrawSphere({ ball.position,ball.radius }, viewProjectionMatrix, viewportMatrix, BLUE);


		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (Key::IsTrigger(DIK_ESCAPE)) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
