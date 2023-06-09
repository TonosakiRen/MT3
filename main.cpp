#define _USE_MATH_DEFINES
#include <Novice.h>
#include "Mymath.h"
#include "Key.h"
#include <cmath>
#include <imgui.h>
#include <Input.h>
const char kWindowTitle[] = "学籍番号";
const int kWindowWidth = 1280;
const int kWindowHeight = 720;

struct Sphere {
	Vector3 pos; // 中心点
	float radius; //半径
};
bool IsCollision(const Sphere& s1, const Sphere& s2) {
	float length = Length(s1.pos - s2.pos);
	if (s1.radius + s2.radius >= length) {
		return true;
	}
	return false;
}
void CameraMove(Vector3& cameraPos , Vector3& cameraRotate) {
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
		cameraPos += cameraX + cameraY;
	}
	else if (wheel != 0) {
		Matrix4x4 rotMat = MakeRotateXYZMatrix(cameraRotate);
		Vector3 cameraZ = GetZAxis(rotMat) * (static_cast<float>(wheel / 120) * 0.5f);
		cameraPos += cameraZ;
	}
	ImGui::Begin("Camera");
	ImGui::DragFloat3("Camera position", &cameraPos.x, 0.01f);
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
		if (kSubdivision / 2 + 1 == zIndex) {
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
			a = a * sphere.radius + sphere.pos;
			b = b * sphere.radius + sphere.pos;
			c = c * sphere.radius + sphere.pos;
			// a,b,cをscreen座標系まで変換
			a = a * viewProjectionMatrix * viewportMatrix;
			b = b * viewProjectionMatrix * viewportMatrix;
			c = c * viewProjectionMatrix * viewportMatrix;

			Novice::DrawLine(int(a.x), int(a.y), int(b.x), int(b.y), color);
			Novice::DrawLine(int(a.x), int(a.y), int(c.x), int(c.y), color);
		}
	}

}
// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	Vector3 scale = { 1.0f,1.0f,1.0f };
	Vector3 rotate = { 0.0f,0.0f,0.0f };
	Vector3 translate = { 0.0f,0.0f,0.0f };
	Vector3 cameraPosition = { 0.0f,1.9f,-6.49f };
	Vector3 cameraRotate = { 0.26f,0.0f,0.0f };

	Matrix4x4 cameraMatrix = MakeAffineMatrix(scale, cameraRotate, cameraPosition);
	Matrix4x4 viewMatrix = Inverse(cameraMatrix);
	Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);

	//Viewportmatrixを作る
	Matrix4x4 viewportMatrix = MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

	Sphere a = { {-3.0f,3.0f,0.0f}, 2.0f};
	Sphere b = { {3.0f,3.0f,0.0f}, 2.0f };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		Key::Input();

		///
		/// ↓更新処理ここから
		///
	
		cameraMatrix = MakeAffineMatrix(scale, cameraRotate, cameraPosition);
		viewMatrix = Inverse(cameraMatrix);

		bool hit = IsCollision(a, b);

		CameraMove(cameraPosition,cameraRotate);

		ImGui::Begin("window");
		ImGui::DragFloat3("CameraTranslate",&cameraPosition.x,0.01f);
		ImGui::DragFloat3("aPos", &a.pos.x, 0.01f);
		ImGui::End();
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		if (hit == true) {
			DrawSphere(a, viewMatrix * projectionMatrix, viewportMatrix, RED);
		}
		else {
			DrawSphere(a, viewMatrix * projectionMatrix,viewportMatrix,WHITE);
		}
		DrawSphere(b, viewMatrix * projectionMatrix, viewportMatrix, WHITE);

		DrawGrid(viewMatrix * projectionMatrix, viewportMatrix);

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
