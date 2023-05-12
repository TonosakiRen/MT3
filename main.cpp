#include <Novice.h>
#include "Mymath.h"
#include "Key.h"
const char kWindowTitle[] = "学籍番号";
const int kWindowWidth = 1280;
const int kWindowHeight = 720;

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	//cross式確認
	Vector3 v1{ 1.2f,-3.9f,2.5f };
	Vector3 v2{ 2.8f,0.4f,-1.3f };
	Vector3 cross = Cross(v1, v2);


	Vector3 kLocalVertices[3] = { {0.0f,0.3f,0.0f},{-0.3f,-0.3f,0.0f},{0.3f,-0.3f,0.0f} };

	Vector3 scale = { 1.0f,1.0f,1.0f };
	Vector3 rotate = { 0.0f,0.0f,0.0f };
	Vector3 translate = { 0.0f,0.0f,0.0f };
	Vector3 cameraPosition = {0.0f,0.0f,-0.5f};


	Matrix4x4 worldMatrix = MakeAffineMatrix(scale, rotate, translate);
	Matrix4x4 cameraMatrix = MakeAffineMatrix(scale, rotate, cameraPosition);
	Matrix4x4 viewMatrix = Inverse(cameraMatrix);
	Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f,float(kWindowWidth) / float(kWindowHeight),0.1f,100.0f);
	///WVPMatrixを作る
	Matrix4x4 worldViewProjectionMatrix = worldMatrix * viewMatrix * projectionMatrix;
	//Viewportmatrixを作る
	Matrix4x4 viewportMatrix = MakeViewportMatrix(0, 0, float(kWindowWidth) , float(kWindowHeight), 0.0f, 1.0f);
	//Screen空間へと頂点を作る
	Vector3 screenVeretices[3];
	for (uint32_t i = 0; i < 3; ++i) {
		//NDCまで変換。Transformを使うと同時座標系->デカルト座標系の処理が行われ、結果的にZDivideが行われることになる
		Vector3 ndcVertex = Transform(kLocalVertices[i], worldViewProjectionMatrix);
		//Viewport変換を行ってScreen
		screenVeretices[i] = Transform(ndcVertex, viewportMatrix);
	}

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		Key::Input();

		///
		/// ↓更新処理ここから
		///

		if (Key::IsPressed(DIK_W)) {
			translate.z += 0.01f;
		}
		if (Key::IsPressed(DIK_S)) {
			translate.z -= 0.01f;
		}
		if (Key::IsPressed(DIK_A)) {
			translate.x -= 0.01f;
		}
		if (Key::IsPressed(DIK_D)) {
			translate.x += 0.01f;
		}

		rotate.y += 0.01f;


		worldMatrix = MakeAffineMatrix(scale, rotate, translate);
		cameraMatrix = MakeAffineMatrix(scale, {0,0,0}, cameraPosition);
		viewMatrix = Inverse(cameraMatrix);
		///WVPMatrixを作る
		worldViewProjectionMatrix = worldMatrix * viewMatrix * projectionMatrix;

		for (uint32_t i = 0; i < 3; ++i) {
			//NDCまで変換。Transformを使うと同時座標系->デカルト座標系の処理が行われ、結果的にZDivideが行われることになる
			Vector3 ndcVertex = Transform(kLocalVertices[i], worldViewProjectionMatrix);
			//Viewport変換を行ってScreen
			screenVeretices[i] = Transform(ndcVertex, viewportMatrix);
		}

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		
		VectorScreenPrintf(0, 0, cross, "Cross");
		Novice::DrawTriangle(int(screenVeretices[0].x), int(screenVeretices[0].y), int(screenVeretices[1].x), int(screenVeretices[1].y), int(screenVeretices[2].x), int(screenVeretices[2].y), RED, kFillModeSolid);

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
