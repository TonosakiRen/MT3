#include "Key.h"
#include <Novice.h>

void Key::Input() {
	// キー入力を受け取る
	memcpy(preKeys, keys, 256);
	Novice::GetHitKeyStateAll(keys);

	
}

bool Key::IsPressed(char keycode) {
	return keys[keycode] != 0;
}

bool Key::IsTrigger(char keycode) {
	return preKeys[keycode] == 0 && keys[keycode] != 0;
}

bool Key::IsRelease(char keycode) {
	return preKeys[keycode] != 0 && keys[keycode] == 0;
}

char Key::keys[256] = { 0 };
char Key::preKeys[256] = { 0 };