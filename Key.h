#pragma once

class Key {

private:

	static char keys[256];
	static char preKeys[256];

public:

	static void Input();

	static bool IsTrigger(char keycode);
	
	static bool IsRelease(char keycode);

	static bool IsPressed(char keycode);
};

