#include "stm32f3xx_hal.h"
#include "LiquidCrystal.h"

typedef unsigned char byte;
byte blank[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte player[] = { 0x00, 0x00, 0x04, 0x0F, 0x0E, 0x0F, 0x04, 0x00 };
byte platform[] = { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 };
byte broken_platform[] = { 0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0x02, 0x01 };
byte blackhole[] = { 0x00, 0x00, 0x00, 0x0E, 0x1F, 0x0E, 0x00, 0x00 };
byte spring[] = { 0x01, 0x01, 0x0F, 0x0B, 0x0F, 0x01, 0x01, 0x01 };
byte enemy[] = { 0x00, 0x04, 0x0E, 0x1E, 0x0E, 0x04, 0x00, 0x00 };
byte bullet[] = { 0x00, 0x00, 0x00, 0x04, 0x0E, 0x04, 0x00, 0x00 };

void set_Characters() {
	createChar(0, blank);
	createChar(1, player);
	createChar(2, platform);
	createChar(3, broken_platform);
	createChar(4, blackhole);
	createChar(5, spring);
	createChar(6, enemy);
	createChar(7, bullet);
}

void show_characters(int row, int col, int character) {
	setCursor(col, row);
	write(character);
}

void set_start_page() {
	setCursor(4, 2);
	print("Doodle  Jump");

	setCursor(5, 3);
	print("(BB: MENU)");
}

void menupage() {
	begin(20, 4);

	setCursor(1, 0);
	print("1: Start Game");

	setCursor(1, 1);
	print("2: Change Name");

	setCursor(1, 2);
	print("3: Help");

	setCursor(1, 3);
	print("4: About");
}

void set_game_page() {
	set_Characters();
	intialize();
}

void set_change_name_page() {
	begin(20, 4);
	setCursor(1, 1);
	print("change name");
}

void set_help_page() {
	begin(20, 4);
	setCursor(1, 1);
	print("help");
}

void set_about_page() {
	begin(20, 4);
	setCursor(1, 1);
	print("about");
}

void set_end_page() {
	begin(20, 4);
	setCursor(1, 1);
	print("mordi");
}
