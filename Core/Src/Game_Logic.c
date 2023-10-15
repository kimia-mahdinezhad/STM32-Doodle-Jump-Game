#include "stm32f3xx_hal.h"
#include "LiquidCrystal.h"
#include "LCD_Util.h"

int position = 1;
int positioning = 0;
int mapping[4][20] = { 0 };
int player_row = 0, player_col = 0;
int jumping = 7;
int last_platform_col = 0, last_platform_row = 0;
int distance = 0;
int counter = 0;
int shooting = 0;
int bullet_row = 0, bullet_col = 0;

int dead_by_enemy = 0;

extern int program_state;
extern int difficulty_level;
extern int score;

void player_dead() {
	if (mapping[player_row][player_col] == 6) {
		begin(20, 4);
		show_characters(player_row, player_col,
				mapping[player_row][player_col]);
		dead_by_enemy = 1;
	} else if (mapping[player_row][player_col] == 4) {
		program_state = end_page;
		set_end_page();
	}

}

void player_right() {
	show_characters(player_row, player_col, mapping[player_row][player_col]);

	player_row++;
	if (player_row > 3) {
		player_row = 0;
	}

	show_characters(player_row, player_col, 1);
	player_dead();
}

void player_left() {
	show_characters(player_row, player_col, mapping[player_row][player_col]);

	player_row--;
	if (player_row < 0) {
		player_row = 3;
	}

	show_characters(player_row, player_col, 1);
	player_dead();
}

void generate_map() {
	show_characters(0, 0, 1);

	show_characters(0, 5, 2);
	mapping[0][5] = 2;

	show_characters(0, 15, 6);
	mapping[0][15] = 6;

	show_characters(1, 6, 2);
	mapping[1][6] = 2;

	show_characters(3, 7, 5);
	mapping[3][7] = 5;

	show_characters(3, 15, 3);
	mapping[3][15] = 3;
}

void possible_jump() {
	if (dead_by_enemy == 0) {
		if (mapping[player_row][player_col - 1] == 2) {
			if (last_platform_col < player_col - 1) {
				distance = player_col - last_platform_col - 1;
				score += (difficulty_level + 1) * distance;
				last_platform_col = player_col - 1;
			}
			jumping = 7;
			position = 1;
		} else if (mapping[player_row][player_col - 1] == 5) {
			jumping = 20;
			position = 1;
		} else if (mapping[player_row][player_col - 1] == 3) {
			mapping[player_row][player_col - 1] = 0;
		}
	}
}

void player_up() {
	show_characters(player_row, player_col, mapping[player_row][player_col]);

	player_col++;
	show_characters(player_row, player_col, 1);

	position++;
	player_dead();
}

void player_down() {
	if (dead_by_enemy == 0) {
		show_characters(player_row, player_col,
				mapping[player_row][player_col]);
	} else {
		show_characters(player_row, player_col, 0);
	}

	player_col--;
	show_characters(player_row, player_col, 1);

	if (player_col == 0) {
		set_end_page();
		program_state = end_page;
		return;
	}

	position--;

	possible_jump();
	player_dead();
}
void shoot() {
	show_characters(bullet_row, bullet_col, mapping[bullet_row][bullet_col]);

	bullet_col++;
	show_characters(bullet_row, bullet_col, 7);

	if (bullet_col == 19) {
		shooting = 0;
	}

	if (mapping[bullet_row][bullet_col] == 6) {
		mapping[bullet_row][bullet_col] = 0;
		show_characters(bullet_row, bullet_col, 0);
		shooting = 0;
	}
}

void update_position() {
	counter++;
	if (shooting && counter % 2 == 0) {
		shoot();
	}
	if (counter % 3 == 0) {
		if (position == jumping + 1) {
			position = -1;
		}
		if (position > 0 && position <= jumping && dead_by_enemy == 0) {
			player_up();
		} else if (position < 0 || dead_by_enemy == 1) {
			player_down();
		}
	}
}

void intialize() {
	begin(20, 4);
	generate_map();
}
