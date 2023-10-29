# STM32 Doodle Jump Game

In this repository you will find all the necessary elements and resources related to the design and implementation of the engaging Doodle Jump game using STM32 and various peripherals. Here's an overview of the project:

## Project Elements

- **STM32**: The core microcontroller responsible for managing the game and its components.

- **Character LCD**: Used for displaying important game information such as the player's score and the menu.

- **Keypad**: Provides input control for navigating through the game menu, moving the character, and shooting.

- **7-Segment Display with IC**: Shows the player's score and the current difficulty level in real-time.

- **UART**: Facilitates communication between the game and a computer terminal, displaying player movements and information.

- **Buzzer**: Creates sound effects to enhance the gaming experience.

- **Volume Module**: Controls the game's difficulty levels, ensuring a challenging yet playable experience.

## Game Overview

The Doodle Jump game starts with a home page that displays the game's title. Pressing the blue button on the board takes you to the game menu, which provides options to start the game and view information about the game. You can select your desired option using the keypad. If you choose to view the About page, it displays the group members' names and the current date from the Real-Time Clock (RTC), and you can return to the menu by pressing a key.

The game board is a vertical grid of 4 columns by 20 rows. As the player ascends, the screen moves upward, revealing more of the game map. The map features stairs that the player's character (Doodler) must jump on to avoid falling off the bottom of the screen. When Doodler lands on a step, they bounce back up. There are also special springs that provide extra jumping height. Be cautious of broken stairs that won't support Doodler's weight.

The game introduces obstacles, including monsters and black holes. Falling into a black hole ends the game, while colliding with a monster disorients Doodler and causes them to fall off the screen. The game provides appropriate sound effects for these actions.

The movement of Doodler is controlled using left and right keys, allowing you to shift horizontally. The screen loops at the edges, enabling continuous movement from left to right. A key is available to shoot arrows upwards, which can destroy monsters.

The game dynamically generates different maps with randomly placed stairs and obstacles. There are ten levels of difficulty, affecting the type and spacing of obstacles and stairs. The game's difficulty can be adjusted using the volume module, ensuring challenging but always solvable scenarios.

Player scores are accumulated based on the current difficulty level. The game displays the player's score on a three-digit 7-segment display with the difficulty level as a decimal point.

The game concludes when Doodler falls into a black hole, at which point the player's name and score are displayed on the LCD and terminal, and the board LEDs flash. After the game ends, pressing a key returns you to the game menu. The default player name is "Doodler," but you can change it by sending a phrase via UART on the menu page. There's a message on the menu screen to inform you of this feature.
