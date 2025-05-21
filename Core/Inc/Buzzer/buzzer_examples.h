#include "buzzer_tones.h"
#include "buzzer.h"

#define BPM_MARIO 128
note_t mario_theme[] = {
  {NOTE_E5, 12}, {NOTE_E5, 12}, {0, 12}, {NOTE_E5, 12},
  {0, 12}, {NOTE_C5, 12}, {NOTE_E5, 12}, {0, 12},
  {NOTE_G5, 12}, {0, 12}, {0, 12}, {0, 12},
  {NOTE_G4, 12}, {0, 12}, {0, 12}, {0, 12},

  {NOTE_C5, 12}, {0, 12}, {0, 12}, {NOTE_G4, 12},
  {0, 12}, {0, 12}, {NOTE_E4, 12}, {0, 12},
  {0, 12}, {NOTE_A4, 12}, {0, 12}, {NOTE_B4, 12},
  {0, 12}, {NOTE_AS4, 12}, {NOTE_A4, 12}, {0, 12},

  {NOTE_G4, 9}, {NOTE_E5, 9}, {NOTE_G5, 9},
  {NOTE_A5, 12}, {0, 12}, {NOTE_F5, 12}, {NOTE_G5, 12},
  {0, 12}, {NOTE_E5, 12}, {0, 12}, {NOTE_C5, 12},
  {NOTE_D5, 12}, {NOTE_B4, 12}, {0, 12}, {0, 12},

  {NOTE_C5, 12}, {0, 12}, {0, 12}, {NOTE_G4, 12},
  {0, 12}, {0, 12}, {NOTE_E4, 12}, {0, 12},
  {0, 12}, {NOTE_A4, 12}, {0, 12}, {NOTE_B4, 12},
  {0, 12}, {NOTE_AS4, 12}, {NOTE_A4, 12}, {0, 12},

  {NOTE_G4, 9}, {NOTE_E5, 9}, {NOTE_G5, 9},
  {NOTE_A5, 12}, {0, 12}, {NOTE_F5, 12}, {NOTE_G5, 12},
  {0, 12}, {NOTE_E5, 12}, {0, 12}, {NOTE_C5, 12},
  {NOTE_D5, 12}, {NOTE_B4, 12}, {0, 12}, {0, 12}
};

#define BPM_PACMAN 105
note_t pacman[] = {
  { NOTE_B4, 16 }, { NOTE_B5, 16 }, { NOTE_FS5, 16 }, { NOTE_DS5, 16 },
  { NOTE_B5, 32 }, { NOTE_FS5, 12 }, { NOTE_DS5, 8 }, { NOTE_C5, 16 },
  { NOTE_C6, 16 }, { NOTE_G6, 16 }, { NOTE_E6, 16 }, { NOTE_C6, 32 }, { NOTE_G6, 12 }, { NOTE_E6, 8 },

  { NOTE_B4, 16 }, { NOTE_B5, 16 }, { NOTE_FS5, 16 }, { NOTE_DS5, 16 }, { NOTE_B5, 32 },
  { NOTE_FS5, 12 }, { NOTE_DS5, 8 }, { NOTE_DS5, 32 }, { NOTE_E5, 32 }, { NOTE_F5, 32 },
  { NOTE_F5, 32 }, { NOTE_FS5, 32 }, { NOTE_G5, 32 }, { NOTE_G5, 32 }, { NOTE_GS5, 32 }, { NOTE_A5, 16 }, { NOTE_B5, 8 }
};

#define BPM_MARIO_LEVEL 145
note_t mario_level_complete[] = {
		{ NOTE_G3, 12 }, { NOTE_C4, 12 }, { NOTE_E4, 12 },
		{ NOTE_G4, 12 }, { NOTE_C5, 12 }, { NOTE_E5, 12 },
		{ NOTE_G5, 4 },
		{ NOTE_E5, 4 },

		{ NOTE_GS3, 12 }, { NOTE_C4, 12 }, { NOTE_DS4, 12 },
		{ NOTE_GS4, 12 }, { NOTE_C5, 12 }, { NOTE_DS5, 12 },
		{ NOTE_GS5, 4 },
		{ NOTE_E5, 4 },

		{ NOTE_AS3, 12 }, { NOTE_D4, 12 }, { NOTE_F4, 12 },
		{ NOTE_AS4, 12 }, { NOTE_D5, 12 }, { NOTE_F5, 12 },
		{ NOTE_AS5, 4 },
		{ NOTE_B5, 16 }, { 0, 48 }, { NOTE_B5, 16 }, { 0, 48 }, { NOTE_B5, 16 }, { 0, 48 },

		{ NOTE_C6, 4 }, { 0, 4 }, { 0, 2 }
};

#define BPM_SYSTEM_SOUND 120
#define error_sound NOTE_A4
#define battery_empty NOTE_G5

note_t air_empty[]= {
		{ NOTE_B7, 16 }, { 0, 16 },
		{ NOTE_B6, 16 }, { 0, 16 },
		{ NOTE_B5, 16 }, { 0, 16 }
};
