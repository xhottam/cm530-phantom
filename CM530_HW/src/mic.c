/*
 * mic.c
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#include "mic.h"
#include "system_init.h"
#include "system_func.h"


//##############################################################################
void Buzzed(u32 mlength, u32 tone) {
	// Twelve-Tone Equal Temperment (12-TET)
	//   1 octave is a doubling of frequency and equal to 1200 cents
	//   1 octave => 12 equally distributed notes (12 intervals/semitones)
	//     so 100 cents per note
	// Tuned to A 440 (440Hz), so 100 cents per note relative to A_5 (440Hz)

	// n [cents] = 1200 log2(b/a)
	// b = a * 2^(n/1200)

	// tone = 1/(2*1e-6*f) = 1/(2*1e-6*440*2^(cents_relative/1200))
	//   using uDelay(), 50% duty cycle, cents relative to A_5

//#define FREQTOTONE(f)    (5000000/f)    // (1/(2*1e-6*f))

	start_countdown_buzzer(mlength);
	while (glBuzzerCounter > 0) {
		GPIO_ResetBits(PORT_BUZZER, PIN_BUZZER);
		uDelay(tone);
		GPIO_SetBits(PORT_BUZZER, PIN_BUZZER);
		uDelay(tone);
	}
}

//##############################################################################
void PlayNote(u32 mlength, buzzed_note_t note, u8 octave) {
	Buzzed(mlength, (u32) (note >> octave));
}
