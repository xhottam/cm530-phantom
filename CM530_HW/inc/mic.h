/*
 * mic.h
 *
 *  Created on: 8 Jun 2016
 *      Author: E1193262
 */

#ifndef CM530_HW_INC_MIC_H_
#define CM530_HW_INC_MIC_H_

#include "stm32f10x_type.h"

typedef enum buzzed_note_e {
// Twelve-Tone Equal Temperment (12-TET)
//   1 octave is a doubling of frequency and equal to 1200 cents
//   1 octave => 12 equally distributed notes (12 intervals/semitones)
//     so 100 cents per note
// Tuned to A 440 (440Hz), so 100 cents per note relative to A_5 (440Hz)
// tone_delay = 1/(2*1e-6*f) = 1/(2*1e-6*440*2^(cents_relative/1200))
//   using uDelay(), 50% duty cycle, cents relative to A_4
                                  // tone_delay // cents, ideal frequency
    NOTE_C                            = 30578,  // -5700, 16.35159783 Hz, C_0
    NOTE_Cs                           = 28862,  // -5600, 17.32391444 Hz, Cs_0
    NOTE_Db                           = 28862,  // -5600, 17.32391444 Hz, Db_0
    NOTE_D                            = 27242,  // -5500, 18.35404799 Hz, D_0
    NOTE_Ds                           = 25713,  // -5400, 19.44543648 Hz, Ds_0
    NOTE_Eb                           = 25713,  // -5400, 19.44543648 Hz, Eb_0
    NOTE_E                            = 24270,  // -5300, 20.60172231 Hz, E_0
    NOTE_F                            = 22908,  // -5200, 21.82676446 Hz, F_0
    NOTE_Fs                           = 21622,  // -5100, 23.12465142 Hz, Fs_0
    NOTE_Gb                           = 21622,  // -5100, 23.12465142 Hz, Gb_0
    NOTE_G                            = 20408,  // -5000, 24.49971475 Hz, G_0
    NOTE_Gs                           = 19263,  // -4900, 25.9566436  Hz, Gs_0
    NOTE_Ab                           = 19263,  // -4900, 25.9565436  Hz, Ab_0
    NOTE_A                            = 18182,  // -4800, 27.5        Hz, A_0
    NOTE_As                           = 17161,  // -4700, 29.13523509 Hz, As_0
    NOTE_Bb                           = 17161,  // -4700, 29.13523509 Hz, Bb_0
    NOTE_B                            = 16198  // -4600, 30.86770633 Hz, B_0
} buzzed_note_t;


/**
 * Easy Function to control the buzzer on the CM-530.
 * @param mlength The length of time [ms] to play the tone.
 * @param tone The length of time [us] of half the period of the wave.
 */
void Buzzed(u32 mlength, u32 tone);
/**
 * Easy Function to play a musical note with the buzzer on the CM-530.
 * To play Middle C (C4) for 10 seconds:
 *   PlayNote( 10000, NOTE_C, 4 );
 * @param mlength The length of time [ms] to play the note.
 * @param note The musical note to be played.
 * @param octave The octave of the note to be played.
 */
void PlayNote(u32 mlength, buzzed_note_t note, u8 octave);


#endif /* CM530_HW_INC_MIC_H_ */
