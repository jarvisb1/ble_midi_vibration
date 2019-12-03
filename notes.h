#ifndef _NOTES_H_
#define _NOTES_H_

#include <Arduino.h>
#include "pitchToNote.h"

#define NUM_NOTES (16)

//C Major
const byte note_pitches[NUM_NOTES] = {pitchC6, pitchD6, pitchE6, pitchF6, pitchG6, pitchA6, pitchB6, pitchC7, pitchD7, pitchE7, pitchF7, pitchG7, pitchA7, pitchB7, pitchC8, pitchD8};

#endif _NOTES_H_
