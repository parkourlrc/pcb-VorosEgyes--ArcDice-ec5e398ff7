#include <Arduino.h>
#include <avr/sleep.h>
#include "dice.h"

// LED group layout on the 3x3 dice face:
//
//   TL  TM  TR
//   ML  CC  MR        CC   = center       (SEGMENT_CENTER)
//   BL  BM  BR        ML/MR = middle row  (SEGMENT_MIDDLE_ROW)
//                      TL/BR = diagonal A  (SEGMENT_DIAGONAL_A)
//                      TR/BL = diagonal B  (SEGMENT_DIAGONAL_B)
//
// The constants below come from include/dice.h so the host-native test
// runner can validate face-mask logic without an AVR toolchain.
using arcdice::kDisplayMask;
using arcdice::kButtonMask;
using arcdice::kDiceFaceMasks;
using arcdice::kCenterMask;
using arcdice::kDiagonalAMask;
using arcdice::kDiagonalBMask;
using arcdice::kMiddleRowMask;
using arcdice::Prng;

volatile bool buttonInterruptPending = false;
// 16 bits of accumulated entropy, kept as two bytes because the ATtiny13A
// is an 8-bit AVR.
volatile uint8_t randomEntropyHi = 0;
volatile uint8_t randomEntropyLo = 0;
volatile uint8_t edgeCounter = 0;
Prng prng;

void showFace(uint8_t face) {
  PORTB = (PORTB & ~kDisplayMask) | kDiceFaceMasks[face];
}

// Folds 16 bits of button-press entropy into the PRNG state.
void mixEntropy(uint16_t entropy) {
  prng.mix(entropy);
}

// Draws a face uniformly from 1..6.
// The PRNG itself lives in include/dice.h so the host-native test runner
// exercises the exact same code the firmware runs.
uint8_t randomFace() {
  return prng.face();
}

// Waits up to `ms`, 1 ms at a time, so a fresh button press can interrupt
// an in-progress animation immediately instead of waiting for it to finish.
bool waitOrPressed(uint16_t ms) {
  for (uint16_t i = 0; i < ms; ++i) {
    delay(1);
    if (buttonInterruptPending && !(PINB & kButtonMask)) {
      return true;
    }
  }
  return false;
}

// Number of spin steps in a real roll (see performRoll()).
const uint8_t kDemoSpinCount = 8;

void runStartupTest() {
  // Running-light demo: chases through the 4 LED groups starting top-right
  // (diagonal B), 3 full laps around, then goes dark.
  const uint8_t sequence[4] = {
    kDiagonalBMask, kMiddleRowMask, kDiagonalAMask, kCenterMask
  };
  for (uint8_t lap = 0; lap < 3; ++lap) {
    for (uint8_t i = 0; i < 4; ++i) {
      PORTB = (PORTB & ~kDisplayMask) | sequence[i];
      delay(20);
    }
  }
  PORTB &= ~kDisplayMask;
}

void goToSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  // Disable BOD during sleep to minimize power draw (~25 uA savings).
  // Must be called within 3 cycles of sleep_enable() per datasheet.
  sleep_bod_disable();
  // Atomic-style sleep: enable interrupts, then sleep, then disable.
  // Using sleep_mode() (which itself does sei+sleep_cpu+cli) races with
  // a pending PCINT and can return immediately. Explicit form is safer.
  sei();
  sleep_cpu();
  sleep_disable();
}

void setup() {
  DDRB |= kDisplayMask;
  DDRB &= ~kButtonMask;
  PORTB |= kButtonMask; // Enable internal pull-up for button

  GIMSK |= (1 << PCIE); // Pin Change Interrupt Enable
  PCMSK |= kButtonMask; // Enable interrupt for the button pin
  sei(); // Enable global interrupts

  mixEntropy((static_cast<uint16_t>(TCNT0) << 8) | TCNT0);
  PORTB &= ~kDisplayMask; // Ensure all display groups are off at start
  runStartupTest();
}

// Runs the spin animation and result display. A fresh button press at any
// point (mid-spin or while the result is shown) restarts the roll right away.
void performRoll() {
  const uint16_t kRollStepMs = 30; // Constant spin speed, no slow-down.
  bool restart = true;
  while (restart) {
    restart = false;
    buttonInterruptPending = false;
    uint8_t rollCount = kDemoSpinCount * 2; // Roll spins twice as long as the demo.
    // The loop draws rollCount faces. With the counter-based PRNG every
    // draw is a fresh avalanche of the counter, so the walk always
    // covers the whole state space.
    for (uint8_t i = 0; i < rollCount; ++i) {
      showFace(randomFace());
      if (waitOrPressed(kRollStepMs)) {
        restart = true;
        break;
      }
    }
    // Last iteration already showed the result; no need to roll once more.
    if (!restart && waitOrPressed(1200)) { // Keep the result visible briefly
      restart = true;
    }
  }
  PORTB &= ~kDisplayMask; // Turn off all display groups
}

void loop() {
  if (buttonInterruptPending) {
    noInterrupts();
    buttonInterruptPending = false;
    uint16_t entropy = (static_cast<uint16_t>(randomEntropyHi) << 8) | randomEntropyLo;
    randomEntropyHi = 0;
    randomEntropyLo = 0;
    interrupts();

    delay(25);
    if (!(PINB & kButtonMask)) {
      mixEntropy(entropy ^ PINB);
      performRoll();
    }
  }

  if (!buttonInterruptPending) {
    goToSleep(); // Go to sleep mode
  }
}

// Accumulate entropy across multiple edges instead of overwriting on
// each interrupt. A single TCNT0 sample is weak; XORing PINB and
// counting edges gives more bits per button press.
//
// 16 bits are accumulated (two bytes) so the full width of the PRNG
// state is refreshed on every press — an 8-bit accumulator could only
// ever reach half of it.
ISR(PCINT0_vect) {
  if (!(PINB & kButtonMask)) {
    randomEntropyLo ^= TCNT0;
    randomEntropyLo ^= PINB;
    randomEntropyHi ^= edgeCounter;
    edgeCounter++;
    buttonInterruptPending = true;
  }
}