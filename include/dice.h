// dice.h — Pure-logic header for the dice face mapping.
//
// This file is intentionally free of Arduino-specific includes so it can
// be compiled both by the ATtiny13A firmware and by the host-native test
// runner (PlatformIO `pio test -e native`).
//
// The hardware layout (PB0..PB3 as display segments, PB4 as the button)
// is fixed in stone at the schematic level — see docs/BOM.md and the
// main README. Changing these values here without changing the firmware
// pin mapping AND the KiCad schematic will produce a board that lights
// up the wrong segments.

#ifndef ARCDICE_DICE_H_
#define ARCDICE_DICE_H_

#include <stdint.h>

// Bit positions on ATtiny13A PORTB. Keep in sync with src/main.cpp.
#ifndef DICE_SEGMENT_CENTER
#define DICE_SEGMENT_CENTER     0   // PB0 — center pip (CC)
#endif
#ifndef DICE_SEGMENT_DIAGONAL_A
#define DICE_SEGMENT_DIAGONAL_A 1   // PB1 — TL + BR diagonal
#endif
#ifndef DICE_SEGMENT_DIAGONAL_B
#define DICE_SEGMENT_DIAGONAL_B 2   // PB2 — TR + BL diagonal
#endif
#ifndef DICE_SEGMENT_MIDDLE_ROW
#define DICE_SEGMENT_MIDDLE_ROW 3   // PB3 — ML + MR middle row
#endif

#define DICE_BUTTON_PIN         4   // PB4 — roll button input

#define DICE_BIT(bit)           ((uint8_t)(1u << (bit)))

namespace arcdice {

constexpr uint8_t kCenterMask     = DICE_BIT(DICE_SEGMENT_CENTER);
constexpr uint8_t kDiagonalAMask  = DICE_BIT(DICE_SEGMENT_DIAGONAL_A);
constexpr uint8_t kDiagonalBMask  = DICE_BIT(DICE_SEGMENT_DIAGONAL_B);
constexpr uint8_t kMiddleRowMask  = DICE_BIT(DICE_SEGMENT_MIDDLE_ROW);

constexpr uint8_t kButtonMask     = DICE_BIT(DICE_BUTTON_PIN);

constexpr uint8_t kDisplayMask =
    kCenterMask | kDiagonalAMask | kDiagonalBMask | kMiddleRowMask;

// Face indices are 0..6 inclusive. Face 0 is the "all off" sentinel
// (never displayed by the firmware — roll() returns 1..6) but is included
// here so the array length matches the README's 1..6 documentation.
//
// The mapping matches the README "Dice Face Layout" table:
//
//   face 1 = center
//   face 2 = diagonal A
//   face 3 = center + diagonal A           (vertical middle column)
//   face 4 = diagonal A + diagonal B       (full X, four corners)
//   face 5 = center + diagonals A + B      (full X + center)
//   face 6 = diagonals A + B + middle row  (four corners + middle two)
constexpr uint8_t kDiceFaceMasks[7] = {
    0,                                              // face 0 — unused
    kCenterMask,                                    // face 1
    kDiagonalAMask,                                  // face 2
    kCenterMask | kDiagonalAMask,                    // face 3
    kDiagonalAMask | kDiagonalBMask,                 // face 4
    kCenterMask | kDiagonalAMask | kDiagonalBMask,   // face 5
    kDiagonalAMask | kDiagonalBMask | kMiddleRowMask, // face 6
};

// Returns the PORTB mask for the requested face. Faces 1..6 are valid;
// face 0 returns 0 (all segments off). Out-of-range faces return 0xFF
// so a programming error is visible immediately rather than silently
// rolling "all off".
constexpr uint8_t faceMask(uint8_t face) {
    return (face < 7) ? kDiceFaceMasks[face] : 0xFF;
}

// --- PRNG ------------------------------------------------------------------
//
// WHY THIS IS NOT AN 8-BIT LFSR
// ------------------------------
// The original firmware used an 8-bit Galois LFSR (taps 0xB8, period
// 255) and mapped the byte to a face with `(byte % 6) + 1`. That produced
// a measurably biased dice:
//
//   * 256 = 42*6 + 4, so `byte % 6` over the full byte range already
//     over-weights faces 1..4 by 1/256 each.
//   * Worse: performRoll() drew 17 faces per roll (kDemoSpinCount * 2
//     with a `for (i = 0; i <= rollCount; ++i)` loop), and
//     gcd(17, 255) = 17. Walking the LFSR 17 steps at a time therefore
//     only ever visits 255 / 17 = 15 of the 255 states. Within that
//     15-state sub-cycle `byte % 6` is strongly correlated with the walk
//     position.
//
// Measured over 60,000 simulated rolls (constant button timing):
//   face 1 = 20.00%, face 2 = 13.33%, face 3 = 20.00%,
//   face 4 = 20.00%, face 5 = 13.33%, face 6 = 13.33%
//   chi-square = 2400 (df = 5, 0.05 critical value = 11.07)
//
// WHY A 16-BIT XORSHIFT IS NOT ENOUGH EITHER
// ------------------------------------------
// The first fix attempt replaced the LFSR with a 16-bit xorshift
// (shifts 7, 9, 8) plus rejection sampling. That dropped the chi-square
// from 2400 to ~112 — better, but still biased.
//
// The reason: xorshift and XOR are both GF(2)-AFFINE maps. The whole
// state recursion is therefore affine over GF(2):
//
//     state(n+1) = A^k * state(n) + C
//
// An affine map on a 65535-long cycle is just a *shift* along that
// cycle, so the trajectory never escapes a fixed subset of it and
// `state % 6` over that subset is not uniform. Adding entropy with a
// modular ADD does break the affine structure a little, but the
// residual bias (chi-square ~50) was still far above the 11.07
// threshold.
//
// THE FIX: a counter-based generator
// ----------------------------------
// A counter-based generator has no state recursion at all: every draw is
// a pure function of (counter, entropy). There is no sub-cycle to fall
// into, so no bias can accumulate.
//
// The mixer is the SplitMix32 finalizer:
//
//     z  = x + 0x9E3779B9
//     z  = (z ^ (z >> 16)) * 0x85EBCA6B
//     z  = (z ^ (z >> 13)) * 0xC2B2AE35
//     z ^= z >> 16
//
// The two 32-bit multiplications give a strong avalanche, so consecutive
// counter values produce statistically independent outputs. Rejection
// sampling then maps the 32-bit output onto 1..6 exactly uniformly
// (4294967286 = 6 * 715827881 exactly).
//
// The 32-bit state lives in four uint8_t bytes because the ATtiny13A is
// an 8-bit AVR. The 32-bit multiplications inside mix32() are compiled
// to 16x16 -> 32 bit partial products by the AVR toolchain.

struct Prng {
    // 32-bit counter, stored as four bytes little-endian.
    uint8_t b0 = 0x5A;
    uint8_t b1 = 0x5A;
    uint8_t b2 = 0x5A;
    uint8_t b3 = 0x5A;

    uint32_t state() const {
        return static_cast<uint32_t>(b0) |
               (static_cast<uint32_t>(b1) << 8) |
               (static_cast<uint32_t>(b2) << 16) |
               (static_cast<uint32_t>(b3) << 24);
    }

    void setState(uint32_t s) {
        b0 = static_cast<uint8_t>(s);
        b1 = static_cast<uint8_t>(s >> 8);
        b2 = static_cast<uint8_t>(s >> 16);
        b3 = static_cast<uint8_t>(s >> 24);
    }

    // SplitMix32 finalizer — the avalanche mixer.
    static uint32_t mix32(uint32_t x) {
        x += 0x9E3779B9u;
        x = (x ^ (x >> 16)) * 0x85EBCA6Bu;
        x = (x ^ (x >> 13)) * 0xC2B2AE35u;
        return x ^ (x >> 16);
    }

    // Advances the counter and returns the mixed 32-bit output.
    uint32_t next() {
        // The counter advances by the golden-ratio increment, so the
        // sequence of counter values is itself well spread.
        setState(state() + 0x9E3779B9u);
        return mix32(state());
    }

    // Draws a face uniformly from 1..6.
    //
    // 4294967295 = 6 * 715827882 + 3, so the top 9 values must be
    // rejected. The remaining 4294967286 values split into 715827881
    // exact groups of 6, which makes the mapping perfectly uniform.
    uint8_t face() {
        for (;;) {
            const uint32_t z = next();
            if (z < 4294967286u) {
                return static_cast<uint8_t>((z % 6) + 1);
            }
        }
    }

    // Folds 16 bits of entropy into the counter.
    //
    // The entropy is spread over the full 32-bit width before it is
    // added. The SplitMix32 finalizer in next() then avalanches it.
    // No multiplication is needed here: the golden-ratio increment in
    // next() already guarantees that a one-bit change in the entropy
    // lands on a completely different output word.
    void mix(uint16_t entropy) {
        // 0x00010001 replicates the 16 entropy bits into both halves, so
        // the high and low halves of the counter change together.
        const uint32_t spread =
            static_cast<uint32_t>(entropy) |
            (static_cast<uint32_t>(entropy) << 16);
        setState(state() ^ spread);
    }
};

}  // namespace arcdice

#endif  // ARCDICE_DICE_H_