// test_prng_distribution.cpp — host-native statistical tests for the
// firmware PRNG and the roll animation.
//
// Run with: pio test -e native
//
// WHY THIS FILE EXISTS
// --------------------
// The original test_prng_sequence_produces_diverse_outputs only asserts
// that every face appears at least once in 600 draws. That assertion
// passes even for a heavily biased generator, so the real defect slipped
// through CI.
//
// THE DEFECT (fixed in this commit)
// --------------------------------
// The old firmware used an 8-bit Galois LFSR (taps 0xB8, period 255) and
// mapped the byte onto a face with `(byte % 6) + 1:
//
//   * 256 = 42*6 + 4, so `byte % 6` over the full byte range already
//     over-weights faces 1..4.
//   * Worse: performRoll() drew 17 faces per roll (the loop was
//     `for (i = 0; i <= rollCount; ++i)` with rollCount = 16), and
//     gcd(17, 255) = 17. Walking the LFSR 17 steps at a time therefore
//     only ever visited 255 / 17 = 15 of the 255 states. Within that
//     15-state sub-cycle `byte % 6` is strongly correlated with the walk
//     position.
//
// Measured over 60,000 simulated rolls (constant button timing):
//   face 1 = 20.00%, face 2 = 13.33%, face 3 = 20.00%,
//   face 4 = 20.00%, face 5 = 13.33%, face 6 = 13.33%
//   chi-square = 2400 (df = 5, 0.05 critical value = 11.07)
//
// THE FIX
// --------
// A counter-based generator with the SplitMix32 finalizer. Every draw is
// a pure function of (counter, entropy) — there is no state recursion and
// therefore no sub-cycle that the walk could get stuck in. Rejection
// sampling maps the 32-bit output onto 1..6 exactly uniformly
// (4294967286 = 6 * 715827881 exactly).
//
// The PRNG lives in include/dice.h as arcdice::Prng, so these tests run
// the exact same code the ATtiny13A firmware runs.

#include <stdint.h>
#include <cstdio>
#include <unity.h>

#include "dice.h"

// ---------------------------------------------------------------------------
// Firmware constants mirrored from src/main.cpp
// ---------------------------------------------------------------------------

namespace firmware {

constexpr uint8_t kDemoSpinCount = 8;

// performRoll(): `uint8_t rollCount = kDemoSpinCount * 2;` then
// `for (uint8_t i = 0; i < rollCount; ++i)` -> rollCount draws.
constexpr uint8_t kDrawsPerRoll = kDemoSpinCount * 2;

}  // namespace firmware

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Pearson chi-square statistic for a 6-bucket histogram against a uniform
// expectation. df = 5, so the 0.05 critical value is 11.07.
static float chiSquare(const uint32_t counts[6], uint32_t total) {
    const float expected = static_cast<float>(total) / 6.0f;
    float chi = 0.0f;
    for (int i = 0; i < 6; ++i) {
        const float d = static_cast<float>(counts[i]) - expected;
        chi += d * d / expected;
    }
    return chi;
}

// Simulates `rolls` button presses from an explicitly seeded PRNG and
// records the face that would be left on the display (the last draw of
// each roll). `entropyFor` supplies the button-press timing entropy.
template <typename EntropyFn>
static void simulateRollsSeeded(arcdice::Prng &prng, uint32_t rolls,
                                EntropyFn entropyFor, uint32_t counts[6]) {
    for (uint32_t r = 0; r < rolls; ++r) {
        prng.mix(entropyFor(r));
        uint8_t face = 1;
        for (uint8_t i = 0; i < firmware::kDrawsPerRoll; ++i) {
            face = prng.face();
        }
        counts[face - 1]++;
    }
}

// Convenience wrapper that uses the default seed.
template <typename EntropyFn>
static void simulateRolls(uint32_t rolls, EntropyFn entropyFor,
                          uint32_t counts[6]) {
    arcdice::Prng prng;
    simulateRollsSeeded(prng, rolls, entropyFor, counts);
}

// Asserts that every face share sits inside a tight band around 16.667%.
//
// Over 600,000 rolls the standard deviation of a face share is about
// 0.048 percentage points, so a +/- 0.5 percentage point band is roughly
// a 10-sigma bound. The old firmware sat at +3.33 points (20.00% instead
// of 16.67%), which is about 69 sigma away — so this band catches the
// defect with a very large margin while never flaking on a correct
// generator.
static void assertSharesWithinBand(const uint32_t counts[6],
                                    uint32_t total, const char *label) {
    for (int i = 0; i < 6; ++i) {
        const float pct = 100.0f * static_cast<float>(counts[i]) /
                          static_cast<float>(total);
        char msg[160];
        snprintf(msg, sizeof(msg),
                 "%s: face %d share %.4f%% is outside 16.167%%..17.167%% "
                 "(count %u of %u)",
                 label, i + 1, pct, counts[i], total);
        TEST_ASSERT_TRUE_MESSAGE(pct > 16.167f && pct < 17.167f, msg);
    }
}

// ---------------------------------------------------------------------------
// Tests — generator invariants
// ---------------------------------------------------------------------------

// The mixer output must never be the rejection sentinel (0xFFFFFFFF),
// which would otherwise be a distinguishable value.
void test_mixer_output_is_never_the_sentinel(void) {
    arcdice::Prng prng;
    for (int i = 0; i < 20000; ++i) {
        TEST_ASSERT_NOT_EQUAL_MESSAGE(0xFFFFFFFFu, prng.next(),
                                      "mixer output is the sentinel value");
    }
}

// Consecutive counter values must produce different mixed words — the
// avalanche property the whole design depends on.
void test_mixer_avalanche_on_consecutive_counters(void) {
    arcdice::Prng prng;
    uint32_t previous = prng.next();
    for (int i = 0; i < 5000; ++i) {
        const uint32_t z = prng.next();
        TEST_ASSERT_NOT_EQUAL_MESSAGE(previous, z,
                                      "consecutive draws produced the same word");
        previous = z;
    }
}

// The face must never leave the 1..6 range.
void test_face_is_always_in_range(void) {
    arcdice::Prng prng;
    for (int i = 0; i < 20000; ++i) {
        const uint8_t face = prng.face();
        TEST_ASSERT_TRUE_MESSAGE(face >= 1 && face <= 6,
                                 "face out of the 1..6 range");
    }
}

// The spin must show more than one face while spinning, otherwise the
// "roll" is a single draw and the entropy mixing has no effect.
void test_spin_shows_multiple_distinct_faces(void) {
    arcdice::Prng prng;
    bool seenFace[7] = {false};
    uint8_t distinct = 0;
    for (uint8_t i = 0; i < firmware::kDrawsPerRoll; ++i) {
        const uint8_t face = prng.face();
        if (!seenFace[face]) {
            seenFace[face] = true;
            distinct++;
        }
    }
    TEST_ASSERT_GREATER_THAN_UINT8(3, distinct);
}

// The roll walk must visit a fresh part of the state space between
// consecutive rolls. The old 17-draw walk over a 255-period LFSR only
// reached 15 distinct states; this test pins the fixed behaviour.
//
// The 60 visited states are kept in a small array and compared by linear
// search: the state is 32 bits wide, so a direct-indexed table would
// need 4 GB.
void test_roll_walk_covers_the_state_space(void) {
    arcdice::Prng prng;
    uint32_t visited[64];
    uint32_t distinct = 0;

    for (int roll = 0; roll < 60; ++roll) {
        for (uint8_t i = 0; i < firmware::kDrawsPerRoll; ++i) {
            prng.face();
        }
        const uint32_t s = prng.state();
        bool seen = false;
        for (uint32_t k = 0; k < distinct; ++k) {
            if (visited[k] == s) {
                seen = true;
                break;
            }
        }
        if (!seen) {
            visited[distinct++] = s;
        }
    }
    // 60 rolls must reach 60 distinct states — no sub-cycle collapse.
    TEST_ASSERT_EQUAL_UINT32(60, distinct);
}

// ---------------------------------------------------------------------------
// Tests — distribution of the displayed face
// ---------------------------------------------------------------------------

// THE core regression test: the displayed face must be uniform.
//
// This is the assertion the old 8-bit LFSR failed with chi-square = 2400
// against a critical value of 11.07.
void test_face_shares_are_within_half_a_percent(void) {
    const uint32_t kRolls = 600000;
    uint32_t counts[6] = {0};
    simulateRolls(kRolls, [](uint32_t) -> uint16_t { return 0x37; }, counts);
    assertSharesWithinBand(counts, kRolls, "constant entropy");
}

// Same uniformity check, but with entropy that varies the way a real
// button press does.
void test_uniform_with_variable_entropy(void) {
    const uint32_t kRolls = 600000;
    uint32_t counts[6] = {0};
    simulateRolls(kRolls, [](uint32_t r) -> uint16_t {
        return static_cast<uint16_t>(0x37 + ((r * 37) % 251));
    }, counts);
    assertSharesWithinBand(counts, kRolls, "variable entropy");
}

// Weak entropy — only 4 distinct values, i.e. a user who presses the
// button with machine-like precision.
void test_uniform_with_weak_entropy(void) {
    const uint32_t kRolls = 600000;
    uint32_t counts[6] = {0};
    simulateRolls(kRolls, [](uint32_t r) -> uint16_t {
        return static_cast<uint16_t>(0x3700 + (r % 4));
    }, counts);
    assertSharesWithinBand(counts, kRolls, "weak entropy");
}

// Zero entropy — the pathological case where the button timing carries
// no information at all. The counter alone must still produce a uniform
// face distribution.
void test_uniform_with_zero_entropy(void) {
    const uint32_t kRolls = 600000;
    uint32_t counts[6] = {0};
    simulateRolls(kRolls, [](uint32_t) -> uint16_t { return 0x0000; }, counts);
    assertSharesWithinBand(counts, kRolls, "zero entropy");
}

// Chi-square uniformity across many independent streams.
//
// A single chi-square test rejects a correct generator 5% of the time by
// construction, so this test runs 24 independent streams and requires
// that at most 5 of them exceed the 0.05 critical value. Under the
// binomial distribution the expected number of exceedances is 1.2 with a
// standard deviation of 1.1, so 5 is a comfortable bound; a genuinely
// biased generator would fail all 24.
void test_chi_square_across_independent_streams(void) {
    const uint32_t kStreams = 24;
    const uint32_t kRollsPerStream = 6000;
    const float critical = 11.07f;  // df=5, p=0.05
    uint32_t exceedances = 0;

    for (uint32_t s = 0; s < kStreams; ++s) {
        arcdice::Prng prng;
        prng.setState(0x5A5A5A5Au + s * 0x9E3779B9u);
        uint32_t counts[6] = {0};
        simulateRollsSeeded(prng, kRollsPerStream,
                            [](uint32_t) -> uint16_t { return 0x37; },
                            counts);
        if (chiSquare(counts, kRollsPerStream) > critical) {
            exceedances++;
        }
    }

    char msg[160];
    snprintf(msg, sizeof(msg),
             "%u of %u independent streams exceeded the 0.05 critical "
             "value (expected ~1.2, allowed 5)",
             exceedances, kStreams);
    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE(5, exceedances, msg);
}

// A single face must never exceed 19% of the rolls. The uniform share is
// 16.67%, so a 19% ceiling leaves generous headroom for sampling noise
// while catching the observed 20% spike.
void test_no_face_exceeds_19_percent(void) {
    const uint32_t kRolls = 60000;
    uint32_t counts[6] = {0};
    simulateRolls(kRolls, [](uint32_t) -> uint16_t { return 0x37; }, counts);

    for (int i = 0; i < 6; ++i) {
        const float pct = 100.0f * static_cast<float>(counts[i]) /
                          static_cast<float>(kRolls);
        char msg[96];
        snprintf(msg, sizeof(msg), "face %d appeared %.2f%% of the time",
                 i + 1, pct);
        TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(19.0f, pct, msg);
    }
}

// ---------------------------------------------------------------------------
// Test runner
// ---------------------------------------------------------------------------

int main(int /*argc*/, char** /*argv*/) {
    UNITY_BEGIN();
    RUN_TEST(test_mixer_output_is_never_the_sentinel);
    RUN_TEST(test_mixer_avalanche_on_consecutive_counters);
    RUN_TEST(test_face_is_always_in_range);
    RUN_TEST(test_spin_shows_multiple_distinct_faces);
    RUN_TEST(test_roll_walk_covers_the_state_space);
    RUN_TEST(test_face_shares_are_within_half_a_percent);
    RUN_TEST(test_uniform_with_variable_entropy);
    RUN_TEST(test_uniform_with_weak_entropy);
    RUN_TEST(test_uniform_with_zero_entropy);
    RUN_TEST(test_chi_square_across_independent_streams);
    RUN_TEST(test_no_face_exceeds_19_percent);
    return UNITY_END();
}
