# rng

Random number generator with SP 800-90B health tests. **Not a certified entropy source.**

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-MulanPSL--2.0-blue)

Part of the [Tape-Out](https://github.com/Tape-Out) IP library: Bluespec IP over the
bus-neutral contracts in [`hwcore`](https://github.com/Tape-Out/hwcore), assembled by
[`xirang`](https://github.com/Tape-Out/xirang). Maturity runs `planned` -> `simulated` ->
`fpga-proven` -> `asic-ready` -> `silicon-proven`.

## Status

Simulated. The IP takes one raw bit per clock cycle from a noise source on `noise_i`. It runs the two continuous health tests of NIST SP 800-90B, section 4.4, on those raw bits, debiases them with a von Neumann corrector, and delivers 32-bit words.

- **Repetition count test:** cutoff C = 1 + ⌈20 / H⌉ for a false positive probability of 2^-20.
- **Adaptive proportion test:** window of 1024 samples, cutoff from table 2 of the standard, including the binary extension that also checks the complementary count. At H = 1 the build computes the exact binomial cutoff with integer arithmetic and refuses to compile if it disagrees with the table.
- **Startup:** after enabling, one full window of 1024 samples must pass both tests before any word is released (section 4.3, item 4).
- **Failure:** either test stops all output and sets its flag. Clearing and setting `en` again reruns the startup tests.

The entropy estimate `hTenths` must come from an assessment of the real noise source. The noise source itself, a ring oscillator cell, is not part of this IP yet, and neither is an approved conditioning function. This is why the output is not a certified entropy source: an entropy assessment and an approved conditioner are required before it can be.

The testbench computes both cutoffs on its own and compares them with the `cutoff` register. It drives several noise models, checks that each test trips only for the failure it is meant to catch, and checks that a word is never delivered twice.

## Registers

| Offset | Register | Fields |
| :--: | :-- | :-- |
| 0x00 | `ctrl` | `en`, `ien` (enabling again restarts the startup tests) |
| 0x04 | `status` | `valid`, `ready`, `rctfail`, `aptfail` |
| 0x08 | `data` | conditioned 32-bit word; reading it takes the word |
| 0x0C | `cutoff` | `rct` (15:0) and `apt` (31:16), the cutoffs computed from `hTenths` |

`hTenths` is the min-entropy estimate per bit in tenths of a bit, one of 2, 4, 6, 8 or 10. The interrupt is `ien` and either a waiting word or a failed test.

## Specification sources

The specifications this IP is implemented against, with their links, digests and the clause-by-clause comparison, are kept on the [`spec` branch](https://github.com/Tape-Out/rng/tree/spec).

## License

Mulan PSL v2.
