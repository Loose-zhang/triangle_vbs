# Replay sample: 2026-05-17 bridge-state capture

- Capture window: 2026-05-17 13:33:10 to about 13:36:18 CST
- Duration: about 3 minutes
- Target mode during live validation: `-v b`
- Purpose: preserve a live dual-base sample for later replay of phase-clock and B-only bridge state-machine behavior

## Files

- `base_2435536.rtcm3`
  - Base A input stream
- `base_2435540.rtcm3`
  - Base B input stream
- `bcep00bkg0.rtcm3`
  - Broadcast ephemeris input stream
- `ssrc00cne0.rtcm3`
  - SSR input stream
- `synthetic_base_b.rtcm3`
  - Synthetic RTCM output emitted by the current VBS pipeline while targeting base B
- `live_vbs.log`
  - Runtime diagnostics from `ntrip_rtcm_obs`

## Capture notes

- During this capture, the live data repeatedly showed only `1` usable B-only bridge candidate.
- Several epochs had bridge residuals below the current RMS threshold, roughly `0.09` to `0.13 cyc`.
- Even with low residuals, the new multi-epoch bridge state correctly stayed at `acquiring/not-enough` because the candidate count was below the configured minimum of `3`.
- This makes the sample useful for regression tests that should verify:
  1. low residuals alone do not trigger `fixed`
  2. `bridge` remains blocked when candidate support is insufficient
  3. phase-clock readiness can be stable while B-only bridge readiness is still not publishable
