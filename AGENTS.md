# AGENTS.md

ZMK module with a PS/2 mouse/TrackPoint driver.

- `src/drivers/ps2/`: PS/2 bus drivers (GPIO and UART).
- `src/drivers/input/input_mouse_ps2.c`: the mouse/TrackPoint driver. Parses
  PS/2 packets and reports Zephyr input events.
- `src/behaviors/behavior_mouse_setting.c`: `&mms` behavior to change
  TrackPoint settings at runtime.
- `src/mouse/input_accelerator.c` (`zmk,input-accelerator`): the pointer
  acceleration, as a ZMK input processor.
- `scripts/mouse_acceleration_models.py`: Python model of the acceleration
  curves, a typer CLI that plots and tabulates them. Keep it in sync with the
  C code. Its dependencies
  are in its PEP 723 header, so run it directly or with `uv run --script`.
- `zmk-config-example/` is gitignored and not part of the module.

## Building and checks

- The module can't be built on its own. Build it through the user's
  zmk-config at `../../zmk-config` (see its `AGENTS.md`), which uses this
  checkout via `ZMK_LOCAL_MODULES` in its `mise.local.toml`:
  `cd ../../zmk-config && just build think_corney_right`.
- Checks: `mise install`, then `prek install`. All checks are hooks in
  `.pre-commit-config.yaml`: clang-format v18.1.8 (same as ZMK v0.3),
  prettier, whitespace checks, and for the Python scripts ruff (config in
  `scripts/ruff.toml`) and ty. Run `just check` (all files) or
  `mise exec -- prek run --files <files>` before handing over.
  `just check-scripts` runs only ruff and ty.
- The ruff and ty hooks are local hooks that use mise's tools, so
  `mise.lock` pins their versions. ty runs in each script's PEP 723
  environment, otherwise it can't resolve the imports. They check every
  `.py` file directly in `scripts/`.
- This repo's `mise.toml` applies even when opened through a symlink, because
  mise resolves the physical path.
- Testing on the hardware is the user's job. Say what you couldn't verify.

## How input flows

1. The driver turns each PS/2 packet into separate events: `REL_X` with
   `sync = !have_y`, then `REL_Y` with `sync = true`. An axis with 0 movement
   sends no event at all.
2. ZMK uses `CONFIG_INPUT_MODE_THREAD`: `input_report` queues the event, and
   the input thread hands events to the listeners one at a time.
3. ZMK's listener runs the input processors on each event (layer overrides
   without `process-next` replace the base processors), adds up the values,
   and sends the HID report on `sync`.
4. A processor gets one event via `handle_event`, can change it in place and
   returns `ZMK_INPUT_PROC_CONTINUE` or `ZMK_INPUT_PROC_STOP`. It can't hold an
   event back or create new ones.

## Accelerator design

- The factor comes from the combined speed of both axes. The processor can't
  wait for Y, so it accelerates each axis as it arrives with the **previous
  report's** factor and calculates the next factor at sync. After a pause of
  more than 200 ms it starts from the curve's factor at speed 0.
- An earlier version had a copy of ZMK's listener (`zmk,input-listener-accel`)
  that accelerated the whole report at sync with the current report's factor.
  It was removed after the processor felt just as good.
- Each axis keeps a float remainder. A new movement (change of sign, or a
  pause of more than 200 ms) discards it and makes the first value at least 1.
  `prev_mov` is the last non-zero movement of each axis, so a pausing axis
  keeps its direction and remainder.
- With `enable-interval-based-speed`, speed is normalized by the time since
  the previous report. That time is measured when the input thread processes
  the event, not when the PS/2 packet arrived, so a delayed report reads as a
  slowdown (seen as ~23 ms gaps in logs). Known, not fixed.
- Known open edge case: a layer switch between the X and Y events of one
  report leaves the processor's report open until the next sync.

## Gotchas

- Kconfig: ZMK's `app/Kconfig` sets `default CBPRINTF_NANO` before module
  Kconfigs are read, so a module can't change that default. Logging floats
  needs `CONFIG_CBPRINTF_COMPLETE=y` set explicitly.
- The accelerator binding includes ZMK's `ip_common.yaml`, because ZMK's
  processor macros read `track-remainders` from every processor node.
- The curve is selected with `DT_INST_ENUM_IDX`, so
  `enum input_accelerator_curve` must match the order of the
  `acceleration-curve` enum in the binding.

## Code style and working with the user

- **Don't commit unless the user explicitly asks.** Leave a reviewable working
  tree diff. When asked, make small separate commits, one per logical change,
  each building on its own, in their style: `feat(accelerator): ...`,
  `refactor(accelerator): ...`, `fix(...)`, `perf(...)`. Never push.
- Keep diffs minimal and easy to review. Refactors must not change behavior;
  ask before changes that alter how movement feels.
- Order functions in the order they're used, so a file reads top to bottom.
  Put prototypes near the top for that.
- Prefer plain `if`/`else` over clever one-liners, and short helpers with
  clear names over long functions.
