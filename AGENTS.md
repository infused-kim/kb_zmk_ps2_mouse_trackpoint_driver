# AGENTS.md

ZMK module with a PS/2 mouse/TrackPoint driver.

- `src/drivers/ps2/`: PS/2 bus drivers (GPIO and UART).
- `src/drivers/input/input_mouse_ps2.c`: the mouse/TrackPoint driver. Parses
  PS/2 packets and reports Zephyr input events.
- `src/behaviors/behavior_mouse_setting.c`: `&mms` behavior to change
  TrackPoint settings at runtime.
- `zmk-config-example/` is gitignored and not part of the module.

The pointer acceleration (`zmk,input-accelerator`) was developed here and
merged as #19, then moved to its own module at
`../kb_zmk_mouse_acceleration`.

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

## Code style and working with the user

- **Don't commit unless the user explicitly asks.** Leave a reviewable working
  tree diff. When asked, make small separate commits, one per logical change,
  each building on its own, in their style: `feat(...): ...`,
  `refactor(...): ...`, `fix(...)`, `perf(...)`. Never push.
- Keep diffs minimal and easy to review. Refactors must not change behavior;
  ask before changes that alter how movement feels.
- Order functions in the order they're used, so a file reads top to bottom.
  Put prototypes near the top for that.
- Prefer plain `if`/`else` over clever one-liners, and short helpers with
  clear names over long functions.
