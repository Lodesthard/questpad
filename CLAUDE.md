# QuestPad

## Build / test
- `cd firmware && cargo build --release` (+ `--features nicenano`) - firmware, target thumbv7em-none-eabihf; build both boards after firmware changes, zero warnings expected
- `sh tools/package.sh` - builds both boards + `firmware/target/questpad.bin`, `questpad-nicenano.uf2`, `dist/*.zip`
- `python host/mapping.py` - host self-test (CI runs it)
- `python host/vrtest.py` - VR-mode check window on real hardware (needs pyusb + libusb-package, dongle in VR mode); `--sim` reads sim live, `--text` terminal, `--boxes` skips the models; non-interactive check: `timeout 8 python -u host/vrtest.py --text` (`-u` or the output is lost when timeout kills it)
- vrtest.py draws `assets/quest_2_controller_low_poly_{left,right}.glb` if present (stdlib GLB loader, vertex clustering `MODEL_CELL`, half turn about y into the controller frame); boxes otherwise. assets/ license unconfirmed: don't commit it without the user's OK
- gui.py right side (DS4/VR modes) = `host/calib.py` CalibPanel: 3D view (same GLB models, `vrtest.mesh_faces`), per-controller axis maps (settings `imu_map` = right, `imu_map_left`) sent live with `settings_set(persist=False)`, saved on "Save to dongle". VR: dongle fusion via `vrtest.Reader` (pyusb; while the panel shows it, vrtest.py gets "Access denied"), else HostTracker = Python port of vr.rs Fusion fed by a Worker tap (`Worker.set_tap(name, fn)`). Change vr.rs Fusion => keep calib.py Fusion in step. `python host/calib.py --test` self-test, `python host/calib.py` / `gui.py --demo` sample motion
- Tk canvas speed: Tk's redraw costs more than the Python drawing (time `update_idletasks` too, not just your code): reuse items (coords/itemconfigure only when changed), no polygon outlines, ~200 faces per model
- Settings layouts: `Bridge.settings_get` normalises (one-map firmware: left map = right, `imu_maps` 1); `settings_set` sends only what the firmware has. Firmware: shorter layouts go through `Settings::from_bytes_over` (flash: over DEFAULT, USB: over current); a pre-`imu_map_left` layout sets both maps
- Serial 0xA1 notification frames: decode with `mapping.frame_notifications(f)`
- Serial tools (bridge.py, link.py, regs.py) fail with `PermissionError` on the COM port while the control panel holds it; vrtest.py uses the VR vendor interfaces: coexists with the panel unless its calibration view is showing in VR mode
- nice!nano: `firmware/flash.sh --nicenano`, or `python host/uf2.py firmware/target/questpad-nicenano.uf2` (finds the NICENANO drive by Board-ID, or reboots a running QuestPad into it)
- Pure refactor of firmware? Prove it: `llvm-objcopy -O binary` the ELF and `cmp` with the previous `firmware/target/questpad.bin` (hw.rs / wdt.rs moves were byte-identical; moving code between modules can change inlining)

## Simulator (sim/)
- `cd sim && cargo run --release -- test` - 41 scenarios (~1 s); keep passing after firmware changes; `test <name>` shows one with its numbers
- `cargo run --release -- budget` - error budget per sensor imperfection; tune vr.rs KP/KI with it + the bias/shake scenarios
- `cargo run --release -- live` - real-time dongle: `python host/vrsim_view.py` (3D), `QUESTPAD_PORT=socket://127.0.0.1:26772 python host/gui.py` (real panel; regs.py/link.py/dsu.py too), VR reports on UDP 26770, flash kept in `sim/target/release/live-flash.bin`
- Firmware modules are included unchanged by `#[path]` in sim/src/main.rs (all but main.rs/USB and selfupdate's RAM routine). All register/flash access must go through `crate::hw::{read, write}`: sim/src/hw.rs emulates TIMER0/PPI/RADIO/CCM/NVMC/RNG/WDT. New peripheral => emulate it there
- sim/fakes/ = embassy-nrf (interrupts, button, LED PWM) and cortex-m (`sys_reset` unwinds with `SysReset`, caught by world.rs / `stack::resets`)
- Scenario names: `stack_*` = full stack (world.rs + air.rs controllers on the air, boot mirrors main.rs; VR on only if the provisioned mode is VR); others = direct (ctrl.rs feeds `gamepad::link_notifications`, VR forced on). Each runs in its own process: firmware statics and flash start blank
- CCM is a keyed checksum (`hw::mic`), not AES: key/IV handling is tested, crypto bytes are not
- Mutation-test new scenarios (break the firmware on purpose, expect FAIL, restore)

## Firmware gotchas
- Radio-side state (`gamepad::HANDS`, `RESTS`, `vr::FUSIONS`) = `static mut` touched only from radio interrupts; USB tasks talk to it via atomics/Signals
- No `f32::sqrt`/`abs`/`round` in core: see `vr.rs` `inv_sqrt`, `gamepad.rs` `round_i16`
- IMU samples are converted to reference units (8.192/°/s, 1024/g) from the controller's register 0x32 (`gamepad::imu_config`, read by link.rs at join). `host/dsu.py` reads raw serial data and ignores 0x32
- VR fusion: `Rest` (at-rest offset) and the integral term must not both correct the bias: `Rest::epoch` resets `vr::Fusion::bias` (without it: 35°/min drift once set down)
- XInput mode = no serial port, the PC can't switch it back: Oculus/Menu + trigger 5 s, or button 3-9 s (nice!nano: wire P1.06/D9 to GND; >=10 s forgets pairing). Settings page 0xDE000 survives reflashing, so the mode does too
- New setting: append it at the end of the settings.rs layout (LEN, to_bytes, from_bytes_over), add its name to `host/bridge.py` SETTINGS, mutation-test with a sim scenario; older layouts keep the base values
- Persisting settings (CMD_SETTINGS_SET persist=1) stops the radio for the flash write and restarts the link: controllers drop and rejoin (<1 s)
- embassy-usb default max 4 interfaces (VR mode uses exactly 4: 2 vendor + CDC 2); more needs `max-interface-count-N` feature
- MS OS 2.0 descriptors: `MSOS_DESC` buffer in main.rs must fit all WinUSB functions (~170 B each)
- New `settings::Mode` variant: update `Mode::from`/`next`, `ui::mode_color`, main.rs USB match, `host/bridge.py` MODES, `host/gui.py` radio + MODE_HELP
- `host/dsu.py` `Rest` duplicates `gamepad.rs` `Rest` (same thresholds) - change both together
- Tk canvas UIs (vrtest.py): fonts follow display scaling but canvas coordinates don't: multiply offsets by `root.winfo_fpixels("1i") / 96`; cap window size to the screen; keep a frame under ~1500 polygons (backface cull + painter sort, ~10 ms)
- Tk: a `tk.StringVar` nobody keeps a reference to is garbage-collected and its label shows empty: use `text=` for static labels
- bridge.py falls back to the bundled pyserial in host/vendor, so every host tool runs standalone

## Protocol contracts
- VR report layout (docs/vr.md) is shared with `monado/questpad_vr.h` and sim/src/report.rs: change all + bump protocol version byte (new flag bits in unused positions don't need a bump)
- Serial CMD_*/EV_* opcodes in `serial.rs` mirror `host/bridge.py`
- Right after a host opens the VR endpoints, the first 1-2 IN reports are stale (endpoint buffer + `vr::REPORTS` Signal): ignore seq gaps when `dongle_ts` jumps >100 ms (vrtest.py `Stats.feed`)

## Repo
- Fork: origin = Lodesthard/questpad, upstream = Jakey-Jakey/questpad
- CI (.github/workflows/build.yml) on push to main / PRs / manual (`gh workflow run build -R Lodesthard/questpad`): mapping.py + sim scenarios + package; enabled on the fork. Check with `gh run list -R Lodesthard/questpad`
- autocrlf on: "LF will be replaced by CRLF" warnings are harmless

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
