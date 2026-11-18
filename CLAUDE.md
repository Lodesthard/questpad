# QuestPad

## Build / test
- `cd firmware && cargo build --release` (+ `--features nicenano`) - firmware, target thumbv7em-none-eabihf; build both boards after firmware changes, zero warnings expected
- `sh tools/package.sh` - builds both boards + `firmware/target/questpad.bin`, `questpad-nicenano.uf2`, `dist/*.zip`
- `python host/mapping.py` - host self-test (CI runs it)
- `python host/vrtest.py` - VR-mode check on real hardware (needs pyusb + libusb-package, dongle in VR mode)
- Pure refactor of firmware? Prove it: `llvm-objcopy -O binary` the ELF and `cmp` with the previous `firmware/target/questpad.bin` (hw.rs / wdt.rs moves were byte-identical; moving code between modules can change inlining)

## Simulator (sim/)
- `cd sim && cargo run --release -- test` - 40 scenarios (~1 s); keep passing after firmware changes; `test <name>` shows one with its numbers
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
- Persisting settings (CMD_SETTINGS_SET persist=1) stops the radio for the flash write and restarts the link: controllers drop and rejoin (<1 s)
- embassy-usb default max 4 interfaces (VR mode uses exactly 4: 2 vendor + CDC 2); more needs `max-interface-count-N` feature
- MS OS 2.0 descriptors: `MSOS_DESC` buffer in main.rs must fit all WinUSB functions (~170 B each)
- New `settings::Mode` variant: update `Mode::from`/`next`, `ui::mode_color`, main.rs USB match, `host/bridge.py` MODES, `host/gui.py` radio + MODE_HELP
- `host/dsu.py` `Rest` duplicates `gamepad.rs` `Rest` (same thresholds) - change both together

## Protocol contracts
- VR report layout (docs/vr.md) is shared with `monado/questpad_vr.h` and sim/src/report.rs: change all + bump protocol version byte (new flag bits in unused positions don't need a bump)
- Serial CMD_*/EV_* opcodes in `serial.rs` mirror `host/bridge.py`

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
