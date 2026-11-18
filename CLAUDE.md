# QuestPad

## Build / test
- `cd firmware && cargo build --release` (+ `--features nicenano`) - firmware, target thumbv7em-none-eabihf; build both boards after firmware changes
- `sh tools/package.sh` - builds both boards + `firmware/target/questpad.bin`, `questpad-nicenano.uf2`, `dist/*.zip`
- `python host/mapping.py` - host self-test (CI runs it)
- `python host/vrtest.py` - VR-mode live check (needs pyusb + libusb-package, dongle in VR mode)
- No hardware in the loop: verify math by porting to Python and simulating

## Firmware gotchas
- Radio-side state (`gamepad::HANDS`, `RESTS`, `vr::FUSIONS`) = `static mut` touched only from radio interrupts; USB tasks talk to it via atomics/Signals
- No `f32::sqrt`/`abs` in core: see `vr.rs` `inv_sqrt`
- embassy-usb default max 4 interfaces (VR mode uses exactly 4: 2 vendor + CDC 2); more needs `max-interface-count-N` feature
- MS OS 2.0 descriptors: `MSOS_DESC` buffer in main.rs must fit all WinUSB functions (~170 B each)
- New `settings::Mode` variant: update `Mode::from`/`next`, `ui::mode_color`, main.rs USB match, `host/bridge.py` MODES, `host/gui.py` radio + MODE_HELP
- `host/dsu.py` `Rest` duplicates `gamepad.rs` `Rest` (same thresholds) - change both together

## Protocol contracts
- VR report layout (docs/vr.md) is shared with `monado/questpad_vr.h`: change both + bump protocol version byte
- Serial CMD_*/EV_* opcodes in `main.rs` mirror `host/bridge.py`

## Repo
- Fork: origin = Lodesthard/questpad, upstream = Jakey-Jakey/questpad
- CI (.github/workflows/build.yml) only on push to main / PRs; Actions disabled on fork until enabled
- autocrlf on: "LF will be replaced by CRLF" warnings are harmless

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
