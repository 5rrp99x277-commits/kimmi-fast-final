# CYW4373 RP2040 emulator v4.19 fixed + runtime BCDC debug patch

Base: user-provided `cyw4373_v419_fixed.zip`.

This package intentionally preserves the working v4.19 boot path:

- boot VID/PID `04b4:bd29`
- runtime VID/PID `04b4:0bdc`
- firmware bulk receive path
- DL_GETSTATE / DL_GO timing
- TinyUSB disconnect/reconnect path
- endpoint descriptors
- WL_REG_ON timing

Patch scope: runtime BCDC control OUT/IN parsing only.

Expected useful UART lines after runtime mount:

```text
CTRL SETUP bm=0x21 bReq=0 wLen=272 boot=0 pending=1 seq=1
CTRL DATA bm=0x21 bReq=0 wLen=272 boot=0 pending=1 seq=1
parse_out: total=272 ...
RUNTIME BCDC GET_VAR cur_etheraddr ...
CTRL -> parse_runtime_out done at DATA
RUNTIME BCDC IN: cmd=262 iovar='cur_etheraddr' resp_len=...
```

Build through GitHub Actions:

```text
Actions -> Build RP2040 CYW4373 Emulator UF2 -> Run workflow
```

Expected UF2 name:

```text
cyw4373_emulator_nohub_checked_v4_19_fixed_bcdc_debug.uf2
```
