# Comms Protocol Optimization — Roadmap

Now that CC↔XPB transport is UDP/Ethernet rather than TTL/9600, the
ACK/retry scheme inherited from the serial design has more headroom
than it needs. This doc captures one already-confirmed fact and one
queued enhancement.

---

## Status

### (a) Telemetry frames carry no ACK — DONE ✅
Verified 2026-05-12 by code inspection + live transcript:

- `HB` (CC → ...) sent with `MessageType::INFO` at
  [src/clearcore/ClearCoreRTM.cpp](../src/clearcore/ClearCoreRTM.cpp)
  line 301.
- `STAT` (XPB → ...) sent with `MessageType::INFO` at
  [src/exp-board/ExpansionBoard.cpp](../src/exp-board/ExpansionBoard.cpp)
  line 610 (and the two other STAT sites).
- XPB receive policy explicitly: *“never ACK telemetry/one-way notices”*
  ([ExpansionBoard.cpp](../src/exp-board/ExpansionBoard.cpp) line 1251).
- CC `STAT;` handler ([ClearCoreRTM.h](../include/ClearCoreRTM.h)
  line 448) consumes and returns without emitting ACK.
- Live capture (`test/log/2026/05/12/102438_pytest.log`) shows zero
  ACK frames following any HB or STAT.

No further work required. Telemetry is already self-healing — if a
frame drops, the next one (250 ms / 1000 ms later) replaces it.

---

## TODO

### (b) Windowed bulk transfer for protocol upload
**Today**, every `PR_DAT;SEQ=N` from XPB is individually ACK'd by CC
before XPB sends `SEQ=N+1`. For a 4-step CSV that's ~10 frames
serialized over five round-trips. For a 50-step CSV it's ~100 frames
over fifty round-trips. On UDP/100 Mbit this is wasted latency —
RTT is < 1 ms but we're treating it like 9600-baud serial where each
ACK saved real wall-clock time.

**Proposed**: replace per-frame ACK with a single windowed validation
at PR_END.

```
Today                       Proposed
─────                       ────────
PR_BEG     →                PR_BEG          →
           ← ACK;PR_BEG=OK             ← ACK;PR_BEG=OK     (kept; gates upload)
PR_DAT 0   →                PR_DAT 0..N−1   →   (back-to-back)
           ← ACK;OK         PR_END;CRC=…    →
PR_DAT 1   →                            ← ACK;PROTO=OK;PHASH=…
           ← ACK;OK                          OR
…                                       ← NACK;MISSING=2,5
PR_DAT N−1 →                PR_DAT 2 →   PR_DAT 5 →   (selective retx)
           ← ACK;OK         PR_END;CRC=…    →
PR_END     →                            ← ACK;PROTO=OK;PHASH=…
           ← ACK;OK
                            (PROTO_RX=OK from CC unchanged at end)
```

#### Wire-level changes
- **XPB**: send `PR_DAT;SEQ=n;DATA=…` frames back-to-back without
  waiting on ACK between each. Hold the full set in a small TX
  buffer until ACK on `PR_END` arrives.
- **CC**: track `PR_DAT;SEQ=` arrivals into a bitmap (max steps is
  bounded — already capped, see `STEP_MAX` in firmware). On
  `PR_END;CRC=…`:
  - If all SEQs present and CRC matches: emit
    `ACK;PROTO=OK;PHASH=…` and proceed to `NOTICE;PROTO_RX=OK`.
  - If gaps: emit `NACK;MISSING=2,5,7;PHASH=…` (comma list, hard cap
    at e.g. 8 entries to keep frame size sane).
- **XPB**: on `NACK;MISSING=…`, retransmit *only* those SEQs, then
  re-send `PR_END;CRC=…`. After K rounds (e.g. 3) of NACK, abort
  and start a fresh PR_BEG cycle.

#### Why keep PR_BEG ACK
PR_BEG carries STEPS/LOOPS/PHASH metadata that CC needs to size the
arrival bitmap. ACK'ing it gates the upload start and gives XPB a
clean "I see you, send away" signal. Cheap, useful, keep.

#### Expected wins
- 4-step CSV: ~600 ms → ~150 ms (target).
- 50-step CSV: ~6 s → ~300 ms (rough estimate, dominated by serial
  send time on XPB's W5500 SPI, not by RTT).
- Cleaner transcripts: one PROTO upload becomes ~PR_BEG + N PR_DAT +
  PR_END + 2 ACKs, instead of 2N+4 frames.
- Lower XPB CPU: half the formatting/checksum work.

#### Risks / things to verify
- W5500 single-socket TX buffer is 2 KB by default. PR_DAT frames are
  short (~30 B with checksum); 50 of them = ~1.5 KB. Fits. For
  larger protocols we'd need to chunk.
- CC's UDP receive path needs to handle bursts. With the current
  ring buffer this is fine, but we should add a regression test that
  uploads a max-size protocol and asserts zero NACK rounds.
- The current `pendingMsg_` retry slot in
  [src/shared/RtmComms.cpp](../src/shared/RtmComms.cpp) is a single
  slot. Bursting PR_DAT means temporarily bypassing the
  ACK-armed-pending mechanism for those frames (they're explicitly
  fire-and-forget within the burst, and PR_END is the only ACK-armed
  frame in the chain).

#### Suggested implementation sketch
1. Add `MessageType::BULK` (no ACK, no retx, no REF — like INFO but
   semantically labelled). Use it for `PR_DAT` only.
2. CC: extend the receive handler to accumulate `PR_DAT` SEQs into a
   bitmap until `PR_END` arrives.
3. CC: extend `PR_END` handler to either ACK (success) or NACK
   (gaps). Keep `NOTICE;PROTO_RX=OK` as the final user-visible
   marker (tests already key off it).
4. XPB: rewrite `uploadProtocolToCC_()` to send-all-then-wait. On
   NACK, do selective retx and re-send PR_END.
5. Add scenario test (`test/scenarios/test_proto_upload_burst.py`)
   that uploads a 20-step CSV and asserts:
   - Single PR_BEG, single PR_END, single PROTO_RX=OK.
   - PR_DAT count == STEPS.
   - Zero NACK frames seen.
   - Total wall-clock ≤ 200 ms.

---

## Out of scope (for this branch)

### (c) Command/response collapse
Today most `REQ:X` patterns generate `ACK;OK` *and* a separate response
frame. e.g. `REQ:PROTO` → `ACK;OK` → `PR_BEG`. The ACK carries no
information the PR_BEG doesn't. Eliminating the bare ACK would shave
another ~10 % off transcript noise but requires touching every
request handler. Worth doing as a separate PR after (b) lands.
