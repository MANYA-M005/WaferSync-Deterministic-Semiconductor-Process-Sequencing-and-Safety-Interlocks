<div align="center">

# ⚙️ WaferSync

### Deterministic Semiconductor Process Sequencing & Safety Interlocks

*A multi-process, hard-real-time wafer-process controller built on QNX Neutrino*

![QNX](https://img.shields.io/badge/RTOS-QNX%20Neutrino-0b5cad?style=for-the-badge)
![Language](https://img.shields.io/badge/Language-C-00599C?style=for-the-badge&logo=c&logoColor=white)
![Board](https://img.shields.io/badge/Target-Raspberry%20Pi%204-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=white)
![Arch](https://img.shields.io/badge/Arch-AArch64-444?style=for-the-badge)

**Team BIT FLIPPERS**

</div>

---

## 📌 Overview

Semiconductor wafer processing must run in a strict order, inside tight timing windows, with safe behaviour the moment anything goes wrong. **WaferSync** runs a five-stage wafer workflow and enforces:

- ✅ correct **stage order** with entry and exit checks
- ⏱️ per-stage **deadline monitoring**
- 🛑 **safety interlocks** for chamber and wafer-handler faults
- 🔁 **checkpoint-verified recovery**, or a **controlled abort** that parks the wafer
- 🧾 a tamper-evident **blackbox flight recorder** for post-mortem analysis

---

## 📑 Table of Contents

1. [Overview](#-overview)
2. [The Five Programs](#-the-five-programs)
3. [Wafer Process](#-wafer-process)
4. [QNX Concepts Used](#-qnx-concepts-used)
5. [Checkpoint and Blackbox](#-checkpoint-and-blackbox)
6. [Hardware (Raspberry Pi 4)](#-hardware-raspberry-pi-4)
7. [Fault-Injection Tests](#-fault-injection-tests)
8. [Future Work](#-future-work)

---

## 🧱 The Five Programs

WaferSync is **five QNX programs**. The launcher starts four independent real-time processes, and they talk **only** through QNX message passing, pulses and shared memory.

| Program | Priority | Responsibility |
|---|:---:|---|
| 🧠 `ws_supervisor` | **60** | Stage sequencing, entry/exit checks, deadlines, recovery and abort |
| 🛡️ `ws_interlock` | **58** | Judges samples and raises or clears the two interlocks |
| 🌡️ `ws_chamber` | 50 | Temperature (DHT11), pressure, seal; runs stages 3 and 4 |
| 🤖 `ws_handler` | 50 | IR wafer position; runs stages 1, 2 and 5; parks the wafer on abort |
| 🖥️ `wafersync` | low | Starts everything, dashboard and keys, LED loop, blackbox disk writer |

> The real-time side never waits on the screen, keyboard or disk. Only the low-priority `wafersync` process does file and terminal I/O.

---

## 🔄 Wafer Process

| # | Stage | Nominal | Deadline |
|:-:|---|---:|---:|
| 1 | Wafer Load | 600 ms | 900 ms |
| 2 | Position Verify | 400 ms | 700 ms |
| 3 | Chamber Stabilize | 800 ms | 1100 ms |
| 4 | Process Active | 1500 ms | 1800 ms |
| 5 | Wafer Unload | 600 ms | 900 ms |

**Chamber limits:** temperature 180–220 · pressure 450–550 · seal must be valid (values are scaled, not raw °C).

### State machine

<img width="962" height="1600" alt="image" src="https://github.com/user-attachments/assets/f883b662-76d5-4897-9799-8836bb1e9c01" />


- **Chamber fault** (temperature, pressure or seal) → protective state. After the fault clears and the checkpoint is verified, the **interrupted stage restarts**. Only **one recovery** is allowed per run.
- **Handler fault** (position or handler health), a **second fault**, or an **invalid checkpoint** → **controlled abort**.

---

## 🧩 QNX Concepts Used

| Area | What we use |
|---|---|
| Processes | `spawnl`, separate address spaces for fault isolation |
| IPC | `ChannelCreate`, `ConnectAttach`, `MsgSend` / `MsgReceive` / `MsgReply` / `MsgError` |
| Pulses | `MsgSendPulse` for abort, interlock, stage start and park events |
| Timers | POSIX timers delivering pulses: 20 ms tick, one-shot stage timers, deadline timer (`CLOCK_MONOTONIC`) |
| Scheduling | `SCHED_FIFO` fixed priorities |
| Priority inheritance | `PTHREAD_PRIO_INHERIT`, `SIGEV_PULSE_PRIO_INHERIT` |
| Shared memory | `shm_open`, `ftruncate`, `mmap`; process-shared mutexes |
| Hardware | `mmap_device_memory` for direct BCM2711 GPIO register access |

---

## 🧾 Checkpoint and Blackbox

- **Process record:** a CRC-protected record of completed stages. Before any recovery, the supervisor checks the CRC and that stages complete in a clean prefix (no stage done before its predecessor).
- **Blackbox:** every key event plus a 100 ms snapshot while active, stored as **hash-chained** records (FNV-1a style, each linked to the previous one). The real-time processes write to a shared-memory ring, and the low-priority launcher writes it to `/tmp/wafersync_blackbox.log`.
- **Replay:** `./wafersync replay` decodes the file and reports corrupt records and gaps.

> The hash chain detects accidental corruption and tampering. It is an integrity check, not cryptographic security.

---

## 🔌 Hardware (Raspberry Pi 4)

| Component | BCM GPIO | Pin | Function |
|---|:---:|:---:|---|
| DHT11 DATA | 4 | 7 | Chamber temperature |
| IR sensor OUT | 17 | 11 | Wafer position |
| 🟢 Green LED | 22 | 15 | On for 3 s after start |
| 🟡 Yellow LED | 23 | 16 | Default; off while red or green is on |
| 🔴 Red LED | 25 | 22 | On while the temperature fault is active |

⚠️ Use 220–330 Ω resistors on the LEDs. Power the IR module from **3.3 V** so its output never exceeds what the Pi's GPIO accepts.

No hardware? Build with `-DNO_HW_GPIO` for a full **simulation mode**. Temperature and wafer position are simulated and faults are injected from the keyboard.

---

### ⌨️ Dashboard keys

| Key | Action | Key | Action |
|:-:|---|:-:|---|
| `s` | Start | `h` | Wafer-position fault |
| `r` | Reset | `a` | Auto-fault at stage 4 |
| `t` | Temperature fault (toggle) | `d` | +1000 ms stage delay |
| `p` | Pressure fault (toggle) | `o` | Out-of-order completion |
| `i` | Ignore IR sensor | `b` | Blackbox view |
| `q` | Quit | | |

---

## 🧪 Fault-Injection Tests

| Test | Injected | Expected response |
|---|---|---|
| Normal run | none | All 5 stages → `COMPLETE` |
| Temperature / pressure fault | `t` / `p` | Chamber interlock → protective → recover when cleared |
| Second fault in one run | `t` twice | Controlled abort |
| Wafer-position fault | `h` or IR sensor | Handler interlock → controlled abort |
| Deadline miss | `d` | Deadline handling → protective state |
| Invalid transition | `o` | Rejected and counted, system stays safe |

---

System Profiler :

<img width="1600" height="868" alt="image" src="https://github.com/user-attachments/assets/b4a1fb62-63e1-4e74-a2ef-126785d9b4ca" />

<img width="1587" height="800" alt="image" src="https://github.com/user-attachments/assets/346639d3-17bc-4004-9050-03b438a23481" />

<img width="1600" height="840" alt="image" src="https://github.com/user-attachments/assets/6b4910d2-7bfb-472e-8a35-ce9b59524454" />


Reuslts :
<img width="1600" height="771" alt="image" src="https://github.com/user-attachments/assets/3eaf9d91-7e49-406c-b47c-61d7db0e229e" />

<img width="1600" height="645" alt="image" src="https://github.com/user-attachments/assets/729c81b4-4631-4f2f-a0c8-022f9a0552e3" />


## 🔭 Future Work

- Real process sensors and industrial equipment interfaces
- Process crash detection and restart supervision
- Hardware emergency shutdown
- Persistent blackbox storage and richer forensics
- Validation under heavy CPU load

---

<div align="center">

**WaferSync** · QNX Neutrino · Real-Time · Safety-Oriented Control

</div>
