/* wafersync_common.h - shared by ALL WaferSync projects (header only, no project of its own)
 * Contains: constants, message/pulse codes, shared-memory layout, GPIO layer, event log,
 *           BLACKBOX recorder, timer helpers.
 * Every project adds this folder to its include path:  -I../wafersync_common
 */
#ifndef WAFERSYNC_COMMON_H
#define WAFERSYNC_COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <process.h>
#include <sys/neutrino.h>
#include <sys/iomsg.h>

/* each project only uses part of this header */
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"

#define N_STAGES 5
#define TICK_MS  20
#define T_LO 180
#define T_HI 220
#define P_LO 450
#define P_HI 550

/* Hardware pins (BCM numbers) */
#define PIN_DHT11   4
#define PIN_IR_OUT  17
#define PIN_LED_GRN 22
#define PIN_LED_YEL 23
#define PIN_LED_RED 25   /* physical pin 22 (pin 18 would be GPIO 24) */
/* Most IR obstacle modules pull OUT low when something is in front of them */
#define IR_ACTIVE_LOW    1
/* 1: object in front of IR = wafer in place (fault when you REMOVE it)
 * 0: object in front of IR = wafer/handler FAULT (wave a hand to trigger it) */
#define IR_BLOCKED_IS_OK 1
/* scaled = 200 + (reading - room_temperature_at_start) * GAIN. GAIN 10 -> +-2 C is OK */
#define DHT_NOMINAL_T 200
#define DHT_GAIN_T    10
/* green LED on-time after key s */
#define GREEN_ON_MS 3000

static const char *STAGE_NAME[N_STAGES] = {"Wafer Load", "Position Verify", "Chamber Stabilize", "Process Active", "Wafer Unload"};
static const int STAGE_MS[N_STAGES]    = {600, 400, 800, 1500, 600};   /* nominal duration  */
static const int DEADLINE_MS[N_STAGES] = {900, 700, 1100, 1800, 900};  /* stage deadline    */

enum { PC_TICK = _PULSE_CODE_MINAVAIL, PC_STAGE_START, PC_ABORT, PC_PARK, PC_DEADLINE, PC_INTERLOCK, PC_CLEAR, PC_STAGE_END };
enum { M_STAGE_DONE = _IO_MAX + 1, M_START, M_RESET, M_STATUS, M_ABORT_DONE, M_SAMPLE_CH, M_SAMPLE_HD };
enum { IL_CHAMBER = 0, IL_HANDLER = 1 };
enum { R_NONE, R_CHAMBER, R_HANDLER, R_DEADLINE, R_ENTRY };
enum { ST_IDLE, ST_RUN, ST_PROTECT, ST_ABORTING, ST_ABORTED, ST_COMPLETE };
enum { SRC_WORKER = 0, SRC_CLI = 1 };

static const char *STATE_NAME[] = {"IDLE", "RUNNING", "PROTECTIVE", "ABORTING", "ABORTED", "COMPLETE"};
static const char *REASON_NAME[] = {"none", "chamber condition", "wafer/handler fault", "stage deadline miss", "entry condition"};

/* ---------- blackbox event types ---------- */
enum { BB_START, BB_STAGE_START, BB_STAGE_DONE, BB_IL_RAISE, BB_IL_CLEAR, BB_PROTECT, BB_RECOVER, BB_ABORT,
       BB_ABORT_DONE, BB_COMPLETE, BB_DEADLINE, BB_REJECT, BB_ENTRY_FAIL, BB_EXIT_FAIL, BB_RESET, BB_INJECT,
       BB_SENSOR, BB_SNAP, BB_N_TYPES };
static const char *BB_NAME[BB_N_TYPES] = {"START", "STAGE_START", "STAGE_DONE", "IL_RAISE", "IL_CLEAR", "PROTECT",
       "RECOVER", "ABORT", "ABORT_DONE", "COMPLETE", "DEADLINE_MISS", "REJECTED", "ENTRY_FAIL", "EXIT_FAIL",
       "RESET", "INJECT", "SENSOR", "SNAPSHOT"};
#define BB_N    1024
#define BB_FILE "/tmp/wafersync_blackbox.log"

/* names of the 4 worker executables (the launcher starts them) */
#define EXE_SUPERVISOR "ws_supervisor"
#define EXE_INTERLOCK  "ws_interlock"
#define EXE_CHAMBER    "ws_chamber"
#define EXE_HANDLER    "ws_handler"

typedef struct { uint16_t type; int a, b, c; int src; } wmsg_t;

typedef struct {                       /* process record (only Supervisor writes) */
    uint32_t seq;
    int      done[N_STAGES];
    long     ns[N_STAGES];
    int      final;                    /* 0 open, 1 complete, 2 aborted */
    uint32_t crc;
} record_t;

typedef struct {
    int  invalid, miss;
    long err_last_us, err_min_us, err_max_us; int have_err;
    long il_resp_us, f2p_us, prot_us, rec_us, down_us, abort_us;
} metrics_t;

typedef struct {
    int state, stage, reason, recoveries, enable, il[2];
    record_t rec; int rec_ok; metrics_t m;
    int temp, press, seal, pos_ok, hnd_ok;
} status_t;

typedef struct {                       /* one blackbox record */
    uint32_t seq;
    long     t_us;                     /* microseconds since start */
    int      type, state, stage, temp, press, pos, a, b;
    uint32_t prev, crc;                /* hash chain */
} bb_rec_t;

typedef union { struct _pulse pulse; wmsg_t m; } recv_t;

/* ---------- plant + shared memory (visible to all 4 processes + CLI) ---------- */
#define LOG_N 10
enum { P_SUP, P_IL, P_HD, P_CH, N_PROC };
typedef struct { volatile int temp, press, seal, pos_ok, hnd_ok, extra_ticks; volatile long fault_ns; } plant_t;
typedef struct {
    plant_t plant;
    volatile int il_active[2], out_enable, auto_stage;
    volatile int pid[N_PROC], chid[N_PROC];   /* where each process can be reached */
    volatile int sens_temp, sens_pos_ok;      /* what the real sensors say (200 / 1 when simulated) */
    volatile int inj_temp, inj_pos;           /* faults injected from the keyboard */
    volatile int hw_on, hw_dht_c, hw_ir_blocked; /* shown on the dashboard */
    volatile int ir_ignore;                   /* key i: ignore the IR sensor */
    volatile int hw_ir_raw;                   /* raw level of the IR pin: 0 or 1 */
    volatile long green_until_ns;             /* green LED stays on until this time (key s) */
    volatile int  led_r, led_g, led_y;        /* LED states, shown on the dashboard */
    volatile int sys_state, sys_stage;        /* supervisor state/stage, published for the blackbox */
    long t_start;
    int log_head; char logbuf[LOG_N][100];
    pthread_mutex_t log_mtx;
    bb_rec_t bb[BB_N];                        /* BLACKBOX ring */
    volatile uint32_t bb_head, bb_prev;
    pthread_mutex_t bb_mtx;
} shm_t;
static shm_t *S;
#define plant      (S->plant)
#define il_active  (S->il_active)
#define out_enable (S->out_enable)
#define auto_stage (S->auto_stage)
#define t_start    (S->t_start)
#define logbuf     (S->logbuf)
#define log_head   (S->log_head)
#define log_mtx    (S->log_mtx)

#define SHM_NAME "/wafersync_shm"
/* create=1 only in the launcher (wafersync); all other processes attach with create=0 */
static void shm_attach(int create) {
    if (create) shm_unlink(SHM_NAME);
    int fd = shm_open(SHM_NAME, create ? (O_CREAT | O_RDWR) : O_RDWR, 0666);
    if (fd < 0) { perror("shm_open (is the wafersync launcher running?)"); exit(1); }
    if (create && ftruncate(fd, sizeof(shm_t)) < 0) { perror("ftruncate"); exit(1); }
    S = mmap(NULL, sizeof(shm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (S == MAP_FAILED) { perror("mmap"); exit(1); }
    close(fd);
    if (create) {
        memset(S, 0, sizeof(shm_t));
        pthread_mutexattr_t ma; pthread_mutexattr_init(&ma);
        pthread_mutexattr_setpshared(&ma, PTHREAD_PROCESS_SHARED);
        pthread_mutexattr_setprotocol(&ma, PTHREAD_PRIO_INHERIT);   /* no priority inversion on the log locks */
        pthread_mutex_init(&log_mtx, &ma);
        pthread_mutex_init(&S->bb_mtx, &ma);
        auto_stage = -1;
        S->sens_temp = DHT_NOMINAL_T; S->sens_pos_ok = 1;
    }
}
/* every process creates its own channel and publishes (pid, chid); others ConnectAttach to it */
static void publish(int who, int chid) { S->chid[who] = chid; __sync_synchronize(); S->pid[who] = getpid(); }
static int connect_peer(int who) {
    while (!S->pid[who]) usleep(10000);
    int c = ConnectAttach(0, S->pid[who], S->chid[who], _NTO_SIDE_CHANNEL, 0);
    if (c < 0) { perror("ConnectAttach"); exit(1); }
    return c;
}

static long now_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1000000000L + t.tv_nsec; }
static void set_prio(int p) { struct sched_param sp; memset(&sp, 0, sizeof sp); sp.sched_priority = p; pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp); }

/* =====================================================================
 *  Raspberry Pi 4 GPIO layer (BCM2711, registers at 0xFE200000)
 *  Build with -DNO_HW_GPIO for simulation only.
 * ===================================================================== */
#ifndef NO_HW_GPIO
#define BCM2711_GPIO_BASE 0xFE200000UL
#define BCM2711_GPIO_SIZE 0x1000UL
#define GPFSEL0 0
#define GPSET0  7
#define GPCLR0  10
#define GPLEV0  13
static volatile uint32_t *gpio_base;
static int hw_ready;

static int hw_gpio_init(void) {
    void *v = mmap_device_memory(NULL, BCM2711_GPIO_SIZE, PROT_READ | PROT_WRITE | PROT_NOCACHE, 0, BCM2711_GPIO_BASE);
    if (v == MAP_FAILED) { fprintf(stderr, "GPIO map failed (run as root?): %s\n", strerror(errno)); return 0; }
    gpio_base = (volatile uint32_t *)v; hw_ready = 1; return 1;
}
static void gpio_set_fsel(int pin, int func) {          /* 0 = input, 1 = output */
    if (!hw_ready) return;
    int reg = GPFSEL0 + pin / 10, sh = (pin % 10) * 3;
    uint32_t v = gpio_base[reg]; v &= ~(7u << sh); v |= ((uint32_t)func & 7u) << sh; gpio_base[reg] = v;
}
static void gpio_write_pin(int pin, int level) {
    if (!hw_ready) return;
    gpio_base[(level ? GPSET0 : GPCLR0) + pin / 32] = 1u << (pin % 32);
}
static int gpio_read_pin(int pin) {
    if (!hw_ready) return -1;
    return (gpio_base[GPLEV0 + pin / 32] & (1u << (pin % 32))) ? 1 : 0;
}
static void gpio_pin_drive_out(int pin, int level) { gpio_set_fsel(pin, 1); gpio_write_pin(pin, level); }
static void gpio_pin_set_in(int pin) { gpio_set_fsel(pin, 0); }
/* BCM2711 pull resistors: mode 0 = none, 1 = pull-up, 2 = pull-down (registers at 0xE4) */
static void gpio_pull(int pin, int mode) {
    if (!hw_ready) return;
    int reg = 57 + pin / 16, sh = (pin % 16) * 2;
    uint32_t v = gpio_base[reg]; v &= ~(3u << sh); v |= ((uint32_t)mode & 3u) << sh; gpio_base[reg] = v;
}

static int wait_level(int pin, int level, long max_us) {
    long dl = now_ns() + max_us * 1000L;
    while (gpio_read_pin(pin) != level) if (now_ns() > dl) return -1;
    return 0;
}
/* DHT11: 40 bits, returns 1 and the integer Celsius value if the checksum is good */
static int dht11_read_raw(int pin, int *out_c) {
    uint8_t b[5] = {0, 0, 0, 0, 0};
    if (!hw_ready) return 0;
    gpio_pin_drive_out(pin, 0); usleep(20 * 1000);        /* start signal: low >= 18 ms */
    gpio_pin_set_in(pin);                                 /* release, pull-up takes it high */
    if (wait_level(pin, 0, 300) || wait_level(pin, 1, 300) || wait_level(pin, 0, 300)) return 0;
    for (int i = 0; i < 40; i++) {
        if (wait_level(pin, 1, 150)) return 0;
        long t = now_ns();
        if (wait_level(pin, 0, 200)) return 0;
        if ((now_ns() - t) / 1000L > 48) b[i / 8] |= (uint8_t)(1u << (7 - (i % 8)));
    }
    if ((uint8_t)(b[0] + b[1] + b[2] + b[3]) != b[4]) return 0;
    *out_c = b[2];
    return 1;
}
#else
static int hw_ready;
static int hw_gpio_init(void) { return 0; }
static void gpio_write_pin(int pin, int level) { (void)pin; (void)level; }
static int gpio_read_pin(int pin) { (void)pin; return -1; }
static void gpio_pin_drive_out(int pin, int level) { (void)pin; (void)level; }
static void gpio_pin_set_in(int pin) { (void)pin; }
static void gpio_pull(int pin, int mode) { (void)pin; (void)mode; }
static int dht11_read_raw(int pin, int *out_c) { (void)pin; (void)out_c; return 0; }
#endif

/* ---------- event log ring (shown on the CLI screen) ---------- */
static void logf_(const char *fmt, ...) {
    char tmp[100]; va_list ap; va_start(ap, fmt); vsnprintf(tmp, sizeof tmp, fmt, ap); va_end(ap);
    pthread_mutex_lock(&log_mtx);
    snprintf(logbuf[log_head % LOG_N], sizeof logbuf[0], "%7ld ms  %s", (now_ns() - t_start) / 1000000L, tmp);
    log_head++;
    pthread_mutex_unlock(&log_mtx);
}

/* =====================================================================
 *  BLACKBOX flight recorder (hash-chained records in shared memory)
 *  bb_log() is called from any process. Only the CLI process writes to disk.
 * ===================================================================== */
static uint32_t bb_hash(const bb_rec_t *r) {
    uint32_t h = 2166136261u ^ r->prev;
    int v[10] = {(int)r->seq, (int)r->t_us, r->type, r->state, r->stage, r->temp, r->press, r->pos, r->a, r->b};
    for (int i = 0; i < 10; i++) h = (h ^ (uint32_t)v[i]) * 16777619u;
    return h;
}
static void bb_log(int type, int a, int b) {
    if (!S) return;
    pthread_mutex_lock(&S->bb_mtx);
    uint32_t i = S->bb_head;
    bb_rec_t *r = &S->bb[i % BB_N];
    r->seq = i; r->t_us = (now_ns() - t_start) / 1000L;
    r->type = type; r->state = S->sys_state; r->stage = S->sys_stage;
    r->temp = plant.temp; r->press = plant.press; r->pos = plant.pos_ok;
    r->a = a; r->b = b;
    r->prev = S->bb_prev; r->crc = bb_hash(r);
    S->bb_prev = r->crc; S->bb_head = i + 1;
    pthread_mutex_unlock(&S->bb_mtx);
}
/* copy records [from, head) out of the ring (clamped to what is still in it) */
static int bb_copy(bb_rec_t *dst, uint32_t from, uint32_t *first) {
    pthread_mutex_lock(&S->bb_mtx);
    uint32_t head = S->bb_head;
    if (head - from > BB_N) from = head - BB_N;
    int n = (int)(head - from);
    for (int k = 0; k < n; k++) dst[k] = S->bb[(from + k) % BB_N];
    pthread_mutex_unlock(&S->bb_mtx);
    *first = from;
    return n;
}
/* returns the number of bad records (hash mismatch or broken chain link) */
static int bb_check(const bb_rec_t *r, int n) {
    int bad = 0;
    for (int k = 0; k < n; k++) {
        if (bb_hash(&r[k]) != r[k].crc) bad++;
        if (k > 0 && r[k].prev != r[k - 1].crc) bad++;
    }
    return bad;
}
static const char *bb_name(int t) { return (t >= 0 && t < BB_N_TYPES) ? BB_NAME[t] : "?"; }
static const char *st_name(int s) { return (s >= 0 && s <= ST_COMPLETE) ? STATE_NAME[s] : "?"; }
static int bb_is_incident(int t) {
    return t == BB_PROTECT || t == BB_ABORT || t == BB_DEADLINE || t == BB_ENTRY_FAIL || t == BB_EXIT_FAIL || t == BB_REJECT;
}
static void bb_print_rec(const bb_rec_t *r) {
    printf("%s %5u %10.3f ms  %-13s %-10s stage=%d T=%3d P=%3d pos=%d a=%-4d b=%d\n",
           bb_is_incident(r->type) ? "!!" : "  ", r->seq, r->t_us / 1000.0, bb_name(r->type), st_name(r->state),
           r->stage, r->temp, r->press, r->pos, r->a, r->b);
}

/* ---------- actuator hook (LEDs are driven only by led_thread in the CLI process) ---------- */
static void actuator_apply(void) { }

/* ---------- timers ---------- */
static timer_t mk_timer(int coid, int code, int value) {
    struct sigevent ev; timer_t t;
    memset(&ev, 0, sizeof ev);
    SIGEV_PULSE_INIT(&ev, coid, SIGEV_PULSE_PRIO_INHERIT, code, value);
    if (timer_create(CLOCK_MONOTONIC, &ev, &t) < 0) { perror("timer_create"); exit(1); }
    return t;
}
static void arm_timer(timer_t t, int first_ms, int period_ms) {
    struct itimerspec it;
    it.it_value.tv_sec = first_ms / 1000;  it.it_value.tv_nsec = (first_ms % 1000) * 1000000L;
    it.it_interval.tv_sec = period_ms / 1000; it.it_interval.tv_nsec = (period_ms % 1000) * 1000000L;
    timer_settime(t, 0, &it, NULL);
}

#endif /* WAFERSYNC_COMMON_H */
