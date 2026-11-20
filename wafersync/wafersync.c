/* wafersync.c - WaferSync LAUNCHER + CLI (the only program you start by hand)
 *
 *   ./wafersync                 creates shared memory, starts the 4 processes, shows the dashboard
 *   ./wafersync nocolor         same, without colours
 *   ./wafersync load            CPU load for the judges (run in background with &)
 *   ./wafersync replay [file]   BLACKBOX post-mortem: decode + verify /tmp/wafersync_blackbox.log
 *
 * It starts these executables, which must be in the SAME FOLDER as wafersync on the Pi:
 *   ws_supervisor  ws_interlock  ws_chamber  ws_handler
 * If a previous run crashed and left processes behind:
 *   slay ws_supervisor ws_interlock ws_chamber ws_handler
 *
 * Also runs the LED loop (led_thread, low priority):
 *   RED    : ON while the temperature fault is injected (key t, or key a auto-fault)
 *   GREEN  : ON for 3 seconds after key s is pressed
 *   YELLOW : ON all the time, but OFF whenever red or green is ON
 *
 * Wiring: DHT11 GPIO4 (pin 7), IR OUT GPIO17 (pin 11), Green GPIO22 (pin 15),
 *         Yellow GPIO23 (pin 16), Red GPIO25 (pin 22). LEDs via 220-330 ohm to GND.
 *
 * Build (hardware): qcc -Vgcc_ntoaarch64le -I../wafersync_common -o wafersync wafersync.c
 * Build (sim only): qcc -Vgcc_ntoaarch64le -DNO_HW_GPIO -I../wafersync_common -o wafersync wafersync.c
 * Run as root on the Pi.
 */
#include "wafersync_common.h"

/* =====================================================================
 *  LED loop (separate thread, low priority) - the only code that drives the LED pins
 * ===================================================================== */
static void *led_thread(void *arg) {
    (void)arg;
    set_prio(10);

    printf("\n[LED DEBUG] LED thread started. hw_on=%d\n", S->hw_on);
    fflush(stdout);

    int last_red = -1, last_green = -1, last_yellow = -1;

    for (;;) {
        int red = S->inj_temp ? 1 : 0;
        long current_time = now_ns();
        int green = (current_time < S->green_until_ns) ? 1 : 0;
        int yellow = (!red && !green) ? 1 : 0;

        S->led_r = red;
        S->led_g = green;
        S->led_y = yellow;

        gpio_write_pin(PIN_LED_RED, red);
        gpio_write_pin(PIN_LED_GRN, green);
        gpio_write_pin(PIN_LED_YEL, yellow);

        if (red != last_red || green != last_green || yellow != last_yellow) {
            int red_readback = gpio_read_pin(PIN_LED_RED);
            int green_readback = gpio_read_pin(PIN_LED_GRN);
            int yellow_readback = gpio_read_pin(PIN_LED_YEL);

            printf("[LED DEBUG] LOGIC R=%d G=%d Y=%d | GPIO READBACK R=%d G=%d Y=%d | hw_on=%d\n",
                   red, green, yellow,
                   red_readback, green_readback, yellow_readback, S->hw_on);
            fflush(stdout);

            last_red = red;
            last_green = green;
            last_yellow = yellow;
        }

        usleep(20 * 1000); /* repeat every 20 ms */
    }

    return NULL;
}

/* =====================================================================
 *  CLI
 * ===================================================================== */
static int to_sup_cli;
static int sup_call(int type, int a, int src, void *reply, int rlen) {
    wmsg_t m; memset(&m, 0, sizeof m); m.type = type; m.a = a; m.src = src;
    return MsgSend(to_sup_cli, &m, sizeof m, reply, rlen);
}
static int use_color = 1;               /* run "./wafersync nocolor" if the terminal shows odd characters */
#define COL(c) (use_color ? (c) : "")
#define RST "\033[0m"
#define RED "\033[31;1m"
#define GRN "\033[32;1m"
#define YEL "\033[33;1m"
#define CYN "\033[36;1m"
#define GRY "\033[90m"

/* ---- blackbox: disk writer (this process only) and on-screen panel ---- */
static FILE *bb_fp;
static uint32_t bb_flushed;
static bb_rec_t bb_cp[BB_N];
static int bb_view;                     /* key b: show the blackbox panel instead of metrics/log */

static void bb_flush(void) {
    if (!bb_fp) return;
    uint32_t first; int n = bb_copy(bb_cp, bb_flushed, &first);
    if (n == 0) return;
    if (first > bb_flushed) fprintf(bb_fp, "# LOST %u records (buffer overrun)\n", first - bb_flushed);
    for (int k = 0; k < n; k++) {
        const bb_rec_t *r = &bb_cp[k];
        fprintf(bb_fp, "%u,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%08x,%08x  # %s %s\n", r->seq, r->t_us, r->type, r->state,
                r->stage, r->temp, r->press, r->pos, r->a, r->b, r->prev, r->crc, bb_name(r->type), st_name(r->state));
    }
    bb_flushed = first + (uint32_t)n;
    fflush(bb_fp); fsync(fileno(bb_fp));
}
static void bb_panel(void) {
    uint32_t head = S->bb_head, from = head > 18 ? head - 18 : 0, first;
    int n = bb_copy(bb_cp, from, &first);
    printf("\n BLACKBOX - last %d events   (!! = incident, key b = back to metrics)\n", n);
    printf("   seq       time  event         state      stage   temp  press pos  a    b\n");
    for (int k = 0; k < n; k++) bb_print_rec(&bb_cp[k]);
}

static const char *pf(long v, long lim) {          /* PASS/FAIL against a deadline */
    static char buf[4][48]; static int k; char *b = buf[k++ & 3];
    if (v <= 0)       snprintf(b, 48, "%s  --  %s", COL(GRY), COL(RST));
    else if (v <= lim) snprintf(b, 48, "%s PASS %s", COL(GRN), COL(RST));
    else              snprintf(b, 48, "%s FAIL %s", COL(RED), COL(RST));
    return b;
}
static void metric(const char *name, long v, long lim) {
    printf("  %-40s %9ld us   < %-7ld us  %s\n", name, v, lim, pf(v, lim));
}

static void show(void) {
    static const char *SHORT[N_STAGES] = {"Load", "Verify", "Stabilize", "Process", "Unload"};
    status_t s; if (sup_call(M_STATUS, 0, SRC_CLI, &s, sizeof s) < 0) return;
    const char *sc = (s.state == ST_RUN || s.state == ST_COMPLETE) ? GRN : (s.state == ST_PROTECT) ? YEL :
                     (s.state == ST_ABORTING || s.state == ST_ABORTED) ? RED : GRY;
    if (use_color) printf("\033[2J\033[H"); else printf("\n\n");
    printf("%s==================== WAFERSYNC  |  Team BIT FLIPPERS  |  13_WaferSync ====================%s\n", COL(CYN), COL(RST));
    printf(" STATE: %s[ %s ]%s   Process-Enable: %s%s%s   Recoveries used: %d/1\n",
           COL(sc), STATE_NAME[s.state], COL(RST), COL(s.enable ? GRN : GRY), s.enable ? "ON" : "off", COL(RST), s.recoveries);
    printf("\n STAGES: ");
    for (int i = 0; i < N_STAGES; i++) {
        const char *tag, *c;
        if (s.rec.done[i])                                   { tag = "OK"; c = GRN; }
        else if (s.state == ST_RUN && i == s.stage)          { tag = ">>"; c = CYN; }
        else if ((s.state == ST_PROTECT || s.state == ST_ABORTING || s.state == ST_ABORTED) && i == s.stage) { tag = "!!"; c = (s.state == ST_PROTECT) ? YEL : RED; }
        else                                                 { tag = ".."; c = GRY; }
        printf("%s[%s] %s%s%s", COL(c), tag, SHORT[i], COL(RST), i < N_STAGES - 1 ? "  >  " : "\n");
    }
    printf("\n CONDITIONS\n");
    int tok = s.temp >= T_LO && s.temp <= T_HI, pok = s.press >= P_LO && s.press <= P_HI;
    printf("   Temp   : %s%4d C%s  (allowed %d-%d)   %s\n", COL(tok ? GRN : RED), s.temp, COL(RST), T_LO, T_HI, tok ? "ok" : "OUT OF RANGE");
    printf("   Press  : %s%4d mT%s (allowed %d-%d)   %s\n", COL(pok ? GRN : RED), s.press, COL(RST), P_LO, P_HI, pok ? "ok" : "OUT OF RANGE");
    printf("   Seal   : %s%s%s    Wafer-position: %s%s%s    Handler: %s%s%s\n",
           COL(s.seal ? GRN : RED), s.seal ? "ok" : "BAD", COL(RST),
           COL(s.pos_ok ? GRN : RED), s.pos_ok ? "ok" : "BAD", COL(RST),
           COL(s.hnd_ok ? GRN : RED), s.hnd_ok ? "ok" : "BAD", COL(RST));
    {
        printf("   Hardware: %s", S->hw_on ? "Raspberry Pi GPIO" : "simulated (no GPIO)");
        if (S->hw_on) printf("   DHT11: %d C   IR pin=%d (%s)%s   LEDs: GREEN %s  YELLOW %s  RED %s", S->hw_dht_c, S->hw_ir_raw, S->hw_ir_blocked ? "blocked" : "clear", S->ir_ignore ? " (ignored)" : "",
                             S->led_g ? "*" : "-", S->led_y ? "*" : "-", S->led_r ? "*" : "-");
        printf("\n");
    }
    printf("\n INTERLOCKS\n");
    printf("   Chamber-condition: %s%s%s     Wafer/handler-fault: %s%s%s     Last reason: %s\n",
           COL(s.il[0] ? RED : GRN), s.il[0] ? "** ACTIVE **" : "clear", COL(RST),
           COL(s.il[1] ? RED : GRN), s.il[1] ? "** ACTIVE **" : "clear", COL(RST), REASON_NAME[s.reason]);
    printf("\n PROCESS RECORD: final=%s   integrity(crc)=%s%s%s\n",
           s.rec.final == 0 ? "open" : s.rec.final == 1 ? "COMPLETE" : "ABORTED",
           COL(s.rec_ok ? GRN : RED), s.rec_ok ? "OK" : "CORRUPT", COL(RST));
    {   uint32_t first; int n = bb_copy(bb_cp, 0, &first); int bad = bb_check(bb_cp, n);
        printf(" BLACKBOX: %u events recorded | hash chain: %s%s%s | file: %s\n",
               S->bb_head, COL(bad ? RED : GRN), bad ? "BROKEN" : "OK", COL(RST), BB_FILE);
    }
    if (bb_view) {
        bb_panel();
    } else {
        long worst = labs(s.m.err_min_us) > labs(s.m.err_max_us) ? labs(s.m.err_min_us) : labs(s.m.err_max_us);
        printf("\n REAL-TIME METRICS                              VALUE        LIMIT        RESULT\n");
        metric("Unsafe condition -> interlock", s.m.il_resp_us, 20000);
        metric("Unsafe condition -> protective state", s.m.f2p_us, 20000);
        metric("Protective-state entry time", s.m.prot_us, 1000);
        metric("Stage timing error (worst)", worst, 5000);
        metric("Stage jitter (max-min)", s.m.err_max_us - s.m.err_min_us, 5000);
        metric("Recovery time (clear -> resumed)", s.m.rec_us, 1000);
        metric("Controlled-abort completion", s.m.abort_us, 200000);
        printf("  Stage-deadline misses: %s%d%s     Rejected (invalid/out-of-order) transitions: %s%d%s\n",
               COL(s.m.miss ? YEL : GRN), s.m.miss, COL(RST), COL(s.m.invalid ? YEL : GRN), s.m.invalid, COL(RST));
        printf("\n EVENT LOG\n");
        pthread_mutex_lock(&log_mtx);
        int n = log_head < LOG_N ? log_head : LOG_N;
        for (int i = log_head - n; i < log_head; i++) printf("  %s\n", logbuf[i % LOG_N]);
        pthread_mutex_unlock(&log_mtx);
    }
    printf("\n keys: s start | d delay | o out-of-order | t temp | p pressure | h handler | a auto-fault | i ignore IR | b blackbox | r reset | q quit\n");
    fflush(stdout);
}

/* ---- blackbox post-mortem:  ./wafersync replay [file] ---- */
static int bb_replay(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return 1; }
    char line[256]; uint32_t last = 0; long n = 0, bad = 0, gaps = 0, inc = 0; int first = 1;
    printf("BLACKBOX REPLAY: %s\n", path);
    printf("   seq       time  event         state      stage   temp  press pos  a    b\n");
    while (fgets(line, sizeof line, f)) {
        bb_rec_t r; unsigned seq, prev, crc; long t;
        if (line[0] == '#') { printf("%s", line); continue; }
        if (sscanf(line, "%u,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%x,%x", &seq, &t, &r.type, &r.state, &r.stage,
                   &r.temp, &r.press, &r.pos, &r.a, &r.b, &prev, &crc) != 12) continue;
        r.seq = seq; r.t_us = t; r.prev = prev; r.crc = crc;
        int ok = (bb_hash(&r) == crc), gap = (!first && prev != last);
        bb_print_rec(&r);
        if (!ok)  { printf("      ** CORRUPT RECORD: hash mismatch **\n"); bad++; }
        if (gap)  { printf("      ** GAP: records missing before this one **\n"); gaps++; }
        if (bb_is_incident(r.type)) inc++;
        first = 0; last = crc; n++;
    }
    fclose(f);
    printf("\nSUMMARY: %ld records, %ld incident events, %ld corrupt, %ld gaps  ->  INTEGRITY: %s\n",
           n, inc, bad, gaps, (bad == 0 && gaps == 0) ? "OK" : "FAILED");
    return 0;
}

/* folder of this executable: the 4 worker executables are started from the same folder */
static void self_dir(const char *argv0, char *dir, size_t len) {
    char buf[512] = "";
#ifdef __QNX__
    if (!_cmdname(buf)) buf[0] = 0;
#endif
    if (!buf[0]) {
        if (strchr(argv0, '/')) snprintf(buf, sizeof buf, "%s", argv0);
        else snprintf(buf, sizeof buf, "./%s", argv0);
    }
    char *slash = strrchr(buf, '/');
    if (slash) { *slash = 0; snprintf(dir, len, "%s", buf[0] ? buf : "/"); }
    else snprintf(dir, len, ".");
}
static pid_t child[4]; static int nchild;
static void stop_all(void) {
    for (int i = 0; i < nchild; i++) if (child[i] > 0) kill(child[i], SIGTERM);
    shm_unlink(SHM_NAME);
}
static void on_signal(int sig) { (void)sig; stop_all(); _exit(0); }

int main(int argc, char **argv) {
    const char *role = argc > 1 ? argv[1] : "main";
    if (!strcmp(role, "nocolor")) { use_color = 0; role = "main"; }
    if (!strcmp(role, "load")) { volatile double x = 0; for (;;) x += (double)(rand() % 1000) * 1.0001; }
    if (!strcmp(role, "replay")) return bb_replay(argc > 2 ? argv[2] : BB_FILE);

    /* ---- launcher + CLI ---- */
    shm_attach(1);
    t_start = now_ns();
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    rename(BB_FILE, BB_FILE ".prev");                       /* keep the previous session */
    bb_fp = fopen(BB_FILE, "w");
    if (bb_fp) fprintf(bb_fp, "# WaferSync blackbox. fields: seq,t_us,type,state,stage,temp,press,pos,a,b,prev,crc\n");
    else logf_("blackbox file not writable: %s", strerror(errno));
    if (!hw_gpio_init()) logf_("GPIO NOT AVAILABLE (%s) - LEDs/sensors are simulated. Run as root?", strerror(errno));
    else {                                      /* set pin directions once, before the other processes start */
        S->hw_on = 1;
        gpio_pin_drive_out(PIN_LED_GRN, 0); gpio_pin_drive_out(PIN_LED_YEL, 1); gpio_pin_drive_out(PIN_LED_RED, 0);
        gpio_pin_set_in(PIN_IR_OUT); gpio_pull(PIN_IR_OUT, 1);   /* pull-up: an unplugged OUT wire reads 1 (clear) */
        gpio_pin_set_in(PIN_DHT11);
    }
    pthread_t led_tid;
    int led_rc = pthread_create(&led_tid, NULL, led_thread, NULL);
    if (led_rc != 0) {
        fprintf(stderr, "[LED DEBUG] ERROR: pthread_create failed: %s (code %d)\n", strerror(led_rc), led_rc);
    } else {
        printf("[LED DEBUG] pthread_create succeeded; LED thread is running.\n");
        fflush(stdout);
    }

    static const char *exes[4] = {EXE_SUPERVISOR, EXE_INTERLOCK, EXE_CHAMBER, EXE_HANDLER};
    char dir[512]; self_dir(argv[0], dir, sizeof dir);
    for (int i = 0; i < 4; i++) {
        char path[640]; snprintf(path, sizeof path, "%s/%s", dir, exes[i]);
        pid_t pid = spawnl(P_NOWAIT, path, path, (char *)NULL);
        if (pid < 0) { fprintf(stderr, "cannot start %s: %s\n  (copy all 5 executables into the same folder)\n", path, strerror(errno)); stop_all(); return 1; }
        child[nchild++] = pid;
    }
    to_sup_cli = connect_peer(P_SUP);
    usleep(300 * 1000);
    show();
    for (;;) {
        fd_set fds; struct timeval tv = {0, 500000};
        FD_ZERO(&fds); FD_SET(0, &fds);
        int keyed = 0;
        if (select(1, &fds, NULL, NULL, &tv) > 0) {
            keyed = 1;
            char c = 0; if (read(0, &c, 1) <= 0) { bb_flush(); stop_all(); break; }
            switch (c) {
            case 's': S->green_until_ns = now_ns() + (long)GREEN_ON_MS * 1000000L;   /* green LED on for 3 s */
                      sup_call(M_START, 0, SRC_CLI, NULL, 0); break;
            case 'r': sup_call(M_RESET, 0, SRC_CLI, NULL, 0); break;
            case 'd': plant.extra_ticks = 50; bb_log(BB_INJECT, 4, 1000); logf_("INJECT: +1000 ms stage delay"); break;
            case 'o': { status_t s; sup_call(M_STATUS, 0, SRC_CLI, &s, sizeof s);
                        bb_log(BB_INJECT, 5, 0);
                        sup_call(M_STAGE_DONE, s.stage < 3 ? 4 : 1, SRC_CLI, NULL, 0); logf_("INJECT: out-of-order completion"); break; }
            case 't': if (S->inj_temp) S->inj_temp = 0; else { plant.fault_ns = now_ns(); S->inj_temp = 1; }
                      bb_log(BB_INJECT, 1, S->inj_temp);
                      logf_("INJECT: temperature fault %s", S->inj_temp ? "ON" : "off"); break;
            case 'p': if (plant.press != 500) plant.press = 500; else { plant.fault_ns = now_ns(); plant.press = 650; }
                      bb_log(BB_INJECT, 2, plant.press);
                      logf_("INJECT: pressure=%d", plant.press); break;
            case 'h': if (S->inj_pos) S->inj_pos = 0; else { plant.fault_ns = now_ns(); S->inj_pos = 1; }
                      bb_log(BB_INJECT, 3, S->inj_pos);
                      logf_("INJECT: wafer-position fault %s", S->inj_pos ? "ON" : "off"); break;
            case 'i': S->ir_ignore = !S->ir_ignore; bb_log(BB_INJECT, 6, S->ir_ignore);
                      logf_("IR sensor %s", S->ir_ignore ? "IGNORED (simulated wafer position)" : "ENABLED"); break;
            case 'a': auto_stage = 3; logf_("ARMED: chamber fault will hit at start of stage 3"); break;
            case 'b': bb_view = !bb_view; break;
            case 'q': bb_flush(); stop_all(); if (bb_fp) fclose(bb_fp); return 0;
            default: break;
            }
        }
        bb_flush();                                       /* low-priority disk write, outside the real-time path */
        { static int last_sig = -1;                       /* redraw on a key, a new event, an IR change or an LED change */
          int sig = log_head * 64 + (S->hw_ir_raw ? 1 : 0) + (S->hw_ir_blocked ? 2 : 0) + (S->ir_ignore ? 4 : 0)
                    + (S->led_r ? 8 : 0) + (S->led_g ? 16 : 0) + (S->led_y ? 32 : 0);
          if (keyed || sig != last_sig) { show(); last_sig = sig; } }
    }
    if (bb_fp) fclose(bb_fp);
    return 0;
}
