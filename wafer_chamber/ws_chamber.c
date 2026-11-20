/* ws_chamber.c - WaferSync CHAMBER MONITOR process (priority 50)
 * Runs stages 2 and 3, samples temp/pressure/seal, reads the DHT11 (low-priority thread).
 * Start it only through the launcher (wafersync). Run as root (GPIO access).
 * Build (hardware): qcc -Vgcc_ntoaarch64le -I../wafersync_common -o ws_chamber ws_chamber.c
 * Build (sim only): qcc -Vgcc_ntoaarch64le -DNO_HW_GPIO -I../wafersync_common -o ws_chamber ws_chamber.c
 */
#include "wafersync_common.h"

static void *chamber_worker(void *arg) {
    (void)arg; set_prio(50);
    int my_chid = ChannelCreate(0); publish(P_CH, my_chid);
    int to_sup = connect_peer(P_SUP);
    int to_il = connect_peer(P_IL);
    int self = ConnectAttach(0, 0, my_chid, _NTO_SIDE_CHANNEL, 0);
    timer_t tk = mk_timer(self, PC_TICK, 0); arm_timer(tk, TICK_MS, TICK_MS);
    timer_t st_timer = mk_timer(self, PC_STAGE_END, 0);
    long end_ns = 0; int active = -1;
    for (;;) {
        recv_t r; int rcvid = MsgReceive(my_chid, &r, sizeof r, NULL);
        if (rcvid > 0) { MsgError(rcvid, ENOSYS); continue; }
        if (rcvid < 0) continue;
        switch (r.pulse.code) {
        case PC_STAGE_START:
            active = r.pulse.value.sival_int;
            end_ns = now_ns() + (long)STAGE_MS[active] * 1000000L;
            arm_timer(st_timer, STAGE_MS[active], 0);              /* exact one-shot stage timer */
            break;
        case PC_STAGE_END:
            if (active >= 0) {
                wmsg_t d; int rep; memset(&d, 0, sizeof d); d.type = M_STAGE_DONE; d.a = active; d.src = SRC_WORKER;
                active = -1; MsgSend(to_sup, &d, sizeof d, &rep, sizeof rep);
            }
            break;
        case PC_ABORT: active = -1; arm_timer(st_timer, 0, 0); break;
        case PC_TICK: {
            plant.temp = S->inj_temp ? 300 : S->sens_temp;          /* DHT11 (or simulated) + injected fault */
            wmsg_t s; int rep; memset(&s, 0, sizeof s);
            s.type = M_SAMPLE_CH; s.a = plant.temp; s.b = plant.press; s.c = plant.seal;
            MsgSend(to_il, &s, sizeof s, &rep, sizeof rep);
            if (active >= 0 && plant.extra_ticks > 0) {                 /* injected stage delay */
                long rem_ms = (end_ns - now_ns()) / 1000000L; if (rem_ms < 1) rem_ms = 1;
                long add_ms = (long)plant.extra_ticks * TICK_MS;
                end_ns += add_ms * 1000000L; arm_timer(st_timer, (int)(rem_ms + add_ms), 0);
                plant.extra_ticks = 0;
            }
            break; }
        default: break;
        }
    }
    return NULL;
}

/* Low-priority DHT11 poller (one read every ~2 s) */
static void *dht11_thread(void *arg) {
    (void)arg; set_prio(20);
    int base = -1;
    for (;;) {
        sleep(2);
        int c = 0, ok = 0;
        for (int tries = 0; tries < 3 && !ok; tries++) { ok = dht11_read_raw(PIN_DHT11, &c); if (!ok) sleep(1); }
        if (!ok) continue;
        S->hw_dht_c = c;
        if (base < 0) { base = c; logf_("DHT11 calibrated: room temperature %d C = chamber %d", c, DHT_NOMINAL_T); }
        int scaled = DHT_NOMINAL_T + (c - base) * DHT_GAIN_T;
        int prev = S->sens_temp;
        S->sens_temp = scaled;
        if (prev >= T_LO && prev <= T_HI && (scaled < T_LO || scaled > T_HI)) plant.fault_ns = now_ns();
        if (scaled != prev) { bb_log(BB_SENSOR, 0, c); logf_("DHT11: %d C -> chamber temp %d", c, scaled); }
    }
    return NULL;
}

int main(void) {
    shm_attach(0);
    hw_gpio_init();
    if (hw_ready) { pthread_t th; pthread_create(&th, NULL, dht11_thread, NULL); }
    chamber_worker(NULL);
    return 0;
}
