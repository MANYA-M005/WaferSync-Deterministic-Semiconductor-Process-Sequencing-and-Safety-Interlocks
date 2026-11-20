/* ws_handler.c - WaferSync WAFER HANDLER process (priority 50)
 * Runs stages 0, 1, 4, reads the IR sensor, parks the wafer on abort, writes blackbox snapshots.
 * Start it only through the launcher (wafersync). Run as root (GPIO access).
 * Build (hardware): qcc -Vgcc_ntoaarch64le -I../wafersync_common -o ws_handler ws_handler.c
 * Build (sim only): qcc -Vgcc_ntoaarch64le -DNO_HW_GPIO -I../wafersync_common -o ws_handler ws_handler.c
 */
#include "wafersync_common.h"

static void *handler_worker(void *arg) {
    (void)arg; set_prio(50);
    int my_chid = ChannelCreate(0); publish(P_HD, my_chid);
    int to_sup = connect_peer(P_SUP);
    int to_il = connect_peer(P_IL);
    int self = ConnectAttach(0, 0, my_chid, _NTO_SIDE_CHANNEL, 0);
    timer_t tk = mk_timer(self, PC_TICK, 0); arm_timer(tk, TICK_MS, TICK_MS);
    timer_t st_timer = mk_timer(self, PC_STAGE_END, 0);
    long end_ns = 0; int active = -1, park = 0, ir_cnt = 0, snap_cnt = 0;
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
        case PC_PARK:  active = -1; arm_timer(st_timer, 0, 0); park = 5; break;
        case PC_TICK: {
            if (S->ir_ignore) { S->sens_pos_ok = 1; ir_cnt = 0; }
            else if (hw_ready) {                                     /* IR sensor, 3-sample debounce */
                int raw = gpio_read_pin(PIN_IR_OUT);
                if (raw >= 0) {
                    int blocked = IR_ACTIVE_LOW ? (raw == 0) : (raw != 0);
                    S->hw_ir_raw = raw; S->hw_ir_blocked = blocked;
                    int ok = IR_BLOCKED_IS_OK ? blocked : !blocked;
                    if (ok == S->sens_pos_ok) ir_cnt = 0;
                    else if (++ir_cnt >= 3) { ir_cnt = 0; if (!ok) plant.fault_ns = now_ns(); S->sens_pos_ok = ok; bb_log(BB_SENSOR, 1, ok); }
                }
            }
            plant.pos_ok = (S->sens_pos_ok && !S->inj_pos) ? 1 : 0;

            wmsg_t s; int rep; memset(&s, 0, sizeof s);
            s.type = M_SAMPLE_HD; s.a = plant.pos_ok; s.b = plant.hnd_ok;
            MsgSend(to_il, &s, sizeof s, &rep, sizeof rep);
            if (active >= 0 && plant.extra_ticks > 0) {                 /* injected stage delay */
                long rem_ms = (end_ns - now_ns()) / 1000000L; if (rem_ms < 1) rem_ms = 1;
                long add_ms = (long)plant.extra_ticks * TICK_MS;
                end_ns += add_ms * 1000000L; arm_timer(st_timer, (int)(rem_ms + add_ms), 0);
                plant.extra_ticks = 0;
            }
            if (++snap_cnt >= 5) {                                      /* blackbox snapshot every 100 ms while active */
                int bst = S->sys_state; snap_cnt = 0;
                if (bst == ST_RUN || bst == ST_PROTECT || bst == ST_ABORTING) bb_log(BB_SNAP, S->hw_ir_raw, S->hw_dht_c);
            }
            if (park > 0 && --park == 0) {
                wmsg_t d; memset(&d, 0, sizeof d); d.type = M_ABORT_DONE;
                MsgSend(to_sup, &d, sizeof d, &rep, sizeof rep);
            }
            break; }
        default: break;
        }
    }
    return NULL;
}

int main(void) {
    shm_attach(0);
    hw_gpio_init();
    handler_worker(NULL);
    return 0;
}
