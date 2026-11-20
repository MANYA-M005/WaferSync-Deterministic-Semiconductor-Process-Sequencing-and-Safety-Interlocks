/* ws_interlock.c - WaferSync INTERLOCK MANAGER process (priority 58)
 * Judges the samples sent by chamber/handler and raises/clears the 2 interlocks.
 * Start it only through the launcher (wafersync).
 * Build: qcc -Vgcc_ntoaarch64le -I../wafersync_common -o ws_interlock ws_interlock.c
 */
#include "wafersync_common.h"

static int il_chid;

static void il_eval(int id, int viol, int *okc, int coid) {
    if (viol) {
        okc[id] = 0;
        if (!il_active[id]) { il_active[id] = 1; bb_log(BB_IL_RAISE, id, 0); actuator_apply(); MsgSendPulse(coid, SIGEV_PULSE_PRIO_INHERIT, PC_INTERLOCK, id); }
    } else if (il_active[id]) {
        if (++okc[id] >= 10) { okc[id] = 0; il_active[id] = 0; bb_log(BB_IL_CLEAR, id, 0); actuator_apply(); MsgSendPulse(coid, SIGEV_PULSE_PRIO_INHERIT, PC_CLEAR, id); }
    }
}

static void *interlock_mgr(void *arg) {
    (void)arg; set_prio(58);
    il_chid = ChannelCreate(0); publish(P_IL, il_chid);
    int coid = connect_peer(P_SUP), okc[2] = {0, 0};
    for (;;) {
        recv_t r; int rcvid = MsgReceive(il_chid, &r, sizeof r, NULL);
        if (rcvid <= 0) continue;
        if (r.m.type == M_SAMPLE_CH) {
            int v = !(r.m.a >= T_LO && r.m.a <= T_HI && r.m.b >= P_LO && r.m.b <= P_HI && r.m.c);
            il_eval(IL_CHAMBER, v, okc, coid);
        } else if (r.m.type == M_SAMPLE_HD) {
            il_eval(IL_HANDLER, !(r.m.a && r.m.b), okc, coid);
        }
        MsgReply(rcvid, EOK, NULL, 0);
    }
    return NULL;
}

int main(void) {
    shm_attach(0);
    interlock_mgr(NULL);
    return 0;
}
