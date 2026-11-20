/* ws_supervisor.c - WaferSync SUPERVISOR process (priority 60)
 * Stage order, entry/exit checks, deadlines, recovery/abort.
 * Start it only through the launcher (wafersync).
 * Build: qcc -Vgcc_ntoaarch64le -I../wafersync_common -o ws_supervisor ws_supervisor.c
 */
#include "wafersync_common.h"

static int sup_chid;

/* ---------- process record ---------- */
static record_t rec;
static pthread_mutex_t rec_mtx = PTHREAD_MUTEX_INITIALIZER;
static uint32_t rec_crc(const record_t *r) {
    uint32_t c = 2166136261u ^ r->seq;
    for (int i = 0; i < N_STAGES; i++) { c = c * 16777619u ^ (uint32_t)r->done[i]; c = c * 16777619u ^ (uint32_t)(r->ns[i] & 0xffffffff); }
    return c * 16777619u ^ (uint32_t)r->final;
}
static void rec_commit(void) { rec.seq++; rec.crc = rec_crc(&rec); }
static int rec_verify(const record_t *r) {
    if (r->crc != rec_crc(r)) return 0;
    for (int i = 1; i < N_STAGES; i++) if (r->done[i] && !r->done[i - 1]) return 0;   /* must be a clean prefix */
    return 1;
}

/* ---------- condition helpers ---------- */
static int chamber_ok(void) { return plant.temp >= T_LO && plant.temp <= T_HI && plant.press >= P_LO && plant.press <= P_HI && plant.seal; }
static int handler_ok(void) { return plant.pos_ok && plant.hnd_ok; }

static int s_state, s_cur, s_reason, s_recov, s_gen;
static long s_stage_start, s_fault_t0, s_abort_t0;
static metrics_t M;
static int to_hd, to_ch, to_self;
static timer_t dl_timer;

/* every state change is published so the blackbox (any process) records the right state */
#define SET_STATE(x) do { s_state = (x); S->sys_state = (x); } while (0)

static void begin_stage(int i);
static void enter_protective(int reason, int ilid);

static void clear_all(void) {
    memset(&rec, 0, sizeof rec); rec_commit();
    memset(&M, 0, sizeof M);
    S->inj_temp = 0; S->inj_pos = 0;
    auto_stage = -1; SET_STATE(ST_IDLE); s_cur = 0; S->sys_stage = 0; s_reason = R_NONE; s_recov = 0; out_enable = 0;
    plant.temp = 200; plant.press = 500; plant.seal = 1; plant.pos_ok = 1; plant.hnd_ok = 1; plant.extra_ticks = 0; plant.fault_ns = 0;
    actuator_apply();
}

static void begin_abort(void) {
    SET_STATE(ST_ABORTING);
    bb_log(BB_ABORT, s_cur, s_reason);
    MsgSendPulse(to_hd, SIGEV_PULSE_PRIO_INHERIT, PC_PARK, 0);
    logf_("CONTROLLED ABORT: parking wafer");
}

static void try_recover(void) {
    long t0 = now_ns();
    pthread_mutex_lock(&rec_mtx); int ok = rec_verify(&rec); pthread_mutex_unlock(&rec_mtx);
    if (!ok) { logf_("checkpoint INVALID -> abort"); begin_abort(); return; }
    if (il_active[0] || il_active[1]) { logf_("interlock still active, waiting"); return; }
    s_recov++;
    SET_STATE(ST_RUN); s_reason = R_NONE;
    bb_log(BB_RECOVER, s_cur, s_recov);
    logf_("RECOVER: checkpoint verified, resume stage %d", s_cur);
    begin_stage(s_cur);
    M.rec_us = (now_ns() - t0) / 1000;                              /* clear -> resumed        */
    M.down_us = (now_ns() - (s_fault_t0 ? s_fault_t0 : t0)) / 1000; /* fault -> resumed */
}

static void enter_protective(int reason, int ilid) {
    long t0 = now_ns();
    s_gen++; arm_timer(dl_timer, 0, 0);          /* cancel deadline timer */
    out_enable = 0; actuator_apply();
    MsgSendPulse(to_hd, SIGEV_PULSE_PRIO_INHERIT, PC_ABORT, 0);
    MsgSendPulse(to_ch, SIGEV_PULSE_PRIO_INHERIT, PC_ABORT, 0);
    SET_STATE(ST_PROTECT); s_reason = reason; s_fault_t0 = t0; s_abort_t0 = t0;
    M.prot_us = (now_ns() - t0) / 1000;
    bb_log(BB_PROTECT, reason, (int)M.prot_us);
    if (plant.fault_ns && (reason == R_CHAMBER || reason == R_HANDLER)) {
        M.f2p_us = (now_ns() - plant.fault_ns) / 1000;
        s_fault_t0 = plant.fault_ns; s_abort_t0 = plant.fault_ns;
    }
    logf_("PROTECTIVE STATE: %s (%ld us)", REASON_NAME[reason], M.prot_us);
    (void)ilid;
    if (reason == R_HANDLER || s_recov >= 1) { begin_abort(); return; }   /* handler fault or 2nd fault -> abort */
    if (reason == R_DEADLINE || reason == R_ENTRY) try_recover();          /* chamber waits for interlock clear */
}

static void begin_stage(int i) {
    int ok = 1;
    if (i > 0 && !rec.done[i - 1]) ok = 0;                     /* predecessor validated */
    if (il_active[0] || il_active[1]) ok = 0;                  /* no active interlock   */
    if (i == 1 && !plant.hnd_ok) ok = 0;
    if ((i == 2 || i == 3) && !plant.seal) ok = 0;
    if (i == 3 && !chamber_ok()) ok = 0;
    if (!ok) { bb_log(BB_ENTRY_FAIL, i, 0); logf_("entry check FAILED for stage %d", i); enter_protective(R_ENTRY, -1); return; }
    s_cur = i; S->sys_stage = i; s_gen++; s_stage_start = now_ns();
    out_enable = 1; actuator_apply();
    arm_timer(dl_timer, DEADLINE_MS[i], 0);
    MsgSendPulse((i == 2 || i == 3) ? to_ch : to_hd, SIGEV_PULSE_PRIO_INHERIT, PC_STAGE_START, i);
    bb_log(BB_STAGE_START, i, DEADLINE_MS[i]);
    logf_("stage %d '%s' started (deadline %d ms)", i, STAGE_NAME[i], DEADLINE_MS[i]);
    if (i == auto_stage) {                                     /* test helper */
        auto_stage = -1; plant.fault_ns = now_ns(); S->inj_temp = 1; plant.temp = 300;
        bb_log(BB_INJECT, 7, i);
        logf_("AUTO-INJECT: temp=300 at start of stage %d", i);
    }
}

static void handle_done(int rcvid, wmsg_t *m) {
    int st = m->a, err = 0;
    if (s_state != ST_RUN || st != s_cur) {
        if (m->src == SRC_WORKER && s_state != ST_RUN) { logf_("late completion ignored (stage %d)", st); }
        else { M.invalid++; bb_log(BB_REJECT, st, s_state); logf_("REJECTED completion of stage %d (current %d, %s)", st, s_cur, STATE_NAME[s_state]); }
        MsgError(rcvid, EBADMSG); return;
    }
    if (st == 2 || st == 3) { if (!chamber_ok()) err = 1; } else if (!handler_ok()) err = 1;   /* exit conditions */
    if (err) { MsgError(rcvid, EBUSY); bb_log(BB_EXIT_FAIL, st, 0); logf_("exit check failed for stage %d", st); enter_protective(st == 2 || st == 3 ? R_CHAMBER : R_HANDLER, -1); return; }
    long ns = now_ns() - s_stage_start, e_us = (ns - (long)STAGE_MS[st] * 1000000L) / 1000;
    M.err_last_us = e_us;
    if (!M.have_err) { M.err_min_us = M.err_max_us = e_us; M.have_err = 1; }
    if (e_us < M.err_min_us) M.err_min_us = e_us;
    if (e_us > M.err_max_us) M.err_max_us = e_us;
    s_gen++; arm_timer(dl_timer, 0, 0);
    pthread_mutex_lock(&rec_mtx); rec.done[st] = 1; rec.ns[st] = ns; rec_commit(); pthread_mutex_unlock(&rec_mtx);
    MsgReply(rcvid, EOK, NULL, 0);
    bb_log(BB_STAGE_DONE, st, (int)e_us);
    logf_("stage %d done (timing error %ld us)", st, e_us);
    if (st == N_STAGES - 1) {
        pthread_mutex_lock(&rec_mtx); rec.final = 1; rec_commit(); pthread_mutex_unlock(&rec_mtx);
        SET_STATE(ST_COMPLETE); out_enable = 0; actuator_apply();
        bb_log(BB_COMPLETE, 0, 0);
        logf_("WAFER PROCESS COMPLETE");
    } else begin_stage(st + 1);
}

static void *supervisor(void *arg) {
    (void)arg; set_prio(60);
    sup_chid = ChannelCreate(0); publish(P_SUP, sup_chid);
    to_self = ConnectAttach(0, 0, sup_chid, _NTO_SIDE_CHANNEL, 0);
    to_hd = connect_peer(P_HD);
    to_ch = connect_peer(P_CH);
    dl_timer = mk_timer(to_self, PC_DEADLINE, 0);
    clear_all();
    for (;;) {
        recv_t r; int rcvid = MsgReceive(sup_chid, &r, sizeof r, NULL);
        if (rcvid < 0) continue;
        if (rcvid == 0) {                                          /* pulses */
            int code = r.pulse.code, val = r.pulse.value.sival_int;
            if (code == PC_DEADLINE && s_state == ST_RUN) {
                M.miss++; bb_log(BB_DEADLINE, s_cur, 0); logf_("DEADLINE MISS stage %d", s_cur); enter_protective(R_DEADLINE, -1);
            } else if (code == PC_INTERLOCK) {
                long t = now_ns();
                if (plant.fault_ns) M.il_resp_us = (t - plant.fault_ns) / 1000;
                logf_("interlock %s raised", val == IL_CHAMBER ? "CHAMBER" : "HANDLER");
                if (s_state == ST_RUN) enter_protective(val == IL_CHAMBER ? R_CHAMBER : R_HANDLER, val);
                else if (s_state == ST_PROTECT && val == IL_HANDLER) { s_reason = R_HANDLER; begin_abort(); }
            } else if (code == PC_CLEAR) {
                logf_("interlock %s cleared", val == IL_CHAMBER ? "CHAMBER" : "HANDLER");
                if (s_state == ST_PROTECT && s_reason == R_CHAMBER && val == IL_CHAMBER) try_recover();
            }
            continue;
        }
        switch (r.m.type) {
        case M_STAGE_DONE: handle_done(rcvid, &r.m); break;
        case M_START:
            if (s_state == ST_IDLE) { MsgReply(rcvid, EOK, NULL, 0); logf_("START"); SET_STATE(ST_RUN); bb_log(BB_START, 0, 0); begin_stage(0); }
            else MsgError(rcvid, EBUSY);
            break;
        case M_RESET:
            s_gen++; arm_timer(dl_timer, 0, 0);                    /* cancel deadline timer */
            MsgSendPulse(to_hd, SIGEV_PULSE_PRIO_INHERIT, PC_ABORT, 0);   /* stop any running stage */
            MsgSendPulse(to_ch, SIGEV_PULSE_PRIO_INHERIT, PC_ABORT, 0);
            clear_all(); bb_log(BB_RESET, 0, 0); logf_("RESET"); MsgReply(rcvid, EOK, NULL, 0); break;
        case M_ABORT_DONE:
            MsgReply(rcvid, EOK, NULL, 0);
            if (s_state == ST_ABORTING) {
                pthread_mutex_lock(&rec_mtx); rec.final = 2; rec_commit(); pthread_mutex_unlock(&rec_mtx);
                M.abort_us = (now_ns() - s_abort_t0) / 1000; SET_STATE(ST_ABORTED);
                bb_log(BB_ABORT_DONE, 0, (int)M.abort_us);
                logf_("ABORT COMPLETE, record sealed (%ld us)", M.abort_us);
            }
            break;
        case M_STATUS: {
            status_t s; memset(&s, 0, sizeof s);
            s.state = s_state; s.stage = s_cur; s.reason = s_reason; s.recoveries = s_recov; s.enable = out_enable;
            s.il[0] = il_active[0]; s.il[1] = il_active[1]; s.m = M;
            pthread_mutex_lock(&rec_mtx); s.rec = rec; pthread_mutex_unlock(&rec_mtx);
            s.rec_ok = rec_verify(&s.rec);
            s.temp = plant.temp; s.press = plant.press; s.seal = plant.seal; s.pos_ok = plant.pos_ok; s.hnd_ok = plant.hnd_ok;
            MsgReply(rcvid, EOK, &s, sizeof s); break; }
        default: MsgError(rcvid, ENOSYS);
        }
    }
    return NULL;
}

int main(void) {
    shm_attach(0);
    supervisor(NULL);
    return 0;
}
