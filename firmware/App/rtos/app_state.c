//
// shared state - implementation
//
// - one static snapshot behind one mutex; every function here is lock, copy,
//   unlock and nothing else, so no task ever waits long on another
// - both mutexes use priority inheritance: if a low task holds one and the
//   imu task wants it, the low task runs at the imu task's priority until it
//   lets go, so a middle task cannot keep the imu task waiting
//

#include "app_state.h"
#include <string.h>
#include "main.h"   // Error_Handler

osMutexId_t        app_i2c_mutex;
osMessageQueueId_t app_fix_queue;
osMessageQueueId_t app_env_queue;

static osMutexId_t state_mutex;

// file scope so the debugger can live-watch it
static app_state_t state;

static const osMutexAttr_t state_mutex_attr = {
    .name = "state",
    .attr_bits = osMutexPrioInherit,
};

static const osMutexAttr_t i2c_mutex_attr = {
    .name = "i2c",
    .attr_bits = osMutexPrioInherit,
};

static void lock(void) {
    osMutexAcquire(state_mutex, osWaitForever);
}

static void unlock(void) {
    osMutexRelease(state_mutex);
}

void app_state_init(void) {
    memset(&state, 0, sizeof(state));
    state_mutex = osMutexNew(&state_mutex_attr);
    app_i2c_mutex = osMutexNew(&i2c_mutex_attr);
    app_fix_queue = osMessageQueueNew(APP_FIX_QUEUE_LEN, sizeof(gps_fix_t), NULL);
    app_env_queue = osMessageQueueNew(APP_ENV_QUEUE_LEN, sizeof(app_env_msg_t), NULL);

    // these come out of a 40 KB heap that is otherwise empty at this point -
    // a NULL here is a configuration mistake, not bad luck
    if ((state_mutex == NULL) || (app_i2c_mutex == NULL) ||
        (app_fix_queue == NULL) || (app_env_queue == NULL)) {
        Error_Handler();
    }
}

void app_state_put_boot(const app_boot_t *v) {
    lock();
    state.boot = *v;
    unlock();
}

void app_state_put_imu(const app_imu_t *v) {
    lock();
    state.imu = *v;
    unlock();
}

void app_state_put_gps(const app_gps_t *v) {
    lock();
    state.gps = *v;
    unlock();
}

void app_state_put_env(const app_env_t *v) {
    lock();
    state.env = *v;
    unlock();
}

void app_state_put_disp(const app_disp_t *v) {
    lock();
    state.disp = *v;
    unlock();
}

void app_state_put_log(const app_log_t *v) {
    lock();
    state.log = *v;
    unlock();
}

void app_state_note_gap(uint32_t ms) {
    lock();
    if (ms > state.second.imu_gap_ms) {
        state.second.imu_gap_ms = ms;
    }
    unlock();
}

void app_state_note_wake(app_task_t t, uint32_t us) {
    lock();
    if (us > state.second.wake_us[t]) {
        state.second.wake_us[t] = us;
    }
    unlock();
}

void app_state_get(app_state_t *out) {
    lock();
    *out = state;
    unlock();
}

void app_state_take(app_state_t *out) {
    lock();
    *out = state;
    memset(&state.second, 0, sizeof(state.second));
    unlock();
}

void app_state_get_fix(gps_fix_t *out) {
    lock();
    *out = state.gps.fix;
    unlock();
}
