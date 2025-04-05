/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_turbo_key

//#include <zephyr/device.h>
#include <drivers/behavior.h>
//#include <zephyr/logging/log.h>

#include <zmk/hid.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <device.h>

//LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_turbo_config {
    int tap_ms;
    int wait_ms;
    int toggle_term_ms;
    const struct zmk_behavior_binding bindings[];
};

struct behavior_turbo_data {
    uint32_t position;
    bool is_active;
    bool is_pressed;

    int32_t press_time;

    // Timer Data
    bool timer_started;
    bool timer_cancelled;
    bool turbo_decided;
    int64_t release_at;
    struct k_work_delayable release_timer;

    int tap_ms;
    int wait_ms;
    uint16_t start_index;
    uint16_t count;
    struct zmk_behavior_binding bindings[];
};

struct behavior_turbo_key_state {
    struct behavior_macro_trigger_state release_state;

    uint32_t press_bindings_count;
};

static int behavior_turbo_key_init(const struct device *dev) {  
    const struct behavior_turbo_key_config *cfg = dev->config;
    struct behavior_turbo_key_state *state = dev->data;
    state->press_bindings_count = cfg->count;
    state->release_state.start_index = cfg->count;
    state->release_state.count = 0;

    LOG_DBG("Precalculate initial release state:");
    for (int i = 0; i < cfg->count; i++) {
        if (handle_control_binding(&state->release_state, &cfg->bindings[i])) {
            // Updated state used for initial state on release.
        } else if (IS_PAUSE(cfg->bindings[i].behavior_dev)) {
            state->release_state.start_index = i + 1;
            state->release_state.count = cfg->count - state->release_state.start_index;
            state->press_bindings_count = i;
            LOG_DBG("Release will resume at %d", state->release_state.start_index);
            break;
        } else {
            // Ignore regular invokable bindings
        }
    }

    return 0; };

static int stop_timer(struct behavior_turbo_data *data) {
    int timer_cancel_result = k_work_cancel_delayable(&data->release_timer);
    if (timer_cancel_result == -EINPROGRESS) {
        // too late to cancel, we'll let the timer handler clear up.
        data->timer_cancelled = true;
    }
    return timer_cancel_result;
}

static void clear_turbo(struct behavior_turbo_data *data) {
    //LOG_DBG("Turbo deactivated");
    data->is_active = false;
    stop_timer(data);
}

static void reset_timer(struct behavior_turbo_data *data, struct zmk_behavior_binding_event event) {
    data->release_at = event.timestamp + data->wait_ms;
    int32_t ms_left = data->release_at - k_uptime_get();
    if (ms_left > 0) {
        k_work_schedule(&data->release_timer, K_MSEC(ms_left));
        //LOG_DBG("Successfully reset turbo timer at position %d", data->position);
    }
}

static void behavior_turbo_timer_handler(struct k_work *item) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(item);
    struct behavior_turbo_data *data =
        CONTAINER_OF(d_work, struct behavior_turbo_data, release_timer);
    if (!data->is_active) {
        return;
    }
    if (data->timer_cancelled) {
        return;
    }
    //LOG_DBG("Turbo timer reached.");
    struct zmk_behavior_binding_event event = {.position = data->position,
                                               .timestamp = k_uptime_get()};
     for (int i = data.start_index; i < data.start_index + data.count; i++) {
        if (!handle_control_binding(&data, &bindings[i])) 
        {
    zmk_behavior_queue_add(event.position, data->bindings[i], true, data->tap_ms);
    zmk_behavior_queue_add(event.position, data->bindings[i], false, 0);
        }
     }
    reset_timer(data, event);
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = device_get_binding(binding->behavior_dev);
    const struct behavior_turbo_config *cfg = dev->config;
    struct behavior_turbo_data *data = dev->data;

    if (!data->is_active) {
        data->is_active = true;

        //LOG_DBG("%d started new turbo", event.position);
        data->press_time = k_uptime_get();
        k_work_init_delayable(&data->release_timer, behavior_turbo_timer_handler);
         for (int i = state.start_index; i < state.start_index + state.count; i++) {
        if (!handle_control_binding(&state, &bindings[i])) 
        {
        zmk_behavior_queue_add(event.position, cfg->bindings[i], true, cfg->tap_ms);
        zmk_behavior_queue_add(event.position, cfg->bindings[i], false, 0);
        }
         }
        reset_timer(data, event);
    } else {
        clear_turbo(data);
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct device *dev = device_get_binding(binding->behavior_dev);
    const struct behavior_turbo_config *cfg = dev->config;
    struct behavior_turbo_data *data = dev->data;

    if (data->is_active) {
        data->is_pressed = false;
        int32_t elapsedTime = k_uptime_get() - data->press_time;
        //LOG_DBG("turbo elapsed time: %d", elapsedTime);
        if (elapsedTime > cfg->toggle_term_ms) {
            clear_turbo(data);
        }
    }
    return 0;
}

static const struct behavior_driver_api behavior_turbo_key_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define _TRANSFORM_ENTRY(idx, node)                                                                \
    {                                                                                              \
        .behavior_dev = DT_LABEL(DT_INST_PHANDLE_BY_IDX(node, bindings, idx)),                     \
        .param1 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param1), (0),       \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param1))),                  \
        .param2 = COND_CODE_0(DT_INST_PHA_HAS_CELL_AT_IDX(node, bindings, idx, param2), (0),       \
                              (DT_INST_PHA_BY_IDX(node, bindings, idx, param2))),                  \
    }

#define TURBO_INST(n)                                                                              \
    static struct behavior_turbo_config behavior_turbo_config_##n = {                              \
        .tap_ms = DT_INST_PROP(n, tap_ms),                                                         \
        .wait_ms = DT_INST_PROP(n, wait_ms),                                                       \
        .toggle_term_ms = DT_INST_PROP(n, toggle_term_ms),                                         \
        .bindings = TRANSFORMED_BEHAVIORS(n)};                                                        \
    static struct behavior_turbo_data behavior_turbo_data_##n = {                                  \
        .tap_ms = DT_INST_PROP(n, tap_ms),                                                         \
        .wait_ms = DT_INST_PROP(n, wait_ms),                                                       \
        .bindings = TRANSFORMED_BEHAVIORS(n)};                                                        \
    DEVICE_DT_INST_DEFINE(n, behavior_turbo_key_init, NULL, &behavior_turbo_data_##n,              \
                          &behavior_turbo_config_##n, APPLICATION,                                 \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_turbo_key_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TURBO_INST)
