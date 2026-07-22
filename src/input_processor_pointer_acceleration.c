/* SPDX-License-Identifier: MIT */

#define DT_DRV_COMPAT haneta007_input_processor_pointer_acceleration

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <drivers/input_processor.h>

struct pointer_acceleration_config {
    uint8_t type;
    size_t codes_len;
    uint16_t slow_threshold;
    uint16_t fast_threshold;
    uint8_t slow_multiplier;
    uint8_t slow_divisor;
    uint8_t fast_multiplier;
    uint8_t fast_divisor;
    uint16_t codes[];
};

static bool handles_code(const struct pointer_acceleration_config *config, uint16_t code) {
    for (size_t i = 0; i < config->codes_len; i++) {
        if (config->codes[i] == code) {
            return true;
        }
    }

    return false;
}

static void clear_remainder(struct zmk_input_processor_state *state) {
    if (state && state->remainder) {
        *state->remainder = 0;
    }
}

static int scale_event(struct input_event *event, uint8_t multiplier, uint8_t divisor,
                       struct zmk_input_processor_state *state) {
    int32_t multiplied = (int32_t)event->value * multiplier;

    if (state && state->remainder) {
        multiplied += *state->remainder;
    }

    int32_t scaled = multiplied / divisor;
    int32_t clamped = CLAMP(scaled, INT16_MIN, INT16_MAX);

    if (state && state->remainder) {
        *state->remainder =
            (scaled == clamped) ? (int16_t)(multiplied - (scaled * divisor)) : 0;
    }

    event->value = clamped;
    return ZMK_INPUT_PROC_CONTINUE;
}

static int pointer_acceleration_handle_event(const struct device *dev, struct input_event *event,
                                             uint32_t param1, uint32_t param2,
                                             struct zmk_input_processor_state *state) {
    const struct pointer_acceleration_config *config = dev->config;

    ARG_UNUSED(param1);
    ARG_UNUSED(param2);

    if (event->type != config->type || !handles_code(config, event->code)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int64_t magnitude = event->value;
    if (magnitude < 0) {
        magnitude = -magnitude;
    }

    if (magnitude <= config->slow_threshold) {
        return scale_event(event, config->slow_multiplier, config->slow_divisor, state);
    }

    clear_remainder(state);

    if (magnitude >= config->fast_threshold) {
        return scale_event(event, config->fast_multiplier, config->fast_divisor, state);
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api pointer_acceleration_driver_api = {
    .handle_event = pointer_acceleration_handle_event,
};

#define POINTER_ACCELERATION_INST(n)                                                               \
    BUILD_ASSERT(DT_INST_PROP(n, slow_threshold) < DT_INST_PROP(n, fast_threshold),               \
                 "slow-threshold must be less than fast-threshold");                              \
    BUILD_ASSERT(DT_INST_PROP(n, slow_multiplier) > 0 && DT_INST_PROP(n, slow_multiplier) <= 16,  \
                 "slow-multiplier must be between 1 and 16");                                    \
    BUILD_ASSERT(DT_INST_PROP(n, slow_divisor) > 0 && DT_INST_PROP(n, slow_divisor) <= 16,        \
                 "slow-divisor must be between 1 and 16");                                       \
    BUILD_ASSERT(DT_INST_PROP(n, fast_multiplier) > 0 && DT_INST_PROP(n, fast_multiplier) <= 16,  \
                 "fast-multiplier must be between 1 and 16");                                    \
    BUILD_ASSERT(DT_INST_PROP(n, fast_divisor) > 0 && DT_INST_PROP(n, fast_divisor) <= 16,        \
                 "fast-divisor must be between 1 and 16");                                       \
    static const struct pointer_acceleration_config pointer_acceleration_config_##n = {            \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .slow_threshold = DT_INST_PROP(n, slow_threshold),                                         \
        .fast_threshold = DT_INST_PROP(n, fast_threshold),                                         \
        .slow_multiplier = DT_INST_PROP(n, slow_multiplier),                                       \
        .slow_divisor = DT_INST_PROP(n, slow_divisor),                                             \
        .fast_multiplier = DT_INST_PROP(n, fast_multiplier),                                       \
        .fast_divisor = DT_INST_PROP(n, fast_divisor),                                             \
        .codes = DT_INST_PROP(n, codes),                                                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, &pointer_acceleration_config_##n, POST_KERNEL,      \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &pointer_acceleration_driver_api);

DT_INST_FOREACH_STATUS_OKAY(POINTER_ACCELERATION_INST)
