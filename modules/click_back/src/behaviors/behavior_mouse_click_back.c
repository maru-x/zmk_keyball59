/*
 * behavior,mouse-click-back  —  press: BTN_DOWN / release: BTN_UP + delayed layer_to
 *
 * © 2025 YourName  (MIT)
 */
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <zmk/hid.h>
#include <zmk/events/mouse_button_changed.h>
#include <zmk/layers.h>

/* ───────────── Devicetree properties ───────────── */
#define PROP_TIMEOUT_MS(node_id) DT_PROP_OR(node_id, timeout_ms, 500)
#define PROP_RETURN_LAYER(node_id) DT_PROP_OR(node_id, return_layer, 0)

/* ───────────── Per-instance context ───────────── */
struct mcb_ctx {
    struct k_work_delayable back_work;
    uint16_t timeout_ms;
    int8_t   return_layer;
    uint8_t  button_mask;
};

/* マクロでインスタンス展開 */
#define DEFINE_MCB_INST(inst)                                                     \
    static struct mcb_ctx mcb_ctx_##inst;                                         \
                                                                                  \
    static void mcb_back_work_##inst(struct k_work *work) {                       \
        ARG_UNUSED(work);                                                         \
        zmk_layer_to(mcb_ctx_##inst.return_layer);                                \
    }                                                                             \
                                                                                  \
    static int mcb_pressed_##inst(struct zmk_behavior_binding *binding,           \
                                  struct zmk_behavior_binding_event event) {      \
        zmk_hid_mouse_button_press(mcb_ctx_##inst.button_mask);                   \
        return ZMK_BEHAVIOR_OPAQUE;                                               \
    }                                                                             \
                                                                                  \
    static int mcb_released_##inst(struct zmk_behavior_binding *binding,          \
                                   struct zmk_behavior_binding_event event) {     \
        zmk_hid_mouse_button_release(mcb_ctx_##inst.button_mask);                 \
        k_work_reschedule(&mcb_ctx_##inst.back_work,                              \
                          K_MSEC(mcb_ctx_##inst.timeout_ms));                     \
        return ZMK_BEHAVIOR_OPAQUE;                                               \
    }                                                                             \
                                                                                  \
    static int mcb_init_##inst(const struct device *dev) {                        \
        ARG_UNUSED(dev);                                                          \
        mcb_ctx_##inst.timeout_ms  = PROP_TIMEOUT_MS(DT_DRV_INST(inst));          \
        mcb_ctx_##inst.return_layer = PROP_RETURN_LAYER(DT_DRV_INST(inst));       \
        mcb_ctx_##inst.button_mask = BIT(binding->param1); /* placeholder */      \
        k_work_init_delayable(&mcb_ctx_##inst.back_work, mcb_back_work_##inst);   \
        return 0;                                                                 \
    }                                                                             \
    SYS_INIT(mcb_init_##inst, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);     \
                                                                                  \
    ZMK_BEHAVIOR_DEFINITION(mouse_click_back_##inst,                              \
        mcb_pressed_##inst, mcb_released_##inst);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_MCB_INST)
