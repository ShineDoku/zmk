#include <zmk/behavior.h>
#include <zmk/macro.h>

static void custom_macro_hold(struct zmk_behavior_binding *binding) {
    while (zmk_behavior_is_held(binding)) {
        zmk_macro_tap_key(J_KEY); // Отправляем клавишу J
        k_msleep(150);              // Задержка 5 миллисекунд
        zmk_macro_tap_key(K_KEY); // Отправляем клавишу K
        k_msleep(150);              // Задержка 5 миллисекунд
    }
}
static void custom_macro_press(struct zmk_behavior_binding *binding) {}

static void custom_macro_release(struct zmk_behavior_binding *binding) {}

ZMK_BEHAVIOR_DEFINE(custom_macro_jk_repeat, custom_macro_press, custom_macro_hold, custom_macro_release);
