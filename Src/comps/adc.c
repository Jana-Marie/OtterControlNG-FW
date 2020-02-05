#include "commands.h"
#include "common.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "stm32g4xx_hal.h"

HAL_COMP(adc);

HAL_PIN(adc1);
HAL_PIN(adc2);

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct adc_ctx_t * ctx = (struct adc_ctx_t *)ctx_ptr;
  struct adc_pin_ctx_t *pins = (struct adc_pin_ctx_t *)pin_ptr;
  PIN(adc1) = ADC1->DR;// / 4096.0 * 3.3 * 104.3 / 4.3;//dclink
  PIN(adc2) = ADC2->DR;
}

hal_comp_t adc_comp_struct = {
    .name      = "adc",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct adc_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
