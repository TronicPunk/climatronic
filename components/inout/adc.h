/*
   ADC control

   reads ADC signals
*/

/* --- Includes ------------------------------------------------------------ */
#include "esp_types.h"
#include "esp_err.h"


/* --- Defines ------------------------------------------------------------- */

#define ADC_KEY1           (0)      // ADC channel KEY1 signal
#define ADC_KEYX           (1)      // ADC channel KEYX signal
#define ADC_USENS          (2)      // ADC channel Usens voltage
#define ADC_CH_MAX         (3)


/* --- Types --------------------------------------------------------------- */

/* --- Prototypes ---------------------------------------------------------- */

int adc_init(void);
uint32_t adc_read_mv(const uint32_t adc_channel);
