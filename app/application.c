#include "application.h"
//#include "jsmn.h"

// Find defailed API description at https://sdk.hardwario.com/
/*
 * Drives directly LED strips using PWM. It has internal timer to turn lights off
 *
 * TODO:
 *  - timeout topic
 *  - transitions topic
 *  - color topic
 *  - test overheat protection
 */

/*
 * MQTT topics without "node/{id}/" prefix (https://developers.hardwario.com/interfaces/mqtt-topics):
 *  Custom consumed topics:
 *      led-pwm/{0-N}/trigger/set
 *          payload:
 *              N - increases trigger count by specified number and refreshess off timer (N=0 just refreshes the timer)
 *          - triggers lighting up with automatic turning off after configured timeout, multiple triggers while lit on does increase off timeout
 *      led-pwm/-/blink/set             int mode
 *          - blinks for predefined interval
 *      led-pwm/{0-N}/brightness/set    float brightness [%]
 *      led-pwm/{0-N}/color/set
 *      led-pwm/{0-N}/timeout/set
 *          {"base":90, "max":1800}
 *          - timeout in seconds for shutting off triggered light. Multiple triggers does increase timeout up to "max"
 *      led-pwm/{0-N}/transitions/set
 *          {"on": 1.5, "off":10, "change":1.0}
 *          - sets up transition times between states in seconds
 *
 *  Standard published topics:
 *      thermometer/0:1/temperature     float   degrees_celsius
 *
 *  Custom published topics:
 *      TODO (on/off events including total length and trigger count, ...)
 *      led-pwm/{0-N}/trigger/state     int trigger_count
 *      led-pwm/-/brightness/state      float brightness [%]
 *      led-pwm/-/event/on              null
 *      led-pwm/-/event/off
 */

// Debug
// #define EEPROM_SIGNATURE        0x58494d31

// Release
// #define EEPROM_SIGNATURE        0x58494d30
#define EEPROM_SIGNATURE        0x58494d32

#define TEMPERATURE_UPDATE_INTERVAL (10 * 1000)
#define TEMPERATURE_TAG_PUB_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define TEMPERATURE_TAG_PUB_VALUE_CHANGE 0.2f

#define LED_STEPS_PER_SECOND    30
#define LED_STEP_TIME           (1000 / LED_STEPS_PER_SECOND)

#define MAX_CHANNELS            9
#define MAX_LEDS                9
#define PWM_BITS                12
#define PWM_MAX                 ((1 << PWM_BITS) - 1)

// topic callback headers
void led_trigger_set(uint64_t *id, const char *topic, void *value, void *param);
void led_brightness_set(uint64_t *id, const char *topic, void *value, void *param);
void led_config_set(uint64_t *id, const char *topic, void *value, void *param);

// scheduler callback headers
static void pwm_timer(void * param);
static void led_timer_off(void * param);

/*
typedef enum
{
BC_RADIO_SUB_PT_BOOL = 0,
BC_RADIO_SUB_PT_INT = 1,
BC_RADIO_SUB_PT_FLOAT = 2,
BC_RADIO_SUB_PT_STRING = 3,
BC_RADIO_SUB_PT_NULL = 4,

} bc_radio_sub_pt_t;
 */

static char mqtt_bufer[256];
// subscribe table, format: topic, expect payload type, callback, user param
static const bc_radio_sub_t mqtt_subs[] = {
    // state/set
//     {"led-pwm/-/config/set", BC_RADIO_SUB_PT_STRING, led_config_set, NULL },
    {"led-pwm/-/config/set", BC_RADIO_SUB_PT_STRING, led_config_set, mqtt_bufer },
    {"led-pwm/-/trigger/set", BC_RADIO_SUB_PT_INT, led_trigger_set, NULL },
    {"led-pwm/-/brightness/set", BC_RADIO_SUB_PT_FLOAT, led_brightness_set, NULL },
    {"led-pwm/0/brightness/set", BC_RADIO_SUB_PT_FLOAT, led_brightness_set, NULL },
    {"led-pwm/1/brightness/set", BC_RADIO_SUB_PT_FLOAT, led_brightness_set, NULL },
    {"led-pwm/2/brightness/set", BC_RADIO_SUB_PT_FLOAT, led_brightness_set, NULL },
};


static const int pwm_channel[MAX_CHANNELS] = {
    BC_PWM_P0,
    BC_PWM_P1,
    BC_PWM_P2,
    BC_PWM_P3,
    BC_PWM_P6,
    BC_PWM_P7,
    BC_PWM_P8,
    BC_PWM_P12,
    BC_PWM_P14
};

typedef enum {
    LED_OFF,
    LED_ON
} led_state_t;

// color channels are in order: W, RGB, RGBW
typedef struct {
    int8_t      channel[4];
    uint16_t    color[4];
    uint8_t     channels;
    bc_tick_t   timeout_base;
    bc_tick_t   timeout_max;
    bc_tick_t   timeout_step;
    bc_tick_t   transition_on;
    bc_tick_t   transition_off;
    bc_tick_t   transition_change;
    float       brightness;
} led_config_t;

typedef struct {
    led_config_t    led_config[MAX_LEDS];
    uint8_t         leds;
    float           alert_temp;
    float           max_temp;
} module_config_t;

// development values
// #define TIMEOUT_BASE     5000
// #define TIMEOUT_MAX     30000
// #define TIMEOUT_STEP     1000

// release values
#define TIMEOUT_BASE      90000
// #define TIMEOUT_MAX     1800000
#define TIMEOUT_MAX      300000
#define TIMEOUT_STEP       5000

#define FADE_ON         1500
#define FADE_OFF        5000
#define FADE_CHANGE      500

#define TEMPERATURE_ALERT   60.0
#define TEMPERATURE_MAX     70.0

static module_config_t init_config = {
    {
        { {0, 0, 0, 0}, {PWM_MAX, PWM_MAX, PWM_MAX, 0}, 1, TIMEOUT_BASE, TIMEOUT_MAX, TIMEOUT_STEP, FADE_ON, FADE_OFF, FADE_CHANGE, 1.0 },
        { {1, 0, 0, 0}, {PWM_MAX, PWM_MAX, PWM_MAX, 0}, 1, TIMEOUT_BASE, TIMEOUT_MAX, TIMEOUT_STEP, FADE_ON, FADE_OFF, FADE_CHANGE, 1.0 },
        { {2, 0, 0, 0}, {PWM_MAX, PWM_MAX, PWM_MAX, 0}, 1, TIMEOUT_BASE, TIMEOUT_MAX, TIMEOUT_STEP, FADE_ON, FADE_OFF, FADE_CHANGE, 1.0 },
        { {3, 0, 0, 0}, {PWM_MAX, PWM_MAX, PWM_MAX, 0}, 1, TIMEOUT_BASE, TIMEOUT_MAX, TIMEOUT_STEP, FADE_ON, FADE_OFF, FADE_CHANGE, 1.0 },
        { {4, 0, 0, 0}, {PWM_MAX, PWM_MAX, PWM_MAX, 0}, 1, TIMEOUT_BASE, TIMEOUT_MAX, TIMEOUT_STEP, FADE_ON, FADE_OFF, FADE_CHANGE, 1.0 },
        { {5, 0, 0, 0}, {PWM_MAX, PWM_MAX, PWM_MAX, 0}, 1, TIMEOUT_BASE, TIMEOUT_MAX, TIMEOUT_STEP, FADE_ON, FADE_OFF, FADE_CHANGE, 1.0 },
        { {6, 0, 0, 0}, {PWM_MAX, PWM_MAX, PWM_MAX, 0}, 1, TIMEOUT_BASE, TIMEOUT_MAX, TIMEOUT_STEP, FADE_ON, FADE_OFF, FADE_CHANGE, 1.0 },
        { {7, 0, 0, 0}, {PWM_MAX, PWM_MAX, PWM_MAX, 0}, 1, TIMEOUT_BASE, TIMEOUT_MAX, TIMEOUT_STEP, FADE_ON, FADE_OFF, FADE_CHANGE, 1.0 },
    },
    2,
    TEMPERATURE_ALERT,
    TEMPERATURE_MAX
};

static module_config_t config;


typedef struct {
    led_state_t             state;
    bc_tick_t               on_time;
    bc_tick_t               off_time;
    bc_scheduler_task_id_t  off_scheduler_id;
    bc_scheduler_task_id_t  transition_scheduler_id;
    uint32_t                trigger_count;
    float                   pwm_current[4];
//     uint16_t                pwm_current[4];
    uint16_t                pwm_target[4];
    uint16_t                steps;
} led_status_t;

static led_status_t  led_status[MAX_LEDS] = { {LED_OFF, 0, 0, 0, 0, 0, {0, 0, 0, 0}, {0, 0, 0, 0}, 0} };

// helper array of led ids, where index=value. It is used because we need pointer to led_id in callback routines
static uint32_t led_ids[MAX_LEDS];

// Overheat flag
static bool overheat = false;

// onboard LED instance
static bc_led_t led;

// Button instance
static bc_button_t button;

// Thermometer instance
typedef struct
{
//     uint8_t number;
    float value;
    bc_tick_t next_pub;
} temperature_event_param_t;

static bc_tmp112_t tmp112;
static temperature_event_param_t temperature_event_param = { .next_pub = 0, .value = NAN };

static uint64_t my_id;

// ==== LED PWM handling ====

// computes off time from led_status
static inline uint32_t off_time_eval(int led_id)
{
    uint32_t tim = config.led_config[led_id].timeout_base + config.led_config[led_id].timeout_step * led_status[led_id].trigger_count;
    if (tim > config.led_config[led_id].timeout_max)
        tim = config.led_config[led_id].timeout_max;
    return tim;
}

// led_status preparation functions
// turns led on and refreshes off timer
static void led_state_on(int led_id) {
    // Can't turn on the LEDs in case of overheating
    if (overheat) {
        bc_log_warning("can't turn on, overheating detected!");
        bc_radio_pub_string("led-pwm/-/trigger/error", "overheat");
        return;
    }

    if (led_status[led_id].state == LED_OFF) {
        for (int c = 0; c < config.led_config[led_id].channels; c++) {
            led_status[led_id].pwm_target[c] = (uint16_t)((float)config.led_config[led_id].color[c] * config.led_config[led_id].brightness);
        }
        led_status[led_id].steps = config.led_config[led_id].transition_on / LED_STEP_TIME;
        led_status[led_id].state = LED_ON;
        led_status[led_id].on_time = bc_scheduler_get_spin_tick();
        bc_log_debug("LED[%i]: turn on, steps = %i, target color = #%03x %03x %03x %03x", led_id, led_status[led_id].steps, led_status[led_id].pwm_target[0], led_status[led_id].pwm_target[1], led_status[led_id].pwm_target[2], led_status[led_id].pwm_target[3]);
    }
    led_status[led_id].off_time = off_time_eval(led_id);

    // setup off scheduler callback
    if (led_status[led_id].off_scheduler_id) {
        bc_scheduler_plan_from_now(led_status[led_id].off_scheduler_id, led_status[led_id].off_time);
    } else {
        led_status[led_id].off_scheduler_id = bc_scheduler_register(led_timer_off, &led_ids[led_id], bc_tick_get() + led_status[led_id].off_time);
    }

    // setup transition scheduller callback
    if (led_status[led_id].transition_scheduler_id) {
        bc_scheduler_plan_from_now(led_status[led_id].transition_scheduler_id, 0);
    } else {
        led_status[led_id].transition_scheduler_id = bc_scheduler_register(pwm_timer, &led_ids[led_id], bc_tick_get());
    }
}

static void led_state_off(int led_id) {
    char pub_json[60];
    sprintf(pub_json, "{\"onTime\":%lld,\"triggerCount\":%ld}", (bc_scheduler_get_spin_tick() - led_status[led_id].on_time) / 1000, led_status[led_id].trigger_count);
    bc_radio_pub_string("led-pwm/-/event/off", pub_json);
    for (int c = 0; c < config.led_config[led_id].channels; c++) {
        led_status[led_id].pwm_target[c] = 0;
    }
    led_status[led_id].steps = config.led_config[led_id].transition_off / LED_STEP_TIME;
    led_status[led_id].state = LED_OFF;
    led_status[led_id].trigger_count = 0;
    led_status[led_id].on_time = 0;

    // setup transition scheduller callback
    if (led_status[led_id].transition_scheduler_id) {
        bc_scheduler_plan_from_now(led_status[led_id].transition_scheduler_id, 0);
    } else {
        led_status[led_id].transition_scheduler_id = bc_scheduler_register(pwm_timer, &led_ids[led_id], bc_tick_get());
    }

    bc_log_debug("LED[%i]: turn off, steps = %i, target color = #%03x %03x %03x %03x", led_id, led_status[led_id].steps, led_status[led_id].pwm_target[0], led_status[led_id].pwm_target[1], led_status[led_id].pwm_target[2], led_status[led_id].pwm_target[3]);
}

static void led_state_change(int led_id) {
    if (led_status[led_id].state == LED_OFF)
        return;

    led_status[led_id].steps = config.led_config[led_id].transition_change / LED_STEP_TIME;

    // setup transition scheduller callback
    if (led_status[led_id].transition_scheduler_id) {
        bc_scheduler_plan_from_now(led_status[led_id].transition_scheduler_id, 0);
    } else {
        led_status[led_id].transition_scheduler_id = bc_scheduler_register(pwm_timer, &led_ids[led_id], bc_tick_get());
    }

    bc_log_debug("LED[%i]: change color, steps = %i, target color = #%02x%02x%02x%02x", led_id, led_status[led_id].steps, led_status[led_id].pwm_target[0], led_status[led_id].pwm_target[1], led_status[led_id].pwm_target[2], led_status[led_id].pwm_target[3]);
}

// sets all channels for led_id to desired current value
static void pwm_set(int led_id) {
    for (int c = 0; c < config.led_config[led_id].channels; c++) {
        bc_pwm_set(pwm_channel[config.led_config[led_id].channel[c]], (uint16_t)led_status[led_id].pwm_current[c]);
    }
}

// bc scheduler callback function to handle transitions between states
static void pwm_timer(void * param)
{
    uint32_t led_id = *(uint32_t *)param;

    if (led_status[led_id].steps > 0) {

        // transition in progress - compute next value
        for (int c = 0; c < config.led_config[led_id].channels; c++) {
            led_status[led_id].pwm_current[c] += (led_status[led_id].pwm_target[c] - led_status[led_id].pwm_current[c]) / led_status[led_id].steps;
        }
        led_status[led_id].steps--;
        bc_scheduler_plan_current_from_now(LED_STEP_TIME);
    } else {

        // transition finished - just place target state to current
        for (int c = 0; c < config.led_config[led_id].channels; c++) {
            led_status[led_id].pwm_current[c] = led_status[led_id].pwm_target[c];
        }
        led_status[led_id].steps = 0;
        bc_log_debug("LED[%i]: finished transition, color = #%03x %03x %03x %03x", (int)led_id, (uint16_t)led_status[led_id].pwm_current[0], (uint16_t)led_status[led_id].pwm_current[1], (uint16_t)led_status[led_id].pwm_current[2], (uint16_t)led_status[led_id].pwm_current[3]);
    }

    // sets pwm outputs to new computed state
    pwm_set(led_id);
}

static void led_timer_off(void * param)
{
    uint32_t led_id = *(uint32_t *)param;

    led_state_off(led_id);
}


// ==== topic callbacks ===
void led_trigger_set(uint64_t *id, const char *topic, void *value, void *param)
{
    // TODO - get led_id from topic
    int trigger_count = *(int *)value;
    uint32_t max_trigger = 0;

    // for now, we apply trigger count to all leds
    for (int l = 0; l < config.leds; l++) {
        led_status[l].trigger_count += trigger_count;
        if (max_trigger < led_status[l].trigger_count)
            max_trigger = led_status[l].trigger_count;

        // refresh the off timer
        led_state_on(l);
    }
    bc_log_debug("Event: increased trigger count by %i", trigger_count);
    bc_radio_pub_uint32("led-pwm/-/trigger/state", &max_trigger);
}


void _set_brightness(int l, float brightness) {
    config.led_config[l].brightness = brightness;
    if (led_status[l].state == LED_ON) {
        for (int c = 0; c < config.led_config[l].channels; c++) {
            led_status[l].pwm_target[c] = (uint16_t)((float)config.led_config[l].color[c] * config.led_config[l].brightness);
        }
        led_state_change(l);
    }
    bc_log_debug("Event: set brightness of LED[%i] to %f", l, brightness);
}

void led_brightness_set(uint64_t *id, const char *topic, void *value, void *param)
{
    static char topic_buff[50] = "led-pwm/-/brightness/state";

    // convert brightness value as we internally use range [0..1] but payload is in percent
    float brightness = *(float *)value / 100.0;

    // some sanity check (in general, we can allow brightness > 100% which may compensate dark base color)
    if (brightness < 0)
        brightness = 0;

    // get led_id from topic - evil hack! - TODO some nice reliable solution
    int led = (uint8_t)(topic[8] - '0');

    if (led < MAX_LEDS) {
        _set_brightness(led, brightness);
        topic_buff[8] = topic[8];
        bc_radio_pub_float(topic_buff, &brightness);
    } else {
        for (int l = 0; l < config.leds; l++)
            _set_brightness(l, brightness);
        float tmp_brightness = brightness;
        bc_radio_pub_float("led-pwm/-/brightness/state", &tmp_brightness);
    }
    bc_config_save();
}


// static inline int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
//   if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
//       strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
//     return 1;
//   }
//   return 0;
// }

/*
typedef struct {
    int8_t      channel[4];
    uint8_t     color[4];
    uint8_t     channels;
    bc_tick_t   timeout_base;
    bc_tick_t   timeout_max;
    bc_tick_t   timeout_step;
    bc_tick_t   transition_on;
    bc_tick_t   transition_off;
    bc_tick_t   transition_change;
    float       brightness;
} led_config_t;
 */

#define SET_CFG(l, var)    if (var > 0) { config.led_config[l].var = var; bc_log_debug("    " #var " = %lld", var) ; }
void led_config_set(uint64_t *id, const char *topic, void *value, void *param) {
/*    static char topic_buff[50] = "led-pwm/-/config/state";
    jsmn_parser parser;
    jsmntok_t tokens[16];
    char * js = (char *)value;

    bool retval = true;

    bc_log_debug("config: led_config_set()");

    // get led_id from topic - evil hack! - TODO some nice reliable solution
    int led = (uint8_t)(topic[8] - '0');

    jsmn_init(&parser);
    int r = jsmn_parse(&parser, js, strlen(js), tokens, sizeof(tokens) / sizeof(tokens[0]));
    bc_log_debug("config: parsing configuration data, records=%i, data='%s'", led, js);

    bc_tick_t   timeout_base = 0;
    bc_tick_t   timeout_max = 0;
    bc_tick_t   timeout_step = 0;
    bc_tick_t   transition_on = 0;
    bc_tick_t   transition_off = 0;
    bc_tick_t   transition_change = 0;

    if (r > 0) {
        for (int i = 0; i < r; i++) {
            if (jsoneq(js, tokens+i, "toBase")) {
                timeout_base = atol(js+tokens[++i].start);
            } else if (jsoneq(js, tokens+i, "toMax")) {
                timeout_max = atol(js+tokens[++i].start);
            } else if (jsoneq(js, tokens+i, "toStep")) {
                timeout_step = atol(js+tokens[++i].start);
            } else if (jsoneq(js, tokens+i, "trOn")) {
                transition_on = atol(js+tokens[++i].start);
            } else if (jsoneq(js, tokens+i, "trOff")) {
                transition_off = atol(js+tokens[++i].start);
            } else if (jsoneq(js, tokens+i, "trChng")) {
                transition_change = atol(js+tokens[++i].start);
            }
        }
        if (led < MAX_LEDS) {
            SET_CFG(led, timeout_base);
            SET_CFG(led, timeout_max);
            SET_CFG(led, timeout_step);
            SET_CFG(led, transition_on);
            SET_CFG(led, transition_off);
            SET_CFG(led, transition_change);
            topic_buff[8] = topic[8];
            bc_radio_pub_bool(topic_buff, &retval);
            bc_log_debug("config: Configuration for LED #%i is set.", led);
        } else {
            for (int l = 0; l < config.leds; l++) {
                SET_CFG(l, timeout_base);
                SET_CFG(l, timeout_max);
                SET_CFG(l, timeout_step);
                SET_CFG(l, transition_on);
                SET_CFG(l, transition_off);
                SET_CFG(l, transition_change);
            }
            bc_radio_pub_bool(topic_buff, &retval);
            bc_log_debug("config: Configuration for all LEDs is set.");
        }
        bc_config_save();
    } else {
        retval = false;
        bc_radio_pub_bool(topic_buff, &retval);
        bc_log_error("config: Error parsing configuration data, error=%i, data='%s'", led, js);
    }*/
}




// Button event callback
void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    // Log button event
    bc_log_debug("APP: Button event: %i", event);

    // Check event source
    if (event == BC_BUTTON_EVENT_PRESS)
    {
        // Toggle LED pin state
        bc_led_pulse(&led, 200);

        // Toggle LED strips
        for (int l = 0; l < config.leds; l++) {
            if (led_status[l].state == LED_ON) {
                led_state_off(l);
            } else {
                led_status[l].trigger_count++;
                led_state_on(l);
            }
        }
    }
}


static void radio_event_handler(bc_radio_event_t event, void *event_param)
{
    (void) event_param;

    bc_led_set_mode(&led, BC_LED_MODE_OFF);

    if (event == BC_RADIO_EVENT_ATTACH)
    {
        bc_led_pulse(&led, 1000);
        bc_log_debug("Radio: attach");
    }
    else if (event == BC_RADIO_EVENT_DETACH)
    {
        bc_led_pulse(&led, 1000);
        bc_log_debug("Radio: detach");
    }
    else if (event == BC_RADIO_EVENT_INIT_DONE)
    {
        my_id = bc_radio_get_my_id();
        bc_log_debug("Radio: init done, id = 0x%" PRIx64, my_id);
    }
}

void tmp112_event_handler(bc_tmp112_t *self, bc_tmp112_event_t event, void *event_param)
{
    float value;
    temperature_event_param_t *param = (temperature_event_param_t *)event_param;

//     bc_log_debug("Temperature event: %i", event);
    if (event != BC_TMP112_EVENT_UPDATE)
    {
        return;
    }

    if (bc_tmp112_get_temperature_celsius(self, &value))
    {
//         bc_log_debug("Temperature event: measured %f °C", value);
        if ((fabsf(value - param->value) >= TEMPERATURE_TAG_PUB_VALUE_CHANGE) || (param->next_pub < bc_scheduler_get_spin_tick()))
        {
            bc_radio_pub_temperature(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE, &value);
            param->value = value;
            param->next_pub = bc_scheduler_get_spin_tick() + TEMPERATURE_TAG_PUB_NO_CHANGE_INTEVAL;
        }

        // temperature monitoring
        if (value > config.max_temp) {
            // when maximum temperature is reached, turn off all LEDs
            overheat = true;
            for (int l = 0; l < config.leds; l++)
                led_state_off(l);
            bc_radio_pub_float("led-pwm/alert/temperature/fatal", &value);
            bc_log_error("FATAL: maximum allowed temperature exceeded: %f > %f °C", value, config.max_temp);
        } else if (value > config.alert_temp) {
            bc_radio_pub_float("led-pwm/alert/temperature/warning", &value);
            bc_log_warning("WARNING: warning temperature limit exceeded: %f > %f °C", value, config.alert_temp);
        } else {
            overheat = false;
        }
    }
    else
    {
        param->value = NAN;
    }

}


 void _pwm_init(twr_pwm_channel_t channel)
 {
     static bool tim2_initialized = false;
     static bool tim3_initialized = false;
     static bool tim21_initialized = false;
     static bool pll_enabled = false;

     if (!pll_enabled)
     {
         twr_system_pll_enable();
         pll_enabled = true;
     }

     if (!tim2_initialized && (channel == TWR_PWM_P0 || channel == TWR_PWM_P1 || channel == TWR_PWM_P2 || channel == TWR_PWM_P3))
     {
         // 5 us * 255 = cca 784 Hz
         twr_pwm_tim_configure(TWR_PWM_TIM2_P0_P1_P2_P3, 1, PWM_MAX);
         tim2_initialized = true;
     }

     if (!tim3_initialized && (channel == TWR_PWM_P6 || channel == TWR_PWM_P7 || channel == TWR_PWM_P8))
     {
         twr_pwm_tim_configure(TWR_PWM_TIM3_P6_P7_P8, 1, PWM_MAX);
         tim3_initialized = true;
     }

     if (!tim21_initialized && (channel == TWR_PWM_P12 || channel == TWR_PWM_P14))
     {
         twr_pwm_tim_configure(TWR_PWM_TIM21_P12_P14, 1, PWM_MAX);
         tim21_initialized = true;
     }
 }


// Application initialization function which is called once after boot
// TODO use bc_pwm_tim_configure for better PWM precission
void application_init(void)
{
    // initialize data structures
    for (int l=0; l<MAX_LEDS; l++)
        led_ids[l] = l;

    // Load configuration
    bc_config_init(EEPROM_SIGNATURE, &config, sizeof(config), &init_config);

    // Initialize logging
    bc_log_init(BC_LOG_LEVEL_DEBUG, BC_LOG_TIMESTAMP_ABS);

    // Initialize radio
    bc_radio_init(BC_RADIO_MODE_NODE_LISTENING);
    bc_radio_set_subs((bc_radio_sub_t *) mqtt_subs, sizeof(mqtt_subs)/sizeof(bc_radio_sub_t));

    // Initialize LED
    bc_led_init(&led, BC_GPIO_LED, false, 0);
    bc_led_set_mode(&led, BC_LED_MODE_ON);

    // Initialize button
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, 0);
    bc_button_set_event_handler(&button, button_event_handler, NULL);
    bc_log_debug("APP: button init");

    // Initialize thermometer sensor on core module
    bc_tmp112_init(&tmp112, BC_I2C_I2C0, 0x49);
    bc_tmp112_set_event_handler(&tmp112, tmp112_event_handler, &temperature_event_param);
    bc_tmp112_set_update_interval(&tmp112, TEMPERATURE_UPDATE_INTERVAL);

    // setup PWM for all configured GPIO ports
    for (int l=0; l<config.leds; l++) {
        for (int c=0; c<config.led_config[l].channels; c++) {
            _pwm_init(pwm_channel[config.led_config[l].channel[c]]);
            bc_pwm_set(pwm_channel[config.led_config[l].channel[c]], 0);
            bc_pwm_enable(pwm_channel[config.led_config[l].channel[c]]);
            bc_log_debug("APP: PWM init, LED=%i, channel=%i, port=%i", l, c, pwm_channel[config.led_config[l].channel[c]]);
        }
    }

    // Now we can advertise ourselves on network
    bc_radio_set_event_handler(radio_event_handler, NULL);
    bc_radio_pairing_request("led-pwm", VERSION);
}

/*
// Application task function (optional) which is called peridically if scheduled
void application_task(void)
{
    static int counter = 0;

    // Log task run and increment counter
    bc_log_info("APP: Task run (count: %d)", ++counter);

//     bc_pwm_set(BC_PWM_P6, 1 << (counter & 0x7));
//     bc_led_set_mode(&led, BC_LED_MODE_TOGGLE);

    // Plan next run of this task in 1000 ms
    bc_scheduler_plan_current_from_now(10000);
}
*/
