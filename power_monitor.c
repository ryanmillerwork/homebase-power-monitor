#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

// ======= I2C / INA226 wiring (Waveshare RP2040-Zero) =======
#define I2C_INST       i2c0
#define PIN_I2C_SDA    0
#define PIN_I2C_SCL    1
#define I2C_FREQ_HZ    100000  // 100 kHz; raise to 400k if you like

// ======= INA226 register map & address =======
#define INA226_REG_CONFIG   0x00
#define INA226_REG_SHUNT    0x01
#define INA226_REG_BUS      0x02
#define INA226_REG_POWER    0x03
#define INA226_REG_CURRENT  0x04
#define INA226_REG_CAL      0x05
#define INA226_ADDR         0x40   // default 7-bit address

typedef struct {
    uint8_t addr;
    float shunt_ohms;
    float i_max;
    float current_lsb; // A/LSB
    float power_lsb;   // W/LSB
} ina226_t;

// ======= Persistent settings in flash (last 4KB sector) =======

#ifndef PICO_FLASH_SIZE_BYTES
#warning "PICO_FLASH_SIZE_BYTES not defined; defaulting to 2MB"
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

#define SETTINGS_OFFSET_FROM_START  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define SETTINGS_XIP_BASE           (XIP_BASE + SETTINGS_OFFSET_FROM_START)

#define SETTINGS_MAGIC   0x53544731u  // 'STG1'
#define SETTINGS_VERSION 1

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    float    min_v;
    float    max_v;
    uint32_t magic_inv;   // ~magic for light corruption check
} settings_t;

// defaults (used if nothing valid in flash yet)
static float g_min_v = 21.0f;
static float g_max_v = 32.2f;

static void settings_save(float min_v, float max_v) {
    settings_t s = {
        .magic = SETTINGS_MAGIC,
        .version = SETTINGS_VERSION,
        .min_v = min_v,
        .max_v = max_v,
        .magic_inv = ~SETTINGS_MAGIC,
    };
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(SETTINGS_OFFSET_FROM_START, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_OFFSET_FROM_START, (const uint8_t *)&s, sizeof(s));
    restore_interrupts(ints);
}

static void settings_load_or_default(void) {
    const settings_t *s = (const settings_t *)SETTINGS_XIP_BASE;
    if (s->magic == SETTINGS_MAGIC &&
        s->magic_inv == ~SETTINGS_MAGIC &&
        s->version == SETTINGS_VERSION &&
        s->max_v > s->min_v &&
        s->max_v < 1000.0f && s->min_v > -100.0f) {
        g_min_v = s->min_v;
        g_max_v = s->max_v;
    } else {
        // initialize sector with defaults so future loads are fast
        settings_save(g_min_v, g_max_v);
    }
}

// ======= I2C low-level helpers =======
static int i2c_w16(uint8_t addr, uint8_t reg, uint16_t val) {
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    int wrote = i2c_write_blocking(I2C_INST, addr, buf, 3, false);
    return (wrote == 3) ? 0 : -1;
}
static int i2c_r1_then_r(uint8_t addr, const uint8_t *wbuf, size_t wn, uint8_t *rbuf, size_t rn) {
    int w = i2c_write_blocking(I2C_INST, addr, wbuf, wn, true);  // repeated start
    if (w != (int)wn) return -1;
    int r = i2c_read_blocking(I2C_INST, addr, rbuf, rn, false);
    return (r == (int)rn) ? 0 : -2;
}
static int i2c_r16(uint8_t addr, uint8_t reg, uint16_t *out) {
    uint8_t rreg = reg, b[2];
    int rc = i2c_r1_then_r(addr, &rreg, 1, b, 2);
    if (rc) return rc;
    *out = ((uint16_t)b[0] << 8) | b[1];
    return 0;
}
static int i2c_rs16(uint8_t addr, uint8_t reg, int16_t *out) {
    uint16_t u;
    int rc = i2c_r16(addr, reg, &u);
    if (rc) return rc;
    *out = (u & 0x8000) ? (int16_t)(u - 0x10000) : (int16_t)u;
    return 0;
}

// ======= INA226 API =======
static int ina226_init(ina226_t *dev, uint8_t addr, float shunt_ohms, float i_max) {
    dev->addr = addr;
    dev->shunt_ohms = shunt_ohms;
    dev->i_max = i_max;
    dev->current_lsb = i_max / 32768.0f;        // A/LSB
    dev->power_lsb   = 25.0f * dev->current_lsb;// W/LSB

    float fcal = 0.00512f / (dev->current_lsb * dev->shunt_ohms);
    if (fcal < 1.0f || fcal > 65535.0f) return -10;
    uint16_t cal = (uint16_t)(fcal + 0.5f);
    if (i2c_w16(dev->addr, INA226_REG_CAL, cal)) return -11;

    // AVG=16, VBUSCT=1.1ms, VSHCT=1.1ms, MODE=111 (cont shunt+bus)
    uint16_t config = (0b100u << 9) | (0b100u << 6) | (0b100u << 3) | 0b111u;
    if (i2c_w16(dev->addr, INA226_REG_CONFIG, config)) return -12;

    return 0;
}
static int ina226_bus_voltage_V(ina226_t *dev, float *v) {
    uint16_t raw; int rc = i2c_r16(dev->addr, INA226_REG_BUS, &raw);
    if (rc) return rc; *v = (float)raw * 1.25e-3f; return 0;
}
static int ina226_shunt_voltage_V(ina226_t *dev, float *v) {
    int16_t raw; int rc = i2c_rs16(dev->addr, INA226_REG_SHUNT, &raw);
    if (rc) return rc; *v = (float)raw * 2.5e-6f; return 0;
}
static int ina226_current_A(ina226_t *dev, float *i) {
    int16_t raw; int rc = i2c_rs16(dev->addr, INA226_REG_CURRENT, &raw);
    if (rc) return rc; *i = (float)raw * dev->current_lsb; return 0;
}
static int ina226_power_W(ina226_t *dev, float *p) {
    uint16_t raw; int rc = i2c_r16(dev->addr, INA226_REG_POWER, &raw);
    if (rc) return rc; *p = (float)raw * dev->power_lsb; return 0;
}

// ======= Utils =======
static float clampf(float x, float lo, float hi){ return x < lo ? lo : (x > hi ? hi : x); }

// detect both "get" and "set" present
static int has_both_get_and_set(const char *s) {
    return strstr(s, "\"get\"") && strstr(s, "\"set\"");
}

// parse {"get":[ ... ]} for tokens: "v","a","w","pct","charging"
static int parse_get_request(const char *s, int *want_v, int *want_a, int *want_w, int *want_pct, int *want_chg) {
    const char *g = strstr(s, "\"get\"");
    if (!g) return 0;
    *want_v = *want_a = *want_w = *want_pct = *want_chg = 0;
    const char *lb = strchr(g, '[');
    const char *rb = lb ? strchr(lb, ']') : NULL;
    if (!lb || !rb || rb <= lb) return 0;

    if (strstr(lb, "\"v\"")        && strstr(lb, "\"v\"")        < rb) *want_v   = 1;
    if (strstr(lb, "\"a\"")        && strstr(lb, "\"a\"")        < rb) *want_a   = 1;
    if (strstr(lb, "\"w\"")        && strstr(lb, "\"w\"")        < rb) *want_w   = 1;
    if (strstr(lb, "\"pct\"")      && strstr(lb, "\"pct\"")      < rb) *want_pct = 1;
    if (strstr(lb, "\"charging\"") && strstr(lb, "\"charging\"") < rb) *want_chg = 1;
    return 1;
}

// parse {"set":{"max_v":..,"min_v":..}}
static int parse_set_request(const char *s, float *max_v, float *min_v, int *changed) {
    const char *st = strstr(s, "\"set\"");
    if (!st) return 0;
    *changed = 0;
    const char *lb = strchr(st, '{');
    const char *rb = lb ? strchr(lb, '}') : NULL;
    if (!lb || !rb || rb <= lb) return 0;

    const char *mx = strstr(lb, "\"max_v\"");
    if (mx && mx < rb) {
        float v;
        if (sscanf(mx, "\"max_v\"%*[^0-9.-]%f", &v) == 1) { *max_v = v; *changed = 1; }
    }
    const char *mn = strstr(lb, "\"min_v\"");
    if (mn && mn < rb) {
        float v;
        if (sscanf(mn, "\"min_v\"%*[^0-9.-]%f", &v) == 1) { *min_v = v; *changed = 1; }
    }
    return 1;
}

// ======= Input accumulator: capture one JSON object { ... } (no newline needed) =======
static int read_json_object(char *out, size_t cap, uint32_t poll_ms) {
    static char buf[512];
    static size_t n = 0;
    static int depth = 0;
    static int in_str = 0;   // inside "..."
    static int esc = 0;      // after backslash
    absolute_time_t until = make_timeout_time_ms(poll_ms);

    while (absolute_time_diff_us(get_absolute_time(), until) > 0) {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) { tight_loop_contents(); continue; }
        char c = (char)ch;

        if (n + 1 >= sizeof(buf)) { n = 0; depth = 0; in_str = 0; esc = 0; } // reset on overflow

        if (!depth) {
            if (c == '{') { buf[n++] = c; depth = 1; in_str = 0; esc = 0; }
            continue;
        }

        buf[n++] = c;
        if (esc) { esc = 0; continue; }
        if (c == '\\') { esc = 1; continue; }
        if (c == '"') { in_str = !in_str; continue; }
        if (in_str) continue;

        if (c == '{') depth++;
        else if (c == '}') {
            depth--;
            if (depth == 0) {
                buf[n] = '\0';
                size_t len = n < cap ? n : cap - 1;
                memcpy(out, buf, len);
                out[len] = '\0';
                n = 0; in_str = 0; esc = 0;
                return (int)len;
            }
        }
    }
    return -1; // no complete object yet
}

int main() {
    stdio_init_all();
    sleep_ms(1500); // allow USB CDC to enumerate

    // Load persisted thresholds (or initialize defaults)
    settings_load_or_default();

    // I2C init
    i2c_init(I2C_INST, I2C_FREQ_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA);
    gpio_pull_up(PIN_I2C_SCL);

    // INA226 init (0.1Ω shunt, 2A full-scale — adjust as needed)
    ina226_t ina;
    int rc = ina226_init(&ina, INA226_ADDR, 0.1f, 2.0f);
    if (rc) {
        printf("{\"error\":\"ina226_init\",\"code\":%d}\n", rc);
        while (1) tight_loop_contents();
    }

    // Announce ready + current thresholds
    char inbuf[256], outbuf[256];

    while (true) {
        int n = read_json_object(inbuf, sizeof(inbuf), 50); // poll every 50 ms
        if (n <= 0) continue;

        if (has_both_get_and_set(inbuf)) {
            printf("{\"error\":\"both_get_and_set\"}\n");
            continue;
        }

        // --- SET handler ---
        int changed = 0;
        float new_max = g_max_v, new_min = g_min_v;
        if (parse_set_request(inbuf, &new_max, &new_min, &changed)) {
            if (changed) {
                // ensure sane ordering
                if (new_max <= new_min) { float t = new_max; new_max = new_min; new_min = t; }
                g_max_v = new_max;
                g_min_v = new_min;
                settings_save(g_min_v, g_max_v);
                snprintf(outbuf, sizeof(outbuf),
                         "{\"ok\":true,\"min_v\":%.3f,\"max_v\":%.3f}\n", g_min_v, g_max_v);
            } else {
                snprintf(outbuf, sizeof(outbuf),
                         "{\"ok\":true,\"min_v\":%.3f,\"max_v\":%.3f}\n", g_min_v, g_max_v);
            }
            fputs(outbuf, stdout);
            continue;
        }

        // --- GET handler ---
        int want_v, want_a, want_w, want_pct, want_chg;
        if (parse_get_request(inbuf, &want_v, &want_a, &want_w, &want_pct, &want_chg)) {
            float vbus=0, i=0, p=0;
            int ok = 1;
            ok &= (ina226_bus_voltage_V(&ina, &vbus) == 0);
            ok &= (ina226_current_A(&ina, &i) == 0);
            if (want_w) ok &= (ina226_power_W(&ina, &p) == 0);
            if (!ok) { fputs("{\"error\":\"i2c_read\"}\n", stdout); continue; }

            char *w = outbuf; size_t rem = sizeof(outbuf); int first = 1;
            w += snprintf(w, rem, "{"); rem = sizeof(outbuf)-(w-outbuf);
            if (want_v)  { w += snprintf(w, rem, "%s\"v\":%.3f", first?"":",", vbus); first=0; rem = sizeof(outbuf)-(w-outbuf); }
            if (want_a)  { w += snprintf(w, rem, "%s\"a\":%.4f", first?"":",", i);    first=0; rem = sizeof(outbuf)-(w-outbuf); }
            if (want_w)  { w += snprintf(w, rem, "%s\"w\":%.4f", first?"":",", p);    first=0; rem = sizeof(outbuf)-(w-outbuf); }
            if (want_pct){
                float pct = 100.0f * clampf((vbus - g_min_v) / (g_max_v - g_min_v), 0.0f, 1.0f);
                w += snprintf(w, rem, "%s\"pct\":%.2f", first?"":",", pct); first=0; rem = sizeof(outbuf)-(w-outbuf);
            }
            if (want_chg){
                int charging = (i > 0.05f) ? 1 : 0; // tweak threshold as needed
                w += snprintf(w, rem, "%s\"charging\":%s", first?"":",", charging?"true":"false");
                first=0; rem = sizeof(outbuf)-(w-outbuf);
            }
            w += snprintf(w, rem, "}\n");
            fputs(outbuf, stdout);
            continue;
        }

        // Unknown request
        fputs("{\"error\":\"bad_request\"}\n", stdout);
    }
}


