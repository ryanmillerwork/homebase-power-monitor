#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif

/*
 * USB CDC JSON protocol (single JSON object per request, no newline needed):
 * - Request must contain either:
 *     {"get":["v","a","w","pct","charging","min_v","max_v","hrs_capacity","hrs_remaining","fw","chg_threshold_a"]}
 *     or
 *     {"get":"all"} (equivalent to requesting every supported field)
 *   or
 *     {"set":{"min_v":<float>,"max_v":<float>,"hrs_capacity":<float>,"chg_threshold_a":<float>}}
 *   but not both in the same object.
 *   You can request or set only the fields you care about; a GET list may
 *   contain any subset of the supported keys, and SET may include any of
 *   min_v, max_v, hrs_capacity (one or more fields).
 * - Example responses:
 *     {"v":28.523,"a":0.1234,"w":3.5123,"pct":67.12,"charging":true,"hrs_remaining":5.0}
 *     {"ok":true,"min_v":21.000,"max_v":32.200,"hrs_capacity":10.0}
 * - Errors: {"error":"both_get_and_set"} | {"error":"bad_request"} | {"error":"i2c_read"} | {"error":"invalid_get_field","field":<str>,"supported":[...]}
 * - Notes:
 *     pct = 100 * clamp((v - min_v)/(max_v - min_v), 0, 1)
 *     hrs_remaining = hrs_capacity * (pct / 100), rounded to 0.1 hr
 *     charging is true when (chg_threshold_a > 0 ? i >= chg_threshold_a : i <= chg_threshold_a)
 *     defaults if unset: min_v = 21.0, max_v = 32.2, hrs_capacity = 10.0, chg_threshold_a = -0.05
 */

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
#define SETTINGS_VERSION 3

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    float    min_v;
    float    max_v;
    float    hrs_capacity;
    float    chg_threshold_a;
    uint32_t magic_inv;   // ~magic for light corruption check
} settings_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    float    min_v;
    float    max_v;
    float    hrs_capacity;
    uint32_t magic_inv;   // ~magic for light corruption check
} settings_v2_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    float    min_v;
    float    max_v;
    uint32_t magic_inv;
} settings_v1_t;

// defaults (used if nothing valid in flash yet)
static float g_min_v = 21.0f;
static float g_max_v = 32.2f;
static float g_hrs_capacity = 10.0f;
static float g_chg_threshold_a = -0.05f; // signed; sign encodes direction
static int   g_ina_ok = 0;

// Supported GET fields (also used for validation and the "all" shortcut)
static const char *k_get_fields[] = {
    "v", "a", "w", "pct", "charging",
    "min_v", "max_v", "hrs_capacity", "hrs_remaining",
    "fw", "chg_threshold_a"
};
static const size_t k_get_fields_count = sizeof(k_get_fields) / sizeof(k_get_fields[0]);

static void settings_save(float min_v, float max_v, float hrs_capacity, float chg_threshold_a) {
    settings_t s = {
        .magic = SETTINGS_MAGIC,
        .version = SETTINGS_VERSION,
        .min_v = min_v,
        .max_v = max_v,
        .hrs_capacity = hrs_capacity,
        .chg_threshold_a = chg_threshold_a,
        .magic_inv = ~SETTINGS_MAGIC,
    };
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(SETTINGS_OFFSET_FROM_START, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_OFFSET_FROM_START, (const uint8_t *)&s, sizeof(s));
    restore_interrupts(ints);
}

static void settings_load_or_default(void) {
    const settings_t *s = (const settings_t *)SETTINGS_XIP_BASE;
    if (s->magic == SETTINGS_MAGIC && s->magic_inv == ~SETTINGS_MAGIC) {
        if (s->version == SETTINGS_VERSION &&
            s->max_v > s->min_v &&
            s->max_v < 1000.0f && s->min_v > -100.0f &&
            s->hrs_capacity > 0.0f && s->hrs_capacity < 10000.0f &&
            s->chg_threshold_a != 0.0f &&
            s->chg_threshold_a > -100.0f && s->chg_threshold_a < 100.0f) {
            g_min_v = s->min_v;
            g_max_v = s->max_v;
            g_hrs_capacity = s->hrs_capacity;
            g_chg_threshold_a = s->chg_threshold_a;
            return;
        }
        if (s->version == 2) {
            const settings_v2_t *v2 = (const settings_v2_t *)SETTINGS_XIP_BASE;
            if (v2->max_v > v2->min_v &&
                v2->max_v < 1000.0f && v2->min_v > -100.0f &&
                v2->hrs_capacity > 0.0f && v2->hrs_capacity < 10000.0f) {
                g_min_v = v2->min_v;
                g_max_v = v2->max_v;
                g_hrs_capacity = v2->hrs_capacity;
                g_chg_threshold_a = -0.05f; // default for legacy settings
                return;
            }
        }
        if (s->version == 1) {
            const settings_v1_t *v1 = (const settings_v1_t *)SETTINGS_XIP_BASE;
            if (v1->max_v > v1->min_v &&
                v1->max_v < 1000.0f && v1->min_v > -100.0f) {
                g_min_v = v1->min_v;
                g_max_v = v1->max_v;
                g_hrs_capacity = 10.0f;
                g_chg_threshold_a = -0.05f;
            }
        }
    }
    // initialize sector with defaults so future loads are fast
    settings_save(g_min_v, g_max_v, g_hrs_capacity, g_chg_threshold_a);
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

static float pct_from_voltage_alt(float vbus, float min_v, float max_v) {
    const float knee_v = 24.0f;
    const float tail_fraction = 0.1f;

    if (max_v <= min_v || knee_v <= min_v || max_v <= knee_v) {
        return clampf((vbus - min_v) / (max_v - min_v), 0.0f, 1.0f);
    }
    if (vbus >= knee_v) {
        float head = (vbus - knee_v) / (max_v - knee_v);
        return clampf(tail_fraction + (1.0f - tail_fraction) * head, 0.0f, 1.0f);
    }
    float tail = (vbus - min_v) / (knee_v - min_v);
    return clampf(tail_fraction * clampf(tail, 0.0f, 1.0f), 0.0f, 1.0f);
}

// detect both "get" and "set" present
static int has_both_get_and_set(const char *s) {
    return strstr(s, "\"get\"") && strstr(s, "\"set\"");
}

// set all GET wants to 1
static void set_all_wants(int *want_v, int *want_a, int *want_w, int *want_pct, int *want_chg, int *want_min_v, int *want_max_v, int *want_hrs_cap, int *want_hrs_rem, int *want_fw, int *want_chg_thr) {
    *want_v = *want_a = *want_w = *want_pct = *want_chg = *want_min_v = *want_max_v = *want_hrs_cap = *want_hrs_rem = *want_fw = *want_chg_thr = 1;
}

// parse {"get":[ ... ]} or {"get":"all"}; validates against supported list
// returns 1 on success, -1 on invalid field, 0 if no get found
static int parse_get_request(const char *s, int *want_v, int *want_a, int *want_w, int *want_pct, int *want_chg, int *want_min_v, int *want_max_v, int *want_hrs_cap, int *want_hrs_rem, int *want_fw, int *want_chg_thr, char *bad_field, size_t bad_field_cap) {
    const char *g = strstr(s, "\"get\"");
    if (!g) return 0;
    *want_v = *want_a = *want_w = *want_pct = *want_chg = *want_min_v = *want_max_v = *want_hrs_cap = *want_hrs_rem = *want_fw = *want_chg_thr = 0;

    // support both {"get":"all"} and {"get":["..."]}
    const char *lb = strchr(g, '[');
    const char *rb = lb ? strchr(lb, ']') : NULL;
    const char *q = strchr(g, '"'); // first quote after "get"
    const char *after_get = q ? q + 1 : g;

    // Shortcut: {"get":"all"}
    const char *colon = strchr(after_get, ':');
    if (colon) {
        const char *quote_val = strchr(colon, '"');
        if (quote_val) {
            const char *quote_val_end = strchr(quote_val + 1, '"');
            if (quote_val_end && (lb == NULL || quote_val < lb)) {
                size_t len = (size_t)(quote_val_end - (quote_val + 1));
                if (len == 3 && strncmp(quote_val + 1, "all", 3) == 0) {
                    set_all_wants(want_v, want_a, want_w, want_pct, want_chg, want_min_v, want_max_v, want_hrs_cap, want_hrs_rem, want_fw, want_chg_thr);
                    return 1;
                }
            }
        }
    }

    if (!lb || !rb || rb <= lb) return 0;

    const char *p = lb;
    while (p && p < rb) {
        const char *q1 = strchr(p, '"');
        if (!q1 || q1 >= rb) break;
        const char *q2 = strchr(q1 + 1, '"');
        if (!q2 || q2 > rb) break;
        size_t len = (size_t)(q2 - (q1 + 1));
        if (len == 0) { p = q2 + 1; continue; }

        // copy token
        size_t copy_len = len < bad_field_cap - 1 ? len : bad_field_cap - 1;
        memcpy(bad_field, q1 + 1, copy_len);
        bad_field[copy_len] = '\0';

        int matched = 0;
        if (len == 3 && strncmp(q1 + 1, "all", 3) == 0) {
            set_all_wants(want_v, want_a, want_w, want_pct, want_chg, want_min_v, want_max_v, want_hrs_cap, want_hrs_rem, want_fw, want_chg_thr);
            matched = 1;
        } else {
            if (len == 1 && bad_field[0] == 'v') { *want_v = 1; matched = 1; }
            else if (len == 1 && bad_field[0] == 'a') { *want_a = 1; matched = 1; }
            else if (len == 1 && bad_field[0] == 'w') { *want_w = 1; matched = 1; }
            else if (len == 3 && strncmp(bad_field, "pct", 3) == 0) { *want_pct = 1; matched = 1; }
            else if (len == 8 && strncmp(bad_field, "charging", 8) == 0) { *want_chg = 1; matched = 1; }
            else if (len == 5 && strncmp(bad_field, "min_v", 5) == 0) { *want_min_v = 1; matched = 1; }
            else if (len == 5 && strncmp(bad_field, "max_v", 5) == 0) { *want_max_v = 1; matched = 1; }
            else if (len == 12 && strncmp(bad_field, "hrs_capacity", 12) == 0) { *want_hrs_cap = 1; matched = 1; }
            else if (len == 13 && strncmp(bad_field, "hrs_remaining", 13) == 0) { *want_hrs_rem = 1; matched = 1; }
            else if (len == 2 && strncmp(bad_field, "fw", 2) == 0) { *want_fw = 1; matched = 1; }
            else if (len == 15 && strncmp(bad_field, "chg_threshold_a", 15) == 0) { *want_chg_thr = 1; matched = 1; }
        }

        if (!matched) {
            return -1; // invalid field captured in bad_field
        }
        p = q2 + 1;
    }

    return 1;
}

// parse {"set":{"max_v":..,"min_v":..,"chg_threshold_a":..}}
static int parse_set_request(const char *s, float *max_v, float *min_v, float *hrs_capacity, float *chg_threshold_a, int *changed, int *saw_chg_thr) {
    const char *st = strstr(s, "\"set\"");
    if (!st) return 0;
    *changed = 0;
    *saw_chg_thr = 0;
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
    const char *hc = strstr(lb, "\"hrs_capacity\"");
    if (hc && hc < rb) {
        float v;
        if (sscanf(hc, "\"hrs_capacity\"%*[^0-9.-]%f", &v) == 1) { *hrs_capacity = v; *changed = 1; }
    }
    const char *ct = strstr(lb, "\"chg_threshold_a\"");
    if (ct && ct < rb) {
        float v;
        if (sscanf(ct, "\"chg_threshold_a\"%*[^0-9.-]%f", &v) == 1) { *chg_threshold_a = v; *changed = 1; *saw_chg_thr = 1; }
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
        // Non-fatal: keep USB CDC alive so the host can still talk to us.
        // We'll answer requests with an explicit INA226-not-found message.
        g_ina_ok = 0;
        // Emit a one-time boot message for visibility (host might miss it if it connects later).
        printf("{\"error\":\"ina226_not_found\",\"message\":\"INA226 not found\",\"code\":%d}\n", rc);
    } else {
        g_ina_ok = 1;
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
        int saw_chg_thr = 0;
        float new_max = g_max_v, new_min = g_min_v, new_hrs_cap = g_hrs_capacity, new_chg_thr = g_chg_threshold_a;
        if (parse_set_request(inbuf, &new_max, &new_min, &new_hrs_cap, &new_chg_thr, &changed, &saw_chg_thr)) {
            if (changed) {
                if (saw_chg_thr) {
                    if (new_chg_thr == 0.0f || new_chg_thr <= -100.0f || new_chg_thr >= 100.0f) {
                        printf("{\"error\":\"invalid_chg_threshold\",\"message\":\"chg_threshold_a must be non-zero between -100 and 100\"}\n");
                        continue;
                    }
                }
                // ensure sane ordering
                if (new_max <= new_min) { float t = new_max; new_max = new_min; new_min = t; }
                if (new_hrs_cap < 0.0f) new_hrs_cap = 0.0f;
                if (new_hrs_cap > 10000.0f) new_hrs_cap = 10000.0f;
                g_max_v = new_max;
                g_min_v = new_min;
                g_hrs_capacity = new_hrs_cap;
                g_chg_threshold_a = new_chg_thr;
                settings_save(g_min_v, g_max_v, g_hrs_capacity, g_chg_threshold_a);
                snprintf(outbuf, sizeof(outbuf),
                         "{\"ok\":true,\"min_v\":%.3f,\"max_v\":%.3f,\"hrs_capacity\":%.1f,\"chg_threshold_a\":%.3f}\n",
                         g_min_v, g_max_v, g_hrs_capacity, g_chg_threshold_a);
            } else {
                snprintf(outbuf, sizeof(outbuf),
                         "{\"ok\":true,\"min_v\":%.3f,\"max_v\":%.3f,\"hrs_capacity\":%.1f,\"chg_threshold_a\":%.3f}\n",
                         g_min_v, g_max_v, g_hrs_capacity, g_chg_threshold_a);
            }
            if (!g_ina_ok) {
                // Always include INA226-not-found message for host-side clarity.
                // Keep the response as JSON (even though the operation may still succeed).
                // Trim trailing newline from outbuf and wrap with error/message prefix.
                size_t len = strlen(outbuf);
                if (len && outbuf[len - 1] == '\n') outbuf[len - 1] = '\0';
                printf("{\"error\":\"ina226_not_found\",\"message\":\"INA226 not found\",\"result\":%s}\n", outbuf);
            } else {
                fputs(outbuf, stdout);
            }
            continue;
        }

        // --- GET handler ---
        int want_v, want_a, want_w, want_pct, want_chg, want_min_v, want_max_v, want_hrs_cap, want_hrs_rem, want_fw, want_chg_thr;
        char bad_field[32] = {0};
        int get_rc = parse_get_request(inbuf, &want_v, &want_a, &want_w, &want_pct, &want_chg, &want_min_v, &want_max_v, &want_hrs_cap, &want_hrs_rem, &want_fw, &want_chg_thr, bad_field, sizeof(bad_field));
        if (get_rc == -1) {
            // Invalid field requested; respond with explicit list of supported fields.
            printf("{\"error\":\"invalid_get_field\",\"field\":\"%s\",\"supported\":[\"v\",\"a\",\"w\",\"pct\",\"charging\",\"min_v\",\"max_v\",\"hrs_capacity\",\"hrs_remaining\",\"fw\",\"chg_threshold_a\"]}\n", bad_field);
            continue;
        }
        if (get_rc == 1) {
            // If INA226 is missing, still answer with a JSON object including the requested
            // non-sensor fields plus an explicit message for host-side clarity.
            if (!g_ina_ok) {
                char *w = outbuf; size_t rem = sizeof(outbuf); int first = 1;
                w += snprintf(w, rem, "{"); rem = sizeof(outbuf)-(w-outbuf);
                w += snprintf(w, rem, "\"error\":\"ina226_not_found\",\"message\":\"INA226 not found\""); first = 0; rem = sizeof(outbuf)-(w-outbuf);
                if (want_fw) { w += snprintf(w, rem, "%s\"fw\":\"%s\"", first?"":",", FW_VERSION); first=0; rem = sizeof(outbuf)-(w-outbuf); }
                if (want_min_v) { w += snprintf(w, rem, "%s\"min_v\":%.3f", first?"":",", g_min_v); first=0; rem = sizeof(outbuf)-(w-outbuf); }
                if (want_max_v) { w += snprintf(w, rem, "%s\"max_v\":%.3f", first?"":",", g_max_v); first=0; rem = sizeof(outbuf)-(w-outbuf); }
                if (want_hrs_cap) { w += snprintf(w, rem, "%s\"hrs_capacity\":%.1f", first?"":",", g_hrs_capacity); first=0; rem = sizeof(outbuf)-(w-outbuf); }
                if (want_chg_thr) { w += snprintf(w, rem, "%s\"chg_threshold_a\":%.3f", first?"":",", g_chg_threshold_a); first=0; rem = sizeof(outbuf)-(w-outbuf); }
                // Note: v/a/w/pct/charging/hrs_remaining require INA226 measurements; omit them when missing.
                w += snprintf(w, rem, "}\n");
                fputs(outbuf, stdout);
                continue;
            }

            float vbus=0, i=0, p=0;
            int ok = 1;
            ok &= (ina226_bus_voltage_V(&ina, &vbus) == 0);
            ok &= (ina226_current_A(&ina, &i) == 0);
            if (want_w) ok &= (ina226_power_W(&ina, &p) == 0);
            if (!ok) { fputs("{\"error\":\"i2c_read\"}\n", stdout); continue; }

            char *w = outbuf; size_t rem = sizeof(outbuf); int first = 1;
            w += snprintf(w, rem, "{"); rem = sizeof(outbuf)-(w-outbuf);
            if (want_fw) { w += snprintf(w, rem, "%s\"fw\":\"%s\"", first?"":",", FW_VERSION); first=0; rem = sizeof(outbuf)-(w-outbuf); }
            if (want_v)  { w += snprintf(w, rem, "%s\"v\":%.3f", first?"":",", vbus); first=0; rem = sizeof(outbuf)-(w-outbuf); }
            if (want_a)  { w += snprintf(w, rem, "%s\"a\":%.4f", first?"":",", i);    first=0; rem = sizeof(outbuf)-(w-outbuf); }
            if (want_w)  { w += snprintf(w, rem, "%s\"w\":%.4f", first?"":",", p);    first=0; rem = sizeof(outbuf)-(w-outbuf); }
            float pct = 0.0f;
            if (want_pct || want_hrs_rem){
                pct = 100.0f * pct_from_voltage_alt(vbus, g_min_v, g_max_v);
            }
            if (want_pct){
                w += snprintf(w, rem, "%s\"pct\":%.2f", first?"":",", pct); first=0; rem = sizeof(outbuf)-(w-outbuf);
            }
            if (want_hrs_rem){
                float hrs_remaining = g_hrs_capacity * pct * 0.01f;
                w += snprintf(w, rem, "%s\"hrs_remaining\":%.1f", first?"":",", hrs_remaining); first=0; rem = sizeof(outbuf)-(w-outbuf);
            }
            if (want_chg){
                int charging = (g_chg_threshold_a > 0.0f) ? (i >= g_chg_threshold_a) : (i <= g_chg_threshold_a);
                w += snprintf(w, rem, "%s\"charging\":%s", first?"":",", charging?"true":"false");
                first=0; rem = sizeof(outbuf)-(w-outbuf);
            }
            if (want_min_v) { w += snprintf(w, rem, "%s\"min_v\":%.3f", first?"":",", g_min_v); first=0; rem = sizeof(outbuf)-(w-outbuf); }
            if (want_max_v) { w += snprintf(w, rem, "%s\"max_v\":%.3f", first?"":",", g_max_v); first=0; rem = sizeof(outbuf)-(w-outbuf); }
            if (want_hrs_cap) { w += snprintf(w, rem, "%s\"hrs_capacity\":%.1f", first?"":",", g_hrs_capacity); first=0; rem = sizeof(outbuf)-(w-outbuf); }
            if (want_chg_thr) { w += snprintf(w, rem, "%s\"chg_threshold_a\":%.3f", first?"":",", g_chg_threshold_a); first=0; rem = sizeof(outbuf)-(w-outbuf); }
            w += snprintf(w, rem, "}\n");
            fputs(outbuf, stdout);
            continue;
        }

        // Unknown request
        fputs("{\"error\":\"bad_request\"}\n", stdout);
    }
}


