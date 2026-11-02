#include "app.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "w25q64.h"
#include "servo.h"
#include "ov7670.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <math.h>

#define IMU_PERIOD_MS 10u
#define BARO_PERIOD_MS 1000u
#define LED_IDLE_PERIOD_MS 500u
#define LED_INCIDENT_PERIOD_MS 50u
#define BTN_DEBOUNCE_MS 40u

#define IMPACT_THRESHOLD_G 2.5f
#define TILT_THRESHOLD_DEG 60.0f
#define COMPLEMENTARY_ALPHA 0.98f

#define FLASH_TEST_ADDR 0x00000000u /* нижній сектор; 0x00700000 часто під BP */
#define JOURNAL_BASE_ADDR 0x00100000u
#define JOURNAL_SLOT_SIZE 0x6000u /* 24 KB = 6×4K sectors */
#define JOURNAL_SLOT_COUNT 8u
#define JOURNAL_MAGIC 0xB10CB001u

#define CLI_LINE_MAX 64u

typedef enum
{
    MODE_CARGO = 0,
    MODE_SITE = 1
} app_mode_t;

typedef enum
{
    SYS_IDLE = 0,
    SYS_ARMED = 1,
    SYS_INCIDENT = 2
} app_sys_state_t;

typedef enum
{
    TRIG_NONE = 0,
    TRIG_BUTTON = 1,
    TRIG_IMPACT = 2,
    TRIG_TILT = 3
} trigger_reason_t;

typedef enum
{
    JRN_IDLE = 0,
    JRN_ERASE,
    JRN_WAIT_ERASE,
    JRN_WRITE_HDR,
    JRN_WRITE_FRAME,
    JRN_DONE
} journal_fsm_t;

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint32_t seq;
    uint32_t tick_ms;
    uint8_t mode;
    uint8_t reason;
    uint8_t reserved[2];
    float ax, ay, az;
    float pitch_deg, roll_deg;
    int16_t temp_x10;
    int32_t pressure_pa;
    uint16_t frame_w;
    uint16_t frame_h;
    uint16_t frame_bytes;
    uint16_t crc;
} journal_hdr_t;

static mpu6050_t s_mpu;
static bmp180_t s_bmp;
static w25q64_t s_flash;
static servo_t s_servo;
static ov7670_t s_camera;

static const board_io_interface_t *s_board = NULL;
static const uart_rx_interface_t *s_cli = NULL;

static app_mode_t s_mode = MODE_CARGO;
static app_sys_state_t s_sys = SYS_IDLE;
static bool s_mpu_ok = false;
static bool s_bmp_ok = false;
static bool s_flash_ok = false;
static bool s_cam_ok = false;

static uint32_t s_last_imu_ms = 0;
static uint32_t s_last_baro_ms = 0;
static uint32_t s_last_led_ms = 0;
static uint32_t s_btn_last_change_ms = 0;
static bool s_btn_stable = false;
static bool s_btn_prev = false;
static bool s_btn_event = false;

static float s_pitch = 0.0f;
static float s_roll = 0.0f;
static bool s_attitude_init = false;
static mpu6050_motion_t s_motion;
static int16_t s_temp_x10 = 0;
static int32_t s_pressure_pa = 0;

static uint8_t s_frame[OV7670_FRAME_BYTES];
static bool s_frame_valid = false;
static bool s_capture_pending = false;

static journal_fsm_t s_jrn = JRN_IDLE;
static journal_hdr_t s_jrn_hdr;
static uint32_t s_jrn_slot = 0;
static uint32_t s_jrn_seq = 0;
static uint32_t s_jrn_erase_idx = 0;
static uint32_t s_jrn_frame_off = 0;
static uint32_t s_jrn_slot_addr = 0;

static char s_cli_line[CLI_LINE_MAX];
static uint8_t s_cli_len = 0;

static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

/** CRC лише по байтах до поля crc (через memcpy — без aliasing/FPU сюрпризів) */
static uint16_t journal_hdr_crc(const journal_hdr_t *hdr)
{
    uint8_t buf[64];
    const uint32_t n = (uint32_t)offsetof(journal_hdr_t, crc);
    if (n > sizeof(buf))
        return 0;
    memcpy(buf, hdr, n);
    return crc16_ccitt(buf, n);
}

static const char *mode_str(app_mode_t m)
{
    return (m == MODE_SITE) ? "site" : "cargo";
}

static const char *sys_str(app_sys_state_t s)
{
    switch (s)
    {
    case SYS_ARMED:
        return "ARMED";
    case SYS_INCIDENT:
        return "INCIDENT";
    default:
        return "IDLE";
    }
}

static const char *trig_str(trigger_reason_t r)
{
    switch (r)
    {
    case TRIG_BUTTON:
        return "button";
    case TRIG_IMPACT:
        return "impact";
    case TRIG_TILT:
        return "tilt";
    default:
        return "none";
    }
}

static void enter_armed(void)
{
    s_sys = SYS_ARMED;
    servo_set_position(&s_servo, SERVO_ANGLE_180);
    if (s_board)
        s_board->led_write(true);
    printf("[SYS] Armed — servo locked (180 deg)\r\n");
}

static void enter_idle(void)
{
    s_sys = SYS_IDLE;
    servo_set_position(&s_servo, SERVO_ANGLE_0);
    if (s_board)
        s_board->led_write(false);
    printf("[SYS] Idle — servo unlocked\r\n");
}

static void start_journal_write(uint32_t tick_ms, trigger_reason_t reason)
{
    memset(&s_jrn_hdr, 0, sizeof(s_jrn_hdr));
    s_jrn_hdr.magic = JOURNAL_MAGIC;
    s_jrn_hdr.seq = ++s_jrn_seq;
    s_jrn_hdr.tick_ms = tick_ms;
    s_jrn_hdr.mode = (uint8_t)s_mode;
    s_jrn_hdr.reason = (uint8_t)reason;
    s_jrn_hdr.ax = s_motion.accel.ax;
    s_jrn_hdr.ay = s_motion.accel.ay;
    s_jrn_hdr.az = s_motion.accel.az;
    s_jrn_hdr.pitch_deg = s_pitch;
    s_jrn_hdr.roll_deg = s_roll;
    s_jrn_hdr.temp_x10 = s_temp_x10;
    s_jrn_hdr.pressure_pa = s_pressure_pa;
    s_jrn_hdr.frame_w = OV7670_QQVGA_W;
    s_jrn_hdr.frame_h = OV7670_QQVGA_H;
    s_jrn_hdr.frame_bytes = s_frame_valid ? (uint16_t)OV7670_FRAME_BYTES : 0u;
    s_jrn_hdr.crc = journal_hdr_crc(&s_jrn_hdr);

    s_jrn_slot = (s_jrn_seq - 1u) % JOURNAL_SLOT_COUNT;
    s_jrn_slot_addr = JOURNAL_BASE_ADDR + s_jrn_slot * JOURNAL_SLOT_SIZE;
    s_jrn_erase_idx = 0;
    s_jrn_frame_off = 0;
    s_jrn = JRN_ERASE;
}

/* pending reason між raise і journal start */
static trigger_reason_t s_pending_trig = TRIG_NONE;
static uint32_t s_pending_trig_tick = 0;

static void raise_incident(uint32_t tick_ms, trigger_reason_t reason)
{
    if (s_sys == SYS_INCIDENT && s_jrn != JRN_IDLE && s_jrn != JRN_DONE)
        return;

    s_sys = SYS_INCIDENT;
    servo_set_position(&s_servo, SERVO_ANGLE_180);
    s_capture_pending = s_cam_ok;
    s_pending_trig = reason;
    s_pending_trig_tick = tick_ms;

    printf("[ALARM] Incident trigger=%s |a|=%.2fg pitch=%.1f roll=%.1f\r\n",
           trig_str(reason),
           (double)s_motion.accel.a_total,
           (double)s_pitch,
           (double)s_roll);

    if (!s_cam_ok)
    {
        s_frame_valid = false;
        start_journal_write(tick_ms, reason);
        s_pending_trig = TRIG_NONE;
    }
}

static void task_button(uint32_t now)
{
    if (!s_board)
        return;

    bool pressed = s_board->button_is_pressed();
    if (pressed != s_btn_prev)
    {
        s_btn_last_change_ms = now;
        s_btn_prev = pressed;
    }
    if ((now - s_btn_last_change_ms) >= BTN_DEBOUNCE_MS)
    {
        if (pressed && !s_btn_stable)
        {
            s_btn_stable = true;
            s_btn_event = true;
        }
        else if (!pressed)
        {
            s_btn_stable = false;
        }
    }
}

static void task_imu(uint32_t now)
{
    if (!s_mpu_ok)
        return;
    if ((now - s_last_imu_ms) < IMU_PERIOD_MS)
        return;
    const float dt = (s_last_imu_ms == 0) ? 0.01f : (float)(now - s_last_imu_ms) * 0.001f;
    s_last_imu_ms = now;

    if (!mpu6050_read_motion(&s_mpu, &s_motion))
        return;

    float accel_pitch = atan2f(s_motion.accel.ay, s_motion.accel.az) * (180.0f / 3.14159265f);
    float accel_roll = atan2f(-s_motion.accel.ax, s_motion.accel.az) * (180.0f / 3.14159265f);

    if (!s_attitude_init)
    {
        s_pitch = accel_pitch;
        s_roll = accel_roll;
        s_attitude_init = true;
    }
    else
    {
        s_pitch = COMPLEMENTARY_ALPHA * (s_pitch + s_motion.gyro.gx * dt) +
                  (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
        s_roll = COMPLEMENTARY_ALPHA * (s_roll + s_motion.gyro.gy * dt) +
                 (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;
    }

    if (s_sys == SYS_ARMED || s_sys == SYS_IDLE)
    {
        if (s_btn_event)
        {
            s_btn_event = false;
            raise_incident(now, TRIG_BUTTON);
            return;
        }
        if (s_motion.accel.a_total > IMPACT_THRESHOLD_G)
        {
            raise_incident(now, TRIG_IMPACT);
            return;
        }
        if (fabsf(s_pitch) > TILT_THRESHOLD_DEG || fabsf(s_roll) > TILT_THRESHOLD_DEG)
        {
            raise_incident(now, TRIG_TILT);
            return;
        }
    }
    else
    {
        s_btn_event = false;
    }
}

static void task_baro(uint32_t now)
{
    if (!s_bmp_ok)
        return;
    if ((now - s_last_baro_ms) < BARO_PERIOD_MS)
        return;
    s_last_baro_ms = now;

    (void)bmp180_read_temperature(&s_bmp, &s_temp_x10);
    (void)bmp180_read_pressure(&s_bmp, &s_pressure_pa);
}

static void task_camera(uint32_t now)
{
    (void)now;
    if (!s_capture_pending || !s_cam_ok)
        return;

    s_capture_pending = false;
    printf("[CAM] Capture start (QQVGA gray)...\r\n");
    s_frame_valid = ov7670_capture_gray(&s_camera, s_frame, 1500);
    printf("[CAM] Capture %s\r\n", s_frame_valid ? "OK" : "FAIL/TIMEOUT");

    if (s_pending_trig != TRIG_NONE)
    {
        start_journal_write(s_pending_trig_tick, s_pending_trig);
        s_pending_trig = TRIG_NONE;
    }
}

static void task_journal(uint32_t now)
{
    (void)now;
    if (!s_flash_ok)
        return;

    switch (s_jrn)
    {
    case JRN_ERASE:
        if (s_jrn_erase_idx >= (JOURNAL_SLOT_SIZE / 4096u))
        {
            s_jrn = JRN_WRITE_HDR;
            break;
        }
        if (!w25q64_erase_sector_4k_start(&s_flash, s_jrn_slot_addr + s_jrn_erase_idx * 4096u))
        {
            printf("[FLASH] Erase start failed\r\n");
            s_jrn = JRN_IDLE;
            break;
        }
        s_jrn = JRN_WAIT_ERASE;
        break;

    case JRN_WAIT_ERASE:
        if (!w25q64_is_busy(&s_flash))
        {
            s_jrn_erase_idx++;
            s_jrn = JRN_ERASE;
        }
        break;

    case JRN_WRITE_HDR:
    {
        uint8_t page[256];
        memset(page, 0xFF, sizeof(page));
        memcpy(page, &s_jrn_hdr, sizeof(s_jrn_hdr));
        if (!w25q64_page_program(&s_flash, s_jrn_slot_addr, page, 256))
        {
            printf("[FLASH] Header write failed\r\n");
            s_jrn = JRN_IDLE;
            break;
        }
        /* Верифікація: раніше «saved» друкувалось навіть коли дані не закріпились */
        {
            journal_hdr_t check;
            memset(&check, 0, sizeof(check));
            if (!w25q64_read_data(&s_flash, s_jrn_slot_addr, (uint8_t *)&check, sizeof(check)) ||
                check.magic != JOURNAL_MAGIC || check.seq != s_jrn_hdr.seq)
            {
                uint8_t sr1 = 0;
                (void)w25q64_read_status1(&s_flash, &sr1);
                printf("[FLASH] Header verify FAIL magic=0x%08lX seq=%lu SR1=0x%02X\r\n",
                       (unsigned long)check.magic,
                       (unsigned long)check.seq,
                       (unsigned)sr1);
                s_jrn = JRN_IDLE;
                break;
            }
        }
        s_jrn_frame_off = 0;
        s_jrn = (s_jrn_hdr.frame_bytes > 0) ? JRN_WRITE_FRAME : JRN_DONE;
        break;
    }

    case JRN_WRITE_FRAME:
    {
        uint32_t remain = s_jrn_hdr.frame_bytes - s_jrn_frame_off;
        uint16_t chunk = (remain > 256u) ? 256u : (uint16_t)remain;
        uint32_t addr = s_jrn_slot_addr + 256u + s_jrn_frame_off;
        if (!w25q64_page_program(&s_flash, addr, &s_frame[s_jrn_frame_off], chunk))
        {
            printf("[FLASH] Frame page write failed @%lu\r\n", (unsigned long)addr);
            s_jrn = JRN_IDLE;
            break;
        }
        s_jrn_frame_off += chunk;
        if (s_jrn_frame_off >= s_jrn_hdr.frame_bytes)
            s_jrn = JRN_DONE;
        break;
    }

    case JRN_DONE:
        printf("[FLASH] Journal slot %lu seq=%lu saved (frame=%u B)\r\n",
               (unsigned long)s_jrn_slot,
               (unsigned long)s_jrn_hdr.seq,
               (unsigned)s_jrn_hdr.frame_bytes);
        s_jrn = JRN_IDLE;
        break;

    case JRN_IDLE:
    default:
        break;
    }
}

static void task_led(uint32_t now)
{
    if (!s_board)
        return;

    uint32_t period = LED_IDLE_PERIOD_MS;
    if (s_sys == SYS_ARMED)
    {
        s_board->led_write(true);
        return;
    }
    if (s_sys == SYS_INCIDENT)
        period = LED_INCIDENT_PERIOD_MS;

    if ((now - s_last_led_ms) >= period)
    {
        s_last_led_ms = now;
        s_board->led_toggle();
    }
}

static void cli_print_help(void)
{
    printf("Commands:\r\n");
    printf("  help              - this help\r\n");
    printf("  status            - system status\r\n");
    printf("  mode cargo|site   - select mode\r\n");
    printf("  arm | disarm      - lock/unlock servo\r\n");
    printf("  dump log          - dump journal headers\r\n");
    printf("  dump frame N      - dump slot N grayscale frame (hex) to PC\r\n");
    printf("  reset             - clear incident -> idle\r\n");
    printf("  sim incident      - software incident trigger\r\n");
}

static bool journal_read_hdr(uint32_t slot, journal_hdr_t *hdr, uint16_t *crc_ok)
{
    uint32_t addr = JOURNAL_BASE_ADDR + slot * JOURNAL_SLOT_SIZE;
    if (!w25q64_read_data(&s_flash, addr, (uint8_t *)hdr, sizeof(*hdr)))
        return false;
    if (hdr->magic != JOURNAL_MAGIC)
        return false;

    uint16_t stored = hdr->crc;
    uint16_t calc = journal_hdr_crc(hdr);
    if (crc_ok)
        *crc_ok = (stored == calc) ? 1u : 0u;
    return true;
}

static void cli_dump_log(void)
{
    if (!s_flash_ok)
    {
        printf("[ERR] Flash not available\r\n");
        return;
    }

    printf("=== Journal dump ===\r\n");
    for (uint32_t i = 0; i < JOURNAL_SLOT_COUNT; i++)
    {
        journal_hdr_t hdr;
        uint16_t crc_ok = 0;
        if (!journal_read_hdr(i, &hdr, &crc_ok))
            continue;

        printf("#%lu seq=%lu t=%lu reason=%u mode=%u |a|=(%.2f,%.2f,%.2f) pitch=%.1f roll=%.1f P=%ld Pa frame=%u crc=%s\r\n",
               (unsigned long)i,
               (unsigned long)hdr.seq,
               (unsigned long)hdr.tick_ms,
               (unsigned)hdr.reason,
               (unsigned)hdr.mode,
               (double)hdr.ax, (double)hdr.ay, (double)hdr.az,
               (double)hdr.pitch_deg, (double)hdr.roll_deg,
               (long)hdr.pressure_pa,
               (unsigned)hdr.frame_bytes,
               crc_ok ? "OK" : "BAD");
    }
    printf("=== end ===\r\n");
}

/**
 * Вивантаження кадру слота N у hex (для хост-скрипта tools/dump_frame.py).
 * Формат:
 *   FRAME_BEGIN slot=N w=W h=H bytes=B
 *   <рядки по 32 байти hex без пробілів>
 *   FRAME_END ok=1 payload_crc=XXXX
 */
static void cli_dump_frame(uint32_t slot)
{
    if (!s_flash_ok)
    {
        printf("[ERR] Flash not available\r\n");
        return;
    }
    if (slot >= JOURNAL_SLOT_COUNT)
    {
        printf("[ERR] slot must be 0..%u\r\n", (unsigned)(JOURNAL_SLOT_COUNT - 1u));
        return;
    }

    journal_hdr_t hdr;
    uint16_t crc_ok = 0;
    if (!journal_read_hdr(slot, &hdr, &crc_ok))
    {
        printf("[ERR] slot %lu: empty (bad magic)\r\n", (unsigned long)slot);
        return;
    }
    if (!crc_ok)
    {
        /* Старі записи могли мати CRC, порахований інакше — кадр все одно віддаємо */
        printf("[WARN] slot %lu: header CRC mismatch (stored=0x%04X calc=0x%04X) — dump anyway\r\n",
               (unsigned long)slot,
               (unsigned)hdr.crc,
               (unsigned)journal_hdr_crc(&hdr));
    }
    if (hdr.frame_bytes == 0 || hdr.frame_w == 0 || hdr.frame_h == 0)
    {
        printf("[ERR] slot %lu: no frame data (frame_bytes=%u)\r\n",
               (unsigned long)slot, (unsigned)hdr.frame_bytes);
        return;
    }

    const uint32_t frame_addr = JOURNAL_BASE_ADDR + slot * JOURNAL_SLOT_SIZE + 256u;
    printf("FRAME_BEGIN slot=%lu w=%u h=%u bytes=%u\r\n",
           (unsigned long)slot,
           (unsigned)hdr.frame_w,
           (unsigned)hdr.frame_h,
           (unsigned)hdr.frame_bytes);
    fflush(stdout);

    /* Один рядок = 32 байти -> 64 hex-символи + CRLF (не printf на кожен байт) */
    static const char hexdig[] = "0123456789ABCDEF";
    uint8_t page[256];
    char line[64 + 2 + 1];
    uint16_t payload_crc = 0xFFFFu;
    uint32_t off = 0;

    while (off < hdr.frame_bytes)
    {
        uint16_t chunk = (uint16_t)((hdr.frame_bytes - off) > 256u ? 256u : (hdr.frame_bytes - off));
        if (!w25q64_read_data(&s_flash, frame_addr + off, page, chunk))
        {
            printf("FRAME_END ok=0 err=read\r\n");
            return;
        }

        uint16_t i = 0;
        while (i < chunk)
        {
            uint16_t n = (uint16_t)(((chunk - i) > 32u) ? 32u : (chunk - i));
            for (uint16_t k = 0; k < n; k++)
            {
                uint8_t byte = page[i + k];
                payload_crc ^= (uint16_t)byte << 8;
                for (int b = 0; b < 8; b++)
                    payload_crc = (payload_crc & 0x8000u)
                                      ? (uint16_t)((payload_crc << 1) ^ 0x1021u)
                                      : (uint16_t)(payload_crc << 1);
                line[k * 2u] = hexdig[byte >> 4];
                line[k * 2u + 1u] = hexdig[byte & 0x0Fu];
            }
            line[n * 2u] = '\r';
            line[n * 2u + 1u] = '\n';
            line[n * 2u + 2u] = '\0';
            fputs(line, stdout);
            i = (uint16_t)(i + n);
        }
        off += chunk;
    }

    printf("FRAME_END ok=1 payload_crc=%04X\r\n", (unsigned)payload_crc);
    fflush(stdout);
}

static void cli_handle_line(uint32_t now, const char *line)
{
    if (strcmp(line, "help") == 0)
    {
        cli_print_help();
    }
    else if (strcmp(line, "status") == 0)
    {
        printf("status mode=%s sys=%s |a|=%.2fg pitch=%.1f roll=%.1f T=%d.%dC P=%ldPa cam=%d flash=%d jrn=%d\r\n",
               mode_str(s_mode), sys_str(s_sys),
               (double)s_motion.accel.a_total,
               (double)s_pitch, (double)s_roll,
               s_temp_x10 / 10, abs(s_temp_x10 % 10),
               (long)s_pressure_pa,
               s_cam_ok ? 1 : 0,
               s_flash_ok ? 1 : 0,
               (int)s_jrn);
    }
    else if (strcmp(line, "mode cargo") == 0)
    {
        s_mode = MODE_CARGO;
        printf("[OK] mode=cargo\r\n");
    }
    else if (strcmp(line, "mode site") == 0)
    {
        s_mode = MODE_SITE;
        printf("[OK] mode=site\r\n");
    }
    else if (strcmp(line, "arm") == 0)
    {
        enter_armed();
    }
    else if (strcmp(line, "disarm") == 0)
    {
        enter_idle();
    }
    else if (strcmp(line, "dump log") == 0)
    {
        cli_dump_log();
    }
    else if (strncmp(line, "dump frame ", 11) == 0)
    {
        /* Без sscanf (nano.specs інколи HardFault на scanf) */
        const char *p = line + 11;
        unsigned slot = 0;
        int digits = 0;
        while (*p >= '0' && *p <= '9')
        {
            slot = slot * 10u + (unsigned)(*p - '0');
            p++;
            digits++;
        }
        if (digits > 0)
        {
            printf("[ACK] dump frame %u\r\n", slot);
            cli_dump_frame(slot);
        }
        else
            printf("[ERR] usage: dump frame N\r\n");
    }
    else if (strcmp(line, "reset") == 0)
    {
        enter_idle();
        s_jrn = JRN_IDLE;
        s_capture_pending = false;
        s_pending_trig = TRIG_NONE;
        printf("[OK] faults cleared\r\n");
    }
    else if (strcmp(line, "sim incident") == 0)
    {
        raise_incident(now, TRIG_BUTTON);
    }
    else if (line[0] != '\0')
    {
        printf("[ERR] Unknown: '%s' (try help)\r\n", line);
    }
}

static void task_cli(uint32_t now)
{
    if (!s_cli)
        return;

    uint8_t b;
    while (s_cli->read_byte(&b))
    {
        if (b == '\r' || b == '\n')
        {
            if (s_cli_len > 0)
            {
                s_cli_line[s_cli_len] = '\0';
                cli_handle_line(now, s_cli_line);
                s_cli_len = 0;
            }
            continue;
        }
        if (s_cli_len + 1u < CLI_LINE_MAX)
            s_cli_line[s_cli_len++] = (char)b;
    }
}

void app_init(const i2c_interface_t *i2c_bus,
              const sccb_interface_t *sccb_bus,
              const spi_interface_t *spi_bus,
              const pwm_interface_t *pwm_bus,
              const board_io_interface_t *board_io,
              const camera_parallel_interface_t *cam_parallel,
              const uart_rx_interface_t *cli_uart,
              void (*delay_ms)(uint32_t))
{
    s_board = board_io;
    s_cli = cli_uart;

    printf("\r\n=======================================================\r\n");
    printf("  Incident Recorder: Black Box (Lab 3 product)\r\n");
    printf("  Super-loop | QQVGA gray cam | ring journal+CRC\r\n");
    printf("=======================================================\r\n");

    if (servo_init(&s_servo, pwm_bus))
    {
        servo_set_position(&s_servo, SERVO_ANGLE_0);
        printf("[OK]     Servo PWM ready\r\n");
    }

    s_mpu_ok = mpu6050_init(&s_mpu, i2c_bus, MPU6050_DEFAULT_ADDR);
    printf(s_mpu_ok ? "[OK]     MPU-6050 ±8g @100Hz\r\n" : "[WARN]   MPU-6050 init failed\r\n");

    /* Wake GY-89 companions (як у ЛР2) */
    uint8_t l3g_wake = 0x0F;
    i2c_bus->write_reg(0x6B, 0x20, &l3g_wake, 1);
    uint8_t lsm_wake = 0x57;
    i2c_bus->write_reg(0x1D, 0x20, &lsm_wake, 1);

    s_bmp_ok = bmp180_init(&s_bmp, i2c_bus, BMP180_DEFAULT_ADDR, delay_ms);
    printf(s_bmp_ok ? "[OK]     BMP180 ready\r\n" : "[WARN]   BMP180 init failed\r\n");

    uint8_t cam_pid = 0, cam_ver = 0;
    if (ov7670_init(&s_camera, sccb_bus, cam_parallel, OV7670_SCCB_ADDR, delay_ms) &&
        ov7670_read_id(&s_camera, &cam_pid, &cam_ver) && cam_pid == 0x76)
    {
        s_cam_ok = ov7670_configure_qqvga_yuv(&s_camera);
        printf(s_cam_ok ? "[OK]     OV7670 QQVGA YUV configured (PID=0x%02X)\r\n" : "[WARN]   OV7670 config failed\r\n",
               cam_pid);
    }
    else
    {
        printf("[WARN]   OV7670 not detected (PID=0x%02X VER=0x%02X)\r\n", cam_pid, cam_ver);
    }

    uint8_t mfg = 0, type = 0, cap = 0;
    if (w25q64_init(&s_flash, spi_bus) && w25q64_read_jedec_id(&s_flash, &mfg, &type, &cap))
    {
        uint8_t sr1 = 0;
        (void)w25q64_read_status1(&s_flash, &sr1);
        bool test_ok = w25q64_self_test(&s_flash, FLASH_TEST_ADDR);
        s_flash_ok = test_ok;
        printf("[%s]  W25Q64 JEDEC %02X %02X %02X SR1=0x%02X self-test=%s\r\n",
               test_ok ? "OK" : "ERROR",
               mfg, type, cap, sr1,
               test_ok ? "PASS" : "FAIL");
    }
    else
    {
        printf("[ERROR]  Flash init failed\r\n");
    }

    printf("\r\nType 'help'. Button B1 / impact / tilt -> incident.\r\n");
    printf("Default state: IDLE (1 Hz LED). Use 'arm' for Armed mode.\r\n\r\n");
}

void app_run_step(uint32_t current_tick_ms)
{
    task_cli(current_tick_ms);
    task_button(current_tick_ms);
    task_imu(current_tick_ms);
    task_baro(current_tick_ms);
    task_camera(current_tick_ms);
    task_journal(current_tick_ms);
    task_led(current_tick_ms);
}
