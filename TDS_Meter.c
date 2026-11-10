// DIY TDS meter for the Raspberry Pi Pico
//
// Wiring:
//   GP16 --[1k resistor]  --+-- electrode 1 (in water)
//                         |
//   GP26 (ADC0) ----------+
//
//   GP17 ------------------- electrode 2 (in water)
//
// Temperature probe (NTC thermistor, e.g. a meat thermometer probe):
//   3V3 (pin 36) --[10k resistor]--+-- probe wire 1
//                                  |
//   GP27 (ADC1) -------------------+
//   GND (pin 33) ------------------- probe wire 2
//
// LCD1602 with I2C backpack:
//   GND -> GND,  VCC -> 3V3 (pin 36) or VBUS (pin 40) if the text is too faint
//   SDA -> GP4 (pin 6),  SCL -> GP5 (pin 7)
//
// The probe is driven with alternating current (polarity flips every
// sample) so the electrodes don't corrode and the reading doesn't drift.
//
// Calibrate: put the probe in water with a known TDS, then type in the
// serial monitor:  c342  (replace 342 with the known ppm) and press Enter.

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

#define DRIVE_A_PIN   16
#define DRIVE_B_PIN   17
#define SENSE_PIN     26
#define SENSE_ADC     0

#define THERM_PIN     27
#define THERM_ADC     1
#define USE_THERMISTOR 1      // 0 = assume 25 C

#define SERIES_OHMS   1000.0f // resistor between GP16 and the probe (brown-black-black-brown-brown)
#define THERM_SERIES  10000.0f
#define THERM_NOMINAL 100000.0f // probe resistance at 25 C (most meat probes are 100k)
#define THERM_BETA    3950.0f

#define LCD_I2C       i2c0
#define LCD_SDA_PIN   4
#define LCD_SCL_PIN   5

#define TDS_FACTOR    0.5f    // ppm per uS/cm (the usual "500 scale")

// Cell constant in 1/cm. Depends on electrode size and spacing.
// Calibrate once, then paste the printed value here so it survives a reboot.
static float cell_k = 0.289f; // calibrated: screw probe, two points (see below)

// Extra resistance in the probe wiring (contacts at the screws etc.).
// It is subtracted from every measurement. Found from two known waters:
// tap = 152 ppm (water report) and tap + 1 g/L salt = 1152 ppm.
#define PROBE_OFFSET_OHMS 332.0f

// Raw sense readings from the last measurement (0.0 - 1.0), for debugging
static float last_r1, last_r2;

// Temperature probe resistance from the last reading, or negative if not connected
static float last_therm_ohms = -1.0f;

static float read_adc_ratio(uint input)
{
    adc_select_input(input);
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += adc_read();
    }
    return (sum / 8.0f) / 4095.0f;
}

// Returns water resistance in ohms, or a negative value if the probe is dry.
static float measure_water_ohms(void)
{
    float total = 0.0f, sum1 = 0.0f, sum2 = 0.0f;
    const int cycles = 64;

    for (int i = 0; i < cycles; i++) {
        // Phase 1: current flows GP16 -> water -> GP17
        gpio_put(DRIVE_A_PIN, 1);
        gpio_put(DRIVE_B_PIN, 0);
        sleep_us(50);
        float r1 = read_adc_ratio(SENSE_ADC);

        // Phase 2: reversed, GP17 -> water -> GP16
        gpio_put(DRIVE_A_PIN, 0);
        gpio_put(DRIVE_B_PIN, 1);
        sleep_us(50);
        float r2 = read_adc_ratio(SENSE_ADC);

        // Both phases give water/(resistor+water); average them
        total += (r1 + (1.0f - r2)) / 2.0f;
        sum1 += r1;
        sum2 += r2;
    }

    // Stop driving the probe between measurements
    gpio_put(DRIVE_A_PIN, 0);
    gpio_put(DRIVE_B_PIN, 0);

    last_r1 = sum1 / cycles;
    last_r2 = sum2 / cycles;

    float r = total / cycles;
    if (r > 0.995f) {
        return -1.0f;
    }
    return SERIES_OHMS * r / (1.0f - r);
}

static float read_temperature_c(void)
{
#if USE_THERMISTOR
    float r = read_adc_ratio(THERM_ADC);
    if (r < 0.02f || r > 0.98f) {
        last_therm_ohms = -1.0f;   // open or shorted, so assume 25 C
        return 25.0f;
    }
    float ohms = THERM_SERIES * r / (1.0f - r);
    last_therm_ohms = ohms;
    float kelvin = 1.0f / (1.0f / 298.15f + logf(ohms / THERM_NOMINAL) / THERM_BETA);
    return kelvin - 273.15f;
#else
    return 25.0f;
#endif
}

// Conductivity in uS/cm, corrected to 25 C
static float conductivity_us(float water_ohms, float temp_c)
{
    float ec = cell_k / water_ohms * 1e6f;
    return ec / (1.0f + 0.02f * (temp_c - 25.0f));
}

// ---- LCD1602 over an I2C backpack (PCF8574) ----
// Backpack bits: P0 = RS, P2 = EN, P3 = backlight, P4-P7 = LCD data D4-D7

#define LCD_RS        0x01
#define LCD_ENABLE    0x04
#define LCD_BACKLIGHT 0x08

static uint8_t lcd_addr = 0;   // 0 means no LCD found

static void lcd_write_byte(uint8_t val)
{
    i2c_write_timeout_us(LCD_I2C, lcd_addr, &val, 1, false, 1000);
}

static void lcd_send_nibble(uint8_t nibble, uint8_t mode)
{
    uint8_t val = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    lcd_write_byte(val | LCD_ENABLE);
    sleep_us(1);
    lcd_write_byte(val);
    sleep_us(50);
}

static void lcd_send(uint8_t val, uint8_t mode)
{
    lcd_send_nibble(val & 0xF0, mode);
    lcd_send_nibble((uint8_t)(val << 4), mode);
}

static void lcd_cmd(uint8_t cmd)
{
    lcd_send(cmd, 0);
    if (cmd < 4) {
        sleep_ms(2);   // clear and home are slow
    }
}

// Returns true if an LCD answered on the I2C bus
static bool lcd_init(void)
{
    i2c_init(LCD_I2C, 100 * 1000);
    gpio_set_function(LCD_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(LCD_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(LCD_SDA_PIN);
    gpio_pull_up(LCD_SCL_PIN);

    // Look for the backpack on every possible address (usually 0x27 or 0x3F).
    // Writing the backlight bit keeps the backlight on while we search.
    lcd_addr = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t val = LCD_BACKLIGHT;
        if (i2c_write_timeout_us(LCD_I2C, addr, &val, 1, false, 2000) == 1) {
            lcd_addr = addr;
            break;
        }
    }
    if (lcd_addr == 0) {
        return false;
    }

    // Standard sequence to put the display into 4-bit mode
    sleep_ms(50);
    lcd_send_nibble(0x30, 0);
    sleep_ms(5);
    lcd_send_nibble(0x30, 0);
    sleep_us(150);
    lcd_send_nibble(0x30, 0);
    lcd_send_nibble(0x20, 0);

    lcd_cmd(0x28);   // 2 lines, 5x8 font
    lcd_cmd(0x0C);   // display on, no cursor
    lcd_cmd(0x06);   // move right after each character
    lcd_cmd(0x01);   // clear
    return true;
}

// Writes a whole row (0 or 1), padding with spaces so old text is erased
static void lcd_print_row(int row, const char *text)
{
    if (lcd_addr == 0) {
        return;
    }
    lcd_cmd(row == 0 ? 0x80 : 0xC0);
    for (int i = 0; i < 16; i++) {
        char c = *text ? *text++ : ' ';
        lcd_send((uint8_t)c, LCD_RS);
    }
}

// Reads a line like "c342" from the serial monitor without blocking
static void check_for_calibration(float water_ohms, float temp_c)
{
    static char line[16];
    static int len = 0;

    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == '\r' || ch == '\n') {
            line[len] = '\0';
            len = 0;
            if (line[0] == 'c' || line[0] == 'C') {
                float known_ppm = strtof(line + 1, NULL);
                if (known_ppm <= 0.0f || water_ohms < 0.0f) {
                    printf("Calibration failed: put the probe in the solution and type e.g. c342\n");
                    return;
                }
                float target_ec = known_ppm / TDS_FACTOR;
                float temp_factor = 1.0f + 0.02f * (temp_c - 25.0f);
                cell_k = target_ec * temp_factor * water_ohms / 1e6f;
                printf("Calibrated! cell_k = %.4f  (paste this into TDS_Meter.c)\n", cell_k);
            }
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = (char)ch;
        }
    }
}

int main()
{
    stdio_init_all();

    gpio_init(DRIVE_A_PIN);
    gpio_set_dir(DRIVE_A_PIN, GPIO_OUT);
    gpio_init(DRIVE_B_PIN);
    gpio_set_dir(DRIVE_B_PIN, GPIO_OUT);

    adc_init();
    adc_gpio_init(SENSE_PIN);
    adc_gpio_init(THERM_PIN);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    bool have_lcd = lcd_init();
    if (have_lcd) {
        lcd_print_row(0, "TDS meter");
        lcd_print_row(1, "Starting...");
    }

    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        float water_ohms = measure_water_ohms();
        if (water_ohms >= 0.0f) {
            water_ohms -= PROBE_OFFSET_OHMS;
            if (water_ohms < 1.0f) {
                water_ohms = 1.0f;
            }
        }
        float temp_c = read_temperature_c();
        gpio_put(PICO_DEFAULT_LED_PIN, 0);

        char row0[17], row1[17];
        if (water_ohms < 0.0f) {
            printf("Probe dry / not connected\n");
            snprintf(row0, sizeof(row0), "TDS:  --- ppm");
            snprintf(row1, sizeof(row1), "Probe dry");
        } else {
            float ec = conductivity_us(water_ohms, temp_c);
            printf("TDS: %6.0f ppm   EC: %6.0f uS/cm   water: %7.0f ohm   temp: %.1f C\n",
                   ec * TDS_FACTOR, ec, water_ohms, temp_c);
            snprintf(row0, sizeof(row0), "TDS: %5.0f ppm", ec * TDS_FACTOR);
            snprintf(row1, sizeof(row1), "EC:%5.0fuS %4.1fC", ec, temp_c);
        }

        printf("   raw: phase1 %.3f  phase2 %.3f\n", last_r1, last_r2);
        if (last_therm_ohms < 0.0f) {
            printf("   temp probe: not connected (using 25 C)\n");
        } else {
            printf("   temp probe: %.0f ohm\n", last_therm_ohms);
        }

        if (have_lcd) {
            printf("   LCD found at address 0x%02X\n", lcd_addr);
            lcd_print_row(0, row0);
            lcd_print_row(1, row1);
        } else {
            printf("   LCD not found on GP4/GP5 - retrying\n");
            have_lcd = lcd_init();
        }

        check_for_calibration(water_ohms, temp_c);
        sleep_ms(1000);
    }
}
