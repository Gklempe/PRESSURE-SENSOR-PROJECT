#include "mcc_generated_files/system/system.h"
#include <stdint.h>
#include <stdbool.h>
//#include <stdio.h>
#include "xc.h"
#include "zssc_config.h"
#include "functions.h"

extern uint16_t readReg16(uint8_t addr);
extern uint8_t  readStatus(void);
extern int32_t  readRawSensorMeasurement(void);
extern uint32_t read_raw_temperature(void);
extern void     putS(const char *s);
extern void     decodeSM1(uint16_t sm1);
extern void     decodeSM2(uint16_t sm2);

// Globals to hold the user’s choices:
static uint8_t  g_code1;
static uint8_t  g_code2;
static bool     g_polarity;



// simple blocking input of one line (max 4 chars)
static void UART_ReadLine(char *buf, uint8_t maxlen) {
    uint8_t pos = 0;
    while (1) {
        if (UART1_IsRxReady()) {
            char c = UART1_Read();
            // echo
            UART1_Write(c);
            if ((c == '\r' || c == '\n') && pos > 0) {
                buf[pos] = '\0';
                break;
            }
            if (pos < maxlen-1 && ((c >= '0' && c <= '9')|| c=='-')) {
                buf[pos++] = c;
            }
        }
    }
}

// top‑level menu
static void UART_SetupSM1Fields(void) {
    char line[8], msg[64];
    int  sel, val;
    uint16_t sm1, new_sm1;
    bool ok = false;


    // 1) Show menu
    putS("\r\nEdit SM_Config1 field:\r\n");
    putS(" 1) PGA1 stage      (bits 3:0, code 0..15)\r\n");
    putS(" 2) PGA2 stage      (bits 6:4, code 0..7)\r\n");
    putS(" 3) Gain polarity   (bit  7, 0=pos,1=invert)\r\n");
    putS(" 4) ADC bits        (bits11:8, code 0..15)\r\n");
    putS(" 5) ADC offset      (bits14:12, code 0..7)\r\n");
    putS(" 6) Reference src   (bit 15, 0=bandgap,1=ratiometric)\r\n");
    putS("Select (1-6): ");
    UART_ReadLine(line, sizeof(line));
    sel = atoi(line);

    // 2) Read current register
    sm1 = ZSSC_ReadSMConfig1();

    // 3) Prompt & set
    switch (sel) {
      case 1:
        putS("\r\nEnter PGA1 code (0-15): ");
        UART_ReadLine(line,sizeof(line));
        val = atoi(line);
        if (val<0||val>15) { putS("Bad code\r\n"); return; }
        //sprintf(msg,"Setting Gain_stage1 code=%d\r\n", val); putS(msg);
        putS("Setting Gain_stage1 code=");
        putDec16(val);
        putS("\r\n");
        ok = ZSSC_SetGainStage1((uint8_t)val);
        break;

      case 2:
        putS("\r\nEnter PGA2 code (0-7): ");
        UART_ReadLine(line,sizeof(line));
        val = atoi(line);
        if (val<0||val>7) { putS("Bad code\r\n"); return; }
        //sprintf(msg,"Setting Gain_stage2 code=%d\r\n", val); putS(msg);
        putS("Setting Gain_stage2 code=");
        putDec16(val);
        putS("\r\n");
        ok = ZSSC_SetGainStage2((uint8_t)val);
        break;

      case 3:
        putS("\r\nEnter polarity (0=pos,1=invert): ");
        UART_ReadLine(line,sizeof(line));
        val = atoi(line);
        if (val!=0&&val!=1){ putS("Bad val\r\n"); return; }
        //sprintf(msg,"Setting Gain_polarity=%d\r\n", val); putS(msg);
        putS("Setting Gain_polarity code=");
        putDec16(val);
        putS("\r\n");
        ok = ZSSC_SetGainPolarity(val!=0);
        break;

      case 4:
        putS("\r\nEnter ADC bits code (0-15): ");
        UART_ReadLine(line,sizeof(line));
        val = atoi(line);
        if (val<0||val>15){ putS("Bad code\r\n"); return; }
        //sprintf(msg,"Setting adc_bits code=%d\r\n", val); putS(msg);
        putS("Setting adc_bits code=");
        putDec16(val);
        putS("\r\n");
        ok = ZSSC_SetAdcBits((uint8_t)val);
        break;

      case 5:
        putS("\r\nEnter ADC offset code (0-7): ");
        UART_ReadLine(line,sizeof(line));
        val = atoi(line);
        if (val<0||val>7){ putS("Bad code\r\n"); return; }
        //sprintf(msg,"Setting adc_offset code=%d\r\n", val); putS(msg);
        putS("Setting adc_offset code=");
        putDec16(val);
        putS("\r\n");
        ok = ZSSC_SetAdcOffset((uint8_t)val);
        break;

      case 6:
        putS("\r\nEnter Reference src (0=bandgap,1=ratiometric): ");
        UART_ReadLine(line,sizeof(line));
        val = atoi(line);
        if (val!=0&&val!=1){ putS("Bad val\r\n"); return; }
        //sprintf(msg,"Setting sel_ref1=%d\r\n", val); putS(msg);
        putS("Setting sel_ref1 code=");
        putDec16(val);
        putS("\r\n");
        ok = ZSSC_SetRefSource(val!=0);
        break;

      default:
        putS("\r\nInvalid selection\r\n");
        return;
    }

    if (!ok) {
        putS("ERROR writing NVM\r\n");
        return;
    }

    // 5) Read back & show whole SM_Config1
    new_sm1 = ZSSC_ReadSMConfig1();
    //sprintf(msg,"\r\nNew SM_Config1 = 0x%04X\r\n", new_sm1); putS(msg);
    putS("\r\nNew SM_Config1 = 0x");
    putHex16(new_sm1);
    putS("\r\n");
    decodeSM1(new_sm1);
    
//    // 7) Read & decode SM2
//    uint16_t sm2 = readReg16(0x15);
//    sprintf(msg, "SM2 = 0x%04X\r\n", sm2);
//    putS(msg);
//    decodeSM2(sm2);
}

static void UART_SetupSM2Fields(void) {
    char line[8], msg[64];
    int sel, val;
    bool ok = false;

    putS("\r\nEdit SM_Config2 field:\r\n");
    putS(" 1) IOFFSC       (0-31)\r\n");
    putS(" 2) Tbiasout     (0-7)\r\n");
    putS(" 3) ADC_en_shift (0=off,1=on)\r\n");
    putS(" 4) PGA_en_shift (0=off,1=on)\r\n");
    putS("Select (1-4): ");
    UART_ReadLine(line,sizeof(line));
    sel = atoi(line);

    switch(sel) {
      case 1:
        putS("\r\nEnter IOFFSC code (0-31): ");
        UART_ReadLine(line,sizeof(line));
        val = atoi(line);
        if (val<0||val>31){ putS("Bad code\r\n"); return; }
        putS("Setting IOFFSC = "); putDec16(val); putS("\r\n");
        ok = ZSSC_SetIoffsc((uint8_t)val);
        break;
      case 2:
        putS("\r\nEnter Tbiasout code (0-7): ");
        UART_ReadLine(line,sizeof(line));
        val = atoi(line);
        if (val<0||val>7){ putS("Bad code\r\n"); return; }
        putS("Setting Tbiasout = "); putDec16(val); putS("\r\n");
        ok = ZSSC_SetTbiasout((uint8_t)val);
        break;
      case 3:
        putS("\r\nEnter ADC_en_shift (0=off,1=on): ");
        UART_ReadLine(line,sizeof(line));
        val = atoi(line);
        if (val!=0&&val!=1){ putS("Bad val\r\n"); return; }
        putS("Setting ADC_en_shift = "); putDec16(val); putS("\r\n");
        ok = ZSSC_SetAdcEnShift(val!=0);
        break;
      case 4:
        putS("\r\nEnter PGA_en_shift (0=off,1=on): ");
        UART_ReadLine(line,sizeof(line));
        val = atoi(line);
        if (val!=0&&val!=1){ putS("Bad val\r\n"); return; }
        putS("Setting PGA_en_shift = "); putDec16(val); putS("\r\n");
        ok = ZSSC_SetPgaEnShift(val!=0);
        break;
      default:
        putS("\r\nInvalid selection\r\n");
        return;
    }

    if (!ok) {
        putS("ERROR writing SM_Config2!\r\n");
        return;
    }

    uint16_t v = ZSSC_ReadSMConfig2();
    putS("\r\nNew SM_Config2 = 0x"); putHex16(v); putS("\r\n");
    decodeSM2(v);
}

void Initialize (void) {
    // sanity-check UART
    putS("Hello, UART!\r\n");
    putS("Probing 0x"); putHex16(SLAVE7); putS("...\r\n");

    // — now carve out RC2/RC4 for bit-bang I²C —
    ANSELCbits.ANSELC2 = 0;
    ANSELCbits.ANSELC4 = 0;
    WPUCbits.WPUC2   = 1;
    WPUCbits.WPUC4   = 1;
    ODCONCbits.ODCC2  = 1;
    ODCONCbits.ODCC4  = 1;
    LATCbits.LATC2   = 0;
    LATCbits.LATC4   = 0;
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC4 = 1;
    // clear any stray PPS
    RC2PPS     = 0x1E;
    RC4PPS     = 0x1F;
    I2C1SCLPPS = 0x12;
    I2C1SDAPPS = 0x14;

    // — power up EVB via RD1, then your I²C loop —
    TRISDbits.TRISD1 = 0;
    LATDbits.LATD1   = 1;
    __delay_ms(10);
}

static void memory_error_check(void) {
    putS("Waiting for Memory‑Error=0...\r\n");
    while (1) {
        uint8_t st = readStatus();
        putS("STATUS=0x");
        putHex8cr(st);
        putS("\r\n");
        if ((st & (1<<2)) == 0) break;
        __delay_ms(50);
    }
    putS("Memory‑Error cleared by GUI!\r\n");
}

static void putDec32(int32_t v) {
    char buf[12];
    bool neg = false;
    if (v < 0) { neg = true; v = -v; }
    int i = 0;
    if (v == 0) buf[i++] = '0';
    else {
        char tmp[12];
        int t = 0;
        while (v) {
            tmp[t++] = '0' + (v % 10);
            v /= 10;
        }
        while (t--) buf[i++] = tmp[t];
    }
    if (neg) {
        // shift right one, insert '-'
        for (int j = i; j > 0; --j) buf[j] = buf[j-1];
        buf[0] = '-';
        i++;
    }
    buf[i] = '\0';
    putS(buf);
}

static void put_mV3(float volts) {
    float   mv = volts * 1000.0f;
    int32_t W  = (int32_t)(mv + (mv >= 0 ? 0.5f : -0.5f));
    int32_t F  = abs(W % 1000);
    W /= 1000;
    putDec32(W);
    putS(".");
    if (F < 100) putS("0");
    if (F <  10) putS("0");
    putDec32(F);
    putS(" mV");
}

// ——— Measurement loop ——————————————————————————
static void MeasurementLoop(void) {
    // 1) grab the PGA & ADC settings once
    uint16_t sm1    = ZSSC_ReadSMConfig1();
    uint16_t sm2    = ZSSC_ReadSMConfig2();

    // 2) decode PGA gains
    uint8_t  g1c =  sm1       & 0x0F;
    uint8_t  g2c = (sm1 >> 4) & 0x07;
    float    G1  = PGA1_gain_val[g1c];
    float    G2  = PGA2_gain_val[g2c];

    // 3) IOFFSC compensation
    uint8_t  offc  =  sm2       & 0x1F;
    float    Voff  = ioffsc_volt[offc];

    // 4) ADC gain‑shift?
    bool     adcEn = ((sm2 >> 8) & 1) != 0;
    float    pipG  = G1 * G2 * (adcEn ? 2.0f : 1.0f);

    // 5) full‑scale counts on the 24‑bit bus
    const float FS = (1UL<<23) - 1;   // 0x00FF FF FF

    putS("\r\n--- AFE Settings ---\r\n");
    putS(" G1           = "); putDec16((int)(G1*100)); putS(" /100\r\n");
    putS(" G2           = "); putDec16((int)(G2*100)); putS(" /100\r\n");
    putS(" Pipeline G   = "); putDec16((int)(pipG*100)); putS(" /100\r\n");
    putS(" IOFFSC [µV]  = "); putDec32((int32_t)(Voff*1e6f)); putS("\r\n");
    putS(" ADC_en_shift = "); putDec16(adcEn); putS("\r\n");
    putS("---------------------\r\n");

    while (1) {
        // if user types 'r', go back out to re‑config
        if (UART1_IsRxReady()) {
            char c = UART1_Read();
            if ((c|0x20) == 'r') { putS("\r\n< re‑configuring >\r\n"); break; }
        }

        // --- 1) fetch 24‑bit *raw* measurement ---
        i2c_start();
          i2c_write_byte_ack((SLAVE7<<1)|0);
          i2c_write_byte_ack(0xA3);
        i2c_stop();

        i2c_start();
          i2c_write_byte_ack((SLAVE7<<1)|1);
          uint8_t b1 = i2c_read_byte(false);
          uint8_t b2 = i2c_read_byte(false);
          uint8_t b3 = i2c_read_byte(true);
        i2c_stop();

        uint32_t raw24 = ((uint32_t)b1<<16) | ((uint32_t)b2<<8) | b3;

        // 2) offset‑binary → signed
        int32_t signed24 = (int32_t)raw24;
        if (signed24 & 0x800000) signed24 |= 0xFF000000;

        // 3) V_ADC [V]
        float Vadc  = ((float)signed24/FS) * VDD_BRIDGE;

        // 4) back‑calculate Vdiff
        float Vdiff = (Vadc + G2*Voff)/pipG - Voff;

        // 5) print it all
        putS("raw24=0x");      putHex24(raw24);
        putS("   signed=");    putDec32(signed24);
        putS("   Vadc=");       put_mV3(Vadc);
        putS("   Vdiff=");      put_mV3(Vdiff);
        putS("\r\n");

        __delay_ms(2000);
    }
}

void main(void) {
    // — bring up everything MCC cares about —
    SYSTEM_Initialize();

    // — now turn on interrupts for UART1 —
    INTERRUPT_GlobalInterruptEnable();
    
    Initialize();
    
    scan_i2c();
    
    I2C1_Initialize();
    
    memory_error_check();
     
    
    while (1) {
        char line[4];
        putS("\r\n=== ZSSC3241 Config Editor ===\r\n");
        putS("  1) Edit SM_Config1\r\n");
        putS("  2) Edit SM_Config2\r\n");
        putS("Select register: ");
        UART_ReadLine(line,sizeof(line));
        int which = atoi(line);

        if (which == 1) {
            UART_SetupSM1Fields();
        }
        else if (which == 2) {
            UART_SetupSM2Fields();
        }
        else {
            putS("Invalid choice\r\n");
            continue;
        }

        // after editing either register, go back into measurement
        MeasurementLoop();  // press 'r' in here to return and re-edit
    }
}