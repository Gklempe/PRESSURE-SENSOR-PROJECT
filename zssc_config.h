#ifndef ZSSC_CONFIG_H
#define ZSSC_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "functions.h"

// I2C & ZSSC command definitions (same as in main.c)

#define SLAVE7         0x28
#define I2C_DELAY_US   5
#define NVM_START      0x40
#define NVM_COUNT      54      // 0x75 − 0x40 + 1

//#define FS_COUNTS  ((1UL<<23) - 1UL)
#define VDD_BRIDGE       1.75f          // excitation voltage in volts

// NVM write command & target address
#define CMD_WRITE_NVM    0xC0
#define ADDR_SM_CONFIG1  0x14
#define ADDR_SM_CONFIG2   0x15    // NVM address for SM_Config2

#define CMD_START_CMD     0xA9   // Enter Command Mode
#define CMD_WRITE_CRC     0x90   // Write new CRC into reg 0x35

#define CMD_MEM_READ_BASE    0x00    // to 0x3F → reads NVM  

// — small UART helpers —
static void putS(const char *s) {
    while (*s) {
        while (!UART1_IsTxReady());
        UART1_Write((uint8_t)*s++);
    }
}

static void putDec16(uint16_t v) {
    char buf[6];
    int  i = 0;
    if (v == 0) {
        buf[i++] = '0';
    } else {
        char tmp[6];
        int  t = 0;
        while (v) {
            tmp[t++] = '0' + (v % 10);
            v /= 10;
        }
        while (t--) buf[i++] = tmp[t];
    }
    buf[i] = '\0';
    putS(buf);
}

// — lookup tables: numeric values rather than strings —
static const float PGA1_gain_val[16] = {
    1.2f,  2.0f,  4.0f,  6.0f, 
   12.0f, 20.0f, 30.0f, 40.0f, 
   60.0f, 80.0f,120.0f,150.0f, 
  200.0f,240.0f,300.0f,   0.0f   // last entry (“n/a”) we treat as 0.0
};
static const float PGA2_gain_val[8]  = {
    1.1f, 1.2f, 1.3f, 1.4f, 
    1.5f, 1.6f, 1.7f, 1.8f
};

static const char * const PGA1_gain_str[16] = {
    "1.2","2","4","6","12","20","30","40",
    "60","80","120","150","200","240","300","n/a"
};

static const char * const PGA2_gain_str[8] = {
    "1.1","1.2","1.3","1.4","1.5","1.6","1.7","1.8"
};

static const char * const polarity_str[2] = {
    "positive","negative"
};

static const char * const adc_bits[16] = {
    "12","13","14","15","16","17","18","19",
    "20","21","22","23","24","n/a","n/a","n/a"
};

static const uint8_t adc_nbits[16] = {
/*0*/ 12,13,14,15,16,17,18,19,
/*8*/ 20,21,22,23,24,24,24,24   // protect against 0xD..F
};

static const char * const adc_offset[8] = {
    "0%","6.25%","12.5%","18.75%","25%","31.25%","37.5%","44%"
};

static const char * const sel_ref1[2] = {
    "bandgap","ratiometric"
};

static const char * const ioffsc[32] = {
    "0mV","-1mV","-2mV","-3mV","-4mV","-5mV","-6mV","-7mV",
    "-8mV","-9mV","-10mV","-11mV","-12mV","-13mV","-14mV","-15mV",
    "0mV","1mV","2mV","3mV","4mV","5mV","6mV","7mV",
    "8mV","9mV","10mV","11mV","12mV","13mV","14mV","15mV"
};

static const float ioffsc_volt[32] = {
  0.000f, -0.001f, -0.002f, -0.003f, -0.004f, -0.005f, -0.006f, -0.007f,
  -0.008f, -0.009f, -0.010f, -0.011f, -0.012f, -0.013f, -0.014f, -0.015f,
   0.000f,  0.001f,  0.002f,  0.003f,  0.004f,  0.005f,  0.006f,  0.007f,
   0.008f,  0.009f,  0.010f,  0.011f,  0.012f,  0.013f,  0.014f,  0.015f
};

static const char * const Tbiasout[8] = {
    "5uA","10uA","20uA","39uA","79uA","157uA","196uA","494uA"
};

static const char * const adc_en_shift[2] = {
    "gain/offset off","gain/offset on"
};

static const char * const pga_en_shift[2] = {
    "CM Adjustment off","CM Adjustment on"
};



static void putHex8cr(uint8_t v) {
    static const char H[] = "0123456789ABCDEF";
    while (!UART1_IsTxReady()); UART1_Write(H[v>>4]);
    while (!UART1_IsTxReady()); UART1_Write(H[v & 0xF]);
    while (!UART1_IsTxReady()); UART1_Write('\r');
    while (!UART1_IsTxReady()); UART1_Write('\n');
}

static void putHex16(uint16_t w) {
    static const char hex[] = "0123456789ABCDEF";
    for (int shift = 12; shift >= 0; shift -= 4) {
        while (!UART1_IsTxReady());
        UART1_Write((uint8_t)hex[(w >> shift) & 0xF]);
    }
    // CRLF
    while (!UART1_IsTxReady()); 
    UART1_Write('\r');
    while (!UART1_IsTxReady()); 
    UART1_Write('\n');
}

// ————— Put 6‑digit hex (24‑bit) —————
static void putHex24(uint32_t v) {
    const char hex[] = "0123456789ABCDEF";
    for (int shift=20; shift>=0; shift-=4) {
        while (!UART1_IsTxReady());
        UART1_Write(hex[(v>>shift)&0xF]);
    }
}

// --- decode & print SM_Config1 fields ---
static void decodeSM1(uint16_t sm1) {
    uint8_t p1    = sm1        & 0x0F;
    uint8_t p2    = (sm1 >> 4) & 0x07;
    uint8_t pol   = (sm1 >> 7) & 0x01;
    uint8_t bits  = (sm1 >> 8) & 0x0F;
    uint8_t off   = (sm1 >> 12)& 0x07;
    uint8_t ref   = (sm1 >> 15)& 0x01;

    putS("SM_Config1 = 0x");     putHex16(sm1); putS("\r\n");

    putS("  PGA1 gain code = 0x");   putHex8cr(p1); putS(" -> "); putS(PGA1_gain_str[p1]); putS("\r\n");
    putS("  PGA2 gain code = 0x");   putHex8cr(p2); putS(" -> "); putS(PGA2_gain_str[p2]); putS("\r\n");

    // total gain
    {
        float tot = PGA1_gain_val[p1] * PGA2_gain_val[p2];
        // convert to m×100 for two‑decimal fixed print
        uint32_t m100 = (uint32_t)(tot * 100.0f + 0.5f);
        putS("  Total gain    = ");
        putDec16(m100/100);                // integer part
        putS(".");
        uint16_t frac = m100 % 100;
        if (frac < 10) putS("0");
        putDec16(frac);                    // two‑digit fraction
        putS("\r\n");
    }

    putS("  Polarity      = "); putS(polarity_str[pol]); putS("\r\n");
    putS("  ADC Bits      = "); putS(adc_bits[bits]);    putS("\r\n");
    putS("  ADC Offset    = "); putS(adc_offset[off]);   putS("\r\n");
    putS("  ADC Reference = "); putS(sel_ref1[ref]);     putS("\r\n");
}

// --- decode & print SM_Config2 fields ---
static void decodeSM2(uint16_t sm2) {
    uint8_t IOFFSC     =  sm2        & 0x1F;      
    uint8_t TBIASout   = (sm2 >> 5)  & 0x07;      
    uint8_t ADC_en_shift   = (sm2 >> 8)  & 0x01;      
    uint8_t PGA_en_shift   = (sm2 >> 9) & 0x01;
    

    putS("SM_Config2 = 0x");    putHex16(sm2);
    putS("  IOFFSC                  = ");    putS(ioffsc[IOFFSC]); putS("\r\n");
    
    putS("  Tbiasout                = ");   putS(Tbiasout[TBIASout]); putS("\r\n");
    
    putS("  ADC Enable Shift        = ");   putS(adc_en_shift[ADC_en_shift]); putS("\r\n");
    
    putS("  PGA Enable Shift        = ");   putS(pga_en_shift[PGA_en_shift]); putS("\r\n");
}

static void scan_i2c(void) {
    putS("Scanning I2C bus:\r\n");
    for (uint8_t addr = 1; addr < 0x78; addr++) {
        i2c_start();
        bool ack = i2c_write_byte_ack((addr<<1)|0);
        i2c_stop();
        if (ack) {
            putS("  Found ACK at 0x");
            putHex8cr(addr);
            putS("\r\n");
        }
    }
    putS("Scan complete.\r\n");
}



// — Read STATUS via direct-command 0x00 (returns two-byte, STATUS in high byte) —
static uint8_t readStatus(void) {
    i2c_start();
      i2c_write_byte_ack((SLAVE7<<1)|0);
      i2c_write_byte_ack(0x00);
    i2c_start();
      i2c_write_byte_ack((SLAVE7<<1)|1);
      uint8_t hi = i2c_read_byte(true);
    i2c_stop();
    return hi;
}

// — read back a 16-bit register (MSB then LSB) —
static uint16_t readReg16(uint8_t ptr) {
    uint8_t hi, lo, status;
    i2c_start();
      i2c_write_byte_ack((SLAVE7<<1)|0);
      //i2c_write_byte_ack(0xC0);             // Read‑NVM command
      i2c_write_byte_ack(ptr);
      i2c_stop();
      __delay_ms(10);
    i2c_start();
      i2c_write_byte_ack((SLAVE7<<1)|1);
      status = i2c_read_byte(false);  // *first* byte = STATUS (ACK)
      hi     = i2c_read_byte(false);
      lo     = i2c_read_byte(true);
    i2c_stop();
    return ((uint16_t)hi << 8) | lo;
}

static int32_t readRawSensorMeasurement(void){
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

    int32_t v = ((int32_t)b1<<16) | ((int32_t)b2<<8) | b3;
    
    /*  KEEP THIS!  ------------------------------------------------- */
    if (v & 0x00800000)          // bit‑23 set?  (negative value)
        v |= 0xFF000000;         // sign–extend 24‑>32 bits
    /*  ------------------------------------------------------------- */
    
    return v;
}

static uint32_t read_raw_temperature(void) 
{
    // 1) send the A4h command
    i2c_start();
      i2c_write_byte_ack((SLAVE7<<1) | 0); // SLA+W
      i2c_write_byte_ack(0xA4);               // Raw‑Temp command
    i2c_stop();

    __delay_ms(1);  // tconv ≈ 500μs…1ms

    // 2) now read 3 bytes (MSB→LSB)
    i2c_start();
      i2c_write_byte_ack((SLAVE7<<1) | 1); // SLA+R
      uint8_t b1 = i2c_read_byte(false);       // ACK after MSB
      uint8_t b2 = i2c_read_byte(false);       // ACK after mid‑byte
      uint8_t b3 = i2c_read_byte(true);        // NACK after LSB
    i2c_stop();

    // 3) assemble into a 24‑bit word
    return ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;
}

// Send a *single‑byte* command (no addr, no data) to the SSC
static void ZSSC_SendCmd(uint8_t cmd) {
    i2c_start();
      i2c_write_byte_ack((SLAVE7<<1)|0);  // SLA+W
      i2c_write_byte_ack(cmd);            // e.g. 0xA9 or 0x39
    i2c_stop();
    // tCMD ≈ 1 ms or so; give a little margin
    __delay_ms(5);
}

//---------------------------------------------------------------------------
// Write a 16‑bit value into the ZSSC’s NVM at register ‘addr’.
// Returns true if the device ack’d all bytes.
//---------------------------------------------------------------------------
static bool ZSSC_WriteReg16_NVM(uint8_t addr, uint16_t word)
{
    ZSSC_SendCmd(CMD_START_CMD);
    
    uint8_t hi = (word >> 8) & 0xFF;
    uint8_t lo =  word       & 0xFF;
    uint8_t wcmd = (uint8_t)(0x40 + (addr & 0x3F));
    
    i2c_start();
      // SLA+W
      i2c_write_byte_ack((SLAVE7 << 1) | 0);
      i2c_write_byte_ack(wcmd);
      i2c_write_byte_ack(hi);
      i2c_write_byte_ack(lo);
    i2c_stop();
    
    // 2) Wait for NVM‐write to finish (STATUS.RDY bit = 0)
    while (readStatus() & 0x01) {
        __delay_ms(5);
    }
    
    // 3) Trigger CRC calculation & store: CMD = 0x90
    i2c_start();
      i2c_write_byte_ack((SLAVE7<<1) | 0);
      i2c_write_byte_ack(CMD_WRITE_CRC);
    i2c_stop();
    
    // 4) Wait again for the CRC‐write to finish (RDY = 0)
    while (readStatus() & 0x01) {
        __delay_ms(5);
    }

    return true;
}

//---------------------------------------------------------------------------
// Read back a 16‑bit register from SRAM/NVM to verify.
//---------------------------------------------------------------------------
static uint16_t ZSSC_ReadReg16(uint8_t addr)
{
    uint8_t hi, lo;

    i2c_start();
     i2c_write_byte_ack((SLAVE7 << 1) | 0);
     i2c_write_byte_ack(addr);
    i2c_stop();

    __delay_ms(10);

    i2c_start();
      i2c_write_byte_ack((SLAVE7 << 1) | 1);
      hi = i2c_read_byte(false);
      lo = i2c_read_byte(true);
    i2c_stop();

    return ((uint16_t)hi << 8) | lo;
}

// Read the current SM_Config1 word from SSC (SRAM or NVM)
static inline uint16_t ZSSC_ReadSMConfig1(void) {
    return readReg16(ADDR_SM_CONFIG1);
}

// Write a full 16-bit SM_Config1 into NVM (enter cmd mode, write, CRC)
static inline bool ZSSC_WriteSMConfig1(uint16_t new_sm1) {
    // 1) Enter Command Mode
    ZSSC_SendCmd(CMD_START_CMD);
    // 2) Write new value into NVM
    uint8_t hi = (new_sm1 >> 8) & 0xFF;
    uint8_t lo = new_sm1 & 0xFF;
    uint8_t wcmd = (uint8_t)(0x40 + (ADDR_SM_CONFIG1 & 0x3F));
    
    i2c_start();
      i2c_write_byte_ack((SLAVE7 << 1) | 0);
      i2c_write_byte_ack(wcmd);
      i2c_write_byte_ack(hi);
      i2c_write_byte_ack(lo);
    i2c_stop();
    
    // 3) Wait for NVM programming to complete (STATUS.RDY=0)
    while (readStatus() & 0x01) __delay_ms(5);
    
    // 4) Update CRC
    ZSSC_SendCmd(CMD_WRITE_CRC);
    while (readStatus() & 0x01) __delay_ms(5);
    
    return true;
}

// High-level helpers: adjust individual fields in SM_Config1

// PGA1 stage bits [3:0]
static inline bool ZSSC_SetGainStage1(uint8_t code) {
    uint16_t cur = ZSSC_ReadSMConfig1();      // read current value
    uint16_t new = (cur & ~0x000F)            // clear bits [3:0] only
                 | (code & 0x0F);            // insert new code
    return ZSSC_WriteReg16_NVM(ADDR_SM_CONFIG1, new);
}

// PGA2 stage bits [6:4]
static inline bool ZSSC_SetGainStage2(uint8_t stage2) {
    uint16_t w = (ZSSC_ReadSMConfig1() & ~(0x07<<4)) | ((stage2&0x07)<<4);
    return ZSSC_WriteSMConfig1(w);
}

// Gain polarity bit [7]
static inline bool ZSSC_SetGainPolarity(bool negative) {
    uint16_t w = ZSSC_ReadSMConfig1();
    w = negative ? (w |  (1<<7)) : (w & ~(1<<7));
    return ZSSC_WriteSMConfig1(w);
}

// adc_bits[3:0] at bits [11:8]   valid code 0..6 (12..18 bit) or up to 0..15
static inline bool ZSSC_SetAdcBits(uint8_t code) {
    uint16_t w = (ZSSC_ReadSMConfig1() & ~(0x0F<<8)) | ((code&0x0F)<<8);
    return ZSSC_WriteSMConfig1(w);
}

// adc_offset[2:0] at bits [14:12]   valid code 0..7
static inline bool ZSSC_SetAdcOffset(uint8_t code) {
    uint16_t w = (ZSSC_ReadSMConfig1() & ~(0x07<<12)) | ((code&0x07)<<12);
    return ZSSC_WriteSMConfig1(w);
}

// sel_ref1 bit [15]   0=internal bandgap, 1=ratiometric
static inline bool ZSSC_SetRefSource(bool ratiometric) {
    uint16_t w = ZSSC_ReadSMConfig1();
    w = ratiometric ? (w |  (1<<15)) : (w & ~(1<<15));
    return ZSSC_WriteSMConfig1(w);
}

static inline uint16_t ZSSC_ReadSMConfig2(void) {
    return readReg16(ADDR_SM_CONFIG2);
}

// IOFFSC bits [4:0]
static inline bool ZSSC_SetIoffsc(uint8_t code) {
    uint16_t w = (ZSSC_ReadSMConfig2() & ~0x001F) | (code & 0x1F);
    return ZSSC_WriteReg16_NVM(ADDR_SM_CONFIG2, w);
}

// Tbiasout bits [7:5]
static inline bool ZSSC_SetTbiasout(uint8_t code) {
    uint16_t w = (ZSSC_ReadSMConfig2() & ~(0x07<<5)) | ((code&0x07)<<5);
    return ZSSC_WriteReg16_NVM(ADDR_SM_CONFIG2, w);
}

// ADC_en_shift bit [8]
static inline bool ZSSC_SetAdcEnShift(bool on) {
    uint16_t w = ZSSC_ReadSMConfig2();
    w = on ? (w |  (1<<8)) : (w & ~(1<<8));
    return ZSSC_WriteReg16_NVM(ADDR_SM_CONFIG2, w);
}

// PGA_en_shift bit [9]
static inline bool ZSSC_SetPgaEnShift(bool on) {
    uint16_t w = ZSSC_ReadSMConfig2();
    w = on ? (w |  (1<<9)) : (w & ~(1<<9));
    return ZSSC_WriteReg16_NVM(ADDR_SM_CONFIG2, w);
}

#endif // ZSSC_CONFIG_H
