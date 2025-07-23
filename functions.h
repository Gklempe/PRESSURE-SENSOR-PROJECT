#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#define I2C_DELAY_US   5

#include <stdint.h>
#include <stdbool.h>

// Helper: 1 second “I²C delay”
static inline void i2c_delay(void) { 
    __delay_ms(I2C_DELAY_US); 
}

//—— open-drain control via TRIS ——————————————————————————
static inline void SDA_HIGH(void) { 
    TRISCbits.TRISC4 = 1; 
}
static inline void SDA_LOW (void) { 
    LATCbits.LATC4 = 0; TRISCbits.TRISC4 = 0; 
}
static inline void SCL_HIGH(void) { 
    TRISCbits.TRISC2 = 1; 
}
static inline void SCL_LOW (void) { 
    LATCbits.LATC2 = 0; TRISCbits.TRISC2 = 0; 
}

static void i2c_start(void) {
    SDA_HIGH(); 
    SCL_HIGH(); 
    i2c_delay();
    SDA_LOW();           
    i2c_delay();
    SCL_LOW();           
    i2c_delay();
}


static void i2c_stop(void) {
    SDA_LOW();  
    i2c_delay();  // ensure SDA low
    SCL_HIGH(); 
    i2c_delay();
    SDA_HIGH(); 
    i2c_delay();  // STOP: SDA↑ while SCL high
}


// send full byte and sample ACK (9ᵗʰ bit)
static bool i2c_write_byte_ack(uint8_t x) {
    // send bits
    for (int i = 7; i >= 0; --i) {
        if (x & (1<<i)) SDA_HIGH(); else SDA_LOW();
        i2c_delay();
        SCL_HIGH();  i2c_delay();
        SCL_LOW();   i2c_delay();
    }
    // release SDA for ACK
    SDA_HIGH();    i2c_delay();
    SCL_HIGH();    i2c_delay();
    bool ack = (PORTCbits.RC4 == 0);
    SCL_LOW();     i2c_delay();
    return ack;
}

static uint8_t i2c_read_byte(bool nack)
{
    uint8_t v = 0;
    for (int i = 7; i >= 0; --i) {
        SDA_HIGH();  // release SDA
        i2c_delay();
        SCL_HIGH();  i2c_delay();
        if (PORTCbits.RC4) v |= (1<<i);
        SCL_LOW();   i2c_delay();
    }
    // ACK/NACK bit
    if (nack)      SDA_HIGH();
    else           SDA_LOW();
    i2c_delay();
    SCL_HIGH(); i2c_delay();
    SCL_LOW();  i2c_delay();
    SDA_HIGH();
    return v;
}

#endif // FUNCTIONS_H