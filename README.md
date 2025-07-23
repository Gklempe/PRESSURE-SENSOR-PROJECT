# PRESSURE-SENSOR-PROJECT

# ZSSC ADC Raw Measurement Reader

This project implements a firmware-level reader for the **ZSSC sensor signal conditioner** family (e.g., ZSSC3240) using **I2C communication**. It reads **raw 24-bit ADC output**, computes the **analog differential input voltage (V<sub>diff</sub>)**, and prints detailed measurement data to a serial terminal via a **PIC microcontroller**.

> ✅ The project reproduces the **same raw ADC behavior and scaling** as observed with the official **ZSSC GUI and communication board**.

---

## 📋 Features

- Low-level I2C interface to ZSSC (e.g., ZSSC3240)
- Reads **raw 24-bit ADC result (CMD 0xA2)** in offset-binary format
- Converts to:
  - **Signed ADC voltage (V<sub>adc</sub>)** in millivolts
  - **Input differential voltage (V<sub>diff</sub>)** in millivolts
- Fully matches GUI behavior (uses `0xA2`, not `0xA3`)
- Displays:
  - PGA gain config (G1, G2)
  - IOFFSC offset voltage
  - ADC pipeline gain
- Supports **re-entry to configuration** via serial `'r'` key
- Outputs precision-formatted debug lines every 2 seconds

---

## 🔧 Hardware Setup

- **Sensor Board**: ZSSC with bridge sensor or potentiometer (used as bridge)
- **Microcontroller**: PIC (any MCU with I2C + UART)
- **Wiring**:
  - VINP → INP (connected internally or by wire)
  - VINN → INN (connected internally or by wire)
  - Bridge reference = V<sub>DDB</sub>
- **Measured bridge span**: ~43.8 mV differential

---

## 🧪 Verification

- Validated against **official ZSSC GUI** using communication board
- GUI and firmware both read:
  - **Raw ADC span**: ~1300 counts for full-scale bridge sweep
  - **Raw values**: 0x400000 ± ~1300 counts
- Voltage differential confirmed with multimeter: 0 ↔ 44 mV swing

---

## 📜 Example Output

```
--- AFE Configuration ---------------------------
 G1           = 120 /100
 G2           = 110 /100
 Pipeline G   = 132 /100
 IOFFSC [μV]  = 0
 ADC_en_shift = 0
--------------------------------------------------
-- Measurements (press 'r' to re-configure) -----

raw24=0x400000   signed=0         Vadc=0.000 mV   Vdiff=0.000 mV
raw24=0x40054E   signed=1262      Vadc=0.875 mV   Vdiff=0.663 mV
raw24=0x3FFAB2   signed=−1298     Vadc=−0.900 mV  Vdiff=−0.681 mV
```

---

## 📚 Technical Notes

- **Command 0xA2**: returns the raw (uncorrected) ADC result in **offset binary**
- **Offset-binary → signed**: `signed24 = raw24 - 2²³`
- **V<sub>adc</sub>**: calculated as  
  ```c
  Vadc = (signed24 / 2^23) * VDD_BRIDGE
  ```
- **V<sub>diff</sub>**: back-calculated from gain pipeline and IOFFSC:
  ```c
  Vdiff = (Vadc + G2 * Voff) / (G1 * G2) - Voff
  ```

---

## 🛠️ Code Highlights

- `put_mV3()` helper for printing voltages with 3 decimal digits in mV
- Clean integer formatting for debug logs
- Fully embedded-safe and delay-controlled measurement loop

---

## 🧰 Dependencies

- C compiler for your PIC microcontroller (e.g. MPLAB XC8)
- UART terminal for monitoring (e.g. PuTTY, TeraTerm)
- I2C and UART libraries / drivers
- Power supply for VDD/VSSB (1.72 V typical)

---

## 📁 Folder Structure

```
├── src/
│   ├── main.c                # main application entry
│   ├── i2c_helpers.c/h       # I2C wrappers
│   ├── uart_helpers.c/h      # UART output functions
│   └── MeasurementLoop.c     # Measurement logic
├── include/
│   └── ZSSC_config.h         # Gain tables and constants
├── README.md                 # You're reading it
```

---

## 👤 Author

**[Your Name]**  
Electrical & Computer Engineering Student  
Personal GitHub Projects – Embedded & Analog

---

## 📜 License

MIT License – feel free to use, modify, and share.
