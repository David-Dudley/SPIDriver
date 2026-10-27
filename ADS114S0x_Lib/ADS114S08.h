/* ******************************************************************************
 *
 * Definitions for the ADS114S08 A/D Converter chip
 *
 * ADS114S0xB Low-Power, Low-Noise, Highly Integrated 6- and 12-Channel
 * 4 kSPS, 16-Bit, Delta-Signa ADC With PGA and Voltage Reference
 *
 * Seems lik it may be a very useful part
 *
 * This device communicates through SPI, Mode 1
 * It uses an actual Byte oriented stream protocol for communications, and uses a
 * Command/Response protocol for control and operation
 *
 * ******************************************************************************/

// Control commands for the ADS114S0x
#define ADS114_NOP          0
#define ADS114_WAKEUP       0x02
#define ADS114_POWERDOWN    0x04
#define ADS114_RESET        0x06
#define ADS114_START        0x08
#define ADS114_STOP         0x0a

// Calibration Commands for the ADS114S0x
#define ADS114_SYOCAL       0x16
#define ADS114_SYGCAL       0x17
#define ADS114_SFOCAL       0x19

// Data Read command
#define ADS114_RDATA        0x12

// Register Read and Write Commands (or in the register ID to read (lower 5 bits) )
// These are 2-byte sequences.
// The first byte is defined as  0b001rrrrr where rrrrr is the starting register to read
// The second byte is defined as 0b000nnnnn where nnnnn is the number of additional registers to read
#define ADS114_RREG         0x20
#define ADS114_WREG         0x40


// Register Addresses
#define ADS114_ID           0x00
#define ADS114_STATUS       0x01
#define ADS114_INPMUX       0x02
#define ADS114_PGA          0x03
#define ADS114_DATARATE     0x04
#define ADS114_REF          0x05
#define ADS114_IDACMAG      0x06
#define ADS114_IDACMUX      0x07
#define ADS114_VBIAS        0x08
#define ADS114_SYS          0x09
#define ADS114_RESERVED     0x0a
#define ADS114_OFCAL0       0x0b
#define ADS114_OFCAL1       0x0c
#define ADS114_RESERVED2    0x0d
#define ADS114_FSCAL0       0x0e
#define ADS114_FSCAL1       0x0f
#define ADS114_GPIODAT      0x10
#define ADS114_GPIOCON      0x11

// Definitions for the ID register
#define ADS114_DEV_ID       0x07

// Definitions for the STATUS register
#define ADS114_FL_POR       0x80
#define ADS114_NRDY         0x40
#define ADS114_FL_REF       0x01

// Now defitions for the Input Multiplexer (INPMUX) register
#define ADS114_MUXP_SHFT    4
#define ADS114_MUXN_SHFT    0
#define ADS114_MUX_AIN0     0x00
#define ADS114_MUX_AIN1     0x01
#define ADS114_MUX_AIN2     0x02
#define ADS114_MUX_AIN3     0x03
#define ADS114_MUX_AIN4     0x04
#define ADS114_MUX_AIN5     0x06
#define ADS114_MUX_AIN6     0x06
#define ADS114_MUX_AIN7     0x07
#define ADS114_MUX_AIN8     0x08
#define ADS114_MUX_AIN9     0x09
#define ADS114_MUX_AIN10    0x0a
#define ADS114_MUX_AIN11    0x0b
#define ADS114_MUX_AINCOM   0x0c

// Definitions for the Gain Setting (PGA) register
#define ADS114_PGA_DISABLE  0x00
#define ADS114_PGA_ENABLE   0x08
#define ADS114_PGA_GAIN1    0
#define ADS114_PGA_GAIN2    1
#define ADS114_PGA_GAIN4    2
#define ADS114_PGA_GAIN8    3
#define ADS114_PGA_GAIN16   4
#define ADS114_PGA_GAIN32   5
#define ADS114_PGA_GAIN64   6
#define ADS114_PGA_GAIN128  7

// Definitions for the Data Rate (DATARATE) register
// ALWAYS or in ADS114_DATARATE_RES into value for this register
#define ADS114_CLK_EXTERN   0x40
#define ADS114_MODE         0x20
#define ADS114_DATARATE_RES 0x10
#define ADS114_RATESEL_2_5  0x00
#define ADS114_RATESEL_5    0x01
#define ADS114_RATESEL_10   0x02
#define ADS114_RATESEL_16_6 0x03
#define ADS114_RATESEL_20   0x04
#define ADS114_RATESEL_50   0x05
#define ADS114_RATESEL_60   0x06
#define ADS114_RATESEL_100  0x07
#define ADS114_RATESEL_200  0x08
#define ADS114_RATESEL_400  0x09
#define ADS114_RATESEL_800  0x0a
#define ADS114_RATESEL_1000 0x0b
#define ADS114_RATESEL_2000 0x0c
#define ADS114_RATESEL_4000 0x0d
#define ADS114_RATESEL_4000 0x0e

// Definitions for the Reference Control (REF) register
#define ADS114_FL_REF_EN    0x40
#define ADS114_NREFB_BUF    0x20
#define ADS114_NREFN_BUF    0x10
#define ADS114_REFP1        0x04
#define ADS114_REF_INTERNAL 0x02
#define ADS114_REF_IN_ON    0x01

// Definitions for the Excitation Current 1 (IDACMAG) register
#define ADS114_IDAC_OFF     0x00
#define ADS114_IDAC_10u     0x01
#define ADS114_IDAC_50u     0x02
#define ADS114_IDAC_100u    0x03
#define ADS114_IDAC_250u    0x04
#define ADS114_IDAC_500u    0x05
#define ADS114_IDAC_750u    0x06
#define ADS114_IDAC_1000u   0x07
#define ADS114_IDAC_1500u   0x08
#define ADS114_IDAC_2000u   0x09

// Definitions for the Excitation Current 2 (IDACMUX) register
#define ADS114_I2MUX_AIN0   0x00
#define ADS114_I2MUX_AIN1   0x10
#define ADS114_I2MUX_AIN2   0x20
#define ADS114_I2MUX_AIN3   0x30
#define ADS114_I2MUX_AIN4   0x40
#define ADS114_I2MUX_AIN5   0x50
#define ADS114_I2MUX_AIN6   0x60
#define ADS114_I2MUX_AIN7   0x70
#define ADS114_I2MUX_AIN8   0x80
#define ADS114_I2MUX_AIN9   0x90
#define ADS114_I2MUX_AIN10  0xa0
#define ADS114_I2MUX_AIN11  0xb0
#define ADS114_I2MUX_AINCOM 0xc0
#define ADS114_I1MUX_AIN0   0x00
#define ADS114_I1MUX_AIN1   0x01
#define ADS114_I1MUX_AIN2   0x02
#define ADS114_I1MUX_AIN3   0x03
#define ADS114_I1MUX_AIN4   0x04
#define ADS114_I1MUX_AIN5   0x05
#define ADS114_I1MUX_AIN6   0x06
#define ADS114_I1MUX_AIN7   0x07
#define ADS114_I1MUX_AIN8   0x08
#define ADS114_I1MUX_AIN9   0x09
#define ADS114_I1MUX_AIN10  0x0a
#define ADS114_I1MUX_AIN11  0x0b
#define ADS114_I1MUX_AINCOM 0x0c

// Definitions for the Sensor Biasing (VBIAS) register
// Bit 7 in this registe must always be 0
#define ADS114_VBIAS_AINC   0x40
#define ADS114_VBIAS_AIN5   0x20
#define ADS114_VBIAS_AIN4   0x10
#define ADS114_VBIAS_AIN3   0x08
#define ADS114_VBIAS_AIN2   0x04
#define ADS114_VBIAS_AIN1   0x02
#define ADS114_VBIAS_AIN0   0x01

// Definitions for the System Control (SYS) Register
#define ADS114_SYS_MON0     0x00
#define ADS114_SYS_MON1     0x20
#define ADS114_SYS_MON2     0x40
#define ADS114_SYS_MON3     0x60
#define ADS114_SYS_MON4     0x80
#define ADS114_SYS_MON5     0xa0
#define ADS114_SYS_MON6     0xc0
#define ADS114_SYS_MON7     0xe0
#define ADS114_SYS_SAMPL1   0x00
#define ADS114_SYS_SAMPL4   0x08
#define ADS114_SYS_SAMPL8   0x10
#define ADS114_SYS_SAMPL16  0x18
#define ADS114_SYS_SPI_TIMR 0x04

// Some registers following are just 8 bit values

// Definitions for the GPIO Data (GPIODAT) register

// Definitions for the GPIO Configuration (GPIOCON) register

