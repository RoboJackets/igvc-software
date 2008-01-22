#define SL_ADDR 0x00
#define DATA    (0x04 / sizeof(unsigned int))
#define CONTROL (0x08 / sizeof(unsigned int))
#define STATUS  (0x0C / sizeof(unsigned int))
#define BAUD    (0x0C / sizeof(unsigned int))
#define SRAM    (0x04 / sizeof(unsigned int))
#define GLED    (0x08 / sizeof(unsigned int))

#define READ    1
#define WRITE   0

/* Control Reg */
#define ACK     (1 << 2)
#define IFLG    (1 << 3)
#define STOP    (1 << 4)
#define START   (1 << 5)
#define TWSIEN  (1 << 6)
#define INTEN   (1 << 7)

/* Status Reg

   MT_xxx - master transmitter
   MR_xxx - master receiver
   ST_xxx - slave transmitter
   SR_xxx - slave receiver
*/

#define MT_START        0x08
#define MR_START        0x08
#define MT_REP_START    0x10
#define MT_SLA_ACK      0x18
#define MT_DATA_ACK     0x28
#define MR_SLA_ACK      0x40
#define MR_DATA_ACK     0x50
#define MR_DATA_NACK    0x58
#define SR_SLA_ACK      0x60
#define SR_DATA_ACK     0x80
#define SR_STOP         0xA0
#define ST_SLA_ACK      0xA8
#define ST_DATA_ACK     0xB8

/* Slave Address Reg */
#define GCE             (1 << 0)
#define SADDR           0x0A            //Orion I2C address

#define AVR_ADDR        (0x07 << 1)     //AVR I2C address

/* I2C commands */

#define LED             0x04
#define SLEEP           0x08
#define ADC_CH0         (0x40 | (1<<0))
#define ADC_CH1         (0x40 | (1<<1))
#define ADC_CH2         (0x40 | (1<<2))
#define ADC_CH3         (0x40 | (1<<3))
#define ADC_CH6         (0x40 | (1<<4))
#define ADC_CH7         (0x40 | (1<<5))

#define WDT_OFF         0
#define WDT_250ms       1
#define WDT_2s          2
#define WDT_8s          3

#define OTP_W           0x81
#define OTP_R		0x80
