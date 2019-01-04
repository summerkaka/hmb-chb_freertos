/**
  ******************************************************************************
  * @file    Project/src/can_app.h
  * @author
  * @version V0.00
  * @date
  * @brief   can_app.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_APP_H
#define __CAN_APP_H
//#ifdef __cplusplus
// extern "C" {
//#endif


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported macro ------------------------------------------------------------*/

#define BITRATE             1000000

#define CANID_CHARGE        0x0d
#define CANID_BROADCAST     0xff
#define CANID_PUMP          0x12
#define CANID_MB            0x00
#define LOCAL_ID            CANID_CHARGE

#define CMD_MAXLENGTH       8

#define CMD_DATA_SYNC       0x60
#define CMD_READ_AIO        0x61
#define CMD_DETECT_ADC      0x62
#define CMD_READ_ZONE       0x63
#define	CMD_READ_ID         0x80
#define	CMD_READ_DIO        0x87
#define	CMD_WRITE_DIO       0x88
#define CMD_WRITE_PULSE     0x90
#define CMD_START_RUN       0x92
#define CMD_WRITE_SP        0x98
#define CMD_CONTACT_OFF     0x99
#define CMD_STOP_RUN        0x9a
#define CMD_DEBUG_RD        0xf0
#define CMD_DEBUG_WR        0xf1

#define CMD_PING            0x00
#define CMD_DLD             0x01
#define CMD_SENDDATA        0x02
#define CMD_REQUEST         0x03
#define CMD_RESET           0x05
#define CMD_ASK_APPAREA     0xaa
#define CMD_JUMPTOAPP       0xab
#define CMD_JUMPTOBL        0xac
#define CMD_WRITECRC        0xad
#define CMD_PROGRAM_START   0xae
#define CMD_PROGRAM_END     0xaf
#define CMD_BAT_RENEW       0xbf

#define MUX_CG_BAT_1_VOL    0x01
#define MUX_CG_BAT_2_VOL    0x02
#define MUX_CG_BAT_1_CUR    0x03
#define MUX_CG_BAT_2_CUR    0x04
#define MUX_CG_BAT_1_TEMP   0x05
#define MUX_CG_BAT_2_TEMP   0x06
#define MUX_CG_BAT_1_LVL    0x07
#define MUX_CG_BAT_2_LVL    0x08
#define MUX_CG_BAT_1_CAP    0x09
#define MUX_CG_BAT_2_CAP    0x0A
#define MUX_CG_BAT_1_REMAIN_TIME  0x0B
#define MUX_CG_BAT_2_REMAIN_TIME  0x0C
#define MUX_CG_BAT_1_CHARGE_TIMES 0x0D
#define MUX_CG_BAT_2_CHARGE_TIMES 0x0E
#define MUX_CG_HEATER_TEMP  0x20
#define MUX_CG_HEATER_DUTY  0x21
#define MUX_CG_ADAPTOR_VOL  0x30
#define MUX_CG_OUTPUT_VOL   0x40
#define MUX_CG_OUTPUT_CUR   0x41
#define MUX_CG_PRESSURE_1   0x50
#define MUX_CG_PRESSURE_2   0x51

#define BLOCK_A	0x01
#define BLOCK_B	0x02
#define BLOCK_C 0x03
// BLOCK A
#define DIO_ADAPTOR_SUPPLY	0x0001
#define DIO_BAT1_SUPPLY		0x0002
#define DIO_BAT2_SUPPLY		0x0004
#define DIO_GCFAN           0x0008
#define DIO_HEATER          0x0010
#define	DIO_BAT1_CHARGE     0x0020
#define DIO_BAT2_CHARGE     0x0040
#define DIO_BAT1_AGED       0x0080
#define DIO_BAT2_AGED       0x0100
#define DIO_VALVE1_SW       0x0200
#define DIO_VALVE2_SW       0x0400
#define DIO_PUMP            0x1000
#define DIO_PUMP_VALVE      0x2000
#define DIO_BOOT_MODE       0x4000      // 1: normal boot, 0: fast boot

// BLOCK_B
#define DIO_HALL_SENSOR     0x0001
#define DIO_LED_WHITE       0x0002
#define DIO_LED_RED         0x0004
#define DIO_CALIFORNIA      0x0008
// BLOCK C
#define DIO_CCB             0x0001

#define GetWordH(ptr)  ((uint32_t)*(uint8_t *)(ptr) << 24       | \
                        (uint32_t)*((uint8_t *)(ptr) + 1) << 16 | \
                        (uint32_t)*((uint8_t *)(ptr) + 2) << 8  | \
                        (uint32_t)*((uint8_t *)(ptr) + 3))

#define GetWordL(ptr)  ((uint32_t)*((uint8_t *)(ptr) + 3) << 24 | \
                        (uint32_t)*((uint8_t *)(ptr) + 2) << 16 | \
                        (uint32_t)*((uint8_t *)(ptr) + 1) << 8  | \
                        (uint32_t)*(ptr))
//#define GetWordL(ptr)   (uint32_t)*(uint32_t *)(ptr)

#define GetShortH(ptr) ((uint16_t)*(uint8_t *)(ptr) << 8  | \
                        (uint16_t)*((uint8_t *)(ptr) + 1))

#define GetShortL(ptr)  (*((uint8_t *)(ptr) + 1) << 8  | \
                        *(uint8_t *)(ptr))
//#define GetShortL(ptr)  (uint32_t)*(uint16_t *)(ptr)

// ptr is h
#define WriteWordH(ptr, value)      do {*(uint8_t *)(ptr)       = (value) >> 24;        \
                                        *(uint8_t *)((ptr) + 1) = (value) >> 16 & 0xff; \
                                        *(uint8_t *)((ptr) + 2) = (value) >> 8 & 0xff;  \
                                        *(uint8_t *)((ptr) + 3) = (value) & 0xff;       \
                                    } while (0)
// ptr is l
#define WriteWordL(ptr, value)      do {*(uint8_t *)((ptr) + 3) = (value) >> 24;        \
                                        *(uint8_t *)((ptr) + 2) = (value) >> 16 & 0xff; \
                                        *(uint8_t *)((ptr) + 1) = (value) >> 8 & 0xff;  \
                                        *(uint8_t *)(ptr)       = (value) & 0xff;       \
                                    } while (0)
// #define WriteWordL(ptr, value)      *(uint32_t *)(ptr) = value
// ptr is h
#define WriteShortH(ptr, value)     do {*(uint8_t *)(ptr)       = (value) >> 8;         \
                                        *(uint8_t *)((ptr) + 1) = (value);              \
                                    } while (0)
// ptr is l
#define WriteShortL(ptr, value)     do {*(uint8_t *)((ptr) + 1) = (value) >> 8;         \
                                        *(uint8_t *)(ptr)       = (value);              \
                                    } while (0)
// #define WriteShortL(ptr, value)     *(uint16_t *)(ptr) = value

#define CMD_SHIFT   0
#define DEST_SHIFT  8
#define SRC_SHIFT   18

#define GET_ID_CMD(id)              (id & ((uint32_t)0xff << 1))

#define GET_ID_DEST(id)             (id & ((uint32_t)0xff << DEST_SHIFT))

#define GET_ID_SRC(id)              (id & ((uint32_t)0xff << 21))

#define WRITE_ID_CMD(id, value)     do {id &= ~((uint32_t)0xff << CMD_SHIFT);   \
                                        id |= value << CMD_SHIFT;               \
                                    } while (0)

#define WRITE_ID_DEST(id, value)    do {id &= ~((uint32_t)0xff << DEST_SHIFT);  \
                                        id |= value << DEST_SHIFT;              \
                                    } while (0)

#define WRITE_ID_SRC(id, value)     do {id &= ~((uint32_t)0xff << SRC_SHIFT);   \
                                        id |= value << SRC_SHIFT;               \
                                    } while (0)

#define IDE_FLAG    0x00080000u

/* Exported types ------------------------------------------------------------*/
typedef union {
    float tfloat;
    int32_t tint32;
} tuType32;

typedef struct {
    uint32_t CmdNum     :8;  //7:0
    uint32_t Target     :8;  //15:8
    uint32_t rsvd0      :2;  //17:16
    uint32_t Src        :8;  //25:18
    uint32_t rsvd1      :3;  //28:26
} stCanId;

typedef union {
	uint32_t 	all;
	stCanId     field;
} tuCanId;

typedef struct {
    tuCanId id;
    uint8_t dlc;
    uint8_t data[CMD_MAXLENGTH];
} stCanPacket;

typedef struct {
    CAN_RxHeaderTypeDef         header;
    uint8_t                     data[8];
} CANMsg_t;


/* Exported constants --------------------------------------------------------*/

/* Exported variables ------------------------------------------------------- */
extern uint16_t dio_a_mask;
extern uint16_t dio_a_value;
extern uint16_t dio_b_mask;
extern uint16_t dio_b_value;
extern uint8_t ccb_rst_times;
extern uint32_t msg_total_recv;
extern uint32_t self_recover_cnt;
extern uint32_t fetch_cnt;
extern uint32_t led_red_lock_time;
extern uint32_t led_white_lock_time;

/* Exported functions ------------------------------------------------------- */
void CAN_Config(void);
void CAN_Listen(void);
void MsgHandler(const void *);


//#ifdef __cplusplus
//}
//#endif
#endif /*__CAN_APP_H */


