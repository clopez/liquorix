/*
 * lirc.h - linux infrared remote control header file
 * last modified 2007/09/27
 */

#ifndef _LINUX_LIRC_H
#define _LINUX_LIRC_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define PULSE_BIT  0x01000000
#define PULSE_MASK 0x00FFFFFF

/*** lirc compatible hardware features ***/

#define LIRC_MODE2SEND(x) (x)
#define LIRC_SEND2MODE(x) (x)
#define LIRC_MODE2REC(x) ((x) << 16)
#define LIRC_REC2MODE(x) ((x) >> 16)

#define LIRC_MODE_RAW                  0x00000001
#define LIRC_MODE_PULSE                0x00000002
#define LIRC_MODE_MODE2                0x00000004
#define LIRC_MODE_LIRCCODE             0x00000010


#define LIRC_CAN_SEND_RAW              LIRC_MODE2SEND(LIRC_MODE_RAW)
#define LIRC_CAN_SEND_PULSE            LIRC_MODE2SEND(LIRC_MODE_PULSE)
#define LIRC_CAN_SEND_MODE2            LIRC_MODE2SEND(LIRC_MODE_MODE2)
#define LIRC_CAN_SEND_LIRCCODE         LIRC_MODE2SEND(LIRC_MODE_LIRCCODE)

#define LIRC_CAN_SEND_MASK             0x0000003f

#define LIRC_CAN_SET_SEND_CARRIER      0x00000100
#define LIRC_CAN_SET_SEND_DUTY_CYCLE   0x00000200
#define LIRC_CAN_SET_TRANSMITTER_MASK  0x00000400

#define LIRC_CAN_REC_RAW               LIRC_MODE2REC(LIRC_MODE_RAW)
#define LIRC_CAN_REC_PULSE             LIRC_MODE2REC(LIRC_MODE_PULSE)
#define LIRC_CAN_REC_MODE2             LIRC_MODE2REC(LIRC_MODE_MODE2)
#define LIRC_CAN_REC_LIRCCODE          LIRC_MODE2REC(LIRC_MODE_LIRCCODE)

#define LIRC_CAN_REC_MASK              LIRC_MODE2REC(LIRC_CAN_SEND_MASK)

#define LIRC_CAN_SET_REC_CARRIER       (LIRC_CAN_SET_SEND_CARRIER << 16)
#define LIRC_CAN_SET_REC_DUTY_CYCLE    (LIRC_CAN_SET_SEND_DUTY_CYCLE << 16)

#define LIRC_CAN_SET_REC_DUTY_CYCLE_RANGE 0x40000000
#define LIRC_CAN_SET_REC_CARRIER_RANGE    0x80000000
#define LIRC_CAN_GET_REC_RESOLUTION       0x20000000

#define LIRC_CAN_SEND(x) ((x)&LIRC_CAN_SEND_MASK)
#define LIRC_CAN_REC(x) ((x)&LIRC_CAN_REC_MASK)

#define LIRC_CAN_NOTIFY_DECODE            0x01000000

/*** IOCTL commands for lirc driver ***/

#define LIRC_GET_FEATURES              _IOR('i', 0x00000000, uint64_t)

#define LIRC_GET_SEND_MODE             _IOR('i', 0x00000001, uint64_t)
#define LIRC_GET_REC_MODE              _IOR('i', 0x00000002, uint64_t)
#define LIRC_GET_SEND_CARRIER          _IOR('i', 0x00000003, uint32_t)
#define LIRC_GET_REC_CARRIER           _IOR('i', 0x00000004, uint32_t)
#define LIRC_GET_SEND_DUTY_CYCLE       _IOR('i', 0x00000005, uint32_t)
#define LIRC_GET_REC_DUTY_CYCLE        _IOR('i', 0x00000006, uint32_t)
#define LIRC_GET_REC_RESOLUTION        _IOR('i', 0x00000007, uint32_t)

/* code length in bits, currently only for LIRC_MODE_LIRCCODE */
#define LIRC_GET_LENGTH                _IOR('i', 0x0000000f, uint64_t)

#define LIRC_SET_SEND_MODE             _IOW('i', 0x00000011, uint64_t)
#define LIRC_SET_REC_MODE              _IOW('i', 0x00000012, uint64_t)
/* Note: these can reset the according pulse_width */
#define LIRC_SET_SEND_CARRIER          _IOW('i', 0x00000013, uint32_t)
#define LIRC_SET_REC_CARRIER           _IOW('i', 0x00000014, uint32_t)
#define LIRC_SET_SEND_DUTY_CYCLE       _IOW('i', 0x00000015, uint32_t)
#define LIRC_SET_REC_DUTY_CYCLE        _IOW('i', 0x00000016, uint32_t)
#define LIRC_SET_TRANSMITTER_MASK      _IOW('i', 0x00000017, uint32_t)

/*
 * to set a range use
 * LIRC_SET_REC_DUTY_CYCLE_RANGE/LIRC_SET_REC_CARRIER_RANGE with the
 * lower bound first and later
 * LIRC_SET_REC_DUTY_CYCLE/LIRC_SET_REC_CARRIER with the upper bound
 */

#define LIRC_SET_REC_DUTY_CYCLE_RANGE  _IOW('i', 0x0000001e, uint32_t)
#define LIRC_SET_REC_CARRIER_RANGE     _IOW('i', 0x0000001f, uint32_t)

#define LIRC_NOTIFY_DECODE             _IO('i', 0x00000020)

#endif
