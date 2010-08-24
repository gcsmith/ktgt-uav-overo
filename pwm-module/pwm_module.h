// pwm_module.h
// Garrett Smith 2010

#ifndef PWM_MODULE__H_
#define PWM_MODULE__H_

// miscellaneous helpers

#define PWM_FIRST   8
#define PWM_COUNT   4

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

// core clock registers

#define CM_CORE_PBASE   0x48004A00UL
#define CM_CORE_SIZE    0x50

#define CM_FCLKEN1_CORE     0x000 
#define CM_FCLKEN3_CORE     0x008
#define CM_ICLKEN1_CORE     0x010
#define CM_ICLKEN3_CORE     0x018
#define CM_IDLEST1_CORE     0x020
#define CM_IDLEST3_CORE     0x028
#define CM_AUTOIDLE1_CORE   0x030
#define CM_AUTOIDLE3_CORE   0x038
#define CM_CLKSEL_CORE      0x040
#define CM_CLKSTCTRL_CORE   0x048
#define CM_CLKSTST_CORE     0x04C

#define CLKSEL_GPT10    (1 << 6)
#define CLKSEL_GPT11    (1 << 7)

// general purpose timer base addresses

#define GPTIMER1_ADDR   0x48318000LU
#define GPTIMER2_ADDR   0x49032000LU
#define GPTIMER3_ADDR   0x49034000LU
#define GPTIMER4_ADDR   0x49036000LU
#define GPTIMER5_ADDR   0x49038000LU
#define GPTIMER6_ADDR   0x4903A000LU
#define GPTIMER7_ADDR   0x4903C000LU
#define GPTIMER8_ADDR   0x4903E000LU
#define GPTIMER9_ADDR   0x49040000LU
#define GPTIMER10_ADDR  0x48086000LU
#define GPTIMER11_ADDR  0x48088000LU
#define GPTIMER_SIZE    4096

// per-timer register offsets

#define GPT_TIDR    0x000
#define GPT_TIOCP   0x010
#define GPT_TISTAT  0x014
#define GPT_TISR    0x018
#define GPT_TIER    0x01C
#define GPT_TWER    0x020
#define GPT_TCLR    0x024
#define GPT_TCRR    0x028
#define GPT_TLDR    0x02C
#define GPT_TTGR    0x030
#define GPT_TWPS    0x034
#define GPT_TMAR    0x038
#define GPT_TCAR1   0x03C
#define GPT_TSICR   0x040
#define GPT_TCAR2   0x044
#define GPT_TPIR    0x048
#define GPT_TNIR    0x04C
#define GPT_TCVR    0x050
#define GPT_TOCR    0x054
#define GPT_TOWR    0x058

// relevant padconf multiplexer register offsets

#define CONTROL_PADCONF_MUX_PBASE           0x48002030LU
#define CONTROL_PADCONF_MUX_SIZE            0xA22
#define CONTROL_PADCONF_UART2_CTS_OFFSET    0x144
#define CONTROL_PADCONF_UART2_RTS_OFFSET    0x146
#define CONTROL_PADCONF_UART2_TX_OFFSET     0x148
#define CONTROL_PADCONF_UART2_RX_OFFSET     0x14A

#endif // PWM_MODULE__H_

