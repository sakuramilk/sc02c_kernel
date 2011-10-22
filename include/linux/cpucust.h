/*
 *  linux/include/linux/cpufreq.h
 *
 *  Copyright (C) 2011 sakuramilk <c.sakuramilk@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _LINUX_CPUCUST_H
#define _LINUX_CPUCUST_H

#define CUST_ARM_CLK_DEFAULT_MAX    (1200000)
#define CUST_ARM_CLK_DEFAULT_MIN    ( 200000)

#ifdef CONFIG_FREQ_OVERCLOCK
/* Overclock */
#define CUST_ARM_V_MAX              (1500000)
#define CUST_ARM_V_MIN              ( 800000)

#define CUST_ARM_CLK_L_MAX          (11)
#define CUST_FREQ_LEVEL_INDEX       L0, L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, CPUFREQ_LEVEL_END,

#define CUST_ARM_CLK_L0             (1600000)
#define CUST_ARM_V_L0               (1425000)
#define CUST_INT_V_L0               (1100000)
#define CUST_CLKDIV_CPU0_L0         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L0         { 5, 0 }
#define CUST_APLL_PMS_L0            ( ((200<<16)|(3<<8)|(0x1)) ) // 8000

#define CUST_ARM_CLK_L1             (1504000)
#define CUST_ARM_V_L1               (1400000)
#define CUST_INT_V_L1               (1100000)
#define CUST_CLKDIV_CPU0_L1         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L1         { 5, 0 }
#define CUST_APLL_PMS_L1            ( ((188<<16)|(3<<8)|(0x1)) ) // 8000

#define CUST_ARM_CLK_L2             (1400000)
#define CUST_ARM_V_L2               (1350000)
#define CUST_INT_V_L2               (1100000)
#define CUST_CLKDIV_CPU0_L2         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L2         { 5, 0 }
#define CUST_APLL_PMS_L2            ( ((350<<16)|(6<<8)|(0x1)) ) // 4000

#define CUST_ARM_CLK_L3             (1200000)
#define CUST_ARM_V_L3               (1300000)
#define CUST_INT_V_L3               (1100000)
#define CUST_CLKDIV_CPU0_L3         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L3         { 5, 0 }
#define CUST_APLL_PMS_L3            ( ((150<<16)|(3<<8)|(0x1)) ) // 8000

#define CUST_ARM_CLK_L4             (1000000)
#define CUST_ARM_V_L4               (1200000)
#define CUST_INT_V_L4               (1100000)
#define CUST_CLKDIV_CPU0_L4         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L4         { 4, 0 }
#define CUST_APLL_PMS_L4            ( ((250<<16)|(6<<8)|(0x1)) ) // 4000

#define CUST_ARM_CLK_L5             ( 800000)
#define CUST_ARM_V_L5               (1100000)
#define CUST_INT_V_L5               (1100000)
#define CUST_CLKDIV_CPU0_L5         { 0, 3, 7, 3, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L5         { 3, 0 }
#define CUST_APLL_PMS_L5            ( ((200<<16)|(6<<8)|(0x1)) ) // 4000

#define CUST_ARM_CLK_L6             ( 500000)
#define CUST_ARM_V_L6               (1000000)
#define CUST_INT_V_L6               (1000000)
#define CUST_CLKDIV_CPU0_L6         { 0, 3, 7, 3, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L6         { 3, 0 }
#define CUST_APLL_PMS_L6            ( ((250<<16)|(6<<8)|(0x2)) ) // 2000

#define CUST_ARM_CLK_L7             ( 200000)
#define CUST_ARM_V_L7               ( 975000)
#define CUST_INT_V_L7               (1000000)
#define CUST_CLKDIV_CPU0_L7         { 0, 1, 3, 1, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L7         { 3, 0 }
#define CUST_APLL_PMS_L7            ( ((200<<16)|(6<<8)|(0x3)) ) // 1000

#define CUST_ARM_CLK_L8             ( 100000)
#define CUST_ARM_V_L8               ( 925000)
#define CUST_INT_V_L8               (1000000)
#define CUST_CLKDIV_CPU0_L8         { 0, 1, 3, 1, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L8         { 3, 0 }
#define CUST_APLL_PMS_L8            ( ((100<<16)|(6<<8)|(0x3)) ) // 1000

#define CUST_ARM_CLK_L9             (  50000)
#define CUST_ARM_V_L9               ( 900000)
#define CUST_INT_V_L9               (1000000)
#define CUST_CLKDIV_CPU0_L9         { 0, 1, 3, 1, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L9         { 3, 0 }
#define CUST_APLL_PMS_L9            ( ((100<<16)|(6<<8)|(0x4)) ) // 500

#define CUST_ARM_CLK_L10            (  25000)
#define CUST_ARM_V_L10              ( 875000)
#define CUST_INT_V_L10              (1000000)
#define CUST_CLKDIV_CPU0_L10        { 0, 1, 3, 1, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L10        { 3, 0 }
#define CUST_APLL_PMS_L10           ( (( 50<<16)|(6<<8)|(0x4)) ) // 500

#define CUST_SUSPEND_CLK_L          L5 // set  800MHz
#define CUST_REBOOT_CLK_L           L3 // set 1200MHz
#define CUST_1000MHZ_CLK_L          L4 // set 1000MHz
#define CUST_DEFAULT_CLK_L_MAX      L3 // set default max freq levex
#define CUST_DEFAULT_CLK_L_MIN      L7 // set default min freq levex

#define CUST_ARM_CLK_1000MHZ        CUST_ARM_CLK_L4
#define CUST_ARM_CLK_500MHZ         CUST_ARM_CLK_L6
#define CUST_ARM_CLK_MAX            CUST_ARM_CLK_L0

#define CPUFREQ_LEVEL_DEFAULT_OFFSET	3

#else

/* Default clock max 1200MHz */
#define CUST_ARM_V_MAX              (1300000)
#define CUST_ARM_V_MIN              ( 700000)

#define CUST_ARM_CLK_L_MAX          (5)
#define CUST_FREQ_LEVEL_INDEX       L0, L1, L2, L3, L4, CPUFREQ_LEVEL_END,

#define CUST_ARM_CLK_L0             (1200000)
#define CUST_ARM_V_L0               (1300000)
#define CUST_CLKDIV_CPU0_L0         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L0         { 5, 0 }
#define CUST_APLL_PMS_L0            ( ((150<<16)|(3<<8)|(0x1)) )

#define CUST_ARM_CLK_L1             (1000000)
#define CUST_ARM_V_L1               (1200000)
#define CUST_CLKDIV_CPU0_L1         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L1         { 4, 0 }
#define CUST_APLL_PMS_L1            ( ((250<<16)|(6<<8)|(0x1)) )

#define CUST_ARM_CLK_L2             ( 800000)
#define CUST_ARM_V_L2               (1100000)
#define CUST_CLKDIV_CPU0_L2         { 0, 3, 7, 3, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L2         { 3, 0 }
#define CUST_APLL_PMS_L2            ( ((200<<16)|(6<<8)|(0x1)) )

#define CUST_ARM_CLK_L3             ( 500000)
#define CUST_ARM_V_L3               (1000000)
#define CUST_CLKDIV_CPU0_L3         { 0, 3, 7, 3, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L3         { 3, 0 }
#define CUST_APLL_PMS_L3            ( ((250<<16)|(6<<8)|(0x2)) )

#define CUST_ARM_CLK_L4             ( 200000)
#define CUST_ARM_V_L4               ( 975000)
#define CUST_CLKDIV_CPU0_L4         { 0, 3, 7, 1, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L4         { 3, 0 }
#define CUST_APLL_PMS_L4            ( ((200<<16)|(6<<8)|(0x3)) )

#define CUST_SUSPEND_CLK_L          L1 // set 1000MHz
#define CUST_REBOOT_CLK_L           L0 // set 1200MHz
#define CUST_DEFAULT_CLK_L_MAX      L0 // set default max freq levex
#define CUST_DEFAULT_CLK_L_MIN      L4 // set default min freq levex

#define CUST_ARM_CLK_1000MHZ        CUST_ARM_CLK_L1
#define CUST_ARM_CLK_500MHZ         CUST_ARM_CLK_L3
#define CUST_ARM_CLK_MAX            CUST_ARM_CLK_L0

#define CPUFREQ_LEVEL_DEFAULT_OFFSET	0

#endif

/* UV table */
extern int exp_UV_mV[CUST_ARM_CLK_L_MAX];

#endif /* _LINUX_CPUCUST_H */
