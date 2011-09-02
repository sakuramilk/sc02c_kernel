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
/* Overclock max 1600MHz */
#define CUST_ARM_V_MAX              (1400000)
#define CUST_ARM_V_MIN              ( 800000)

#define CUST_ARM_CLK_L_MAX          (7)
#define CUST_FREQ_LEVEL_INDEX       L0, L1, L2, L3, L4, L5, L6, CPUFREQ_LEVEL_END,

#define CUST_ARM_CLK_L0             (1504000)
#define CUST_ARM_V_L0               (1400000)
#define CUST_CLKDIV_CPU0_L0         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L0         { 5, 0 }
#define CUST_APLL_PMS_L0            ( ((188<<16)|(3<<8)|(0x1)) ) // 8000

#define CUST_ARM_CLK_L1             (1200000)
#define CUST_ARM_V_L1               (1275000)
#define CUST_CLKDIV_CPU0_L1         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L1         { 5, 0 }
#define CUST_APLL_PMS_L1            ( ((150<<16)|(3<<8)|(0x1)) ) // 8000

#define CUST_ARM_CLK_L2             (1000000)
#define CUST_ARM_V_L2               (1175000)
#define CUST_CLKDIV_CPU0_L2         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L2         { 4, 0 }
#define CUST_APLL_PMS_L2            ( ((250<<16)|(6<<8)|(0x1)) ) // 4000

#define CUST_ARM_CLK_L3             ( 800000)
#define CUST_ARM_V_L3               (1075000)
#define CUST_CLKDIV_CPU0_L3         { 0, 3, 7, 3, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L3         { 3, 0 }
#define CUST_APLL_PMS_L3            ( ((200<<16)|(6<<8)|(0x1)) ) // 4000

#define CUST_ARM_CLK_L4             ( 500000)
#define CUST_ARM_V_L4               ( 975000)
#define CUST_CLKDIV_CPU0_L4         { 0, 3, 7, 3, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L4         { 3, 0 }
#define CUST_APLL_PMS_L4            ( ((250<<16)|(6<<8)|(0x2)) ) // 2000

#define CUST_ARM_CLK_L5             ( 200000)
#define CUST_ARM_V_L5               ( 950000)
#define CUST_CLKDIV_CPU0_L5         { 0, 3, 7, 1, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L5         { 3, 0 }
#define CUST_APLL_PMS_L5            ( ((200<<16)|(6<<8)|(0x3)) ) // 1000

#define CUST_ARM_CLK_L6             ( 100000)
#define CUST_ARM_V_L6               ( 950000)
#define CUST_CLKDIV_CPU0_L6         { 0, 3, 7, 1, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L6         { 3, 0 }
#define CUST_APLL_PMS_L6            ( ((100<<16)|(6<<8)|(0x3)) ) // 1000


#define ARMCLOCK_HIGH               CUST_ARM_CLK_L1
#define ARMCLOCK_MID                CUST_ARM_CLK_L2
#define ARMCLOCK_LOW                CUST_ARM_CLK_L4
#define CUST_SUSPEND_LV             L3
#define CUST_REBOOT_LV              L1

#else

/* Default clock max 1200MHz */
#define CUST_ARM_V_MAX              (1200000)
#define CUST_ARM_V_MIN              ( 800000)

#define CUST_ARM_CLK_L_MAX          (5)
#define CUST_FREQ_LEVEL_INDEX       L0, L1, L2, L3, L4, CPUFREQ_LEVEL_END,

#define CUST_ARM_CLK_L0             (1200000)
#define CUST_ARM_V_L0               (1275000)
#define CUST_CLKDIV_CPU0_L0         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L0         { 5, 0 }
#define CUST_APLL_PMS_L0            ( ((150<<16)|(3<<8)|(0x1)) )

#define CUST_ARM_CLK_L1             (1000000)
#define CUST_ARM_V_L1               (1175000)
#define CUST_CLKDIV_CPU0_L1         { 0, 3, 7, 3, 4, 1, 7 }
#define CUST_CLKDIV_CPU1_L1         { 4, 0 }
#define CUST_APLL_PMS_L1            ( ((250<<16)|(6<<8)|(0x1)) )

#define CUST_ARM_CLK_L2             ( 800000)
#define CUST_ARM_V_L2               (1075000)
#define CUST_CLKDIV_CPU0_L2         { 0, 3, 7, 3, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L2         { 3, 0 }
#define CUST_APLL_PMS_L2            ( ((200<<16)|(6<<8)|(0x1)) )

#define CUST_ARM_CLK_L3             ( 500000)
#define CUST_ARM_V_L3               ( 975000)
#define CUST_CLKDIV_CPU0_L3         { 0, 3, 7, 3, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L3         { 3, 0 }
#define CUST_APLL_PMS_L3            ( ((250<<16)|(6<<8)|(0x2)) )

#define CUST_ARM_CLK_L4             ( 200000)
#define CUST_ARM_V_L4               ( 950000)
#define CUST_CLKDIV_CPU0_L4         { 0, 3, 7, 1, 3, 1, 7 }
#define CUST_CLKDIV_CPU1_L4         { 3, 0 }
#define CUST_APLL_PMS_L4            ( ((200<<16)|(6<<8)|(0x3)) )

#define ARMCLOCK_HIGH               CUST_ARM_CLK_L0
#define ARMCLOCK_MID                CUST_ARM_CLK_L1
#define ARMCLOCK_LOW                CUST_ARM_CLK_L3
#define CUST_SUSPEND_LV             L2
#define CUST_REBOOT_LV              L0

#endif

/* UV table */
extern int exp_UV_mV[CUST_ARM_CLK_L_MAX];

#endif /* _LINUX_CPUCUST_H */
