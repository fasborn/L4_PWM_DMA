#include "delay.h"

//#define DWT_CONTROL *(volatile unsigned long *)0xE0001000
//#define SCB_DEMCR   *(volatile unsigned long *)0xE000EDFC


//void DWT_Init(void)
//{
//    SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // ????????? ???????????? ???????
//    DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // ????????? ???????
//}

//void delay_micros(uint32_t us)
//{
//    uint32_t us_count_tic =  us * (SystemCoreClock / 10000000); // ???????? ???-?? ?????? ?? 1 ??? ? ???????? ?? ???? ????????
//    DWT->CYCCNT = 0U; // ???????? ???????
//    while(DWT->CYCCNT < us_count_tic);
//}


//void stopwatch_reset(void)
//{
//    /* Enable DWT */
//    DEMCR |= DEMCR_TRCENA; 
//    *DWT_CYCCNT = 0;             
//    /* Enable CPU cycle counter */
//    DWT_CTRL |= CYCCNTENA;
//}

//inline uint32_t stopwatch_getticks()
//{
//    return CPU_CYCLES;
//}

//inline void stopwatch_delay(uint32_t ticks)
//{
//    uint32_t end_ticks = ticks + stopwatch_getticks();
//    while(1)
//    {
//            if (stopwatch_getticks() >= end_ticks)
//                    break;
//    }
//}

//uint32_t CalcNanosecondsFromStopwatch(uint32_t nStart, uint32_t nStop)
//{
//    uint32_t nDiffTicks;
//    uint32_t nSystemCoreTicksPerMicrosec;

//    // Convert (clk speed per sec) to (clk speed per microsec)
//    nSystemCoreTicksPerMicrosec = CLK_SPEED / 1000000;

//    // Elapsed ticks
//    nDiffTicks = nStop - nStart;

//    // Elapsed nanosec = 1000 * (ticks-elapsed / clock-ticks in a microsec)
//    return 1000 * nDiffTicks / nSystemCoreTicksPerMicrosec;
//}
