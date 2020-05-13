/*
 * File: scheduler.c
 *
 * Written by duvallee in 2018
 *
 */
#include "main.h"
#include "string.h"

// ---------------------------------------------------------------------------
#define MAX_SCHEDULER_COUNT                              1                          // max 8
#define SCHEDULER_FLAG_SIZE                              32                         // does not modified

#define RESET_EVENT_FLAG                                 0
#define SET_EVENT_FLAG                                   1

#define UNLOCK                                           0
#define LOCK                                             1

// ---------------------------------------------------------------------------
typedef struct SOFT_TIMER_STRUCT
{
   uint32_t cycle_ms;                                    // 
   uint32_t elapse_ms;
   uint8_t expire_count;
   volatile uint8_t lock;

   TIMER_FN fn;
} SOFT_TIMER;

// ---------------------------------------------------------------------------
typedef struct WAIT_EVENT_STRUCT
{
   uint8_t event;

   volatile uint8_t event_flag;
   volatile uint8_t duplicate_event_flag;
   volatile uint8_t lock;

   uint8_t status;
   WAIT_EVENT_FN fn;
} WAIT_EVENT;

// ---------------------------------------------------------------------------
typedef struct SCHEDULER_STRUCT
{
   uint32_t sheduler_time_ms;

   uint32_t soft_timer_flag[MAX_SCHEDULER_COUNT];

   SOFT_TIMER soft_timer[MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE];
   WAIT_EVENT event[MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE];
} SCHEDULER;

// ---------------------------------------------------------------------------
static SCHEDULER g_scheduler;

// ***************************************************************************
// Fuction      : HAL_SYSTICK_Callback()
// Description  : 
// 
//
// ***************************************************************************
void HAL_SYSTICK_Callback()
{
   uint32_t index                                        = 0;
   uint32_t bits                                         = 0;
   unsigned char i;
   g_scheduler.sheduler_time_ms++;

   for (i = 0; i < (MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE); i++)
   {
      if (g_scheduler.soft_timer[i].cycle_ms != 0)
      {
         if (g_scheduler.soft_timer[i].cycle_ms <= g_scheduler.soft_timer[i].elapse_ms)
         {
             g_scheduler.soft_timer[i].elapse_ms         = 1;
            index                                        = i / SCHEDULER_FLAG_SIZE;
            bits                                         = i % SCHEDULER_FLAG_SIZE;
            g_scheduler.soft_timer_flag[index]           |= (0x1 << bits);
         }
         else
         {
             g_scheduler.soft_timer[i].elapse_ms++;
         }
      }
   }

}


// ***************************************************************************
// Fuction      : scheduler_run()
// Description  : 
// 
//
// ***************************************************************************
void scheduler_run()
{
   uint32_t index                                        = 0;
   uint32_t bits                                         = 0;
   unsigned char i;
   for (i = 0; i < (MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE); i++)
   {
      index                                              = i / SCHEDULER_FLAG_SIZE;
      bits                                               = i % SCHEDULER_FLAG_SIZE;
      if (g_scheduler.event[i].event_flag == SET_EVENT_FLAG)
      {
         g_scheduler.event[i].lock                       = LOCK;
         (*g_scheduler.event[i].fn)(g_scheduler.event[i].status);
         g_scheduler.event[i].lock                       = UNLOCK;
         g_scheduler.event[i].event_flag                 = RESET_EVENT_FLAG;
      }
      else if (g_scheduler.event[i].duplicate_event_flag == SET_EVENT_FLAG)
      {
         g_scheduler.event[i].event_flag                 = SET_EVENT_FLAG;
         g_scheduler.event[i].duplicate_event_flag       = RESET_EVENT_FLAG;
      }
      if ((g_scheduler.soft_timer_flag[index] & (0x1 << bits)) != 0)
      {
         // call
         g_scheduler.soft_timer_flag[index]              &= ~(0x1 << bits);

         if (g_scheduler.soft_timer[i].expire_count != INFINIT_TIMER_COUNT)
         {
            g_scheduler.soft_timer[i].elapse_ms          = 0;
            g_scheduler.soft_timer[i].lock               = LOCK;
            (*g_scheduler.soft_timer[i].fn)(g_scheduler.sheduler_time_ms);
            g_scheduler.soft_timer[i].lock               = UNLOCK;
         }
         else
         {
            if (g_scheduler.soft_timer[i].expire_count == 0)
            {
               g_scheduler.soft_timer[i].cycle_ms        = 0;
               g_scheduler.soft_timer[i].fn              = NULL;
               break;
            }
            g_scheduler.soft_timer[i].expire_count--;
            g_scheduler.soft_timer[i].elapse_ms          = 0;
            g_scheduler.soft_timer[i].lock               = LOCK;
            (*g_scheduler.soft_timer[i].fn)(g_scheduler.sheduler_time_ms);
            g_scheduler.soft_timer[i].lock               = UNLOCK;
         }
      }
   }
}

// ***************************************************************************
// Fuction      : scheduler_init()
// Description  : 
// 
//
// ***************************************************************************
void scheduler_init()
{
   unsigned char i;

   memset(&g_scheduler, 0, sizeof(g_scheduler));
   for (i = 0; i < (MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE); i++)
   {
      g_scheduler.event[i].event                         = NO_EVENT_ID;
      g_scheduler.event[i].event_flag                    = RESET_EVENT_FLAG;
      g_scheduler.event[i].duplicate_event_flag          = RESET_EVENT_FLAG;
      g_scheduler.event[i].lock                          = UNLOCK;
      g_scheduler.event[i].status                        = 0;
      g_scheduler.event[i].fn                            = NULL;
   }
}

// ***************************************************************************
// Fuction      : get_scheduler_time()
// Description  : 
// 
//
// ***************************************************************************
uint32_t get_scheduler_time(void)
{
   return g_scheduler.sheduler_time_ms;
}

// ***************************************************************************
// Fuction      : diff_scheduler_time()
// Description  : 
// 
//
// ***************************************************************************
uint32_t diff_scheduler_time(uint32_t cur_time, uint32_t pre_time)
{
   if (cur_time >= pre_time)
   {
      return (cur_time - pre_time);
   }
   return (cur_time + (0xFFFFFFFF - pre_time));
}


// ***************************************************************************
// Fuction      : add_timer()
// Description  : 
// 
//
// ***************************************************************************
int add_timer(uint32_t timer_ms, uint8_t count, TIMER_FN fn)
{
   unsigned char i;
   for (i = 0; i < (MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE); i++)
   {
      if (g_scheduler.soft_timer[i].fn == NULL)
      {
         g_scheduler.soft_timer[i].cycle_ms              = timer_ms;
         g_scheduler.soft_timer[i].elapse_ms             = 0;
         if (count == 0)
         {
            g_scheduler.soft_timer[i].expire_count       = INFINIT_TIMER_COUNT;
         }
         else
         {
            g_scheduler.soft_timer[i].expire_count       = count;
         }
         g_scheduler.soft_timer[i].lock                  = UNLOCK;
         g_scheduler.soft_timer[i].fn                    = fn;
         return 0;
      }
   }
   return -1;
}

// ***************************************************************************
// Fuction      : add_event()
// Description  : 
// 
//
// ***************************************************************************
uint8_t add_event(uint16_t wait_ms, uint8_t status, WAIT_EVENT_FN fn)
{
   unsigned char i;
   for (i = 0; i < (MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE); i++)
   {
      if (g_scheduler.event[i].event == NO_EVENT_ID)
      {
         g_scheduler.event[i].event                      = i;
         g_scheduler.event[i].event_flag                 = RESET_EVENT_FLAG;
         g_scheduler.event[i].duplicate_event_flag       = RESET_EVENT_FLAG;
         g_scheduler.event[i].lock                       = UNLOCK;
         g_scheduler.event[i].status                     = status;
         g_scheduler.event[i].fn                         = fn;
         return (g_scheduler.event[i].event);
      }
   }
   return NO_EVENT_ID;
}

// ***************************************************************************
// Fuction      : get_event_count()
// Description  : 
// 
//
// ***************************************************************************
int get_event_count(void)
{
   int count                                             = 0;
   unsigned char i;
   for (i = 0; i < (MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE); i++)
   {
      if (g_scheduler.event[i].event != NO_EVENT_ID)
      {
         count++;
      }
   }
   return count;
}

// ***************************************************************************
// Fuction      : set_event()
// Description  : 
// 
//
// ***************************************************************************
uint8_t set_event(uint8_t event, uint8_t status)
{
   if (event >= (MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE))
   {
      return NO_EVENT_ID;
   }
   if (g_scheduler.event[event].event != event)
   {
      return NO_EVENT_ID;
   }
   if (g_scheduler.event[event].lock == LOCK)
   {
      g_scheduler.event[event].duplicate_event_flag      = SET_EVENT_FLAG;
     
   }
   g_scheduler.event[event].event_flag                   = SET_EVENT_FLAG;
   g_scheduler.event[event].status                       = status;
   return (g_scheduler.event[event].event);
}

// ***************************************************************************
// Fuction      : get_event_status()
// Description  : 
// 
//
// ***************************************************************************
uint8_t get_event_status(uint8_t event)
{
   if (event >= (MAX_SCHEDULER_COUNT * SCHEDULER_FLAG_SIZE))
   {
      return NO_EVENT_ID;
   }
   if (g_scheduler.event[event].event != event)
   {
      return NO_EVENT_ID;
   }
   return (g_scheduler.event[event].status);
}



