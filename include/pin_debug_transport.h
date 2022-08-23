/*
 * Copyright (c) Nordic Semiconductor ASA. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor ASA.
 * The use, copying, transfer or disclosure of such information is prohibited except by
 * express written agreement with Nordic Semiconductor ASA.
 */

/**
  @addtogroup pin_debug_transport
  @{
  @defgroup pin_debug_transport_interface PIN_DEBUG_TRANSPORT_INTERFACE
  @{

  @file

  @brief The physical layer of the debug system.

*/

/* Header guard */
#ifndef PIN_DEBUG_TRANSPORT_H__
#define PIN_DEBUG_TRANSPORT_H__
#ifdef NRF5340_XXAA_NETWORK
#ifndef NRF53_SERIES
#define NRF53_SERIES
#endif
#endif
#include "nrf.h"
#include "nrf_peripherals.h"
#ifdef NRF53_SERIES
#ifdef NRF5340_XXAA_NETWORK
#include "mpsl_dppi_protocol_api.h"
#ifndef CU_TEST
#define NRF_P0          NRF_P0_NS
#define NRF_P1          NRF_P1_NS
#define NRF_GPIOTE      NRF_GPIOTE_NS
#define NRF_TIMER0      NRF_TIMER0_NS
#define NRF_RADIO       NRF_RADIO_NS
#endif
#endif
#endif

/* For production code, PIN_DEBUG_ENABLE is not defined, causing all debug pin macros to be
 * defined to cause a compilation error */
#ifdef PIN_DEBUG_ENABLE


#if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
#define GPIO_PIN_END   50
#else
#define GPIO_PIN_END   31
#endif

#define GPIO_PIN_START 0
#define NUM_DEBUG_PINS 16

#ifdef NRF53_SERIES
#define DBP0_PIN  (4)
#define DBP1_PIN  (5)
#define DBP2_PIN  (6)
#define DBP3_PIN  (7)
#define DBP4_PIN  (25)
#define DBP5_PIN  (26)
#define DBP6_PIN  (32+0)
#define DBP7_PIN  (32+10)
#define DBP8_PIN  (32+11)
#define DBP9_PIN  (32+12)
#define DBP10_PIN (32+13)
#define DBP11_PIN (32+14)
#define DBP12_PIN (32+15)
#define DBP13_PIN (27)
#define DBP14_PIN (32+2)
#define DBP15_PIN (32+3)
#elif defined(NRF52840_XXAA)
#define DBP0_PIN  (3)
#define DBP1_PIN  (4)
#define DBP2_PIN  (28)
#define DBP3_PIN  (29)
#define DBP4_PIN  (30)
#define DBP5_PIN  (31)
#define DBP6_PIN  (32+1)
#define DBP7_PIN  (32+10)
#define DBP8_PIN  (32+11)
#define DBP9_PIN  (32+12)
#define DBP10_PIN (32+13)
#define DBP11_PIN (32+14)
#define DBP12_PIN (32+15)
#define DBP13_PIN (2)
#define DBP14_PIN (26)
#define DBP15_PIN (27)
#else
#define DBP0_PIN  (3)
#define DBP1_PIN  (4)
#define DBP2_PIN  (28)
#define DBP3_PIN  (29)
#define DBP4_PIN  (30)
#define DBP5_PIN  (31)
#define DBP6_PIN  (11)
#define DBP7_PIN  (19)
#define DBP8_PIN  (20)
#define DBP9_PIN  (22)
#define DBP10_PIN (23)
#define DBP11_PIN (24)
#define DBP12_PIN (25)
#define DBP13_PIN (2)
#define DBP14_PIN (26)
#define DBP15_PIN (27)
#endif

/* Introducing a delay to slow down pin toggling where needed */
#define DBP_DELAY {__NOP(); __NOP(); __NOP(); __NOP();}

#define DBP0  (1ULL << DBP0_PIN)
#define DBP1  (1ULL << DBP1_PIN)
#define DBP2  (1ULL << DBP2_PIN)
#define DBP3  (1ULL << DBP3_PIN)
#define DBP4  (1ULL << DBP4_PIN)
#define DBP5  (1ULL << DBP5_PIN)
#define DBP6  (1ULL << DBP6_PIN)
#define DBP7  (1ULL << DBP7_PIN)
#define DBP8  (1ULL << DBP8_PIN)
#define DBP9  (1ULL << DBP9_PIN)
#define DBP10 (1ULL << DBP10_PIN)
#define DBP11 (1ULL << DBP11_PIN)
#define DBP12 (1ULL << DBP12_PIN)
#define DBP13 (1ULL << DBP13_PIN)
#define DBP14 (1ULL << DBP14_PIN)
#define DBP15 (1ULL << DBP15_PIN)

/* Three interfaces are supported and can be used interchangeably to set and clear single pins:
 * e.g.
 * DBP0_ON,  DBP_ON(0) & DEBUG_PIN_ON(DBP0)  will all cause Pin 0 to be set
 * DBP0_OFF, DBP_OFF(0)& DEBUG_PIN_OFF(DBP0) will all cause Pin 0 to be cleared
 *
 * The third interface, DEBUG_PIN_ON(x)/DEBUG_PIN_OFF(x), supports driving multiple pins at the same time
 * e.g.
 * DEBUG_PIN_ON(DBP0|DBP1) will cause Pin 0 and Pin 1 to be set simultaneously
 */

/* (1) Raw interface - Direct access to GPIO registers */

#define GPIO_PIN_CNF_DEFAULT  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | \
                              (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) | \
                              (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos) | \
                              (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)

#define DBP_CNF_IN(_pin)    {(NRF_P0->PIN_CNF[_pin] = GPIO_PIN_CNF_DEFAULT | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos));}
#define DBP_CNF_IN_P1(_pin) {(NRF_P1->PIN_CNF[_pin] = GPIO_PIN_CNF_DEFAULT | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos));}
#define DBP_CNF_OUT(_pin)   {(NRF_P0->PIN_CNF[_pin] = GPIO_PIN_CNF_DEFAULT | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));}
#define DBP_CNF_OUT_P1(_pin){(NRF_P1->PIN_CNF[_pin] = GPIO_PIN_CNF_DEFAULT | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));}
static inline void debug_pin_outcnf(uint64_t bitf)
{
  for (uint8_t _i = GPIO_PIN_START; _i <= GPIO_PIN_END; _i++)
  {
    if (bitf & ((uint64_t)1<<_i))
    {
      if (_i <32)
      {
        DBP_CNF_OUT(_i);
      }
      else
      {
        #if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
        DBP_CNF_OUT_P1(_i)
        #endif
      }
    }
  }
};

static inline void debug_pin_incnf(uint64_t bitf)
{
  for (uint8_t _i = GPIO_PIN_START; _i <= GPIO_PIN_END; _i++)
  {
    if (bitf & ((uint64_t)1<<_i))
    {
      if (_i <32)
      {
        DBP_CNF_IN(_i);
      }
      else
      {
        #if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
        DBP_CNF_IN_P1(_i)
        #endif
      }
    }
  }
};

static inline void debug_pin_enable(uint64_t bitf)
{
  debug_pin_outcnf(bitf);

  uint32_t p0 = (uint32_t)(bitf & 0xFFFFFFFF);
  NRF_P0->DIRSET = p0;
  NRF_P0->OUTCLR = p0;
  #if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
  uint32_t p1 = (uint32_t)((bitf>>32) & 0xFFFFFFFF);
  NRF_P1->DIRSET = p1;
  NRF_P1->OUTCLR = p1;
  #endif
};

static inline void debug_pin_disable(uint64_t bitf)
{
  debug_pin_incnf(bitf);

  uint32_t p0 = (uint32_t)(bitf & 0xFFFFFFFF);
  NRF_P0->DIRCLR = p0;
  NRF_P0->OUTCLR = p0;
  #if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
  uint32_t p1 = (uint32_t)((bitf>>32) & 0xFFFFFFFF);
  NRF_P1->DIRCLR = p1;
  NRF_P1->OUTCLR = p1;
  #endif
}

/* Make a GPIO pin accessible from the network core instead of the application core.
 * Must be called from the application core, and before enabling the pin on the network core.
 */
static inline void debug_pin_netcore(uint32_t pin)
{
#ifdef NRF53_SERIES
  if (pin < 32)
  {
    NRF_P0->PIN_CNF[pin] |=
      GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
  }
  else
  {
    pin = pin - 32;
    NRF_P1->PIN_CNF[pin] |=
      GPIO_PIN_CNF_MCUSEL_NetworkMCU << GPIO_PIN_CNF_MCUSEL_Pos;
  }
#endif
}

static inline void debug_pin_gpiote_cnf(uint32_t pin, uint32_t gpiote_ch, uint32_t polarity)
{
#if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
  uint32_t port;
  if (pin < 32)
  {
    port = 0;
  }
  else
  {
    port = 1;
    pin = pin - 32;
  }
#endif
  NRF_GPIOTE->CONFIG[gpiote_ch] =
    (polarity << GPIOTE_CONFIG_POLARITY_Pos) |
#if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
    (port                          << GPIOTE_CONFIG_PORT_Pos)     |
#endif
    (pin                           << GPIOTE_CONFIG_PSEL_Pos)     |
    (GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos)     |
    (0                             << GPIOTE_CONFIG_OUTINIT_Pos);
}

#ifdef NRF53_SERIES
/* These defines are taken from hal_ppi_nrf53.h and should be kept up to date
 * with that file.
 */
#define HAL_DPPI_REM_EVENTS_START_CHANNEL_IDX     3U
#define HAL_DPPI_RADIO_EVENTS_READY_CHANNEL_IDX   MPSL_DPPI_RADIO_PUBLISH_READY_CHANNEL_IDX
#define HAL_DPPI_RADIO_EVENTS_ADDRESS_CHANNEL_IDX MPSL_DPPI_RADIO_PUBLISH_ADDRESS_CHANNEL_IDX
#define HAL_DPPI_RADIO_EVENTS_END_CHANNEL_IDX     MPSL_DPPI_RADIO_PUBLISH_END_CHANNEL_IDX
#define HAL_DPPI_RADIO_EVENTS_DISABLED_CH_IDX     MPSL_DPPI_RADIO_PUBLISH_DISABLED_CH_IDX
#define HAL_DPPI_RADIO_EVENTS_BCMATCH_IDX         10U
/* Since events can only be connected to one DPPI channel on nRF53, the correct
 * channels must be used for events with fixed channels.
 * If you wish to use one event on more than one pin, it can be added with a
 * fixed channel here, but collision with the channels assigned automatically in
 * this file must be avoided.
 */
static inline uint32_t fixed_channel_check(volatile uint32_t * event)
{
  if (event == &NRF_TIMER0->PUBLISH_COMPARE[0])
  {
    return HAL_DPPI_REM_EVENTS_START_CHANNEL_IDX;
  }
#ifdef NRF5340_XXAA_NETWORK
  else if (event == &NRF_RADIO->PUBLISH_READY)
  {
    return HAL_DPPI_RADIO_EVENTS_READY_CHANNEL_IDX;
  }
  else if (event == &NRF_RADIO->PUBLISH_ADDRESS)
  {
    return HAL_DPPI_RADIO_EVENTS_ADDRESS_CHANNEL_IDX;
  }
  else if (event == &NRF_RADIO->PUBLISH_END)
  {
    return HAL_DPPI_RADIO_EVENTS_END_CHANNEL_IDX;
  }
  else if (event == &NRF_RADIO->PUBLISH_DISABLED)
  {
    return HAL_DPPI_RADIO_EVENTS_DISABLED_CH_IDX;
  }
  else if (event == &NRF_RADIO->PUBLISH_BCMATCH)
  {
    return HAL_DPPI_RADIO_EVENTS_BCMATCH_IDX;
  }
#endif
  else
  {
    return 0;
  }
}

#define SUBSCRIBE_EN_Pos (31UL)                      /*!< Position of EN field (this is the same for all SUBSCRIBE-registers) */
#define SUBSCRIBE_EN_Msk (0x1UL << SUBSCRIBE_EN_Pos) /*!< Bit mask of EN field. */
#define PUBLISH_EN_Pos   (31UL)                      /*!< Position of EN field (this is the same for all PUBLISH-registers) */
#define PUBLISH_EN_Msk   (0x1UL << PUBLISH_EN_Pos)   /*!< Bit mask of EN field. */

#endif /* NRF53_SERIES */

static inline void debug_pin_ppi_cnf(uint32_t ppi_ch, volatile uint32_t * event, volatile uint32_t * task)
{
#if defined(DPPI_PRESENT)
  *task = ppi_ch | SUBSCRIBE_EN_Msk;
  *event = ppi_ch | PUBLISH_EN_Msk;
  NRF_DPPIC_NS->CHENSET = (1 << ppi_ch);
#elif defined(PPI_PRESENT)
  NRF_PPI->CH[ppi_ch].TEP    = (uint32_t)(task);
  NRF_PPI->CH[ppi_ch].EEP    = (uint32_t)(event);
  NRF_PPI->CHENSET = (1 << ppi_ch);
#else
  #error "Neither PPI nor DPPI are present"
#endif
}

/* NOTE:
 * Since these are static inline in a header file, all setup should be done from the same file.
 * Otherwise the PPI channel would restart counting from zero and configuration would be overwritten.
 */
#ifdef NRF53_SERIES
static uint32_t ppi_ch = 14;
#else
static uint32_t ppi_ch = 0;
#endif /* NRF53_SERIES */
static uint32_t gpiote_ch = 0;

#ifdef NRF53_SERIES
typedef struct
{
  volatile uint32_t * gpiote_task;
  volatile uint32_t * event;
}not_fixed_config;
static not_fixed_config m_not_fixed_config[8];
static uint32_t m_i;
#endif

/* Create a pulse on GPIO pins for two events, setting the GPIO pin high on one event, and clearing it on the other.
 * Example for nRF52: debug_pin_event_pulse(DBP_PIN(5), &NRF_RADIO->EVENTS_READY, &NRF_RADIO->EVENTS_DISABLED);
 * Example for nRF53: debup_pin_event_pulse(DBP_PIN(5), &NRF_RADIO->PUBLISH_READY, &NRF_RADIO->PUBLISH_DISABLED);
 * Note: On NRF53, all radio registers, including NRF_RADIO->PUBLISH_* are reset before every radio event.
 * In order to use for example TX_READY, you need to handle that. One way can be to do apply this patch manualy
 * (be aware of side-effects depending on what you're debugging):
 * |   +++ b/stack/libs/hal/src/hal_rcs.c
 * |  -  NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);
 * |  -  NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Enabled  << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);
 * |  +  //NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);
 * |  +  //NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Enabled  << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);
 */
static inline void debug_pin_event_pulse(uint32_t pin, volatile uint32_t * event_set, volatile uint32_t * event_clr)
{
#ifdef NRF53_SERIES
  uint32_t fixed_chan_set = fixed_channel_check(event_set);
  if (fixed_chan_set)
  {
    debug_pin_ppi_cnf(fixed_chan_set, event_set, &NRF_GPIOTE->SUBSCRIBE_SET[gpiote_ch]);
  }
  else
  {
    m_not_fixed_config[m_i].gpiote_task = &NRF_GPIOTE->SUBSCRIBE_SET[gpiote_ch];
    m_not_fixed_config[m_i].event = event_set;
    debug_pin_ppi_cnf(ppi_ch++, m_not_fixed_config[m_i].event, m_not_fixed_config[m_i].gpiote_task );
    m_i++;
  }
  uint32_t fixed_chan_clr = fixed_channel_check(event_clr);
  if (fixed_chan_clr)
  {
    debug_pin_ppi_cnf(fixed_chan_clr, event_clr, &NRF_GPIOTE->SUBSCRIBE_CLR[gpiote_ch]);
  }
  else
  {
    m_not_fixed_config[m_i].gpiote_task = &NRF_GPIOTE->SUBSCRIBE_CLR[gpiote_ch];
    m_not_fixed_config[m_i].event = event_clr;
    debug_pin_ppi_cnf(ppi_ch++, m_not_fixed_config[m_i].event, m_not_fixed_config[m_i].gpiote_task );
    m_i++;
  }
#else
  debug_pin_ppi_cnf(ppi_ch++, event_set, &NRF_GPIOTE->TASKS_SET[gpiote_ch]);
  debug_pin_ppi_cnf(ppi_ch++, event_clr, &NRF_GPIOTE->TASKS_CLR[gpiote_ch]);
#endif /* NRF53_SERIES */
  debug_pin_gpiote_cnf(pin, gpiote_ch++, GPIOTE_CONFIG_POLARITY_None);
}

/* Toggle an event out to a GPIO pin.
 * Example for nRF52: debug_pin_event(DBP_PIN(5), &NRF_RADIO->EVENTS_READY);
 * Example for nRF53: debug_pin_event(DBP_PIN(5), &NRF_RADIO->PUBLISH_READY);
 * Note: On NRF53, all radio registers, including NRF_RADIO->PUBLISH_* are reset before every radio event.
 * In order to use for example TX_READY, you need to handle that. One way can be to do apply this patch manualy
 * (be aware of side-effects depending on what you're debugging):
 * |   +++ b/stack/libs/hal/src/hal_rcs.c
 * |  -  NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);
 * |  -  NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Enabled  << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);
 * |  +  //NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);
 * |  +  //NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Enabled  << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);
 */
static inline void debug_pin_event(uint32_t pin, volatile uint32_t * event)
{
#ifdef NRF53_SERIES
  uint32_t fixed_channel = fixed_channel_check(event);
  if (fixed_channel)
  {
    debug_pin_ppi_cnf(fixed_channel, event, &NRF_GPIOTE->SUBSCRIBE_OUT[gpiote_ch]);
  }
  else
  {
    m_not_fixed_config[m_i].gpiote_task = &NRF_GPIOTE->SUBSCRIBE_OUT[gpiote_ch];
    m_not_fixed_config[m_i].event = event;
    debug_pin_ppi_cnf(ppi_ch++, m_not_fixed_config[m_i].event, m_not_fixed_config[m_i].gpiote_task );
    m_i++;
  }
#else
  debug_pin_ppi_cnf(ppi_ch++, event, &NRF_GPIOTE->TASKS_OUT[gpiote_ch]);
#endif /* NRF53_SERIES */
  debug_pin_gpiote_cnf(pin, gpiote_ch++, GPIOTE_CONFIG_POLARITY_Toggle);
}

/* Function to write all !fixed debug pins PUBLISH registers back to what they where set to originaly.
 * Some Publish register (&NRF_TIMER0_NS->PUBLISH_COMPARE[1]), are dynamicaly changed, and DPPI chan enabled/disabled during a radio event.
 * In order to use these registers, you can call this function. A nice place to add it is in rem.c, at the end of rem_sig_end().
 *
 */
static inline void debug_pin_reconfigure(void)
{
#ifdef NRF53_SERIES
  for (uint32_t i= 0; i < m_i; i++)
  {
    *m_not_fixed_config[i].event = *m_not_fixed_config[i].gpiote_task;
  }
#endif
}

/* Toggle GPIO pin when the ppi_channel is triggered
 * Example on nRF52: To get debug pin 8 toggled every time timer0 is cleared by pre-programed
 * ppi channel 30(RTC0->EVENTS_COMPARE[0]->TIMER0->TASKS_CLEAR):
 * debug_pin_event_fork(DBP_PIN(8), 30);
 */
static inline void debug_pin_event_fork(uint32_t pin, uint32_t ppi_channel)
{
#ifdef NRF53_SERIES
  NRF_GPIOTE->SUBSCRIBE_OUT[gpiote_ch] = ppi_channel | SUBSCRIBE_EN_Msk;
#else
  NRF_PPI->FORK[ppi_channel].TEP    = (uint32_t)(&NRF_GPIOTE->TASKS_OUT[gpiote_ch]);
#endif
  debug_pin_gpiote_cnf(pin, gpiote_ch++, GPIOTE_CONFIG_POLARITY_Toggle);
}

#ifndef NRF53_SERIES
/* Make MWU generate a event when SoftDevice writes to a an address.

  This should only be used with SoftDevices which dont use MWU themselfs.
  Which means softdevices without include feature mwu (from softdevice_variants_config.json)

  Example to get MWU->0 event when SoftDevice writes to NRF_RADIO->PACKETPTR
  debug_event_mwu(0, &NRF_RADIO->PACKETPTR);

  That event can then be used to toggle a pin like this:
  debug_pin_event(DBP_PIN(2), &NRF_MWU->EVENTS_REGION[0].WA);
  */
static inline void debug_event_mwu(uint32_t region, volatile uint32_t *address)
{
  #if defined(NRF_MWU)
  NRF_MWU->REGION[region].START = (uint32_t)address;
  NRF_MWU->REGION[region].END = ((uint32_t)address) + 3;
  NRF_MWU->REGIONENSET = 1 << (region*2);
  #endif
}
#endif /* NRF53_SERIES */

#define DEBUG_PIN_ENABLE(_bitf1)  {debug_pin_enable(_bitf1);}
#define DEBUG_PIN_DISABLE(_bitf2) {debug_pin_disable(_bitf2);}

static inline void debug_pin_on(uint64_t bitf)
{
  uint32_t p0 = (uint32_t)(bitf & 0xFFFFFFFF);
  NRF_P0->OUTSET = p0;
  #if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
  uint32_t p1 = (uint32_t)((bitf>>32) & 0xFFFFFFFF);
  NRF_P1->OUTSET = p1;
  #endif
}

static inline void debug_pin_off(uint64_t bitf)
{
  uint32_t p0 = (uint32_t)(bitf & 0xFFFFFFFF);
  NRF_P0->OUTCLR = p0;
  #if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
  uint32_t p1 = (uint32_t)((bitf>>32) & 0xFFFFFFFF);
  NRF_P1->OUTCLR = p1;
  #endif
}

static inline void debug_pin_toggle(uint64_t bitf)
{
  uint32_t p0 = (uint32_t)(bitf & 0xFFFFFFFF);
  uint32_t p0_state = NRF_P0->OUT;
  NRF_P0->OUTCLR =   p0_state  & p0;
  NRF_P0->OUTSET = (~p0_state) & p0;
  #if defined(NRF52840_XXAA) || defined(NRF53_SERIES)
  uint32_t p1 = (uint32_t)((bitf>>32) & 0xFFFFFFFF);
  uint32_t p1_state = NRF_P1->OUT;
  NRF_P1->OUTCLR =   p1_state  & p1;
  NRF_P1->OUTSET = (~p1_state) & p1;
  #endif
}

#define DEBUG_PIN_ON(_bitf)   {debug_pin_on(_bitf);}
#define DEBUG_PIN_OFF(_bitf)  {debug_pin_off(_bitf);}
#define DEBUG_PIN_TOGGLE(_bitf) {debug_pin_toggle(_bitf);}


/* (2) Enumarate interface - Access using pin indexes */
static const uint32_t dbp_pin_mapping[NUM_DEBUG_PINS] =
{
  DBP0_PIN,
  DBP1_PIN,
  DBP2_PIN,
  DBP3_PIN,
  DBP4_PIN,
  DBP5_PIN,
  DBP6_PIN,
  DBP7_PIN,
  DBP8_PIN,
  DBP9_PIN,
  DBP10_PIN,
  DBP11_PIN,
  DBP12_PIN,
  DBP13_PIN,
  DBP14_PIN,
  DBP15_PIN
};

static const uint64_t dbp_mapping[NUM_DEBUG_PINS] =
{
  DBP0,
  DBP1,
  DBP2,
  DBP3,
  DBP4,
  DBP5,
  DBP6,
  DBP7,
  DBP8,
  DBP9,
  DBP10,
  DBP11,
  DBP12,
  DBP13,
  DBP14,
  DBP15
};

#define DBP_PORTA_START 0
#define DBP_PORTA_END   7
#define DBP_PORTB_START 8
#define DBP_PORTB_END   15
#define DBP_PORTS_START DBP_PORTA_START
#define DBP_PORTS_END   DBP_PORTB_END

#define DBP_ENABLE(_pin)  DEBUG_PIN_ENABLE(dbp_mapping[_pin])
#define DBP_DISABLE(_pin) DEBUG_PIN_DISABLE(dbp_mapping[_pin])

#define DBP_ON(_pin)  DEBUG_PIN_ON(dbp_mapping[_pin])
#define DBP_OFF(_pin) DEBUG_PIN_OFF(dbp_mapping[_pin])
#define DBP_PIN(_pin) dbp_pin_mapping[_pin]
#define DBP_TOGGLE(_pin) DEBUG_PIN_TOGGLE(dbp_mapping[_pin])

/* (3) Individual macro interface - Access using separate macros */
#define DBP_PORTA ((uint64_t)(DBP0 | DBP1 | DBP2  | DBP3  | DBP4  | DBP5  | DBP6  | DBP7))
#define DBP_PORTB ((uint64_t)(DBP8 | DBP9 | DBP10 | DBP11 | DBP12 | DBP13 | DBP14 | DBP15))
#define DBP_PORTS (DBP_PORTA | DBP_PORTB)

#define DBP_PORTA_ENABLE  DEBUG_PIN_ENABLE(DBP_PORTA)
#define DBP_PORTA_DISABLE DEBUG_PIN_DISABLE(DBP_PORTA)
#define DBP_PORTB_ENABLE  DEBUG_PIN_ENABLE(DBP_PORTB)
#define DBP_PORTB_DISABLE DEBUG_PIN_DISABLE(DBP_PORTB)
#define DBP_PORTS_ENABLE  DEBUG_PIN_ENABLE(DBP_PORTS)
#define DBP_PORTS_DISABLE DEBUG_PIN_DISABLE(DBP_PORTS)

#define DBP_PORTA_ON  DEBUG_PIN_ON(DBP_PORTA)
#define DBP_PORTA_OFF DEBUG_PIN_OFF(DBP_PORTA)
#define DBP_PORTB_ON  DEBUG_PIN_ON(DBP_PORTB)
#define DBP_PORTB_OFF DEBUG_PIN_OFF(DBP_PORTB)
#define DBP_PORTS_ON  DEBUG_PIN_ON(DBP_PORTS)
#define DBP_PORTS_OFF DEBUG_PIN_OFF(DBP_PORTS)

#define DBP0_ENABLE   DEBUG_PIN_ENABLE(DBP0)
#define DBP0_DISABLE  DEBUG_PIN_DISABLE(DBP0)
#define DBP1_ENABLE   DEBUG_PIN_ENABLE(DBP1)
#define DBP1_DISABLE  DEBUG_PIN_DISABLE(DBP1)
#define DBP2_ENABLE   DEBUG_PIN_ENABLE(DBP2)
#define DBP2_DISABLE  DEBUG_PIN_DISABLE(DBP2)
#define DBP3_ENABLE   DEBUG_PIN_ENABLE(DBP3)
#define DBP3_DISABLE  DEBUG_PIN_DISABLE(DBP3)
#define DBP4_ENABLE   DEBUG_PIN_ENABLE(DBP4)
#define DBP4_DISABLE  DEBUG_PIN_DISABLE(DBP4)
#define DBP5_ENABLE   DEBUG_PIN_ENABLE(DBP5)
#define DBP5_DISABLE  DEBUG_PIN_DISABLE(DBP5)
#define DBP6_ENABLE   DEBUG_PIN_ENABLE(DBP6)
#define DBP6_DISABLE  DEBUG_PIN_DISABLE(DBP6)
#define DBP7_ENABLE   DEBUG_PIN_ENABLE(DBP7)
#define DBP7_DISABLE  DEBUG_PIN_DISABLE(DBP7)
#define DBP8_ENABLE   DEBUG_PIN_ENABLE(DBP8)
#define DBP8_DISABLE  DEBUG_PIN_DISABLE(DBP8)
#define DBP9_ENABLE   DEBUG_PIN_ENABLE(DBP9)
#define DBP9_DISABLE  DEBUG_PIN_DISABLE(DBP9)
#define DBP10_ENABLE  DEBUG_PIN_ENABLE(DBP10)
#define DBP10_DISABLE DEBUG_PIN_DISABLE(DBP10)
#define DBP11_ENABLE  DEBUG_PIN_ENABLE(DBP11)
#define DBP11_DISABLE DEBUG_PIN_DISABLE(DBP11)
#define DBP12_ENABLE  DEBUG_PIN_ENABLE(DBP12)
#define DBP12_DISABLE DEBUG_PIN_DISABLE(DBP12)
#define DBP13_ENABLE  DEBUG_PIN_ENABLE(DBP13)
#define DBP13_DISABLE DEBUG_PIN_DISABLE(DBP13)
#define DBP14_ENABLE  DEBUG_PIN_ENABLE(DBP14)
#define DBP14_DISABLE DEBUG_PIN_DISABLE(DBP14)
#define DBP15_ENABLE  DEBUG_PIN_ENABLE(DBP15)
#define DBP15_DISABLE DEBUG_PIN_DISABLE(DBP15)

#define DBP0_ON   DEBUG_PIN_ON(DBP0)
#define DBP0_OFF  DEBUG_PIN_OFF(DBP0)
#define DBP1_ON   DEBUG_PIN_ON(DBP1)
#define DBP1_OFF  DEBUG_PIN_OFF(DBP1)
#define DBP2_ON   DEBUG_PIN_ON(DBP2)
#define DBP2_OFF  DEBUG_PIN_OFF(DBP2)
#define DBP3_ON   DEBUG_PIN_ON(DBP3)
#define DBP3_OFF  DEBUG_PIN_OFF(DBP3)
#define DBP4_ON   DEBUG_PIN_ON(DBP4)
#define DBP4_OFF  DEBUG_PIN_OFF(DBP4)
#define DBP5_ON   DEBUG_PIN_ON(DBP5)
#define DBP5_OFF  DEBUG_PIN_OFF(DBP5)
#define DBP6_ON   DEBUG_PIN_ON(DBP6)
#define DBP6_OFF  DEBUG_PIN_OFF(DBP6)
#define DBP7_ON   DEBUG_PIN_ON(DBP7)
#define DBP7_OFF  DEBUG_PIN_OFF(DBP7)
#define DBP8_ON   DEBUG_PIN_ON(DBP8)
#define DBP8_OFF  DEBUG_PIN_OFF(DBP8)
#define DBP9_ON   DEBUG_PIN_ON(DBP9)
#define DBP9_OFF  DEBUG_PIN_OFF(DBP9)
#define DBP10_ON  DEBUG_PIN_ON(DBP10)
#define DBP10_OFF DEBUG_PIN_OFF(DBP10)
#define DBP11_ON  DEBUG_PIN_ON(DBP11)
#define DBP11_OFF DEBUG_PIN_OFF(DBP11)
#define DBP12_ON  DEBUG_PIN_ON(DBP12)
#define DBP12_OFF DEBUG_PIN_OFF(DBP12)
#define DBP13_ON  DEBUG_PIN_ON(DBP13)
#define DBP13_OFF DEBUG_PIN_OFF(DBP13)
#define DBP14_ON  DEBUG_PIN_ON(DBP14)
#define DBP14_OFF DEBUG_PIN_OFF(DBP14)
#define DBP15_ON  DEBUG_PIN_ON(DBP15)
#define DBP15_OFF DEBUG_PIN_OFF(DBP15)

#else
/* If PIN_DEBUG_ENABLE is not defined, all of the macros will force a compilation error */

#define FORCE_ERROR char pin_debug_enable_is_undefined[-1];


#define GPIO_PIN_START            FORCE_ERROR
#define GPIO_PIN_END              FORCE_ERROR
#define NUM_DEBUG_PINS            FORCE_ERROR

#define DBP0                      FORCE_ERROR
#define DBP1                      FORCE_ERROR
#define DBP2                      FORCE_ERROR
#define DBP3                      FORCE_ERROR
#define DBP4                      FORCE_ERROR
#define DBP5                      FORCE_ERROR
#define DBP6                      FORCE_ERROR
#define DBP7                      FORCE_ERROR
#define DBP8                      FORCE_ERROR
#define DBP9                      FORCE_ERROR
#define DBP10                     FORCE_ERROR
#define DBP11                     FORCE_ERROR
#define DBP12                     FORCE_ERROR
#define DBP13                     FORCE_ERROR
#define DBP14                     FORCE_ERROR
#define DBP15                     FORCE_ERROR

#define DBP_DELAY                 FORCE_ERROR

#define GPIO_PIN_CNF_DEFAULT      FORCE_ERROR

#define DBP_CNF_IN(_pin)          FORCE_ERROR
#define DBP_CNF_OUT(_pin)         FORCE_ERROR

#define DEBUG_PIN_ENABLE(_bitf1)  FORCE_ERROR
#define DEBUG_PIN_DISABLE(_bitf2) FORCE_ERROR

#define DEBUG_PIN_ON(_bitf)       FORCE_ERROR
#define DEBUG_PIN_OFF(_bitf)      FORCE_ERROR
#define DEBUG_PIN_TOGGLE(_bitf)   FORCE_ERROR

#define DBP_PORTA_START           FORCE_ERROR
#define DBP_PORTA_END             FORCE_ERROR
#define DBP_PORTB_START           FORCE_ERROR
#define DBP_PORTB_END             FORCE_ERROR
#define DBP_PORTS_START           FORCE_ERROR
#define DBP_PORTS_END             FORCE_ERROR

#define DBP_ENABLE(_pin)          FORCE_ERROR
#define DBP_DISABLE(_pin)         FORCE_ERROR

#define DBP_ON(_pin)              FORCE_ERROR
#define DBP_OFF(_pin)             FORCE_ERROR
#define DBP_TOGGLE(_pin)          FORCE_ERROR

#define DBP_PORTA                 FORCE_ERROR
#define DBP_PORTB                 FORCE_ERROR
#define DBP_PORTS                 FORCE_ERROR

#define DBP_PORTA_ENABLE          FORCE_ERROR
#define DBP_PORTA_DISABLE         FORCE_ERROR
#define DBP_PORTB_ENABLE          FORCE_ERROR
#define DBP_PORTB_DISABLE         FORCE_ERROR
#define DBP_PORTS_ENABLE          FORCE_ERROR
#define DBP_PORTS_DISABLE         FORCE_ERROR

#define DBP_PORTA_ON              FORCE_ERROR
#define DBP_PORTA_OFF             FORCE_ERROR
#define DBP_PORTB_ON              FORCE_ERROR
#define DBP_PORTB_OFF             FORCE_ERROR
#define DBP_PORTS_ON              FORCE_ERROR
#define DBP_PORTS_OFF             FORCE_ERROR

#define DBP0_ENABLE               FORCE_ERROR
#define DBP0_DISABLE              FORCE_ERROR
#define DBP1_ENABLE               FORCE_ERROR
#define DBP1_DISABLE              FORCE_ERROR
#define DBP2_ENABLE               FORCE_ERROR
#define DBP2_DISABLE              FORCE_ERROR
#define DBP3_ENABLE               FORCE_ERROR
#define DBP3_DISABLE              FORCE_ERROR
#define DBP4_ENABLE               FORCE_ERROR
#define DBP4_DISABLE              FORCE_ERROR
#define DBP5_ENABLE               FORCE_ERROR
#define DBP5_DISABLE              FORCE_ERROR
#define DBP6_ENABLE               FORCE_ERROR
#define DBP6_DISABLE              FORCE_ERROR
#define DBP7_ENABLE               FORCE_ERROR
#define DBP7_DISABLE              FORCE_ERROR
#define DBP8_ENABLE               FORCE_ERROR
#define DBP8_DISABLE              FORCE_ERROR
#define DBP9_ENABLE               FORCE_ERROR
#define DBP9_DISABLE              FORCE_ERROR
#define DBP10_ENABLE              FORCE_ERROR
#define DBP10_DISABLE             FORCE_ERROR
#define DBP11_ENABLE              FORCE_ERROR
#define DBP11_DISABLE             FORCE_ERROR
#define DBP12_ENABLE              FORCE_ERROR
#define DBP12_DISABLE             FORCE_ERROR
#define DBP13_ENABLE              FORCE_ERROR
#define DBP13_DISABLE             FORCE_ERROR
#define DBP14_ENABLE              FORCE_ERROR
#define DBP14_DISABLE             FORCE_ERROR
#define DBP15_ENABLE              FORCE_ERROR
#define DBP15_DISABLE             FORCE_ERROR

#define DBP0_ON                   FORCE_ERROR
#define DBP0_OFF                  FORCE_ERROR
#define DBP1_ON                   FORCE_ERROR
#define DBP1_OFF                  FORCE_ERROR
#define DBP2_ON                   FORCE_ERROR
#define DBP2_OFF                  FORCE_ERROR
#define DBP3_ON                   FORCE_ERROR
#define DBP3_OFF                  FORCE_ERROR
#define DBP4_ON                   FORCE_ERROR
#define DBP4_OFF                  FORCE_ERROR
#define DBP5_ON                   FORCE_ERROR
#define DBP5_OFF                  FORCE_ERROR
#define DBP6_ON                   FORCE_ERROR
#define DBP6_OFF                  FORCE_ERROR
#define DBP7_ON                   FORCE_ERROR
#define DBP7_OFF                  FORCE_ERROR
#define DBP8_ON                   FORCE_ERROR
#define DBP8_OFF                  FORCE_ERROR
#define DBP9_ON                   FORCE_ERROR
#define DBP9_OFF                  FORCE_ERROR
#define DBP10_ON                  FORCE_ERROR
#define DBP10_OFF                 FORCE_ERROR
#define DBP11_ON                  FORCE_ERROR
#define DBP11_OFF                 FORCE_ERROR
#define DBP12_ON                  FORCE_ERROR
#define DBP12_OFF                 FORCE_ERROR
#define DBP13_ON                  FORCE_ERROR
#define DBP13_OFF                 FORCE_ERROR
#define DBP14_ON                  FORCE_ERROR
#define DBP14_OFF                 FORCE_ERROR
#define DBP15_ON                  FORCE_ERROR
#define DBP15_OFF                 FORCE_ERROR

#endif /* PIN_DEBUG_ENABLE */

/* SPI debug functions, implemented in hal.c in SoftDevice. */
void hal_spi_init(uint32_t clk_pin, uint32_t mosi_pin); /*eample use: hal_spi_init(DBP0_PIN, DBP1_PIN);*/
void hal_spi_putc(uint8_t c);
void hal_spi_puts(uint8_t const * str);
void hal_spi_printf(const char * fmt, ...);

/* The fundamental Serial debug macros.  These are internal, and are used by the other macros. */
/* DBS is short for DeBugSerial */
#ifdef SERIAL_DEBUG_ENABLE

#error "Serial debugging has not been ported yet"

#include "test_console.h"

#define DBS_PUT_STRING(string) {test_console_put_string(string);}
#define DBS_PUT_LINE(string)   {test_console_put_line(string);}
#define DBS_PUT_NEWLINE()      {test_console_put_newline();}
#define DBS_PUT_CHAR(ch)       {test_console_put_char(ch);}
#define DBS_PUT_DECBYTE(b)     {test_console_put_decbyte(b);}
#define DBS_PUT_DECWORD(w)     {test_console_put_decword(w);}
#define DBS_PUT_HEXBYTE(b)     {test_console_put_hexbyte(b);}
#define DBS_PUT_HEXWORD(w)     {test_console_put_hexword(w);}

#else

#define DBS_PUT_STRING(string) {}
#define DBS_PUT_LINE(string)   {}
#define DBS_PUT_NEWLINE()      {}
#define DBS_PUT_CHAR(ch)       {}
#define DBS_PUT_DECBYTE(b)     {}
#define DBS_PUT_DECWORD(w)     {}
#define DBS_PUT_HEXBYTE(b)     {}
#define DBS_PUT_HEXWORD(w)     {}

#endif /* #ifdef SERIAL_DEBUG_ENABLE */


#endif /* PIN_DEBUG_TRANSPORT_H__ */

/**
  @}
  @}
*/