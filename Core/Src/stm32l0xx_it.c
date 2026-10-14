/**
  ******************************************************************************
  * @file         stm32l0xx_it.c
  * @brief        Interrupt Service Routines – Ontvanger/Basisstation
  *
  *               Bevat de USART2 IRQ-handler en de standaard systeem-handlers.
  *
  * BELANGRIJK:
  *   De USART2_IRQHandler roept HAL_UART_IRQHandler() aan, die op zijn beurt:
  *     - Bij RXNE: HAL_UART_RxCpltCallback() aanroept (state machine in main.c)
  *     - Bij ORE/FE/NE: HAL_UART_ErrorCallback() aanroept (error handler in main.c)
  *
  *   EXTRA VEILIGHEID: Vóór de HAL-aanroep wissen we handmatig de error-vlaggen.
  *   Dit is een "belt-and-suspenders" aanpak: soms wist de HAL niet alle vlaggen
  *   correct in oudere library-versies, en bij een analoge RF-link is het
  *   catastrofaal als ook maar één error-vlag ongewist blijft.
  ******************************************************************************
  */

#include "main.h"

/* Externe referentie naar de UART handle in main.c */
extern UART_HandleTypeDef huart2;

/* ==========================================================================
 *  CORTEX-M0+ SYSTEEM INTERRUPT HANDLERS
 * ========================================================================== */

/**
  * @brief  Non-Maskable Interrupt handler.
  */
void NMI_Handler(void)
{
  while (1)
  {
    /* Hang bij NMI */
  }
}

/**
  * @brief  Hard Fault handler.
  */
void HardFault_Handler(void)
{
  while (1)
  {
    /* Hang bij Hard Fault */
  }
}

/**
  * @brief  System Service Call handler (SVCall).
  */
void SVC_Handler(void)
{
  /* Niet gebruikt */
}

/**
  * @brief  Pendable Service Call handler (PendSV).
  */
void PendSV_Handler(void)
{
  /* Niet gebruikt */
}

/**
  * @brief  SysTick Timer handler – HAL tijdbasis (1 ms tick).
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/* ==========================================================================
 *  PERIFERIE INTERRUPT HANDLERS
 * ========================================================================== */

/**
  * @brief  USART2 Global Interrupt Handler.
  *
  *         Dit is het hart van de RF-ontvangst. Elke byte die de USART
  *         hardware ontvangt (of elke error die optreedt) triggert deze ISR.
  *
  *         DEFENSIEVE ERROR CLEARING:
  *         We wissen ORE/FE/NE/PE vlaggen VOORDAT we de HAL-handler aanroepen.
  *         Reden: de HAL_UART_IRQHandler() leest de ISR-register en routeert
  *         naar de juiste callback. Maar bij sommige HAL-versies en edge-cases
  *         (bijv. meerdere errors tegelijk, wat bij RF-ruis constant voorkomt)
  *         kan de HAL in een inconsistente staat raken. Door de vlaggen eerst
  *         handmatig te wissen, garanderen we dat de periferie nooit blokkeert.
  *
  *         Dit is de "dubbele zekerheid" strategie:
  *           1. Hier: handmatige clear van alle error-vlaggen
  *           2. In HAL_UART_ErrorCallback(): nog een keer, plus herstart RX
  */
void USART2_IRQHandler(void)
{
  /*
   * --- Stap 1: Agressief alle error-vlaggen wissen ---
   *
   * Op de STM32L0 worden USART error-vlaggen gewist door naar de
   * ICR (Interrupt Clear Register) te schrijven. De __HAL_UART_CLEAR_FLAG
   * macro doet precies dit.
   *
   * We controleren niet eens OF de vlag gezet is – we wissen gewoon alles.
   * Bij 2400 baud en continue ruis is de overhead verwaarloosbaar, en
   * de zekerheid is het waard.
   */
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);  /* Overrun Error     */
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_FEF);   /* Framing Error     */
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_NEF);   /* Noise Error       */
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF);   /* Parity Error      */

  /*
   * --- Stap 2: HAL-handler aanroepen ---
   *
   * De HAL leest het ISR-register opnieuw en routeert naar:
   *   - HAL_UART_RxCpltCallback() bij RXNE (geldig byte ontvangen)
   *   - HAL_UART_ErrorCallback() bij resterende errors
   *
   * Omdat we de error-vlaggen hierboven al gewist hebben, zal de HAL
   * in de meeste gevallen direct naar de RxCplt callback gaan.
   * Als er tóch een error is die we niet gevangen hebben, vangt de
   * ErrorCallback in main.c het op als tweede verdedigingslinie.
   */
  HAL_UART_IRQHandler(&huart2);
}
