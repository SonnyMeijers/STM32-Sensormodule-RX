/**
  ******************************************************************************
  * @file         stm32l0xx_hal_msp.c
  * @brief        MSP Initialisatie – UART GPIO & Klok configuratie
  *
  *               Dit bestand bevat de "MCU Support Package" callbacks die
  *               door de HAL aangeroepen worden tijdens HAL_UART_Init().
  *               Hier configureren we de fysieke pinnen en periferie-klokken.
  *
  * HARDWARE:     STM32L031F6P6TR (TSSOP-20) – Ontvanger/Basisstation
  * PINOUT:
  *   PA2 (AF4) → USART2_TX → PC seriële monitor
  *   PA3 (AF4) → USART2_RX ← analoge RF-ontvanger data-uitgang
  ******************************************************************************
  */

#include "main.h"

/**
  * @brief  UART MSP Initialisatie – aangeroepen door HAL_UART_Init().
  *
  *         Configureert:
  *         1. USART2 periferie-klok (APB1)
  *         2. GPIOA klok
  *         3. PA2 als Alternate Function Push-Pull (TX)
  *         4. PA3 als Alternate Function Push-Pull (RX)
  *         5. USART2 NVIC interrupt (voor interrupt-driven ontvangst)
  *
  * @param  huart  Pointer naar de UART handle
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (huart->Instance == USART2)
  {
    /* --- Periferie-klokken activeren --- */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* --- GPIO Configuratie ---
     *
     * PA2 = USART2_TX (Alternate Function 4)
     * PA3 = USART2_RX (Alternate Function 4)
     *
     * AF4 is de USART2 alternate function op de STM32L031.
     * Zie Datasheet Table 16 "Alternate function port A".
     *
     * Pull-up op RX (PA3):
     *   De analoge RF-ontvanger heeft een open-collector of zwakke
     *   push-pull uitgang. Een interne pull-up voorkomt dat de lijn
     *   gaat zweven wanneer de ontvanger-module even geen output drijft,
     *   wat extra valse start-bits en framing errors zou genereren.
     */
    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;   /* 2400 bd = laag */
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* --- NVIC: Activeer USART2 interrupt ---
     *
     * Prioriteit 0 (hoogste): de UART-interrupt mag NIET gemist worden.
     * Bij een analoge RF-link komen bytes asynchroon en onvoorspelbaar
     * binnen; elke gemiste RXNE-interrupt kan leiden tot een ORE en
     * dataverlies.
     */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

/**
  * @brief  UART MSP De-Initialisatie – aangeroepen door HAL_UART_DeInit().
  *
  *         Schakelt periferie-klok en GPIO-pinnen uit.
  *         Wordt normaal niet aangeroepen in deze applicatie.
  *
  * @param  huart  Pointer naar de UART handle
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}
