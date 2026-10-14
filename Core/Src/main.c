/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : RF Basisstation (Ontvanger / RX) – STM32L031F6P6TR
  *
  * ============================================================================
  *  SYSTEEMOVERZICHT
  * ============================================================================
  *
  * HARDWARE PLATFORM : STM32L031F6P6TR (TSSOP-20) – identiek aan de zender
  * SYSTEM CLOCK      : MSI @ 2.097 MHz (Range 5) – laag verbruik, exact 2400 bd
  * UART              : USART2 @ 2400 baud, 8N1
  *                     PA3 (RX) ← analoge RF-ontvanger datauitgang
  *                     PA2 (TX) → seriële monitor op PC (debug / weergave)
  *
  * ROL              : Continue ontvangst van MQ-135 PPM-metingen via een
  *                     discrete analoge ASK/OOK RF-link op 433 MHz.
  *
  * ============================================================================
  *  RF-PROTOCOL (identiek aan de Zender / TX)
  * ============================================================================
  *
  *   ┌──────────────────┬───────────┬──────────────────┬─────┐
  *   │  4× 0xAA         │  1× 0xFF  │  ASCII PPM data  │ \n  │
  *   │  (Preamble/AGC)  │  (Sync)   │  bv. "250"       │     │
  *   └──────────────────┴───────────┴──────────────────┴─────┘
  *
  * WAAROM DIT PROTOCOL:
  *
  *   1. PREAMBLE (0xAA = 10101010b, 4 bytes):
  *      De analoge RF-ontvanger heeft een Automatic Gain Control (AGC) die
  *      tijd nodig heeft om op het signaalniveau in te locken. De regelmatige
  *      bitafwisseling van 0xAA biedt een "trainingssignaal" waarmee de
  *      AGC-versterking convergeert vóórdat echte data begint. Zonder preamble
  *      worden de eerste bytes vrijwel altijd corrupt ontvangen.
  *
  *   2. SYNC BYTE (0xFF = 11111111b):
  *      Markeert ondubbelzinnig het begin van de payload. 0xFF kan onmogelijk
  *      verward worden met het alternerende preamble-patroon (0xAA) of met
  *      geldige ASCII-tekens in de payload (0x30–0x39 voor cijfers).
  *      De ontvanger negeert alle ruis en preamble-bytes en wacht uitsluitend
  *      op deze synchronisatiebyte.
  *
  *   3. ASCII PAYLOAD:
  *      Menselijk leesbaar, variabele lengte, eenvoudig te debuggen.
  *
  *   4. NEWLINE TERMINATOR ('\n' = 0x0A):
  *      Eenvoudige end-of-frame detectie.
  *
  * ============================================================================
  *  WAAROM DE STATE MACHINE EN ERROR HANDLING ESSENTIEEL ZIJN
  * ============================================================================
  *
  * PROBLEEM: De PA3 (RX) pin hangt rechtstreeks aan de datauitgang van een
  * analoge RF-ontvanger (bijv. SYN480R, RXB6, XY-MK-5V). Wanneer de zender
  * NIET actief zendt, produceert de ontvanger-module puur thermische ruis:
  * willekeurige spanningsovergangen die de USART-hardware interpreteert als
  * corrupte databytes.
  *
  * GEVOLG ZONDER BESCHERMING:
  *   - Framing Errors (FE): De USART detecteert geen geldig stop-bit in de
  *     ruis → FE-vlag wordt gezet.
  *   - Noise Errors (NE): De USART detecteert instabiele logische niveaus
  *     tijdens sampling → NE-vlag wordt gezet.
  *   - Overrun Errors (ORE): De USART ontvangt een nieuw byte terwijl het
  *     vorige nog niet gelezen is → ORE-vlag wordt gezet.
  *
  *   Als deze error-vlaggen NIET actief gewist worden, blokkeert de
  *   USART-periferie volledig: de RXNE-interrupt stopt met vuren, en de
  *   MCU hangt effectief vast. Dit gebeurt binnen milliseconden na power-on
  *   door de continue ruisstroom.
  *
  * OPLOSSING:
  *   1. STATE MACHINE: Negeert alle data totdat de specifieke sync-byte
  *      (0xFF) wordt gedetecteerd. Hierdoor worden alle ruisbytes en
  *      preamble-bytes systematisch weggegooid.
  *   2. ERROR CALLBACK: De HAL_UART_ErrorCallback() wist agressief alle
  *      ORE/FE/NE-vlaggen en herstart de UART RX-interrupt, zodat de
  *      periferie nooit geblokkeerd raakt.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/

/**
  * @brief  State Machine statussen voor het RF-ontvangstprotocol.
  *
  *         STATE_WAIT_SYNC : Negeer alle inkomende bytes (ruis + preamble)
  *                           totdat de sync-byte (0xFF) arriveert.
  *         STATE_PAYLOAD   : Verzamel ASCII payload-bytes totdat '\n' of
  *                           de bufferlimiet bereikt is.
  */
typedef enum
{
  STATE_WAIT_SYNC = 0,  /* Wacht op 0xFF sync-byte                         */
  STATE_PAYLOAD   = 1   /* Ontvang ASCII PPM-data tot '\n' of buffer vol   */
} RX_State_t;

/* Private defines -----------------------------------------------------------*/
#define SYNC_BYTE         0xFF    /* Synchronisatiebyte na de preamble       */
#define PAYLOAD_MAX_LEN   8       /* Max. payload bytes (bijv. "9999\n" + marge) */
#define TX_BUF_LEN        40      /* Uitvoerbuffer voor geformatteerde tekst */

/* Private variables ---------------------------------------------------------*/

/* --- HAL Handles --- */
UART_HandleTypeDef huart2;

/* --- State Machine variabelen (volatile: gedeeld met ISR) --- */
static volatile RX_State_t rx_state     = STATE_WAIT_SYNC;
static volatile uint8_t    rx_complete  = 0;   /* Vlag: compleet pakket ontvangen */

/* --- Ontvangstbuffers --- */
static uint8_t  rx_byte;                        /* 1-byte HAL_UART_Receive_IT buffer */
static char     payload_buf[PAYLOAD_MAX_LEN + 1]; /* +1 voor null-terminator          */
static volatile uint8_t payload_idx = 0;        /* Huidige schrijfpositie in buffer   */

/* --- Kopiebuffer voor main-loop verwerking (buiten ISR-context) --- */
static char     payload_copy[PAYLOAD_MAX_LEN + 1];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void UART_Send_String(const char *str);

/* ==========================================================================
 *  MAIN
 * ========================================================================== */

/**
  * @brief  Applicatie-ingangspunt.
  *
  *         Initialiseert hardware, start UART RX-interrupt, en draait een
  *         oneindige lus die wacht op compleet ontvangen PPM-pakketten.
  *
  *         De MCU blijft actief (geen Stop Mode) omdat de ontvanger continu
  *         moet luisteren. WFI (Wait For Interrupt) wordt gebruikt om de
  *         CPU-kern te pauzeren tussen interrupts, wat stroom bespaart
  *         zonder de USART-periferie uit te schakelen.
  */
int main(void)
{
  /* ------ HAL & Klok Initialisatie ------ */
  HAL_Init();
  SystemClock_Config();

  /* ------ Periferie Initialisatie ------ */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* ------ Opstartbericht naar seriële monitor ------ */
  UART_Send_String("\r\n========================================\r\n");
  UART_Send_String("  RF Basisstation (RX) – STM32L031\r\n");
  UART_Send_String("  USART2 @ 2400 bd | PA3=RX  PA2=TX\r\n");
  UART_Send_String("  Wacht op RF-pakketten...\r\n");
  UART_Send_String("========================================\r\n\r\n");

  /* ------ Start eerste UART RX Interrupt-ontvangst ------ */
  /*
   * HAL_UART_Receive_IT() configureert de USART om exact 1 byte te ontvangen
   * via interrupt. Bij ontvangst wordt HAL_UART_RxCpltCallback() aangeroepen.
   * Bij een error wordt HAL_UART_ErrorCallback() aangeroepen.
   */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  /* ====== HOOFDLUS ====== */
  while (1)
  {
    /*
     * Controleer of de State Machine een volledig pakket heeft ontvangen.
     * De vlag 'rx_complete' wordt door de ISR/callback op 1 gezet zodra
     * '\n' of de bufferlimiet bereikt is.
     */
    if (rx_complete)
    {
      /* --- Kopieer payload veilig (ISR kan ondertussen doorgaan) --- */
      /*
       * Hoewel rx_complete al gezet is (en de state machine terug in
       * WAIT_SYNC staat), maken we een lokale kopie van de payload
       * voor thread-safety.
       */
      memcpy(payload_copy, (const char *)payload_buf, sizeof(payload_copy));
      payload_copy[PAYLOAD_MAX_LEN] = '\0';  /* Garandeer null-terminatie */

      /* --- Formatteer en verstuur naar PC via PA2 (TX) --- */
      char tx_buf[TX_BUF_LEN];
      int len = snprintf(tx_buf, sizeof(tx_buf),
                         "Ontvangen: %s PPM\r\n", payload_copy);

      if (len > 0 && len < (int)sizeof(tx_buf))
      {
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buf, (uint16_t)len, 200);
      }

      /* --- Reset vlag: klaar voor volgend pakket --- */
      rx_complete = 0;
    }

    /*
     * WFI (Wait For Interrupt):
     * Pauzeer de CPU-kern totdat de volgende interrupt binnenkomt.
     * Dit is "Sleep Mode" op de STM32L0: de CPU-klok stopt, maar alle
     * periferie (inclusief USART2) blijft actief. Stroomverbruik daalt
     * van ~1 mA (Run) naar ~0.5 mA (Sleep), afhankelijk van de
     * kloksnelheid en actieve periferie.
     *
     * Zodra de USART2 RXNE-interrupt vuur, wordt de CPU gewekt en
     * hervat de while(1) lus.
     */
    __WFI();
  }
}

/* ==========================================================================
 *  HAL UART CALLBACKS (aangeroepen vanuit ISR-context)
 * ========================================================================== */

/**
  * @brief  UART RX Complete Callback – State Machine Protocol Decoder.
  *
  *         Deze functie wordt aangeroepen door de HAL vanuit de USART2_IRQHandler
  *         telkens wanneer exact 1 byte succesvol ontvangen is (RXNE-interrupt).
  *
  *         STATEMACHINE LOGICA:
  *
  *         ┌─────────────────┐   byte != 0xFF    ┌─────────────────┐
  *         │                 │ ◄──────────────── │                 │
  *         │  WAIT_SYNC      │                    │    (ruis /      │
  *         │  (initieel)     │                    │     preamble)   │
  *         │                 │ ──────────────── ► │                 │
  *         └────────┬────────┘   byte == 0xFF     └─────────────────┘
  *                  │
  *                  ▼
  *         ┌─────────────────┐   byte == '\n'     ┌─────────────────┐
  *         │                 │   of buffer vol    │  rx_complete=1  │
  *         │  PAYLOAD        │ ──────────────── ► │  → WAIT_SYNC    │
  *         │  (verzamel)     │                    │                 │
  *         │                 │ ◄── anders:        └─────────────────┘
  *         │                 │     sla byte op
  *         └─────────────────┘
  *
  * @param  huart  Pointer naar de UART handle die de callback triggerde
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  switch (rx_state)
  {
    /* ------------------------------------------------------------------ */
    case STATE_WAIT_SYNC:
    /* ------------------------------------------------------------------ */
      /*
       * In deze state negeren we ALLES behalve de sync-byte (0xFF).
       *
       * Dit is cruciaal voor een analoge RF-link:
       *   - Thermische ruis genereert willekeurige bytes → genegeerd
       *   - Preamble bytes (0xAA) → genegeerd
       *   - Alleen 0xFF triggert de overgang naar payload-ontvangst
       *
       * De kans dat ruis toevallig exact 0xFF produceert is ~1/256 per
       * ruisbyte. Echter, zelfs als dat gebeurt, zal de daaropvolgende
       * "payload" onzin bevatten en kan de main-loop dat eventueel
       * valideren (optioneel).
       */
      if (rx_byte == SYNC_BYTE)
      {
        /* Sync ontvangen! Reset buffer en begin payload-ontvangst. */
        payload_idx = 0;
        memset((char *)payload_buf, 0, sizeof(payload_buf));
        rx_state = STATE_PAYLOAD;
      }
      /* Alle andere bytes (ruis, 0xAA preamble) worden stilzwijgend genegeerd. */
      break;

    /* ------------------------------------------------------------------ */
    case STATE_PAYLOAD:
    /* ------------------------------------------------------------------ */
      if (rx_byte == '\n' || payload_idx >= PAYLOAD_MAX_LEN)
      {
        /*
         * End-of-frame gedetecteerd:
         *   - '\n' terminator ontvangen, OF
         *   - Bufferlimiet bereikt (bescherming tegen runaway data)
         *
         * Null-termineer de string en signaleer de main-loop.
         */
        payload_buf[payload_idx] = '\0';
        rx_complete = 1;
        rx_state = STATE_WAIT_SYNC;
      }
      else
      {
        /*
         * Geldig payload-byte: sla op in buffer.
         * We accepteren alleen printbare ASCII-tekens (optionele filter).
         * Dit voorkomt dat een stray ruisbyte de payload vervuilt.
         */
        if (rx_byte >= 0x20 && rx_byte <= 0x7E)
        {
          payload_buf[payload_idx] = (char)rx_byte;
          payload_idx++;
        }
        /*
         * Niet-printbare bytes (behalve '\n' die hierboven al afgevangen
         * wordt) worden stilzwijgend overgeslagen. Dit biedt een extra
         * laag robuustheid tegen corrupte bytes na de sync.
         */
      }
      break;

    /* ------------------------------------------------------------------ */
    default:
    /* ------------------------------------------------------------------ */
      /* Onbekende state: veilige terugval naar sync-wacht. */
      rx_state = STATE_WAIT_SYNC;
      break;
  }

  /*
   * HERSTART UART RX INTERRUPT:
   * De HAL stopt automatisch met luisteren na het ontvangen van het
   * gevraagde aantal bytes (1 in ons geval). We MOETEN de ontvangst
   * hier opnieuw starten, anders stopt de interrupt-chain en ontvangen
   * we nooit meer data.
   */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/**
  * @brief  UART Error Callback – KRITISCH voor analoge RF-ontvangst.
  *
  *         ╔══════════════════════════════════════════════════════════════╗
  *         ║  ZONDER DEZE FUNCTIE LOOPT DE STM32 DIRECT VAST!           ║
  *         ║                                                            ║
  *         ║  De analoge RF-ontvanger produceert continue thermische     ║
  *         ║  ruis op PA3 (RX) wanneer de zender niet actief is.        ║
  *         ║  Deze ruis genereert permanent Framing Errors (FE),        ║
  *         ║  Noise Errors (NE) en Overrun Errors (ORE) in de USART    ║
  *         ║  hardware. Als deze niet gewist worden, blokkeert de       ║
  *         ║  USART-periferie en stopt de RXNE-interrupt met vuren.     ║
  *         ╚══════════════════════════════════════════════════════════════╝
  *
  * @param  huart  Pointer naar de UART handle met de error
  *
  * TECHNISCHE ACHTERGROND:
  *
  *   - FE (Framing Error): De USART verwacht een logic-high stop-bit op het
  *     einde van elk frame. Ruis produceert willekeurige niveaus, waardoor
  *     het stop-bit vaak gemist wordt → FE.
  *
  *   - NE (Noise Error): De USART samplet elke bit meerdere keren (3 of 5x
  *     bij 16x oversampling). Als de samples niet unanimous zijn, wordt NE
  *     gezet. Op een ruisige RF-lijn is dit bijna continu het geval.
  *
  *   - ORE (Overrun Error): Als een nieuw byte binnenkomt terwijl het
  *     vorige byte nog niet uit het Receive Data Register (RDR) gelezen is,
  *     gaat het nieuwe byte verloren en wordt ORE gezet. Bij hoge ruisfrequentie
  *     en trage verwerking kan dit optreden.
  *
  *   Op de STM32L0 worden deze vlaggen gewist door te schrijven naar het
  *   Interrupt Clear Register (ICR), wat de HAL-macro's hieronder doen.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  /* ------ Wis ALLE mogelijke UART-error vlaggen ------ */

  /*
   * ORE (Overrun Error) – Bit ORECF in ICR register
   * Moet als EERSTE gewist worden: ORE blokkeert verdere RX-operaties
   * het meest agressief.
   */
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
  }

  /*
   * FE (Framing Error) – Bit FECF in ICR register
   * Veelvoorkomend op een ruisige RF-lijn door ontbrekende stop-bits.
   */
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
  }

  /*
   * NE (Noise Error) – Bit NECF in ICR register
   * Bijna continu actief op een analoge RF-link vanwege de ruis.
   */
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);
  }

  /*
   * PE (Parity Error) – Voor de volledigheid, hoewel we geen pariteit
   * gebruiken (8N1). Defensief programmeren.
   */
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);
  }

  /*
   * KRITISCHE STAP: Herstart de UART RX Interrupt.
   *
   * Na een error stopt de HAL automatisch de RX-interrupt chain.
   * We MOETEN deze hier opnieuw starten, anders is de ontvanger
   * permanent doof na de eerste ruisbursts.
   *
   * Daarnaast resetten we de state machine naar WAIT_SYNC: als we
   * midden in een payload zaten toen de error optrad, is die payload
   * sowieso corrupt en kan beter weggegooid worden.
   */
  rx_state = STATE_WAIT_SYNC;
  payload_idx = 0;

  /*
   * HARDENING FIX: Forceer de HAL interne state naar READY.
   *
   * Na een error kan de HAL de RxState op HAL_UART_STATE_BUSY_RX laten staan
   * (afhankelijk van de exact HAL-versie en welke error er optrad). Als dat
   * gebeurt, weigert HAL_UART_Receive_IT() de ontvangst te herstarten
   * omdat hij denkt dat er al een actieve ontvangst loopt → permanent dove
   * ontvanger.
   *
   * Door RxState expliciet op READY te zetten vóór de Receive_IT aanroep,
   * garanderen we dat de herstart ALTIJD slaagt, ongeacht de HAL-versie.
   */
  huart->RxState = HAL_UART_STATE_READY;
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/* ==========================================================================
 *  KLOK- EN PERIFERIE-CONFIGURATIE
 * ========================================================================== */

/**
  * @brief  Systeemklok Configuratie.
  *
  *         MSI (Multi-Speed Internal) oscillator op Range 5 = 2.097 MHz.
  *         Identiek aan de zender-configuratie voor symmetrische baudrate.
  *
  *         Geen PLL – directe MSI naar SYSCLK voor minimaal stroomverbruik.
  *
  *         Bij 2.097 MHz en 2400 baud:
  *           Divisor = 2097000 / 2400 = 873.75
  *           Baudrate error ≈ 0.03% → ruim binnen specificatie.
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* --- MSI oscillator op Range 5 (2.097 MHz) --- */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState       = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange  = RCC_MSIRANGE_5;   /* 2.097 MHz */
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;     /* Geen PLL nodig */

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* --- Klokboom: geen delers, alles op 2.097 MHz --- */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK
                               | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  USART2 Initialisatie: 2400 baud, 8N1.
  *
  *         PA3 (AF4) = USART2_RX ← analoge RF-ontvanger data-uitgang
  *         PA2 (AF4) = USART2_TX → PC seriële monitor
  *
  *         Mode = TX_RX: we ontvangen RF-data en sturen geformatteerde
  *         tekst naar de PC over dezelfde USART-periferie.
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 2400;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  GPIO Initialisatie.
  *
  *         Schakelt GPIOA-klok in. De eigenlijke AF-configuratie van
  *         PA2/PA3 wordt afgehandeld in HAL_UART_MspInit() (zie
  *         stm32l0xx_hal_msp.c).
  */
static void MX_GPIO_Init(void)
{
  /* Activeer GPIOA-klok (nodig voor PA2/PA3) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* ==========================================================================
 *  HULPFUNCTIES
 * ========================================================================== */

/**
  * @brief  Verstuur een null-terminated string via USART2 (blocking).
  * @param  str  Pointer naar de te verzenden string
  */
static void UART_Send_String(const char *str)
{
  HAL_UART_Transmit(&huart2, (const uint8_t *)str,
                    (uint16_t)strlen(str), 500);
}

/**
  * @brief  Error Handler – oneindige lus bij fatale initialisatiefout.
  *
  *         In productie: implementeer watchdog-reset of foutmelding.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    /* Hang hier bij fatale fout. */
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Rapporteert de naam van het bronbestand en het regelnummer
  *         waar een assert_param fout is opgetreden.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
