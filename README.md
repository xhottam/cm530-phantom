# CM530 PhantomX Hexapod — Documentación Técnica

## Tabla de Contenidos

1. [Descripción General](#1-descripción-general)
2. [Arquitectura del Proyecto](#2-arquitectura-del-proyecto)
3. [Hardware CM530 — Inicialización](#3-hardware-cm530--inicialización)
4. [Bucle de Control Principal](#4-bucle-de-control-principal)
5. [Inicialización de la Aplicación](#5-inicialización-de-la-aplicación)
6. [Control de Entrada — Commander](#6-control-de-entrada--commander)
7. [Sistema de Marcha (Gaits)](#7-sistema-de-marcha-gaits)
8. [Cinemática Inversa (IK)](#8-cinemática-inversa-ik)
9. [Suavizado de Movimiento](#9-suavizado-de-movimiento)
10. [Referencia de Configuración (`Hex_Cfg.h`)](#10-referencia-de-configuración-hex_cfgh)
11. [Guía de Personalización](#11-guía-de-personalización)
12. [Compilación y Build](#12-compilación-y-build)
13. [Referencias](#13-referencias)

---

## 1. Descripción General

Firmware para el controlador **Robotis CM-530** (STM32F103) que implementa el proyecto
**Lynxmotion Phoenix** adaptado al robot **PhantomX Hexapod** de Trossen Robotics.
El código ejecuta cinemática inversa (IK) completa para 6 patas, control de marcha,
balance dinámico y comunicación con servos AX-12 vía bus Dynamixel.

```
Procesador : STM32F103 (Cortex-M3, 72 MHz)
Servos     : 18 × AX-12 (3 por pata: Coxa, Fémur, Tibia)
Comunicación: USART / ZigBee (ArbotiX Commander o Odroid)
Build      : arm-none-eabi-gcc + Make → CM530.hex / CM530.bin
```

---

## 2. Arquitectura del Proyecto

```
cm530-phantom-main/
├── Makefile                        ← Build system (arm-none-eabi-gcc)
├── stm32.ld                        ← Linker script STM32F103
├── CM530_APP/
│   ├── inc/
│   │   ├── Hex_Cfg.h               ← ★ CONFIGURACIÓN PRINCIPAL DEL HEXÁPODO
│   │   ├── Phoenix.h               ← Tipos, estructuras, tablas seno/coseno
│   │   ├── Phoenix_Input_Commander.h ← Lógica de control (mandos, modos)
│   │   ├── Phoenix_Driver_AX12.h   ← Driver de servos AX-12
│   │   ├── BioloidEx.h             ← Interpolación de poses Bioloid
│   │   ├── dynamixel.h             ← Protocolo Dynamixel v1
│   │   ├── InputController.h       ← Interfaz de control abstracta
│   │   └── ServoDriver.h           ← Interfaz del driver de servos
│   └── src/
│       ├── main.c                  ← Bucle principal + IK + gaits
│       ├── BioloidEx.c             ← Interpolación de poses
│       ├── dynamixel.c             ← Comunicación bus Dynamixel
│       ├── serial.c                ← USART serie
│       ├── zigbee.c                ← Módulo ZigBee / Commander
│       └── stm32f10x_it.c          ← Manejadores de interrupciones
├── CM530_HW/
│   ├── inc/                        ← Periféricos hardware (ADC, LEDs, botones…)
│   └── src/
└── stm32f10x_lib/                  ← Biblioteca STM32 SPL (Standard Peripheral Library)
```

---

## 3. Hardware CM530 — Inicialización


### Tabla Maestra de Periféricos y Conflictos de Multiplexación

| Bus | Módulo Periférico | Función Principal | Pines por Defecto | Pines con Remapeo (Remap) | ⚠️ Periféricos en Conflicto Directo (Mismos Pines) |
|---|---|---|---|---|---|
| APB2 | USART1 | Puerto Serie (Tu config) | PA9 (TX), PA10 (RX) | PB6 (TX), PB7 (RX) | TIM1_CH2 / TIM1_CH3 (Si activas estos canales de PWM en PA9/PA10, rompes el USART1). |
| APB2 | TIM1 | Motor Control | PA8..PA11, PB12..PB15 | PE7..PE15 | USART1 (En PA9/PA10). |
| APB2 | TIM8 | Motor Control | PC6, PC7, PC8, PC9 | No tiene remapeo | USART3 (Si usas el remapeo parcial de USART3 a PC10/PC11, físicamente están uno al lado del otro en el silicio, cuidado con ruido). |
| APB2 | SPI1 | Bus SPI Alta Vel. | PA5, PA6, PA7 | PB3, PB4, PB5 | ADC12_IN5/IN6/IN7 y DAC_OUT2 (PA5). |
| APB2 | ADC1/2/3 | Conversión Analógica | PA0 a PA7, PB0, PB1, PC0 a PC5 | No se remapean | USART2 (PA2/PA3 comparten líneas analógicas ADC_IN2/IN3). |
| APB1 | USART2 | Universal Serial Port (Tu config) | PA2 (TX), PA3 (RX) | PD5 (TX), PD6 (RX) | TIM2_CH3 / TIM2_CH4 y TIM5_CH3 / TIM5_CH4 y ADC12_IN2/IN3. (Si usas el pin físico para el timer, destruyes la comunicación). |
| APB1 | USART3 | Universal Serial Port (Tu config) | PB10 (TX), PB11 (RX) | PC10/PC11 (Partial) PD8/PD9 (Full) | I2C2 (SCL/SDA usan exactamente PB10/PB11 por defecto. Si activas I2C2, el USART3 muere). TIM2 (Si haces Partial Remap 1 de TIM2, se muda aquí). |
| APB1 | TIM2 | General-Purpose Timer (Tu Millis) | PA0, PA1, PA2, PA3 | Múltiples (PA15, PB3, PB10, PB11) | USART2 (Por defecto en PA2/PA3) USART3 (Si usas remapeo de TIM2 a PB10/PB11). |
| APB1 | I2C1 / I2C2 | Interfaz I2C | PB6, PB7 (I2C1) PB10, PB11 (I2C2) | PB8, PB9 (Solo I2C1) | USART3 (Conflicto total y crítico en PB10/PB11 con I2C2). |
| APB1 | UART4 / UART5 | Puertos Serie Básicos | PC10/PC11 (U4) PC12/PD2 (U5) | No tiene remapeo | USART3 (Si remapeas USART3 a PC10/PC11, colisiona con UART4). SPI3 (Usa PC10/PC11/PC12 en su remapeo). |
| APB1 | CAN1 | Bus Automotriz | PA11 (RX), PA12 (TX) | PB8/PB9 o PD0/PD1 | USB (Comparte exactamente PA11/PA12. No puedes usar USB y CAN al mismo tiempo en pines nativos). |
| APB1 | USB | Puerto USB 2.0 | PA11 (DM), PA12 (DP) | Pines fijos | CAN1 (Mismo conflicto físico que el anterior). |
| AHB | SDIO | Lector Tarjetas SD | PC8 a PC12, PD2 | Ruteado fijo | UART4, UART5 y SPI3 (Todos pelean por las líneas altas del Puerto C si intentas usarlos a la vez). |


### `SysInit()` — Secuencia de arranque

```c
void SysInit() {
    ReBootToBootLoader = 0;  // Evita arranque accidental en bootloader
    RCC_Configuration();     // 1. Relojes y PLL
    NVIC_Configuration();    // 2. Vector de interrupciones
    GPIO_Configuration();    // 3. Pines E/S
    SysTick_Configuration(); // 4. Temporizador del sistema
    Timer_Configuration();   // 5. TIM2 (millis)
    ADC_Configuration();     // 6. Conversores analógico-digital
    USART_Configuration();   // 7. Puertos serie
}
```

---

### `RCC_Configuration()` — Relojes del Sistema

**Resultado final:** CPU a **72 MHz** mediante PLL.

```
HSE (cristal externo 8 MHz)
    └─→ PLL × 9
           └─→ SYSCLK = 72 MHz
                 ├─→ HCLK  = 72 MHz   (AHB,  Div1)
                 ├─→ PCLK2 = 72 MHz   (APB2, Div1)  ← USART1, ADC, GPIO
                 └─→ PCLK1 = 36 MHz   (APB1, Div2)  ← USART3, UART5, TIM2
```

**Flash latency:** 2 ciclos de espera (obligatorio a 72 MHz).

#### Cómo funciona el árbol de relojes

El **PLL** (Phase-Locked Loop) es un circuito analógico que multiplica la frecuencia
del cristal externo (8 MHz × 9 = 72 MHz). Una vez configurado, el núcleo Cortex-M3
corre a 72 MHz sin consumir instrucciones adicionales.

Los **prescalers APB1/APB2** son divisores de frecuencia en el bus de periféricos.
El STM32F103 tiene el bus APB1 limitado a **36 MHz máximo por diseño de silicio**,
de ahí el divisor obligatorio /2.

#### Por qué existen dos velocidades de bus

ST divide los periféricos en dos dominios para equilibrar consumo y complejidad:

- **APB2 (72 MHz)** — periféricos "rápidos": USART1, ADC1/2, GPIO
- **APB1 (36 MHz)** — periféricos "lentos": USART3, UART5, TIM2, TIM3…

#### Implicaciones concretas en este proyecto

**USART1 (Dynamixel) @ APB2 = 72 MHz**

$$\text{BRR} = \frac{72\,000\,000}{1\,000\,000} = 72 \quad \Rightarrow \quad \text{1 Mbps exacto, error = 0\%}$$

**USART3/UART5 (PC, ZigBee) @ APB1 = 36 MHz**

$$\text{BRR} = \frac{36\,000\,000}{57\,600} = 625 \quad \Rightarrow \quad \text{57600 bps exacto, error = 0\%}$$

**ADC @ APB2 = 72 MHz — limitación importante**

El ADC del STM32F103 tiene frecuencia máxima de **14 MHz**. Se requiere un
prescaler adicional exclusivo del ADC:

```
PCLK2 = 72 MHz  →  ADC_Prescaler /6  →  ADC clock = 12 MHz  ✓ (< 14 MHz)
```

**TIM2 @ APB1 = 36 MHz — regla especial del STM32**

> Cuando `APBx_Prescaler ≠ 1`, el reloj efectivo de los timers de ese bus
> es **PCLK × 2** (multiplicador automático del hardware).

```
PCLK1 = 36 MHz  →  TIM2 clock = 36 MHz × 2 = 72 MHz  (efectivo)
```

Por eso el cálculo del período de 1 ms usa 72 MHz como base, no 36 MHz:

```c
// PSC=71 → 72MHz/72 = 1MHz tick → ARR=999 → 1ms exacto  ✓
// Sin el ×2 automático (hipotético): PSC=35 → 36MHz/36 = 1MHz  (también válido)
// pero los valores del prescaler serían diferentes
```

#### Resumen de frecuencias efectivas por periférico

| Periférico | Bus | Frecuencia efectiva | Implicación |
|---|---|---|---|
| USART1 (Dynamixel) | APB2 | 72 MHz | 1 Mbps sin error |
| ADC1/ADC2 | APB2 | 72 MHz → /6 = **12 MHz** | Necesita prescaler /6 para no superar 14 MHz |
| GPIO | APB2 | 72 MHz | Conmutación máxima (50 MHz configurado) |
| USART3 (PC/Odroid) | APB1 | 36 MHz | 57600 bps sin error |
| UART5 (ZigBee) | APB1 | 36 MHz | 57600 bps sin error |
| TIM2 | APB1 | 36 MHz × **×2** = **72 MHz** | Multiplicador interno compensa el /2 del APB1 |

**Periféricos habilitados:**

| Bus | Periférico | Condición |
|---|---|---|
| APB2 | GPIOA, GPIOB, GPIOC | Siempre |
| APB2 | ADC1, ADC2 | Siempre |
| APB2 | USART1 | `#define USING_DYNAMIXEL` |
| APB1 | USART3 | `#define USING_PC_UART` |
| APB1 | UART5 | `#define USING_ZIGBEE` |
| APB1 | TIM2 | Siempre |

Los tres `#define` se activan en `CM530_HW/inc/usart.h`.

---

### `NVIC_Configuration()` — Interrupciones

#### Vector Table

```c
// Offset 0x3000 — reserva los primeros 12 KB al bootloader Robotis
NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
// Dirección base: 0x08003000  (también configurada en stm32.ld)
```

#### Grupo de prioridades

```c
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
// 2 bits preemption (4 niveles) + 3 bits sub-priority (8 niveles)
```

#### Tabla de prioridades

| Interrupción | Periférico | Preemption | Sub-priority | Función |
|---|---|---|---|---|
| `SystemHandler_SysTick` | SysTick | 0 | 0 | Tick del sistema (máxima prioridad) |
| `USART1_IRQChannel` | USART1 | 0 | **1** | RX Dynamixel (bus AX-12) |
| `TIM2_IRQChannel` | TIM2 | 0 | **2** | Contador milisegundos |
| `USART3_IRQChannel` | USART3 | 0 | **3** | RX PC_UART / Odroid |
| `UART5_IRQChannel` | UART5 | 0 | **3** | RX ZigBee / Commander |

> USART1 (Dynamixel) tiene mayor prioridad que el host (sub 3), garantizando que
> las respuestas de los servos AX-12 no se pierdan.

---

#### Manejadores de interrupción activos (`stm32f10x_it.c`)

Solo cinco de los ~60 handlers del archivo tienen implementación real.
El resto son stubs vacíos o bucles infinitos para fault exceptions.

---

**`SysTickHandler`** — Temporizador de retardos bloqueantes

```
Fuente   : SysTick core timer — HCLK/8 = 9 MHz
Período  : 10 µs  (SysTick_SetReload(90) — USING_SYSTICK_10US activo)
Prioridad: preemption=0, sub=0  (máxima del sistema)
Llama a  : ISR_Delay_Base()
```

Decrementa cinco contadores de cuenta atrás compartidos con el código principal:

| Contador | Decrementado cada | Usado por |
|---|---|---|
| `glDelayCounter` | 10 µs | `uDelay()` / `mDelay()` — espera bloqueante |
| `glCountdownCounter` | 10 µs | `StartCountdown()` — timeout genérico |
| `glBuzzerCounter` | 10 µs | `Buzzed()` — duración del pitido |
| `glDxlTimeoutCounter` | 10 µs | `dxl_hal_timeout()` — timeout RX Dynamixel |
| `glPcuTimeoutCounter` | 10 µs | `pcu_hal_timeout()` — timeout RX PC_UART |

Cuando todos los contadores llegan a 0: **deshabilita el propio SysTick** para
eliminar carga de ISR. Se reactiva automáticamente en la siguiente llamada a
`uDelay()`, `StartCountdown()`, etc.

---

**`TIM2_IRQHandler`** — Contador de milisegundos no bloqueante

```
Fuente   : APB1 × 2 = 72 MHz  →  PSC=71  →  ARR=999  →  1 ms exacto
Trigger  : TIM_IT_Update (auto-reload en cada overflow del ARR)
Prioridad: preemption=0, sub=2
Llama a  : TimerInterrupt_1ms()
```

```c
void TimerInterrupt_1ms(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        Millis_TIM2++;                    // fuente de getMillis_TIM2()
        if (gw1msCounter > 0)
            gw1msCounter--;               // usado por StartDiscount/CheckTimeOut
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
```

`Millis_TIM2` se usa en todo el firmware como base de tiempo no bloqueante:
sincronización del ciclo de control, timeouts del Commander (1250 ms),
espera de 3 s al arranque en `setupPhoenix()`.

---

**`USART1_IRQHandler`** — Recepción del bus Dynamixel

```
Periferico: USART1 (APB2 72 MHz)  →  PB6=TX / PB7=RX  [remap activo]
Baudrate  : 1 000 000 bps  (BRR=72, error=0%)
RS-485    : PB4=ENABLE_TXD / PB5=ENABLE_RXD
Prioridad : preemption=0, sub=1  ← más alta que USART3 y UART5
Guard     : #ifdef USING_DYNAMIXEL
Llama a   : RxD_DXL_Interrupt() → dxl_hal_rx()
```

Lee un byte del registro `USART1->DR` y lo deposita en el buffer circular
`gbpDxlBuffer[1023]`. Esta ISR captura las respuestas de los servos AX-12
(lecturas de voltaje, posición) sin perder bytes aunque el CPU esté en pleno
cálculo IK.

> La prioridad sub=1 (frente a sub=3 de los otros UART) garantiza que a 1 Mbps
> — un byte cada 10 µs — la ISR siempre sirve el byte antes de que llegue
> el siguiente, evitando desbordamiento del registro DR.

---

**`USART3_IRQHandler`** — Recepción del canal PC / Odroid

```
Periferico: USART3 (APB1 36 MHz)  →  PB10=TX / PB11=RX
Baudrate  : 57 600 bps  (BRR=625, error=0%)
Prioridad : preemption=0, sub=3
Guard     : #ifdef USING_PC_UART
Llama a   : RxD_PCU_Interrupt() → pcu_put_queue()
```

Lee un byte de `USART3->DR` y lo deposita en el buffer circular
`gbpPcuBuffer[32]`. Adicionalmente detecta la secuencia de reboot:
15 bytes consecutivos `'#'` activan `ReBootToBootLoader`, que permite
reflashear el firmware de forma remota.

> **Atención:** buffer de solo 32 bytes. A 57600 bps un byte llega cada
> ~174 µs. Si el bucle principal no lee el buffer durante más de ~5.5 ms,
> el buffer se desborda y los bytes más antiguos se sobreescriben.

---

**`UART5_IRQHandler`** — Recepción del módulo ZigBee / Commander

```
Periferico: UART5 (APB1 36 MHz)  →  PC12=TX / PD2=RX
Baudrate  : 57 600 bps  (BRR=625, error=0%)
Prioridad : preemption=0, sub=3
Guard     : #ifdef USING_ZIGBEE
Llama a   : RxD_ZIG_Interrupt() → zgb_hal_rx()
```

Lee un byte de `UART5->DR` y lo deposita en el buffer circular
`gbpZigBuffer[1023]`. El Commander ArbotiX envía tramas de 7 bytes a ~30 Hz:

```
[0xFF | rightV | rightH | leftV | leftH | buttons | ext | checksum]
```

`Commander_ReadMsgs_Odroid()` valida la trama y actualiza `g_InControlState`.
Si pasan más de `ARBOTIX_TO = 1250 ms` sin trama válida, el robot se apaga.
El mismo canal sirve para `TerminalMonitor()` cuando el robot está apagado.

---

**Resumen de handlers activos**

| Handler | Periférico | Prioridad | Frecuencia | Función principal |
|---|---|---|---|---|
| `SysTickHandler` | SysTick | 0/0 (máxima) | 100 kHz (10 µs) | Decrementar timers bloqueantes |
| `TIM2_IRQHandler` | TIM2 | 0/2 | 1 kHz (1 ms) | Incrementar `Millis_TIM2` |
| `USART1_IRQHandler` | USART1 | 0/1 | Hasta 100 kHz | Buffer RX Dynamixel |
| `USART3_IRQHandler` | USART3 | 0/3 | Hasta 5.76 kHz | Buffer RX PC/Odroid |
| `UART5_IRQHandler` | UART5 | 0/3 | Hasta 5.76 kHz | Buffer RX ZigBee/Commander |

---

### `GPIO_Configuration()` — Mapa de Pines

#### Puerto A

| Pin | Dirección | Señal | Uso |
|---|---|---|---|
| PA0 | OUT PP | `PIN_SIG_MOT1P` | Puerto OLLO 1+ |
| PA1 | OUT PP | `PIN_SIG_MOT1M` | Puerto OLLO 1− |
| PA2 | OUT PP | `PIN_SIG_MOT2P` | Puerto OLLO 2+ |
| PA3 | OUT PP | `PIN_SIG_MOT2M` | Puerto OLLO 2− |
| PA5 | ANALOG IN | `PIN_ADC1` | ADC externo 1 (multiplexado) |
| PA6 | OUT PP | `PIN_BUZZER` | Zumbador |
| PA8 | OUT PP | `PIN_SIG_MOT5P` | Puerto OLLO 5+ |
| PA11 | OUT PP | `PIN_SIG_MOT5M` | Puerto OLLO 5− |
| PA12 | OUT PP | `PIN_ZIGBEE_RESET` | Reset módulo ZigBee |
| PA14 | INPUT PULL-UP | `PIN_SW_RIGHT` | Botón derecho |
| PA15 | INPUT PULL-UP | `PIN_SW_LEFT` | Botón izquierdo |

#### Puerto B

| Pin | Dirección | Señal | Uso |
|---|---|---|---|
| PB3 | INPUT PULL-UP | `PIN_SW_START` | Botón Start |
| PB4 | OUT PP | `PIN_ENABLE_TXD` | RS-485 DXL TX enable |
| PB5 | OUT PP | `PIN_ENABLE_RXD` | RS-485 DXL RX enable |
| PB6 | AF PP | `PIN_DXL_TXD` | USART1 TX → Bus Dynamixel |
| PB7 | INPUT FLOAT | `PIN_DXL_RXD` | USART1 RX ← Bus Dynamixel |
| PB8 | OUT PP | `PIN_SIG_MOT6P` | Puerto OLLO 6+ |
| PB9 | OUT PP | `PIN_SIG_MOT6M` | Puerto OLLO 6− |
| PB10 | AF PP | `PIN_PC_TXD` | USART3 TX → PC/Odroid |
| PB11 | INPUT FLOAT | `PIN_PC_RXD` | USART3 RX ← PC/Odroid |
| PB12 | OUT PP | `PIN_LED_AUX` | LED AUX |
| PB13 | OUT PP | `PIN_LED_MANAGE` | LED MANAGE |
| PB14 | OUT PP | `PIN_LED_PROGRAM` | LED PROGRAM |
| PB15 | OUT PP | `PIN_LED_PLAY` | LED PLAY |

#### Puerto C

| Pin | Dirección | Señal | Uso |
|---|---|---|---|
| PC0 | ANALOG IN | `PIN_ADC0` | ADC externo 0 (multiplexado) |
| PC1 | OUT PP | `PIN_ADC_SELECT0` | Selector multiplexor ADC bit 0 |
| PC2 | OUT PP | `PIN_ADC_SELECT1` | Selector multiplexor ADC bit 1 |
| PC3 | ANALOG IN | `PIN_VDD_VOLT` | Tensión de batería (VBUS) |
| PC4 | INPUT PULL-UP | `PIN_MIC` | Micrófono |
| PC6 | OUT PP | `PIN_SIG_MOT3P` | Puerto OLLO 3+ |
| PC7 | OUT PP | `PIN_SIG_MOT3M` | Puerto OLLO 3− |
| PC8 | OUT PP | `PIN_SIG_MOT4P` | Puerto OLLO 4+ |
| PC9 | OUT PP | `PIN_SIG_MOT4M` | Puerto OLLO 4− |
| PC10 | INPUT PULL-UP | `PIN_SW_DOWN` | Botón abajo |
| PC11 | INPUT PULL-UP | `PIN_SW_UP` | Botón arriba |
| PC12 | AF PP | `PIN_ZIGBEE_TXD` | UART5 TX → Módulo ZigBee |
| PC13 | OUT PP | `PIN_LED_POWER` | LED POWER |
| PC14 | OUT PP | `PIN_LED_TXD` | LED TXD |
| PC15 | OUT PP | `PIN_LED_RXD` | LED RXD |

#### Puerto D

| Pin | Dirección | Señal | Uso |
|---|---|---|---|
| PD2 | INPUT FLOAT | `PIN_ZIGBEE_RXD` | UART5 RX ← Módulo ZigBee |

**Remaps activos:**
```c
GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);      // USART1 → PB6/PB7
GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); // Libera PA13/PA14/PA15 del JTAG
```

---

### `SysTick_Configuration()` — Tick del Sistema

Fuente: AHB/8 = 9 MHz. Configuración en `CM530_HW/inc/system_func.h`:

| Macro | Recarga | Período | Estado |
|---|---|---|---|
| `USING_SYSTICK_1000US` | 9000 | 1 ms | comentada |
| `USING_SYSTICK_100US` | 900 | 100 µs | comentada |
| **`USING_SYSTICK_10US`** | **90** | **10 µs** | **✓ ACTIVA** |
| `USING_SYSTICK_1US` | 9 | 1 µs | comentada |

ISR `SysTickHandler()` → `ISR_Delay_Base()` cada 10 µs, decrementa:

```c
glDelayCounter--;       // uDelay() / mDelay()
glCountdownCounter--;   // StartCountdown()
glBuzzerCounter--;      // Buzzed()
glDxlTimeoutCounter--;  // timeout RX Dynamixel
glPcuTimeoutCounter--;  // timeout RX PC_UART
```

El SysTick se **desconecta automáticamente** cuando todos los contadores llegan a cero.

---

### `Timer_Configuration()` — TIM2 (Contador de Milisegundos)

TIM2 proporciona `getMillis_TIM2()`, base de tiempo no bloqueante del firmware.

```
72 MHz / (71+1) / (999+1) = 1 000 Hz  →  1 ms exacto
PSC = 71  →  tick = 1 µs    ARR = 999  →  overflow cada 1 ms
Interrupción: TIM_IT_Update en cada overflow
```

ISR `TIM2_IRQHandler()` → `TimerInterrupt_1ms()` incrementa `Millis_TIM2`.
Usado en: sincronización de ciclo, timeouts Commander, `ServoMoveTime`.

---

### SysTick vs TIM2 — Comparativa

| | SysTick | TIM2 |
|---|---|---|
| **Tipo** | Cortex-M3 integrado | Periférico STM32 |
| **Resolución** | 10 µs | 1 ms exacto |
| **ISR** | `ISR_Delay_Base()` | `TimerInterrupt_1ms()` |
| **Patrón** | **Bloqueante** (espera activa) | **No bloqueante** (timestamps) |
| **Estado** | On/Off por demanda | Siempre activo |
| **Prioridad** | Sub 0 (máxima) | Sub 2 |
| **Analogía Arduino** | `delay()` | `millis()` |

> Retardos de hardware (UART timeouts, buzzer) → **SysTick**.
> Temporización del bucle de control → **TIM2**.

---

### `ADC_Configuration()` — Convertidores Analógico-Digital

| ADC | Canal | Pin | Señal |
|---|---|---|---|
| ADC1 | Canal 10 | PC0 | `SIG_ADC0` (multiplexor ext.) |
| ADC2 | Canal 5 | PA5 | `SIG_ADC1` (multiplexor ext.) |

Tensión de batería: PC3 (`PIN_VDD_VOLT`), leída por `ReadAnalog(VBUS)`.
Multiplexor 2-bit en PC1/PC2 selecciona entre sensores de los puertos OLLO.

---

### `USART_Configuration()` — Puertos Serie

#### USART1 — Bus Dynamixel

```
Velocidad : 1 000 000 bps  │  Pines: PB6(TX) / PB7(RX)  [remap]
RS-485    : PB4 (TX enable) / PB5 (RX enable)
Buffer RX : 1023 bytes  │  ISR: USART1_IRQ sub-priority 1
```

#### USART3 — PC UART / Odroid

```
Velocidad : 57 600 bps  │  Pines: PB10(TX) / PB11(RX)
Buffer RX : 32 bytes (circular)  │  ISR: USART3_IRQ sub-priority 3
Protocolo : 0xFF 0xFF + ángulos (signo + low + high) × 6 patas + checksum
```

#### UART5 — ZigBee / Commander

```
Velocidad : 57 600 bps  │  Pines: PC12(TX) / PD2(RX)
Buffer RX : 1023 bytes  │  ISR: UART5_IRQ sub-priority 3
Tramas    : 7 bytes a ~30 Hz (ArbotiX Commander)
```

Activar/desactivar comentando los `#define` en `CM530_HW/inc/usart.h`.

---

## 4. Bucle de Control Principal

```
─────────────────────────────────────────────────────────────────────
ARRANQUE (una sola vez)
─────────────────────────────────────────────────────────────────────
SysInit()       ← relojes, NVIC, GPIO, SysTick, TIM2, ADC, USART  [ver §3]
setupPhoenix()  ← estado inicial del robot                         [ver §5]

─────────────────────────────────────────────────────────────────────
BUCLE PRINCIPAL (infinito)
─────────────────────────────────────────────────────────────────────
lTimerStart = getMillis_TIM2()

┌─ FASE 1: ENTRADA ──────────────────────────────────────────────────
│  DoBackgroundProcess()
│  CheckVoltage()                 ← VBUS < 10.0 V → apaga robot
│  if (!g_fLowVoltageShutdown):
│      CommanderInputController_ControlInput()
│          ← lee trama del Commander/Odroid por UART
│          ← actualiza g_InControlState (TravelLength, BodyPos, BodyRot1,
│             GaitType, BalanceMode, LegLiftHeight, SpeedControl…)
│  WriteOutputs()                 ← stub vacío (reservado para LEDs)
└────────────────────────────────────────────────────────────────────

┌─ FASE 2: CONTROL DE PATA INDIVIDUAL ───────────────────────────────
│  SingleLegControl()    ← si OPT_SINGLELEG y SelectedLeg ≠ 255
│  DoBackgroundProcess()
└────────────────────────────────────────────────────────────────────

┌─ FASE 3: SECUENCIA DE MARCHA ──────────────────────────────────────
│  GaitSeq()
│      ← TravelLength vs cTravelDeadZone → TravelRequest
│      ← Gait(i) → GaitPosX/Y/Z[i], GaitRotY[i]  para cada pata
│      ← avanza GaitStep (1 … StepsInGait → 1)
│  DoBackgroundProcess()
└────────────────────────────────────────────────────────────────────

┌─ FASE 4: BALANCE (solo si BalanceMode == 1) ───────────────────────
│  Reset TotalTrans{X,Y,Z} = 0,  Total{X,Y,Z}Bal1 = 0
│  for patas derechas (0..2):  DoBackgroundProcess(); BalCalcOneLeg()
│  for patas izquierdas (3..5): DoBackgroundProcess(); BalCalcOneLeg()
│  BalanceBody()  ← promedia / BalanceDivFactor(6)
└────────────────────────────────────────────────────────────────────

┌─ FASE 5: CINEMÁTICA INVERSA ────────────────────────────────────────
│  IKSolution = IKSolutionWarning = IKSolutionError = 0
│  for patas derechas (0..2):
│      DoBackgroundProcess()
│      BodyFK()  ← desplazamiento por rotación/traslación del cuerpo
│      LegIK()   ← CoxaAngle1[i], FemurAngle1[i], TibiaAngle1[i]
│  for patas izquierdas (3..5):
│      DoBackgroundProcess(); BodyFK(); LegIK()
│  CheckAngles()  ← clamp a límites de Hex_Cfg.h
└────────────────────────────────────────────────────────────────────

┌─ FASE 6A: ROBOT ENCENDIDO ─────────────────────────────────────────
│  ServoMoveTime = NomGaitSpeed + InputTimeDelay×2 + SpeedControl
│                  [+ BALANCE_DELAY si BalanceMode]
│  DoBackgroundProcess()
│  StartUpdateServos()  ← ángulos → posición AX-12, envía estado a Odroid
│  if (algún GaitPos > cGPlimit):
│      espera: do { DoBackgroundProcess(); }
│              while (getMillis_TIM2() < lTimerStart + PrevServoMoveTime)
│  CommitServoDriver(ServoMoveTime)  ← arranca nueva interpolación
│
└─ FASE 6B: ROBOT APAGADO ───────────────────────────────────────────
   if (recién apagado): ServoMoveTime=600, StartUpdateServos(), Buzzed×3, wait(600ms)
   IdleTime()        ← driver de servos en reposo
   TerminalMonitor() ← atiende comandos debug por ZigBee
   mDelay(20)

─────────────────────────────────────────────────────────────────────
PrevServoMoveTime = ServoMoveTime  │  fPrev_RobotOn = fRobotOn  → volver al inicio
```

**Tiempo de ciclo típico** (andando, sin balance):

| Fase | Tiempo |
|---|---|
| Entrada + Commander | ~2–5 ms |
| GaitSeq + IK × 6 | ~6–11 ms |
| Espera `lTimeWaitEnd` | ~`ServoMoveTime` ms |
| **Total** | **~60–80 ms** |

> Detalle de las funciones de arranque:
> - [`setupPhoenix()`](#5-inicialización-de-la-aplicación) — estado inicial del robot, secuencia de 14 pasos
> - [`DoBackgroundProcess()`](#5-inicialización-de-la-aplicación) — motor de interpolación de servos y alarma de batería

---

## 5. Inicialización de la Aplicación

---

### 5.1 `setupPhoenix()` — Secuencia de inicio

Llamada una sola vez desde `main()` tras `SysInit()`. Configura el estado inicial
completo del robot antes de entrar en el bucle de control.

```
 1. Espera 3 s               ← while (getMillis_TIM2() < 3000)
 2. Banner por ZigBee
 3. initMemoryUsageTest()    ← mapea heap/stack, imprime diagnóstico RAM
 4. BioloidControllerEx()    ← lee posiciones REALES de los 18 servos AX-12
 5. Servo_Init()             ← configura driver AX-12, return delay = 0
 6. Posiciones iniciales     ← carga cInitPosX/Y/Z[6] desde Flash
 7. ResetLegInitAngles()     ← recalcula ángulos de coxa por defecto
 8. Estado de control        ← cero cuerpo, cero rotaciones
 9. Gait inicial             ← GaitType=0, GaitStep=1, LegLiftHeight=50
10. GaitSelect()             ← carga APG[0] (Ripple 12) en gaitCur
11. PrintGaitsTable()        ← vuelca tabla de marchas por ZigBee
12. CommanderInputController_Init() ← ControlMode = WALKMODE
13. ServoMoveTime=150, fRobotOn=0, g_fLowVoltageShutdown=FALSE
14. printMemoryUsage()
```

> La espera de 3 s (paso 1) permite que el bus Dynamixel y los módulos serie se
> estabilicen antes de que `BioloidControllerEx()` (paso 4) lea posiciones reales
> de los servos. Sin esta espera, los servos no responderían y la primera
> interpolación arrancaría desde valores erróneos causando movimientos bruscos.

#### Estado inicial del robot tras `setupPhoenix()`

| Variable | Valor | Descripción |
|---|---|---|
| `fRobotOn` | `0` | Apagado — espera comando del Commander |
| `GaitType` | `0` | Ripple 12 (primer gait) |
| `LegLiftHeight` | `50` mm | Altura de elevación de patas |
| `BalanceMode` | `0` | Balance desactivado |
| `GaitStep` | `1` | Primer paso del ciclo |
| `BodyPos` | `{0,0,0}` | Cuerpo en posición neutra |
| `BodyRot1` | `{0,0,0}` | Sin rotación de cuerpo |
| `SelectedLeg` | `255` | Sin pata seleccionada |
| `SpeedControl` | `0` | Sin ajuste de velocidad adicional |
| `ServoMoveTime` | `150` ms | Tiempo de movimiento inicial |
| `LegPosX/Y/Z[i]` | `cInitPosX/Y/Z[i]` | Posición geométrica de reposo |
| `ControlMode` | `WALKMODE` | Modo marcha por defecto |

---

### 5.2 `DoBackgroundProcess()` — Procesamiento de fondo

Macro que se expande a `BackgroundProcess()` cuando `OPT_BACKGROUND_PROCESS` está
activo, o desaparece en compilación si no:

```c
#ifdef OPT_BACKGROUND_PROCESS
    #define DoBackgroundProcess()   BackgroundProcess()
#else
    #define DoBackgroundProcess()
#endif
```

Cada llamada ejecuta dos tareas:

| Tarea | Función | Descripción |
|---|---|---|
| Interpolación | `BioloidControllerEx_interpolateStep(FALSE)` | Avanza un frame de movimiento de servos |
| Monitor batería | `Battery_Monitor_Alarm()` | Alarma acústica si VBUS < 11.5 V |

**Por qué se llama 8+ veces por ciclo:**
Los cálculos de IK tardan ~5–10 ms. Sin llamadas periódicas, los servos solo
recibirían posiciones intermedias una vez por ciclo (~60 ms), produciendo movimiento
entrecortado. Con `frameLength = 33 ms` se generan **2–3 frames por ciclo**.

**Alarma de batería:**

```c
#define VBUS_LOW_LIMIT  115   // 11.5 V — adc.h
if (volt < VBUS_LOW_LIMIT) { Buzzed(500,100); Buzzed(500,5000); × 2 }
```

| Umbral | Valor | Acción |
|---|---|---|
| `VBUS_LOW_LIMIT` | 11.5 V | 4 pitidos — el robot sigue funcionando |
| `cTurnOffVol` | 10.0 V | Apagado completo del robot |

---

### 5.3 Motor de Interpolación de Servos

#### Conceptos clave

**Doble buffer:**
Los 18 servos nunca saltan directamente a la posición objetivo. Se gestionan con
dos arrays paralelos:

```
pose_[18]     ← posición actual interpolada (lo que el servo tiene ahora)
nextpose_[18] ← posición objetivo           (lo que queremos que alcance)
```

**Ticks — unidades de posición del AX-12:**

```
Rango: 0–1023   (10 bits)
  0   = ≈   0°  (mínimo)
 512  = ≈ 150°  (centro neutro — cPFConst)
1023  = ≈ 300°  (máximo)
```
$$1\ \text{tick} \approx 0.293°$$

Conversión ángulo IK (décimas de grado) → ticks:
```c
posicion_AX12 = (angulo × 128) / 375 + 512
// coxa 450 (45°)  →  (450×128)/375 + 512 = 666 ticks
```

**`ServoMoveTime` — tiempo disponible para el movimiento (ms):**

```c
ServoMoveTime = NomGaitSpeed          // 60–80 ms  (velocidad base del gait)
              + InputTimeDelay × 2    // 0–256 ms  (posición del joystick)
              + SpeedControl;         // 0–2000 ms (ajuste manual L6+rightH)
// + BALANCE_DELAY (100 ms) si BalanceMode activo
// Parado: ServoMoveTime = 200 + SpeedControl
```
Mayor `ServoMoveTime` → movimiento más lento y suave.

**Estados del motor:**

```
interpolating = 0  →  IDLE     Sin movimiento. interpolateStep() retorna al instante.
                                 ↑ CommitServoDriver() calcula speed_[i] → RUNNING
interpolating = 1  →  RUNNING  Cada 33 ms: pose_[i] ± speed_[i] → SYNC_WRITE DXL
                                 ↑ Cuando TODOS los servos llegan → IDLE
```

**Comportamiento de bloqueo según `fWait`:**

| `fWait` | Quién lo usa | Comportamiento |
|---|---|---|
| `FALSE` | `DoBackgroundProcess()` | Retorna si el frame no ha vencido |
| `TRUE` | `BeginServoUpdate()` | Siempre espera hasta que el frame termine |

---

#### Las 4 fases del ciclo de interpolación

**FASE 1 — `StartUpdateServos()` — Preparar la pose objetivo**

```
BeginServoUpdate()
    └─→ interpolateStep(TRUE)       ← bloquea hasta terminar el frame en curso

OutputServoInfoForLeg(i) × 6
    └─→ ángulo × 128 / 375 + 512   ← convierte IK a ticks AX-12
        aplica inversión cXXInv[i]
    └─→ setNextPose(id, ticks)      ← llena nextpose_[]
```

Al final: `nextpose_[]` contiene las 18 posiciones objetivo en ticks AX-12.

---

**FASE 2 — `CommitServoDriver(ServoMoveTime)` — Calcular velocidades**

```
frames    = ServoMoveTime / 33 + 1
delta     = |nextpose_[i] − pose_[i]|   (ticks a recorrer)
speed_[i] = delta / frames + 1          (ticks por frame, mínimo 1)

interpolating = 1
nextframe_    = now + 33 ms
```

Ejemplo — `ServoMoveTime = 60 ms`, servo que debe moverse 50 ticks (≈ 14.6°):
```
frames = 2   speed_[i] = 26 ticks/frame
Frame 1: pose += 26  →  26
Frame 2: pose += 26  →  52 > 50  →  snap final: pose = 50  ✓
```

---

**FASE 3 — `DoBackgroundProcess()` × N — Avanzar frame a frame**

Cada llamada ejecuta `interpolateStep(FALSE)`:

```
¿interpolating == 0?           → retorna (nada que hacer)
¿now < nextframe_ − 10 ms?    → retorna (frame aún no vencido)
Espera: while (now < nextframe_)
nextframe_ += 33 ms

Para cada servo i:
    Si |diff| < speed_[i]   → pose_[i] = nextpose_[i]  (snap final)
    Si diff > 0              → pose_[i] += speed_[i]
    Si diff < 0              → pose_[i] −= speed_[i]

¿Todos en destino?  → interpolating = 0
→ SYNC_WRITE: [id, pos_low, pos_high] × 18 en un único paquete Dynamixel
```

`WAIT_SLOP_FACTOR = 10 ms` — permite ejecutar el frame hasta 10 ms antes de su
vencimiento, compensando imprecisiones del scheduler de interrupciones.

---

**FASE 4 — `BeginServoUpdate()` — Cierre del ciclo**

`interpolateStep(TRUE)` — bloquea hasta que `nextframe_` vence y ejecuta el frame.
Garantiza que `nextpose_[]` no se sobreescribe con nuevos ángulos IK antes de que
el frame anterior haya terminado, evitando saltos bruscos.

---

#### Cronogramas (ServoMoveTime = 60 ms, 2 frames)

**Vista de alto nivel por ciclo:**

```
t= 0 ms  CommitServoDriver(60)   → interpolating=1, nextframe_=33
t= 5 ms  DoBackgroundProcess()   → 5 < 23 (33−10)  RETORNA
t=33 ms  DoBackgroundProcess()   → 33 ≥ 23          EJECUTA frame 1 → SYNC_WRITE
                                                     nextframe_=66
t=40 ms  DoBackgroundProcess()   → 40 < 56 (66−10)  RETORNA
t=60 ms  BeginServoUpdate()      → fWait=TRUE        espera hasta t=66
                                                     EJECUTA frame 2 → SYNC_WRITE
                                                     interpolating=0
```

**Detalle de `interpolateStep(FALSE)` — decisión en cada llamada:**

```
Llamada  now    nextframe_  now < nextframe_−10?  Acción
──────────────────────────────────────────────────────────────────
  #1      5      38          5 < 28  →  SÍ        RETORNA (0 coste)
  #2     15      38         15 < 28  →  SÍ        RETORNA (0 coste)
  #3     28      38         28 < 28  →  NO        espera hasta 38 → FRAME 1
                                                  SYNC_WRITE, nextframe_=71
  #4     45      71         45 < 61  →  SÍ        RETORNA (0 coste)
  #5     55      71         55 < 61  →  SÍ        RETORNA (0 coste)
──── BeginServoUpdate() fWait=TRUE ────────────────────────────────
  #6     60      71           —                   espera hasta 71 → FRAME 2
                                                  SYNC_WRITE, interpolating=0
```

---

#### Variables de estado

| Variable | Tipo | Descripción |
|---|---|---|
| `pose_[18]` | `unsigned int[]` | Posición actual interpolada (ticks AX-12) |
| `nextpose_[18]` | `unsigned int[]` | Posición objetivo (ticks AX-12) |
| `speed_[18]` | `int[]` | Ticks por frame para cada servo |
| `id_[18]` | `unsigned char[]` | IDs Dynamixel (tabla `cPinTable`) |
| `poseSize` | `int` | Servos activos = 18 (`NUMSERVOS`) |
| `interpolating` | `unsigned char` | 0 = IDLE, 1 = RUNNING |
| `nextframe_` | `unsigned long` | Timestamp (ms) del próximo frame |
| `frameLength` | `u8` | Duración de frame en ms (= 33) |

---



### Modos de Control

Ciclan con botón **LT**:

| Modo | Constante | Descripción |
|---|---|---|
| 0 | `WALKMODE` | Marcha: joystick izq = avance/lateral, der = rotación |
| 1 | `TRANSLATEMODE` | Traslación del cuerpo sin mover los pies |
| 2 | `ROTATEMODE` | Rotación del cuerpo (pitch, roll, yaw) |
| 3 | `SINGLELEGMODE` | Control manual de una pata individual |

### Mapa de Botones

| Botón | Función |
|---|---|
| **LT** | Cicla entre modos (Walk → Translate → Rotate → Single Leg) |
| **RT** | En Walk: cicla Normal/Double Height × Normal/Double Travel |
| **R1** | En Walk: cambia gait │ En Single Leg: siguiente pata |
| **R2** | En Walk: alterna método de joystick 1 (izq.) / 2 (der.) |
| **L4** | Activa/desactiva modo Balance |
| **L5** | Sentarse / Levantarse (`g_BodyYOffset` 0 ↔ 35 mm) |
| **L6 + RightJoy** | Subir/bajar cuerpo, ajustar anchura de patas, ajustar velocidad |

### Parámetros de Comunicación

```c
#define cTravelDeadZone  6      // Zona muerta joystick
#define ARBOTIX_TO       1250   // ms sin trama → robot se apaga
```

---

## 7. Sistema de Marcha (Gaits)

### Estructura `PHOENIXGAIT`

```c
typedef struct _PhoenixGait {
    short NomGaitSpeed;      // Velocidad nominal (ms/paso)
    byte  StepsInGait;       // Pasos totales del ciclo
    byte  NrLiftedPos;       // Posiciones con pata elevada (1/2/3/5)
    byte  FrontDownPos;      // Paso en que la pata toca el suelo
    byte  LiftDivFactor;     // Divisor del TravelLength en fases parciales
    byte  TLDivFactor;       // Pasos que la pata está en el suelo
    byte  HalfLiftHeight;    // Altura a mitad de trayectoria
    byte  GaitLegNr[6];      // Orden de paso {RR, RM, RF, LR, LM, LF}
} PHOENIXGAIT;
```

### Marchas Disponibles (Hexápodo)

| Índice | Nombre | ms/paso | Pasos | `NrLiftedPos` | `HalfLiftHeight` | Descripción |
|---|---|---|---|---|---|---|
| 0 | Ripple 12  | 80 | 12 | 3 |  8 | Ondulado — más suave y estable |
| 1 | Tripod 8   | 80 |  8 | 3 |  4 | Trípode lento — 3 patas por ciclo |
| 2 | Triple 12  | 60 | 12 | 3 |  8 | Trípode triple — equilibrio/velocidad |
| 3 | Triple 16  | 60 | 16 | 5 | 10 | Trípode con 5 posiciones elevadas |
| 4 | Wave 24    | 80 | 24 | 3 | 20 | Ola — máxima estabilidad |
| 5 | Tripod 6   | 60 |  6 | 2 |  4 | Trípode rápido — mínimo ciclo |

> Selección con botón **R1** cuando el robot está en reposo.

### Velocidades Base

```c
// Hex_Cfg.h
#define DEFAULT_GAIT_SPEED   60   // ms/paso — gaits rápidos (índices 2, 3, 5)
#define DEFAULT_SLOW_GAIT    80   // ms/paso — gaits lentos (índices 0, 1, 4)
```

---

## 8. Cinemática Inversa (IK)

### Concepto

Dos etapas encadenadas por pata, 6 patas por ciclo. Toda la trigonometría usa
**aritmética entera** con tablas precalculadas en Flash (sin punto flotante).

```
[LegPosX/Y/Z + GaitPos + BodyPos]
         │
         ▼
   BodyFK()     ← Cinemática directa del cuerpo → BodyFKPosX/Y/Z
         │
         ▼
   LegIK()      ← IK 3DOF → CoxaAngle1[], FemurAngle1[], TibiaAngle1[]
         │
         ▼
  CheckAngles() ← Clamp a límites de Hex_Cfg.h
         │
         ▼
OutputServoInfoForLeg() → posición AX-12 → bus Dynamixel
```

### `BodyFK()` — Cinemática Directa del Cuerpo

Calcula el desplazamiento producido por la rotación/traslación del cuerpo sobre
el punto de anclaje de cada coxa.

```c
// Rotaciones aplicadas (g_InControlState + corrección de balance):
// Pitch (X)  → BodyRot1.x + TotalXBal1
// Yaw (Y)    → BodyRot1.y + GaitRotY[i] + TotalYBal1
// Roll (Z)   → BodyRot1.z + TotalZBal1
```

**Salidas:** `BodyFKPosX`, `BodyFKPosY`, `BodyFKPosZ`

### `LegIK()` — Cinemática Inversa de Pata

**Modelo geométrico:**
```
[Coxa joint] ── Coxa (52 mm) ── [Femur joint] ── Fémur (66 mm) ── [Tibia joint] ── Tibia (113 mm) ── [PIE]
```

**Cálculo (aritmética entera con tablas Flash):**

1. $\theta_{Coxa} = \text{atan2}(X_{pie}, Z_{pie}) - \text{cCoxaAngle1}[i]$
2. $XZ_{pie} = \sqrt{X^2 + Z^2} - L_{Coxa}$
3. Ley del coseno (problema 2R):

$$\theta_{Tibia} = \arccos\!\left(\frac{D^2 - L_F^2 - L_T^2}{2 L_F L_T}\right) - \text{TibiaHornOffset}$$

$$\theta_{Femur} = \arctan2(Y, XZ) - \arccos\!\left(\frac{D^2 + L_F^2 - L_T^2}{2 D L_F}\right) + \text{FemurHornOffset}$$

**Indicadores de solución:**

| Variable | Significado |
|---|---|
| `IKSolution = 1` | Solución válida |
| `IKSolutionWarning = 1` | En el límite del espacio de trabajo |
| `IKSolutionError = 1` | Pie fuera de alcance mecánico |

### `CheckAngles()` — Validación de Límites

```c
CoxaAngle1[i]  = constrain(CoxaAngle1[i],  cCoxaMin1[i],  cCoxaMax1[i]);
FemurAngle1[i] = constrain(FemurAngle1[i], cFemurMin1[i], cFemurMax1[i]);
TibiaAngle1[i] = constrain(TibiaAngle1[i], cTibiaMin1[i], cTibiaMax1[i]);
```

### Balance Dinámico

Activo con **L4** (`BalanceMode == 1`). `BalCalcOneLeg()` acumula totales de posición
para cada pata; `BalanceBody()` los promedia con `BalanceDivFactor` (= 6) y produce
una corrección de cuerpo que se inyecta en `BodyFK()` del ciclo siguiente.

```c
#define BalanceDivFactor  CNT_LEGS   // = 6
#define BALANCE_DELAY     100        // ms extra de ServoMoveTime
```

### Tablas Matemáticas (Flash)

| Tabla | Tamaño | Precisión | Uso |
|---|---|---|---|
| `GetSin[180]` | 180 words | 0.5°/paso | Seno 0°–90°, escala × 10000 |
| `GetACos[277]` | 277 bytes | 3 tramos | ArcCoseno 0–1, escala × 10000 |

---

## 9. Suavizado de Movimiento

Seis mecanismos en capas producen el movimiento fluido del robot.

---

### Capa 1 — Zona muerta y efecto sneaking

```c
#define cTravelDeadZone  6   // joystick ignorado si < 6
g_InControlState.InputTimeDelay = 128 - max(max(abs(lx), abs(ly)), abs(rightH));
```

| Deflexión joystick | `InputTimeDelay` | Efecto en `ServoMoveTime` |
|---|---|---|
| Centrado (≤6) | ~128 | Robot en reposo |
| ¼ recorrido | ~96 | Arranque lento ("sneaking") |
| ½ recorrido | ~64 | Velocidad media |
| Máximo (~125) | ~3 | Velocidad máxima |

---

### Capa 2 — `SmoothControl()` — Inercia del cuerpo (Translate/Rotate)

```c
#define SmDiv  4   // Phoenix.h — divisor recomendado: 3 a 5
// Por ciclo: CtrlMoveOut += |diff| / SmDiv  (con snap directo si |diff| ≤ 4)
g_InControlState.BodyPos.x  = SmoothControl((lx*2/3),  BodyPos.x,  SmDiv);
g_InControlState.BodyPos.z  = SmoothControl((ly*2/3),  BodyPos.z,  SmDiv);
g_InControlState.BodyRot1.y = SmoothControl((rightH*2), BodyRot1.y, SmDiv);
```

| `SmDiv` | Comportamiento |
|---|---|
| 2–3 | Sigue al joystick rápido, menos suave |
| **4** | **Valor actual** |
| 5–6 | Muy suave, retardo notable |

---

### Capa 3 — `ServoMoveTime` — Duración del movimiento

```c
// El parámetro más importante para el carácter del movimiento:
ServoMoveTime = gaitCur.NomGaitSpeed          // 60–80 ms base
              + (InputTimeDelay * 2)           // 0–256 ms (joystick)
              + g_InControlState.SpeedControl; // 0–2000 ms (ajuste manual)
if (BalanceMode) ServoMoveTime += BALANCE_DELAY; // +100 ms

// Ajuste manual en tiempo real con L6 + rightH:
SpeedControl += rightH / 16;   // clamp [0, 2000] — buzzer confirma
```

---

### Capa 4 — Interpolación Bioloid (~30 Hz)

```c
#define BIOLOID_FRAME_LENGTH  33   // ms/frame

// interpolateSetup: speed_[i] = delta / frames + 1
// interpolateStep:  pose_[i] ± speed_[i]  cada frame → writePose() SYNC_WRITE
```

| `BIOLOID_FRAME_LENGTH` | Frames/ciclo | Suavidad |
|---|---|---|
| 16 ms | ~4 | Máxima, mayor carga bus |
| **33 ms** | **~2** | **Valor actual** |
| 50 ms | ~1 | Menor suavidad |

---

### Capa 5 — Perfil de trayectoria de pata

```
    ───────●───────●────────●────────●──────  suelo
        FrontDown  ½ sub.  cima   ½ baj.
```

| Parámetro | Efecto |
|---|---|
| `NrLiftedPos` | Nº de posiciones intermedias (1/2/3/5) |
| `HalfLiftHeight` | Altura intermedia: `3×LH/(3+H)` |
| `LegLiftHeight` | Altura máxima: 50 mm normal / 80 mm doble (RT) |

---

### Capa 6 — Sincronización de paso (`bExtraCycle`)

```c
#define cGPlimit  2   // mm — umbral pata "en suelo"
// Espera que termine el paso anterior antes de empezar el nuevo:
do { DoBackgroundProcess(); } while (getMillis_TIM2() < lTimerStart + PrevServoMoveTime);
```

---

### Resumen de palancas

| Mecanismo | Archivo | Efecto |
|---|---|---|
| `cTravelDeadZone` | `Phoenix_Input_Commander.h` | Umbral arranque marcha |
| `SmDiv` | `Phoenix.h` | Inercia cuerpo en Translate/Rotate |
| `DEFAULT_GAIT_SPEED` | `Hex_Cfg.h` | Velocidad base gaits rápidos |
| `DEFAULT_SLOW_GAIT` | `Hex_Cfg.h` | Velocidad base gaits lentos |
| `SpeedControl` | en tiempo real (L6+rightH) | Retardo global adicional |
| `BIOLOID_FRAME_LENGTH` | `BioloidEx.h` | Resolución interpolación servos |
| `LegLiftHeight` | en tiempo real (RT) | Altura de paso |
| `HalfLiftHeight` | `APG[]` en `main.c` | Forma trayectoria pata |
| `NrLiftedPos` | `APG[]` en `main.c` | Posiciones intermedias pata |
| `BALANCE_DELAY` | `Phoenix.h` | Pausa extra en modo balance |
| `cGPlimit` | `main.c` | Tolerancia sincronización de paso |

---

## 10. Referencia de Configuración (`Hex_Cfg.h`)

> **Único archivo a modificar** para adaptar el firmware a distintas variantes
> de PhantomX o para cambiar el comportamiento del robot.

### Flags de compilación

| `#define` | Descripción |
|---|---|
| `USECOMMANDER` | Activa el controlador ArbotiX Commander |
| `OPT_SINGLELEG` | Activa el modo de control de pata individual |
| `USE_AX12_DRIVER` | Usa el driver AX-12 |
| `OPT_BACKGROUND_PROCESS` | Habilita interpolación en background |
| `HEXMODE` (en `main.c`) | Modo hexápodo — alternativa: `OCTOMODE` |
| `//#define MILLIS` | Debug de tiempos por serie |
| `//#define SENSOR` | Lectura de sensores IR/analógicos |
| `//#define DEBUG_BIOLOIDEX` | Debug del controlador Bioloid |

### Dimensiones físicas (mm)

```c
#define cXXCoxaLength    52
#define cXXFemurLength   66
#define cXXTibiaLength  113
```

### Geometría del cuerpo

| Pata | OffsetX (mm) | OffsetZ (mm) | Ángulo coxa (×0.1°) |
|---|---|---|---|
| RR | −60 | +120 | −450 |
| RM | −100 | 0 | 0 |
| RF | −60 | −120 | +450 |
| LR | +60 | +120 | −450 |
| LM | +100 | 0 | 0 |
| LF | +60 | −120 | +450 |

```c
#define X_COXA   60  │  #define Y_COXA   60  │  #define M_COXA  100
#define cHexInitXZ  147  │  #define CHexInitY  25  │  #define MAX_BODY_Y  150
```

### IDs de servos

| Pata | Coxa | Fémur | Tibia |
|---|---|---|---|
| RR | 8 | 10 | 12 |
| RM | 14 | 16 | 18 |
| RF | 2 | 4 | 6 |
| LR | 7 | 9 | 11 |
| LM | 13 | 15 | 17 |
| LF | 1 | 3 | 5 |

### Límites de articulaciones (décimas de grado)

| Articulación | Mínimo | Máximo |
|---|---|---|
| Coxa | −750 (−75°) | +750 (+75°) |
| Fémur | −900 (−90°) | +900 (+90°) |
| Tibia | −1020 (−102°) | +670 (+67°) |

### Calibración

```c
#define cFemurHornOffset1   -150   // ajustar en múltiplos de ±150 (= ±15°)
#define cTibiaHornOffset1    200
```

### Protección de batería

```c
#define cTurnOffVol  100   // < 10.0 V → apagado del robot
#define cTurnOnVol   110   // > 11.0 V → reencendido automático
// VBUS_LOW_LIMIT = 115 (11.5 V) → alarma acústica previa  [adc.h]
```

### Driver AX-12

```c
#define cPwmMult   128    // posición_servo = (ángulo × 128) / 375 + 512
#define cPwmDiv    375
#define cPFConst   512

#define USE_AX12_SPEED_CONTROL        // velocidad individual por servo
// Factor: 848 × recorrido / tiempo — clamp [26, 1023] ticks

#define VOLTAGE_MIN_TIME_BETWEEN_CALLS   150    // ms
#define VOLTAGE_MAX_TIME_BETWEEN_CALLS  1000    // ms
#define VOLTAGE_TIME_TO_ERROR           3000    // ms
```

---

## 11. Guía de Personalización

### Ajustar velocidad general
```c
#define DEFAULT_GAIT_SPEED  60   // reducir para ir más rápido
#define DEFAULT_SLOW_GAIT   80
```

### Calibrar postura inicial
```c
#define cHexInitXZ  147   // aumentar para separar más las patas
```

### Calibrar ángulo neutro de servos
```c
#define cFemurHornOffset1  -150   // múltiplos de ±150
#define cTibiaHornOffset1   200
```

### Adaptar a otro robot
```c
#define cXXCoxaLength    52    // medir en mm
#define cXXFemurLength   66
#define cXXTibiaLength  113
```

### Cambiar protección de batería
```c
#define cTurnOffVol  100   // × 0.1 V
#define cTurnOnVol   110
```

### Ajustar límites mecánicos
```c
#define cXXTibiaMin1   -1020
#define cXXTibiaMax1    670
```

### Monitor Terminal — comandos debug (via ZigBee, robot apagado)

| Comando | Función |
|---|---|
| `M` | Muestra el menú |
| `P` | Imprime uso de memoria |
| `G` | Imprime tabla de gaits |
| `A` | Imprime tabla de registros AX-12 |
| `C#id#(e\|r\|a)#` | Lee Control Table de un servo |
| `X#id#R\|W#size#addr#` | Lee/escribe registro de un servo |
| `R` | Reset por software |
| `L` | Ajuste dinámico de posición de patas |

---

## 12. Compilación y Build

Requiere `arm-none-eabi-gcc` en el PATH.

```bash
make           # Genera CM530.elf, CM530.hex, CM530.bin, CM530.lss
make clean     # Limpia objetos intermedios
```

Flashear con **RoboPlus Manager** o `dfu-util` usando `CM530.bin`.
El firmware ocupa Flash desde `0x08003000` (tras los 12 KB del bootloader).

---

## 13. Referencias

- Proyecto original: [Lynxmotion Phoenix](https://github.com/KurtE/Arduino_Phoenix_Parts)
- Hardware: [PhantomX Hexapod MkII – Trossen Robotics](https://www.trossenrobotics.com/)
- Controlador: [Robotis CM-530](https://emanual.robotis.com/docs/en/parts/controller/cm-530/)
- Servos: [AX-12A Dynamixel](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
