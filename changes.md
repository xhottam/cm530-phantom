# Registro de Cambios

---

## [2026-08-28] PC_UART — Ampliar buffer RX y corregir inicialización de índices

### Motivación

El buffer de recepción del canal PC_UART (USART3 / Odroid) tenía un tamaño de
**32 bytes**, 32 veces menor que los buffers de Dynamixel y ZigBee (1023 bytes),
a pesar de operar a la misma velocidad (57600 bps).

A 57600 bps un byte llega cada ~174 µs. Con 32 bytes el buffer se llenaba en:

```
t_overflow = 32 × 174 µs ≈ 5.5 ms
```

El bucle principal de control tarda ~60–80 ms por ciclo, por lo que si el Odroid
emite datos de forma continua el buffer se desbordaba antes de que
`Commander_ReadMsgs_Odroid()` pudiera vaciarlo. El desbordamiento era **silencioso**
(pérdida del byte más antiguo sin flag de error), lo que producía:

1. **Desincronización de trama** — checksums incorrectos en los frames de 7 bytes.
2. **Apagado por timeout** — tramas inválidas consecutivas superaban `ARBOTIX_TO = 1250 ms`.

Adicionalmente, `gbPcuWrite` y `gbPcuRead` se declaraban sin inicializador explícito,
siendo inconsistente con el resto de buffers del sistema (`gbZigWrite`, `gbZigRead`,
`gbDxlWrite`, `gbDxlRead`) que sí tienen `= 0`.

### Cambios realizados

**Archivo:** `CM530_HW/inc/usart.h`

**1. Ampliar buffer PC_UART:**
```c
// Antes:
#define PC_UART_BUFFER_LENGTH  32

// Después:
#define PC_UART_BUFFER_LENGTH  1023
```

**2. Inicialización explícita de índices del buffer circular:**
```c
// Antes:
static volatile u16 gbPcuWrite, gbPcuRead;

// Después:
static volatile u16 gbPcuWrite = 0, gbPcuRead = 0;
```

### Resultado

| Parámetro | Antes | Después |
|---|---|---|
| Capacidad buffer | 32 bytes | 1023 bytes |
| Tiempo hasta overflow | ~5.5 ms | ~178 ms |
| Margen sobre ciclo de control (80 ms) | ✗ Negativo | ✓ +98 ms |
| Inicialización índices | Implícita | Explícita `= 0` |
| Consistencia con DXL / ZigBee | ✗ | ✓ |

**Impacto en RAM:** +991 bytes sobre los ~20 KB de SRAM disponibles (~5%).

### Archivos modificados

| Archivo | Símbolo | Cambio |
|---|---|---|
| `CM530_HW/inc/usart.h` | `PC_UART_BUFFER_LENGTH` | 32 → 1023 |
| `CM530_HW/inc/usart.h` | `gbPcuWrite`, `gbPcuRead` | Inicialización explícita `= 0` |

---

## [2026-08-28] TIM2 — Reemplazar rolling-compare CC1 por modo auto-reload Update

### Motivación

El diseño original de `Timer_Configuration()` usaba un patrón de **rolling compare** sobre TIM2:
- Contador libre con período máximo de 65535 ticks (~658 ms hasta overflow).
- La ISR calculaba dinámicamente el siguiente valor de compare:
  ```c
  capture = TIM_GetCapture1(TIM2);
  TIM_SetCompare1(TIM2, capture + (vu16)100);
  ```

**Problema:** si entre `TIM_GetCapture1()` y `TIM_SetCompare1()` el contador TIM2
superaba el nuevo valor de compare (posible con preemption de una ISR de mayor prioridad),
la siguiente interrupción CC1 no se producía hasta el próximo overflow del contador de
16-bit (~658 ms). Durante ese tiempo `Millis_TIM2` dejaba de incrementar, bloqueando
todo el bucle de control principal.

---

### Cambios realizados

#### `CM530_HW/src/system_init.c` — `Timer_Configuration()`

**Antes:**
```c
TIM_TimeBaseStructure.TIM_Period    = 65535;  // contador libre
TIM_TimeBaseStructure.TIM_Prescaler = 0;
TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_Timing;
TIM_OCInitStructure.TIM_Pulse       = (vu16)100;
TIM_OC1Init(TIM2, &TIM_OCInitStructure);
TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
```

**Después:**
```c
// 72 MHz / (71+1) / (999+1) = 1000 Hz → 1 ms exacto
TIM_TimeBaseStructure.TIM_Period    = 999;   // ARR: auto-reload cada 1000 ticks
TIM_TimeBaseStructure.TIM_Prescaler = 71;    // PSC: 72 MHz / 72 = 1 MHz tick
TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);   // dispara en overflow del ARR
```

Código eliminado: `TIM_OCInitTypeDef`, `TIM_OCStructInit()`, `TIM_PrescalerConfig()`,
`TIM_OC1Init()`, `TIM_OC1PreloadConfig()`.

---

#### `CM530_HW/src/system_func.c` — `TimerInterrupt_1ms()`

**Antes:**
```c
void TimerInterrupt_1ms(void) //OLLO CONTROL
{
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        Millis_TIM2++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        capture = TIM_GetCapture1(TIM2);
        TIM_SetCompare1(TIM2, capture + (vu16)100);  // ← cálculo dinámico eliminado
        if (gw1msCounter > 0)
            gw1msCounter--;
    }
}
```

**Después:**
```c
void TimerInterrupt_1ms(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        Millis_TIM2++;
        if (gw1msCounter > 0)
            gw1msCounter--;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  // al final, tras escribir contadores
    }
}
```

Cambios adicionales en la ISR:
- `TIM_ClearITPendingBit()` movido al **final** del handler: garantiza que los
  contadores se actualizan antes de limpiar el flag, evitando re-entrada espuria
  si una ISR de mayor prioridad preempta entre medio.
- Eliminadas las llamadas a `TIM_GetCapture1()` y `TIM_SetCompare1()`.
- La variable global `capture` queda sin uso (puede eliminarse en una limpieza futura).

---

### Cálculo del nuevo período

| Parámetro | Valor | Descripción |
|---|---|---|
| Reloj fuente TIM2 | 72 MHz | APB1 × 2 (PCLK1=36 MHz, multiplicador de timer = ×2) |
| Prescaler (PSC) | 71 | Divisor = 72 → tick = 1 µs (1 MHz) |
| Auto-reload (ARR) | 999 | Overflow cada 1000 ticks = 1 ms |
| Frecuencia ISR | **1000 Hz** | Período **1 ms exacto** |

$$f_{ISR} = \frac{72\,000\,000}{(71+1)\times(999+1)} = \frac{72\,000\,000}{72\,000} = 1000\,\text{Hz}$$

**Antes** (rolling compare): ~995 Hz (~1.004 ms, acumulaba deriva)  
**Ahora** (auto-reload): 1000 Hz exacto, determinista, sin riesgo de miss.

---

### Archivos modificados

| Archivo | Función |
|---|---|
| `CM530_HW/src/system_init.c` | `Timer_Configuration()` |
| `CM530_HW/src/system_func.c` | `TimerInterrupt_1ms()` |

---

## [2026-08-28] NVIC — Corrección: `NVIC_Init()` faltante para UART5 (ZigBee)

### Motivación

En `NVIC_Configuration()`, el bloque de configuración de UART5 (ZigBee/Commander)
rellenaba correctamente la estructura `NVIC_InitStructure` pero **omitía la llamada
a `NVIC_Init()`**. Como resultado, la estructura era inmediatamente sobreescrita con
los valores de TIM2, y `NVIC_Init()` registraba TIM2 en lugar de UART5.

La ISR `UART5_IRQHandler` nunca era habilitada en el hardware NVIC, por lo que la
recepción por interrupción del módulo ZigBee/Commander no funcionaba.

### Cambio realizado

**Archivo:** `CM530_HW/src/system_init.c` — `NVIC_Configuration()`

**Antes:**
```c
	// Enable the UART5 Interrupt (Zigbee)
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// ← NVIC_Init() ausente: la configuración de UART5 nunca se aplicaba

	// Enable the TIM2 global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;  // sobreescribía UART5
	...
	NVIC_Init(&NVIC_InitStructure);  // registraba TIM2, no UART5
```

**Después:**
```c
	// Enable the UART5 Interrupt (Zigbee)
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  // ← añadido

	// Enable the TIM2 global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	...
	NVIC_Init(&NVIC_InitStructure);
```

### Efecto

| | Antes | Después |
|---|---|---|
| UART5 en NVIC | ✗ No registrado | ✓ Registrado (preemption 0, sub 3) |
| ISR `UART5_IRQHandler` | Nunca disparaba | Dispara en cada byte recibido |
| Recepción ZigBee | Solo por polling | Por interrupción (comportamiento esperado) |

### Archivo modificado

| Archivo | Función |
|---|---|
| `CM530_HW/src/system_init.c` | `NVIC_Configuration()` |