# DeepSea Level 3 DC Fast Charger - Herramienta de Diagnóstico CAN

Herramienta de diagnóstico desarrollada en Python 3 sin dependencias externas, utilizando SocketCAN en crudo para monitorear módulos de potencia, decodificar telemetría, filtrar ruido en el bus y reensamblar tramas múltiples de diagnóstico estilo ISO-TP bajo condiciones adversas de red.

## Decisiones de Arquitectura y Diseño

### 1. Elección del Mecanismo de Filtrado
La estrategia de filtrado se implementó en espacio de usuario inmediatamente después de extraer la trama del socket:

- **Filtrado de Ruido (`0x200`–`0x2FF`):** El requerimiento exige que el tráfico ajeno ni interrumpa la ejecución ni sea contabilizado en el total de tramas válidas procesadas (`frames_processed`). Los identificadores CAN se normalizan con una máscara de 11 bits (`can_id & 0x7FF`) y se evalúan contra los límites del rango de ruido:
  ```python
  if NOISE_ID_MIN <= can_id <= NOISE_ID_MAX:
      continue
  ```

- **Justificación:** Manejar el descarte en espacio de usuario en lugar de configurar filtros en el kernel (`CAN_RAW_FILTER`) garantiza un control determinista sobre el contador exacto exigido por el evaluador automatizado. Esto previene discrepancias por tramas descartadas a nivel de socket y mantiene la compatibilidad entre interfaces CAN físicas y virtuales (`vcan`).

### 2. Reensamblado ISO-TP y Manejo de Estados

Los mensajes de identificación multi-trama (`0x6F0`–`0x6F3`) siguen el formato de tramas de ISO 15765-2. El motor de reensamblado se diseñó bajo principios estrictos de memoria acotada:

- **Indexación y Memoria Acotada:**

  El sistema opera con un número fijo de hasta 4 módulos de potencia. El estado se gestiona mediante un diccionario estático preasignado exclusivamente para los identificadores `0x6F0`, `0x6F1`, `0x6F2` y `0x6F3`. No se crean llaves dinámicas en tiempo de ejecución, lo que asegura una complejidad espacial $O(1)$ y descarta fugas de memoria (*memory leaks*) en operaciones prolongadas.
- **Manejo de Escenarios Anómalos en el Bus:**
  1. **Tramas Consecutivas Huérfanas (CF):** Se ignoran inmediatamente si se reciben sin una sesión activa de First Frame (`session is None`).
  2. **Reinicio de Primera Trama (FF):** Si arriba un nuevo FF mientras un mensaje previo está en curso, se sobreescribe y reinicia el búfer de ese módulo, comenzando el reensamblado desde cero sin arrastrar fragmentos obsoletos.
  3. **Reclamos de Longitud Excesiva:** Si un FF declara una longitud mayor a 64 bytes (o menor a 8 bytes), la trama es rechazada y el búfer del módulo se limpia (`None`).
  4. **Secuencia Fuera de Orden:** Cada trama consecutiva valida de forma estricta el contador `expected_seq` (módulo 16). Cualquier discrepancia anula de inmediato la sesión de reensamblado en curso.
  5. **Abandono Repetido:** Dado que los registros de memoria están fijos por módulo y acotados a un tamaño máximo de 64 bytes, los intentos incompletos o abandonados no generan acumulación de memoria en el sistema.
- **Registro Temporal:** La captura del timestamp en nanosegundos (`time.monotonic_ns()`) se ejecuta en el instante exacto en que se valida el último byte de la carga útil, cumpliendo con la precisión requerida para las métricas del evaluador.

### 3. Modos de Ejecución

La herramienta soporta dos modos de operación:

- **Modo Consola / Dashboard (`python3 main.py --iface vcan0`):** Presenta un panel interactivo en la terminal refrescado a 5 Hz mediante secuencias de escape ANSI estándar. Muestra telemetría eléctrica, estado lógico de cada módulo, cadenas de identificación y las últimas 5 fallas registradas, sin recurrir a librerías de interfaz como `ncurses`.
- **Modo Evaluador (`python3 main.py --iface vcan0 --grader`):** Transmite eventos en formato NDJSON (un objeto JSON por línea) directamente a `stdout` con vaciado inmediato de búfer (`flush=True`), emitiendo registros de `telemetry`, `fault`, `diag_complete` y el conteo periódico y final de `stats`.
