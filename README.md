# Microcontrollers-project (Radar y laser)

Proyecto de **microcontroladores** desarrollado en **C** en **STM32CubeIDE** correspondiente a la parte de microcontroladores del trabajo de SED: **Radar y laser**.

---

El sistema implementa una aplicación completa sobre un microcontrolador (STM32), que incluye:

- Lectura de **sensores de distancia** (VL53L0X).
- Procesamiento de datos (detección de objetivos, filtrado, getsion de los mismo).
- Representación gráfica tipo **radar** en una pantalla **ILI9341**.
- Uso de **Display LCD** para configuraciones realizadas por el usuario (menus).
- Uso de conversores ADC.
- Arquitectura modular y escalable.

El proyecto está orientado a demostrar el uso conjunto de:
- GPIO  
- I2C  
- SPI  
- DMA  
- Timers  
- Programación estructurada en C  

---

### Sensor de distancia (VL53L0X)
- Lectura periódica mediante **I2C**.
- Filtrado de medidas inválidas.
- Conversión a distancia real en mm.
- Asociación distancia–ángulo para el radar.

### Gestión de objetivos
- Almacenamiento de objetivos detectados.
- Comparación por margen de igualdad (ángulo/distancia).
- Eliminación y actualización dinámica.

### Visualización (ILI9341)
- Comunicación por **I2C**.
- Dibujo de píxeles, líneas y rectángulos.
- Representación tipo radar:
- Uso de **DMA** para acelerar proceso.

### Menús (ILI9341)
- Comunicación por **SPI**.
- Gestión de menús con maquina de estados.
- Visualización de opciones a seleccionar por el usuario.

### Seleccion de opciones (potenciomentros y botonera)
- Personalización por el usuario.
  - Selección de menús para gestionar los estados.
  - Acción de apuntar y disparo.
  - Establecimiento de variables (de distancia y rotación).

### Optimización hardware
- DMA para envío de datos a la pantalla.
- Uso eficiente de buffers temporales.
- Evita bloqueos en el `main loop`.

---

## Flujo de funcionamiento
1. Inicialización del hardware (HAL, GPIO, SPI, I2C, DMA).
2. Inicialización de sensores y pantalla.
3. Bucle principal:
   - Lectura del sensor.
   - Procesamiento de la medida.
   - Detección / actualización de objetivos.
   - Dibujado del radar en pantalla.
   - Actuacion del laser (posicionamiento y disparo)
4. Actualización continua en tiempo real.

---

El proyecto permite escoger al usuario:
- Distancia máxima de detección.
- Rotacion maxima del radar.
- Modo manual o automatico para el funcionamiento de la torreta.

