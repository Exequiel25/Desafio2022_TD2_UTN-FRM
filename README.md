**Trabajo Práctico Nº5 (Desafío 2022):**

**Manejo de Comunicaciones seriales y conversores ADC con Microcontroladores ARM**

**N° de integrantes:** 1

**Nombre:** Magni Genre, Exequiel Juan

**Información sobre la actividad realiazada**

`	`A continuación se presenta la información requerida sobre el desafío.

**Contenido**

`	`Dentro de la carpeta del desafío se encuentran 3 códigos con sus respectivas simulaciones. El primero en realizarse fue “Code\_USART\_only”, el cuál únicamente muestra los datos por comunicación serial, luego se implementó la presentación por Display de 7 segmentos “Code\_7seg\_only”, el cuál presentó problemas con respecto al espacio que se requería para la memoria de programa (solo alcanzó para enviar el valor del ADC por USART pero no el valor de tensión). Como solución a esto último, se implementó un añadido de hardware (Circuito Integrado 4511) para disminuir el espacio en memoria requerido por el código y la cantidad de entradas y salidas utilizadas del STM32F103C6. Este código se encuentra en la carpeta “Code\_USART\_BCD\_7seg”.

`	`En la carpeta “Proteus” se encuentran 3 archivos “.pdsprj” correspondientes a las simulaciones de cada solución propuesta al desafío.

`	`Dentro del directorio se halla además, dos archivos pdf, lo cuales fueron utilizados para realizar la configuración de los registros. Dichos pdf son stm32f103c6.pdf respectivo a la información del procesador requerido, y stm32f10xxx.pdf donde se encuentra la información técnica para la configuración de registros de la familia STM32F10.

`	`El código contiene comentarios (en inglés) donde se explica cada bloque de código.

**Compilación en eUCCvm**

`	`Se creó un directorio con el comando mkdir ~/desafio2022 para realizar las compilaciones de los códigos. El procedimiento es similar al utilizado en la “Actividad N°49 – Año 2022”. De hecho, el código base utilizado, a partir del cuál se realizaron modificaciones es el indicado en el punto “1.2.3”, el cuál está ubicado en la ruta: /home/cdlt/arm/STM32F103X6/STM32\_UART\_Examples-master/printf

A continuación se muestran los comandos utilizados dentro de la máquina virtual:

\1) Code\_USART\_only


\2)  Code\_7seg\_only

\3)  Code\_USART\_BCD\_7seg

**Descripción breve del programa original:**

`	`El archivo main.c dentro de la carpeta src que este programa recibe un dato por un pin RX Serie de USART2 y retransmite el carácter leído pero con el agregado de “RX:” por el pin TX de la USART2

**Descripción breve del programa ya modificado:**

`	`El archivo main.c dentro de la carpeta src obtiene un valor analógico (señal de tensión) por el pin A0, realiza la conversión al valor equivalente de tensión y envía por el puerto serie USART2 el valor obtenido del conversor (ADC) y el valor de tensión (V) en períodos de 2 segundos.

`	`Los elementos utilizados para la simulación son los indicados en la consigna del desafio. El archivo “main.hex” generado se encuentra en el directorio “code”. Se seteó la frecuencia del chip en Proteus con 8 MHz. La versión de Proteus utilizada es 8.11

\1) Code\_USART\_only

\2)  Code\_7seg\_only


\3)  Code\_USART\_BCD\_7seg

