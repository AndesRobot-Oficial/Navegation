# Puertos fijos de Martha (udev)

El ESP32 de la base y el RPLIDAR A2M8 aparecen los dos como `/dev/ttyUSBx`, y el número cambia
según el orden en que se conectan. Además los dos usan el mismo chip USB-serie (CP2102), así que
no se distinguen por fabricante: hay que usar el número de serie de cada adaptador. Con la regla
de `99-martha.rules` quedan siempre como **`/dev/martha_esp32`** y **`/dev/martha_lidar`**.

La regla es la misma en los repos de Estructura y de Navegación (el ESP32 es de Estructura y el
LiDAR de Navegación, pero en el laptop se instala un solo archivo). Si la cambias, cópiala al otro.

1. En el laptop de Martha, con los dos conectados: `./robot.sh puertos`
   Muestra cada `/dev/ttyUSBx` con su `idVendor`, `idProduct` y `serial`. Para saber cuál es
   cuál, desconecta uno y vuelve a correrlo.
2. Copia los números de serie en `99-martha.rules` (reemplaza `PONER_SERIAL_ESP32` y
   `PONER_SERIAL_LIDAR`). Si los dos salen **iguales**, usa el "plan B" que explica el archivo.
3. Instala la regla:
   ```bash
   sudo cp docker/udev/99-martha.rules /etc/udev/rules.d/
   ```
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
   ```bash
   ls -l /dev/martha_esp32 /dev/martha_lidar
   ```

Mientras tanto se puede usar el puerto directo: `ESP32=/dev/ttyUSB0 ./robot.sh serial`.
