# WSN-ROS2-Taller2
“Taller 2 – Introducción a ROS 2: implementación de red de sensores, nodos y entorno Docker.”

Este repositorio contiene el desarrollo del **Taller 2 de Redes de Sensores**, cuyo objetivo fue implementar una red mínima de nodos en **ROS 2 Jazzy** ejecutándose dentro de **Docker**, aplicando el modelo publicación–suscripción sobre **DDS**.

## 📘 Descripción general

Se creó un **workspace ROS 2** y un paquete Python `sensor_program` con los siguientes nodos:

- **`sensor_node`**: simula un sensor de temperatura y publica valores aleatorios (20–30 °C) en el tópico `/sensor_data` cada segundo.  
- **`reader_node`**: se suscribe a `/sensor_data` y registra los mensajes recibidos.  
- **`reader_node2`**: segundo suscriptor a `/sensor_data` para validar múltiples lectores en paralelo.  
- **`plotter_node`**: se suscribe a `/sensor_data`, acumula los valores y **genera cada 5 s** un gráfico `sensor_plot.png` en un **volumen compartido** con el host.

El entorno se automatizó con un **Dockerfile** basado en `osrf/ros:jazzy-desktop` e incluye las dependencias (por ejemplo, `matplotlib`) y la estructura estándar de ROS 2.

## ⚙️ Estructura del proyecto

```
ros2_ws/
├── src/
│   └── sensor_program/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/sensor_program
│       ├── sensor_program/
│       │   ├── __init__.py
│       │   ├── sensor_node.py
│       │   ├── reader_node.py
│       │   ├── reader_node2.py
│       │   └── plotter_node.py
└── Dockerfile
```

> Nota: el gráfico `sensor_plot.png` se guarda en el volumen compartido del host, no dentro de este árbol (ver “Ejecución con volumen”).

## 🐳 Opciones de ejecución

### Opción A) Usar la imagen publicada en Docker Hub (recomendado)

Imagen pública: **`cmartinvs/ros2taller2`**

```bash
# 1) Descargar imagen
docker pull cmartinvs/ros2taller2

# 2) Ejecutar con volumen compartido (Windows):
docker run -it --name mi_ros_compartido ^
  -v "C:\Users\marti\ros2_ws_shared:/ros2_ws_shared" ^
  cmartinvs/ros2taller2
```

> Cambia `C:\Users\marti\ros2_ws_shared` por tu carpeta local.  
> Dentro del contenedor, el volumen estará disponible en `/ros2_ws_shared`.

### Opción B) Construir localmente desde este repositorio

```bash
# En el directorio donde está el Dockerfile
docker build -t my-ros2-app .

# Ejecutar con volumen compartido (Windows)
docker run -it --name mi_ros_compartido ^
  -v "C:\Users\marti\ros2_ws_shared:/ros2_ws_shared" ^
  my-ros2-app
```

## 🚀 Compilación y ejecución de nodos

Dentro del contenedor:

```bash
# Compilar (si hiciste cambios)
cd /ros2_ws
colcon build
source install/setup.bash

# Terminal 1 (publicador)
ros2 run sensor_program sensor_node

# Terminal 2 (primer lector)
ros2 run sensor_program reader_node

# Terminal 3 (segundo lector)
ros2 run sensor_program reader_node2

# Terminal 4 (graficador)
ros2 run sensor_program plotter_node
```

- El `plotter_node` genera/actualiza el gráfico cada 5 s en:
  ```
  /ros2_ws_shared/data/sensor_plot.png
  ```
  Por lo tanto, desde tu **Windows** quedará disponible en:
  ```
  C:\Users\marti\ros2_ws_shared\data\sensor_plot.png
  ```

## 🧩 Volumen compartido (host ↔ contenedor)

El volumen se monta así:

```
Host (Windows)                          Contenedor
C:\Users\marti\ros2_ws_shared   <-->    /ros2_ws_shared
```

Uso típico dentro de esa carpeta:

```
/ros2_ws_shared/
├── src/…   (opcional si quieres editar fuentes desde el host)
└── data/
    └── sensor_plot.png   (salida del plotter_node)
```

Ventajas: edición directa desde tu IDE del host, persistencia de gráficos y resultados, desarrollo iterativo sin reconstruir la imagen.

## 🧰 Tecnologías

- ROS 2 **Jazzy Jalisco**
- Python 3
- Docker
- Matplotlib (para `plotter_node`)

## ✅ Verificación (sugerencias)

- Con `reader_node` y `reader_node2` en ejecución, ambos deben imprimir los **mismos** mensajes recibidos de `sensor_node`.  
- Comprueba que `sensor_plot.png` se actualiza cada 5 s en `C:\Users\marti\ros2_ws_shared\data\`.

## 📦 Imagen en Docker Hub

- Repositorio: **`cmartinvs/ros2taller2`**  
- Pull rápido:  
  ```bash
  docker pull cmartinvs/ros2taller2
  ```

## 📚 Créditos

**Carlos Martín Vinces Segovia**  
Facultad de Ingeniería - Universidad de Cuenca
Materia: Redes de Sensores

---

### Tips opcionales

- Si cambias código dentro de `/ros2_ws/src/...`, vuelve a ejecutar:
  ```bash
  cd /ros2_ws && colcon build && source install/setup.bash
  ```
- Si `plotter_node` no arranca, verifica que `matplotlib` esté instalado en la imagen (el Dockerfile ya lo incluye).
- Para limpiar compilados: `rm -rf build install log` y recompilar.
