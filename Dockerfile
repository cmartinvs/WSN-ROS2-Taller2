# 1. Usar la imagen oficial de ROS 2 Jazzy con herramientas de escritorio
FROM osrf/ros:jazzy-desktop

# Definir el shell a usar para los comandos RUN, lo que permite encadenar comandos
SHELL ["/bin/bash", "-c"]

# 2. Instalar las dependencias necesarias, incluyendo matplotlib desde apt
# Esto automatiza la instalación de las extensiones de colcon para Python.
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    nano \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# 3. Crear el directorio del workspace
WORKDIR /ros2_ws/src

# 4. Crear el paquete de ROS 2
RUN source /opt/ros/jazzy/setup.bash && \
    ros2 pkg create --build-type ament_python sensor_program --license MIT

# 5. Copiar los archivos de los nodos al interior del contenedor
COPY sensor_node.py /ros2_ws/src/sensor_program/sensor_program/
COPY reader_node.py /ros2_ws/src/sensor_program/sensor_program/
COPY plotter_node.py /ros2_ws/src/sensor_program/sensor_program/

# 6. Copiar el archivo setup.py modificado
COPY setup.py /ros2_ws/src/sensor_program/

# 7. Regresar a la raíz del workspace y compilar el proyecto
WORKDIR /ros2_ws
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build

# 8. Configurar el punto de entrada para usar el script oficial de ROS
ENTRYPOINT ["/ros_entrypoint.sh"]

# El comando por defecto al iniciar el contenedor será una terminal bash
CMD ["bash"]
