# plotter_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import os
import re # Usaremos expresiones regulares para extraer el número del mensaje

# --- NOTA IMPORTANTE ---
# Este nodo requiere la librería 'matplotlib'.
# Asegúrate de instalarla en tu Dockerfile añadiendo:
# RUN pip install matplotlib
# O ejecutando `pip install matplotlib` dentro de tu contenedor.

class PlotterNode(Node):
    def __init__(self):
        # 1. Inicializar el nodo con el nombre 'plotter_node'
        super().__init__('plotter_node')

        # 2. Crear un suscriptor al tópico 'sensor_data'
        # Llama a 'listener_callback' cada vez que recibe un mensaje.
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.get_logger().info('Nodo plotter iniciado y suscrito a sensor_data.')

        # 3. Lista para almacenar los datos de temperatura
        self.temperature_data = []
        self.time_steps = []
        self.current_time_step = 0

        # 4. Definir la ruta donde se guardará la imagen
        # Usamos os.path.join para construir la ruta de forma segura.
        self.plot_save_path = '/ros2_ws_shared/data'
        self.plot_filename = os.path.join(self.plot_save_path, 'sensor_plot.png')

        # 5. Asegurarse de que el directorio de datos exista
        if not os.path.exists(self.plot_save_path):
            os.makedirs(self.plot_save_path)
            self.get_logger().info(f"Directorio '{self.plot_save_path}' creado.")

        # 6. Crear un temporizador que ejecute la función de graficado cada 5 segundos
        self.plot_timer = self.create_timer(5.0, self.generate_plot)

    def listener_callback(self, msg):
        """
        Esta función se ejecuta cada vez que se recibe un mensaje del tópico.
        Extrae el dato numérico y lo guarda en la lista.
        """
        self.get_logger().info(f'Plotter recibió: "{msg.data}"')
        
        # Usar una expresión regular para encontrar el número en el string
        # Esto hace el código más robusto a cambios en el texto.
        numeric_value = re.search(r'\d+', msg.data)
        
        if numeric_value:
            try:
                # Convertir el valor encontrado a un número entero y añadirlo a la lista
                temp = int(numeric_value.group(0))
                self.temperature_data.append(temp)
                self.time_steps.append(self.current_time_step)
                self.current_time_step += 1
            except (ValueError, IndexError):
                self.get_logger().warn(f"No se pudo convertir el dato a número: {msg.data}")

    def generate_plot(self):
        """
        Esta función es llamada por el temporizador cada 5 segundos.
        Genera y guarda la gráfica si hay datos disponibles.
        """
        # Solo generar la gráfica si tenemos al menos un dato
        if not self.temperature_data:
            self.get_logger().info('No hay datos para graficar todavía...')
            return

        # Configuración de la gráfica
        plt.figure(figsize=(10, 6)) # Tamaño de la figura
        plt.plot(self.time_steps, self.temperature_data, marker='o', linestyle='-', color='b')
        
        # Añadir títulos y etiquetas para mayor claridad
        plt.title('Evolución de la Temperatura del Sensor')
        plt.xlabel('Muestras (Tiempo)')
        plt.ylabel('Temperatura (°C)')
        plt.grid(True) # Añadir una cuadrícula
        plt.ylim(min(self.temperature_data) - 2, max(self.temperature_data) + 2) # Ajustar límites del eje Y

        # Guardar la gráfica en el archivo especificado
        try:
            plt.savefig(self.plot_filename)
            self.get_logger().info(f'¡Gráfica guardada exitosamente en {self.plot_filename}!')
        except Exception as e:
            self.get_logger().error(f'Error al guardar la gráfica: {e}')
        finally:
            # Es importante cerrar la figura para liberar memoria
            plt.close()

def main(args=None):
    rclpy.init(args=args)
    plotter_node = PlotterNode()
    rclpy.spin(plotter_node)
    
    # Destruir el nodo explícitamente
    plotter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
