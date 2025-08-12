import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageFilter, StorageOptions, ConverterOptions
import matplotlib.pyplot as plt
import numpy as np
import os

# A classe para desserializar as mensagens
from nav_msgs.msg import Odometry

def analyze_rosbag_odom(bag_file_path):
    """
    Função principal para analisar os dados de odometria de um rosbag.
    """
    if not os.path.isdir(bag_file_path):
        print(f"Erro: O diretório do rosbag '{bag_file_path}' não foi encontrado.")
        return

    print(f"Iniciando a análise do rosbag: {bag_file_path}")

    storage_options = StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_name = "/odom"
    storage_filter = StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)

    timestamps = []
    x_positions = []
    y_positions = []

    if not reader.has_next():
        print(f"Nenhuma mensagem encontrada no tópico {topic_name}. Verifique se o tópico foi gravado.")
        return

    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, Odometry)

        timestamps.append(t)
        x_positions.append(msg.pose.pose.position.x)
        y_positions.append(msg.pose.pose.position.y)

    print(f"Análise concluída. Foram extraídas {len(x_positions)} mensagens do tópico {topic_name}.")

    # Converter as listas para arrays NumPy
    timestamps = np.array(timestamps)
    x_positions = np.array(x_positions)
    y_positions = np.array(y_positions)

    # --- Lógica para identificar a origem do arquivo e definir o título do gráfico ---
    title_suffix = ""
    is_sp_bag = "vwalker_sp" in bag_file_path
    is_local_bag = "vwalker_local" in bag_file_path
    is_eua_bag = "vwalker_eua" in bag_file_path
    
    if is_local_bag:
        title_suffix = "Configuração Local"
    elif is_sp_bag:
        title_suffix = "Instância em São Paulo, Brasil"
    elif is_eua_bag:
        title_suffix = "Instância nos EUA"
    else:
        title_suffix = "Origem Desconhecida"

    if is_sp_bag:
        print("Detectado 'vwalker_sp'. Invertendo e espelhando os eixos X e Y para a plotagem.")
        x_positions_temp = x_positions.copy()
        x_positions = y_positions.copy()
        # A linha abaixo foi modificada para espelhar a trajetória
        y_positions = x_positions_temp.copy() * -1

    # --- Análise e Métricas ---
    
    duration_sec = (timestamps[-1] - timestamps[0]) / 1e9
    frequency = len(x_positions) / duration_sec
    print(f"\n--- Métricas Gerais ---")
    print(f"Duração do rosbag (odom): {duration_sec:.2f} segundos")
    print(f"Frequência média de publicação: {frequency:.2f} Hz")

    path_length = 0
    for i in range(1, len(x_positions)):
        dx = x_positions[i] - x_positions[i-1]
        dy = y_positions[i] - y_positions[i-1]
        path_length += np.sqrt(dx**2 + dy**2)
    print(f"Distância total percorrida (aproximada): {path_length:.2f} metros")

    # --- Visualização dos Dados (Apenas Posição) ---
    print("\n--- Gerando Gráfico da Trajetória ---")
    
    plt.figure(figsize=(8, 8))
    plt.plot(x_positions, y_positions, label='Trajetória Odometria')
    
    # Atualiza os labels dos eixos se a inversão foi feita
    xlabel = "Posição X (m)" if not is_sp_bag else "Posição Y (m)"
    ylabel = "Posição Y (m)" if not is_sp_bag else "Posição X (m)"
    
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    # Define o título do gráfico com base na origem do arquivo
    plt.title(f"Trajetória da Odometria no Plano XY - {title_suffix}")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    
    plt.show()

if __name__ == '__main__':
    while True:
        bag_name = input("Digite o nome do rosbag que deseja analisar (ex: vwalker_local_01): ")
        if not bag_name:
            print("Nenhum nome inserido. Saindo...")
            break
            
        analyze_rosbag_odom(bag_name)
