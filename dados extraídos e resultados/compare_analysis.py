import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageFilter, StorageOptions, ConverterOptions
import matplotlib.pyplot as plt
import numpy as np
import os

# Classes de mensagens necessárias
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32

# Lista dos rosbags que serão analisados
BAG_PATHS = ["vwalker_local_01", "vwalker_sp_01", "vwalker_eua_01"]
VELOCITY_THRESHOLD = 0.05 # Limite de velocidade para detectar movimento (m/s)
FORCE_THRESHOLD = 5.0 # Limite de força para detectar aplicação (N)

def find_first_event_timestamp(timestamps, values, threshold, direction='up'):
    """
    Encontra o timestamp do primeiro ponto onde o valor cruza o limiar.
    'direction' pode ser 'up' (acima do limiar) ou 'down' (abaixo do limiar).
    """
    if direction == 'up':
        for i in range(len(values)):
            if values[i] > threshold:
                return timestamps[i]
    else: # 'down'
        for i in range(len(values)):
            if values[i] < threshold:
                return timestamps[i]
    return None

def find_movement_timestamps(bag_file_path):
    """
    Encontra o primeiro instante de movimento e o último timestamp do rosbag.
    Retorna (start_time, end_time) ou (None, None) se não encontrar.
    """
    if not os.path.isdir(bag_file_path):
        print(f"Erro: O diretório do rosbag '{bag_file_path}' não foi encontrado.")
        return None, None

    storage_options = StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    storage_filter = StorageFilter(topics=["/odom"])
    reader.set_filter(storage_filter)
    
    start_time = None
    end_time = None
    
    if not reader.has_next():
        return None, None
        
    last_t = None
    while reader.has_next():
        _, data, t = reader.read_next()
        msg = deserialize_message(data, Odometry)
        linear_vel = msg.twist.twist.linear.x
        
        if start_time is None and linear_vel > VELOCITY_THRESHOLD:
            start_time = t
        
        last_t = t
            
    if start_time is not None and last_t is not None:
        end_time = last_t
        return start_time, end_time
    else:
        return None, None

def extract_data_from_bag(bag_file_path, topic_info, start_time, end_time):
    """
    Extrai dados de um tópico dentro de um intervalo de tempo.
    """
    storage_options = StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    storage_filter = StorageFilter(topics=[topic_info['topic_name']])
    reader.set_filter(storage_filter)

    timestamps = []
    data_values = []
    
    while reader.has_next():
        _, data, t = reader.read_next()
        
        # Filtra os dados pelo intervalo de tempo
        if start_time and end_time and (t < start_time or t > end_time):
            continue

        msg = deserialize_message(data, topic_info['msg_type'])
        
        value = 0.0
        if topic_info['topic_name'] == "/odom":
            value = msg.twist.twist.linear.x
        elif topic_info['topic_name'] in ["/force/raw"]:
            value = msg.wrench.force.z
            
        timestamps.append(t)
        data_values.append(value)

    return np.array(timestamps), np.array(data_values)

def main():
    all_metrics = {bag_name: {} for bag_name in BAG_PATHS}
    analysis_intervals = {}

    topic_configs = [
        {'topic_name': '/odom', 'msg_type': Odometry, 'label': 'Velocidade Linear (m/s)'},
        {'topic_name': '/force/raw', 'msg_type': WrenchStamped, 'label': 'Força Aplicada (N)'}
    ]

    # --- Passo 1: Encontrar os tempos de início e fim da análise ---
    for bag_path in BAG_PATHS:
        start_t, end_t = find_movement_timestamps(bag_path)
        if start_t and end_t:
            analysis_intervals[bag_path] = (start_t, end_t)
            print(f"Análise de '{bag_path}' vai começar em {start_t} e terminar em {end_t}.")
        else:
            print(f"Não foi possível detectar o início do movimento em '{bag_path}'.")
            analysis_intervals[bag_path] = (None, None)

    # --- Passo 2: Extração de dados e cálculo de métricas ---
    for bag_path in BAG_PATHS:
        print(f"\n--- Analisando dados do circuito de {bag_path} ---")
        start_t, end_t = analysis_intervals[bag_path]
        
        for topic_config in topic_configs:
            topic_name = topic_config['topic_name']
            
            timestamps, data_values = extract_data_from_bag(bag_path, topic_config, start_t, end_t)

            if timestamps is not None and len(data_values) > 0:
                # --- ALTERAÇÃO: Cálculo da média sobre todos os dados no intervalo,
                mean_value = np.mean(data_values)
                std_dev = np.std(data_values)

                duration = (timestamps[-1] - timestamps[0]) / 1e9

                all_metrics[bag_path][topic_name] = {
                    'timestamps': timestamps,
                    'data_values': data_values,
                    'mean': mean_value,
                    'std_dev': std_dev,
                    'duration': duration
                }
                print(f"  Tópico {topic_name}: Média = {mean_value:.4f}, Desvio Padrão = {std_dev:.4f}")


    # --- Apresentação dos Resultados em Tabela ---
    print("\n" + "="*80)
    print("                      RELATÓRIO COMPARATIVO DE MÉTRICAS                      ")
    print("="*80)
    
    print(f"{'Métrica':<30} | {'vwalker_local_01':<15} | {'vwalker_sp_01':<15} | {'vwalker_eua_01':<15}")
    print("-" * 80)
    
    # Formatação da duração do movimento
    local_duration = all_metrics[BAG_PATHS[0]].get("/odom", {}).get("duration", "N/A")
    sp_duration = all_metrics[BAG_PATHS[1]].get("/odom", {}).get("duration", "N/A")
    usa_duration = all_metrics[BAG_PATHS[2]].get("/odom", {}).get("duration", "N/A")
    
    local_duration_str = f'{local_duration:.2f}' if isinstance(local_duration, (int, float)) else local_duration
    sp_duration_str = f'{sp_duration:.2f}' if isinstance(sp_duration, (int, float)) else sp_duration
    usa_duration_str = f'{usa_duration:.2f}' if isinstance(usa_duration, (int, float)) else usa_duration
    
    print(f"{'Duração do Movimento (s)':<30} | {local_duration_str:<15} | {sp_duration_str:<15} | {usa_duration_str:<15}")
    print("-" * 80)

    # Formatação das médias das métricas
    for topic_config in topic_configs:
        metric_label = topic_config['label']
        mean_local = all_metrics[BAG_PATHS[0]].get(topic_config['topic_name'], {}).get('mean', 'N/A')
        mean_sp = all_metrics[BAG_PATHS[1]].get(topic_config['topic_name'], {}).get('mean', 'N/A')
        mean_usa = all_metrics[BAG_PATHS[2]].get(topic_config['topic_name'], {}).get('mean', 'N/A')
        
        mean_local_str = f'{mean_local:.4f}' if isinstance(mean_local, (int, float)) else mean_local
        mean_sp_str = f'{mean_sp:.4f}' if isinstance(mean_sp, (int, float)) else mean_sp
        mean_usa_str = f'{mean_usa:.4f}' if isinstance(mean_usa, (int, float)) else mean_usa
        
        print(f"{metric_label:<30} | {mean_local_str:<15} | {mean_sp_str:<15} | {mean_usa_str:<15}")

    # --- Gerar Gráficos Comparativos em janelas separadas ---
    print("\nGerando gráficos comparativos (um para cada métrica)...")
    
    for topic_config in topic_configs:
        fig = plt.figure(figsize=(12, 6))
        fig.suptitle(f"Comparativo de: {topic_config['label']}", fontsize=16)
        ax = fig.add_subplot(1, 1, 1)
        
        for bag_path in BAG_PATHS:
            metrics = all_metrics.get(bag_path, {}).get(topic_config['topic_name'])
            if metrics:
                relative_time = (metrics['timestamps'] - metrics['timestamps'][0]) / 1e9
                ax.plot(relative_time, metrics['data_values'], label=bag_path)
        
        ax.set_xlabel("Tempo (s)")
        ax.set_ylabel(topic_config['label'].split('(')[0].strip())
        ax.grid(True)
        ax.legend()
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    plt.show()

if __name__ == '__main__':
    main()
