import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
import math
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rclpy.time import Time

def get_topics_and_types(reader):
    """
    Obtém os tópicos e seus tipos do arquivo bag.
    """
    topics = {}
    for topic_info in reader.get_all_topics_and_types():
        topics[topic_info.name] = topic_info.type
    return topics

def analyze_bag_file(bag_file_path, location):
    """
    Analisa um único arquivo bag e plota a trajetória da tartaruga vs. a trajetória ideal,
    além de calcular o IAE e o tempo de inicialização do controle.
    """
    print(f"Analisando {bag_file_path}...")
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                                 output_serialization_format='cdr'))
    except RuntimeError as e:
        raise RuntimeError(f"Não foi possível abrir o arquivo '{bag_file_path}'. Verifique o caminho. Erro: {e}")

    topic_types = get_topics_and_types(reader)
    
    pose_x = []
    pose_y = []
    
    # Listas para armazenar dados para o cálculo do IAE
    real_poses = []
    real_timestamps = []

    # Variáveis para o cálculo do tempo de inicialização
    first_pose_ts = None
    first_cmd_vel_ts = None
    
    start_time = None
    
    # Parâmetros da trajetória ideal
    period = 20.0
    start_x, start_y = 5.544445, 5.544445
    time_offset = period / 4.0
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        # Armazena os timestamps da primeira mensagem de cada tópico para o cálculo de inicialização
        if topic == '/turtle1/pose' and first_pose_ts is None:
            first_pose_ts = timestamp
        elif topic == '/turtle1/cmd_vel' and first_cmd_vel_ts is None:
            first_cmd_vel_ts = timestamp

        # Se já tivermos os timestamps de inicialização, podemos parar de procurá-los.
        # No entanto, o loop precisa continuar para coletar todos os dados de pose.

        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)

        if start_time is None:
            start_time = timestamp

        # Converte o timestamp para segundos a partir do início da gravação
        elapsed_time = (timestamp - start_time) * 1e-9

        if topic == '/turtle1/pose':
            pose_x.append(msg.x)
            pose_y.append(msg.y)
            real_poses.append((msg.x, msg.y))
            real_timestamps.append(elapsed_time)
    
    # --- RESULTADOS DOS CÁLCULOS ---
    print("\n--- Resultado do Cálculo ---")

    # Cálculo do Tempo de Inicialização
    if first_pose_ts is not None and first_cmd_vel_ts is not None:
        startup_latency_ns = first_cmd_vel_ts - first_pose_ts
        startup_latency_ms = startup_latency_ns / 1000000.0
        print(f"Tempo de Inicialização do controle: {startup_latency_ms:.2f} ms")
    else:
        print("Não foi possível calcular o tempo de inicialização (falta de mensagens de pose ou cmd_vel).")

    # Cálculo da Integral do Erro Absoluto (IAE)
    iae = 0.0
    if real_poses:
        errors = []
        for i, (real_x, real_y) in enumerate(real_poses):
            current_time = real_timestamps[i]
            
            # Calcula a posição ideal da tartaruga no mesmo instante de tempo
            t_lemniscate = (current_time + time_offset) / (period / (2 * math.pi))
            x_ideal = start_x + 2 * math.cos(t_lemniscate) / (1 + math.sin(t_lemniscate)**2)
            y_ideal = start_y + 2 * math.sin(t_lemniscate) * math.cos(t_lemniscate) / (1 + math.sin(t_lemniscate)**2)
            
            # Calcula a distância euclidiana entre a posição real e ideal
            distance = math.sqrt((real_x - x_ideal)**2 + (real_y - y_ideal)**2)
            errors.append(distance)

        # Aproxima a integral usando a regra do trapézio
        for i in range(len(errors) - 1):
            dt = real_timestamps[i+1] - real_timestamps[i]
            iae += 0.5 * (errors[i] + errors[i+1]) * dt
        
        print(f"Integral do Erro Absoluto (IAE): {iae:.2f}")
    else:
        print("Não foram encontrados dados de posição no arquivo bag para calcular o IAE.")
        
    # --- Plotar a Trajetória ---
    if pose_x and pose_y:
        plt.figure(figsize=(8, 8))
        plt.plot(pose_x, pose_y, label='Trajetória Real', color='blue')
        
        # Gerar a trajetória ideal da lemniscata para o plot
        num_points = 200
        t_ideal = np.linspace(0, period, num_points + 1) + time_offset
        x_ideal_plot = start_x + 2 * np.cos(t_ideal / (period / (2 * np.pi))) / (1 + np.sin(t_ideal / (period / (2 * np.pi)))**2)
        y_ideal_plot = start_y + 2 * np.sin(t_ideal / (period / (2 * np.pi))) * np.cos(t_ideal / (period / (2 * np.pi))) / (1 + np.sin(t_ideal / (period / (2 * np.pi)))**2)
        
        plt.plot(x_ideal_plot, y_ideal_plot, label='Trajetória esperada', linestyle='--', color='red')
        
        # Define o título do gráfico com quebra de linha
        title_map = {
            'local': 'Trajetória da Tartaruga x Trajetória esperada\ncom nó de controle sendo executado na Máquina Local.',
            'sp': 'Trajetória da Tartaruga x Trajetória esperada\ncom nó de controle sendo executado em VM instanciada em São Paulo, Brasil.',
            'eua': 'Trajetória da Tartaruga x Trajetória esperada\ncom nó de controle sendo executado em VM instanciada no Norte da Califórnia, EUA.'
        }
        plt.title(title_map.get(location, "Trajetória da Tartaruga x Trajetória esperada"), fontsize=10)
        plt.xlabel("Posição X")
        plt.ylabel("Posição Y")
        plt.legend()
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')
        
        # Determina o nome do arquivo de imagem a partir do caminho do bag
        image_name = bag_file_path.replace('/', '_').replace('.db3', '')
        plt.savefig(f"trajetoria_{image_name}.png")
        plt.close()
        print(f"\nTrajetória salva em trajetoria_{image_name}.png")
    else:
        print("Não foram encontrados dados de posição no arquivo bag.")

if __name__ == '__main__':
    try:
        rclpy.init(args=None)
        
        # --- Pede o nome da pasta ---
        bag_folder = input("Digite o nome da pasta do arquivo bag (ex: turtlesim_local_01): ")
        bag_file_name = f'{bag_folder}/{bag_folder}_0.db3'
        
        # Pergunta sobre a localização da análise
        location_input = input("Onde a análise foi executada? (local, sp, eua): ").lower()
        if location_input not in ['local', 'sp', 'eua']:
            print("Entrada inválida. Usando 'local' como padrão.")
            location_input = 'local'

        try:
            analyze_bag_file(bag_file_name, location_input)
        except Exception as e:
            print(f"Erro ao processar o arquivo {bag_file_name}: {e}")
            
    finally:
        if rclpy.ok():
            rclpy.shutdown()
