import geopandas as gpd
import numpy as np
import folium
import dubins
from pyproj import Transformer

# Константы дрона
DRONE_SPEED = 20  # м/с
ENERGY_PER_METER = 5  # Дж/м
TURN_ENERGY_FACTOR = 1.2  # Коэффициент для поворотов
BATTERY_CAPACITY = 213480  # Дж (эквивалент 59.3 Вт·ч, как у DJI Mavic 2)

# Константы ACO
ACO_ANTS = 20  # Количество муравьёв
ACO_ITERATIONS = 100  # Количество итераций
ACO_ALPHA = 1.0  # Влияние феромонов
ACO_BETA = 2.0  # Влияние эвристики (обратного расстояния)
ACO_RHO = 0.1  # Скорость испарения феромонов
ACO_Q = 100  # Количество откладываемого феромона

def load_and_transform_points(file_path):
    """Загружает GeoJSON и преобразует координаты из WGS84 в UTM и обратно."""
    gdf = gpd.read_file(file_path)
    transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)
    transformer_to_wgs84 = Transformer.from_crs("EPSG:32636", "EPSG:4326", always_xy=True)
    
    points_utm = []
    points_wgs84 = []
    node_ids = []
    for _, row in gdf.iterrows():
        lon, lat = row.geometry.x, row.geometry.y
        x, y = transformer_to_utm.transform(lon, lat)
        points_utm.append((x, y, row['Orientation']))
        points_wgs84.append((lat, lon))
        node_ids.append(row['NodeID'])
    
    return points_utm, points_wgs84, node_ids, transformer_to_wgs84

def calculate_dubins_distances(points_utm, turn_radius):
    """Вычисляет матрицу расстояний по траекториям Дубинса."""
    n = len(points_utm)
    distances = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if i != j:
                q0 = points_utm[i]
                q1 = points_utm[j]
                path = dubins.shortest_path(q0, q1, turn_radius)
                distances[i][j] = path.path_length()
    return distances

def ant_colony_optimization(n, distances):
    """Решает задачу оптимизации маршрута с помощью ACO."""
    pheromones = np.ones((n, n)) * 0.1
    eta = 1 / (distances + 1e-10)
    best_path = None
    best_length = float('inf')

    for _ in range(ACO_ITERATIONS):
        paths = []
        lengths = []
        for _ in range(ACO_ANTS):
            visited = [0]
            while len(visited) < n:
                current = visited[-1]
                unvisited = [i for i in range(n) if i not in visited]
                probs = [(pheromones[current][j]**ACO_ALPHA) * (eta[current][j]**ACO_BETA) for j in unvisited]
                probs = probs / np.sum(probs)
                next_point = np.random.choice(unvisited, p=probs)
                visited.append(next_point)
            visited.append(0)  # Возвращение в начало
            length = sum(distances[visited[i]][visited[i+1]] for i in range(n))
            paths.append(visited)
            lengths.append(length)
            if length < best_length:
                best_length = length
                best_path = visited.copy()
        
        pheromones *= (1 - ACO_RHO)
        for path, length in zip(paths, lengths):
            for i in range(n):
                pheromones[path[i]][path[i+1]] += ACO_Q / length
    
    return best_path, best_length

def generate_trajectory(points_utm, best_path, turn_radius, transformer_to_wgs84, sample_step=0.1):
    """Генерирует траекторию Дубинса для лучшего пути."""
    n = len(points_utm)
    full_trajectory = []
    for i in range(n):
        q0 = points_utm[best_path[i]]
        q1 = points_utm[best_path[i+1]]
        path = dubins.shortest_path(q0, q1, turn_radius)
        path_points = path.sample_many(sample_step)[0]
        path_wgs84 = [[transformer_to_wgs84.transform(x, y)[1], transformer_to_wgs84.transform(x, y)[0]] 
                      for x, y, _ in path_points]
        full_trajectory.append(path_wgs84)
    return full_trajectory

def calculate_energy(points_utm, best_path, turn_radius, drone_speed):
    """Вычисляет расход энергии для маршрута."""
    total_energy = 0
    segment_times = []  # Список времён для каждого сегмента
    n = len(points_utm)
    for i in range(n):
        q0 = points_utm[best_path[i]]
        q1 = points_utm[best_path[i+1]]
        path = dubins.shortest_path(q0, q1, turn_radius)
        segment_length = path.path_length()
        segment_energy = segment_length * ENERGY_PER_METER * TURN_ENERGY_FACTOR
        segment_time = segment_length / drone_speed  # Время для сегмента
        total_energy += segment_energy
        segment_times.append(segment_time)
    return total_energy, segment_times

def create_map(points_wgs84, node_ids, trajectory, output_file="quadcopter_route.html"):
    """Создаёт и сохраняет карту с точками и траекторией."""
    center_lat = sum(lat for lat, _ in points_wgs84) / len(points_wgs84)
    center_lon = sum(lon for _, lon in points_wgs84) / len(points_wgs84)
    m = folium.Map(location=[center_lat, center_lon], zoom_start=15)
    
    for (lat, lon), node_id in zip(points_wgs84, node_ids):
        folium.Marker(location=[lat, lon], popup=node_id, 
                      icon=folium.Icon(color="red", icon="circle")).add_to(m)
    
    for path_segment in trajectory:
        folium.PolyLine(locations=path_segment, color="blue", weight=2).add_to(m)
    
    m.save(output_file)
    return output_file

def main():
    """Основная функция для выполнения программы."""
    # Параметры
    turn_radius = 30  # Радиус поворота

    # Загрузка и преобразование данных
    points_utm, points_wgs84, node_ids, transformer_to_wgs84 = load_and_transform_points("forest_points.geojson")
    n = len(points_utm)

    # Расчёт расстояний
    distances = calculate_dubins_distances(points_utm, turn_radius)

    # Оптимизация маршрута
    best_path, best_length = ant_colony_optimization(n, distances)

    # Генерация траектории
    trajectory = generate_trajectory(points_utm, best_path, turn_radius, transformer_to_wgs84)

    # Расчёт энергии и времени
    total_energy, segment_times = calculate_energy(points_utm, best_path, turn_radius, DRONE_SPEED)
    energy_remaining = BATTERY_CAPACITY - total_energy
    energy_percentage = (energy_remaining / BATTERY_CAPACITY) * 100

    # Общее время как сумма времён сегментов
    total_time = sum(segment_times)

    # Создание карты
    map_file = create_map(points_wgs84, node_ids, trajectory)

    # Вывод результатов
    print("Лучший порядок точек:", [node_ids[i] for i in best_path])
    print("Общая длина маршрута:", best_length)
    print("Общий расход энергии:", total_energy, "Дж")
    print("Оставшаяся энергия:", energy_remaining, "Дж")
    print("Общее время полёта:", total_time, "с (", round(total_time / 60,2), "мин)")
    print("\nВремя для каждого сегмента:")
    for i, (time, start_idx, end_idx) in enumerate(zip(segment_times, best_path[:-1], best_path[1:])):
        print(f"Сегмент {node_ids[start_idx]} → {node_ids[end_idx]}: {time:.2f} с")
    print(f"Остаток заряда аккумулятора: {energy_percentage:.2f}%")
    print("Энергии достаточно" if energy_remaining >= 0 else "Внимание: энергии недостаточно!")
    print(f"Карта сохранена как '{map_file}'. Открой её в браузере!")

if __name__ == "__main__":
    main()