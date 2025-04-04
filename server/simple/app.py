from flask import Flask, request, jsonify
import geopandas as gpd
import numpy as np
import folium
import dubins
from pyproj import Transformer

app = Flask(__name__)

def process_points(points):
    transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)
    transformer_to_wgs84 = Transformer.from_crs("EPSG:32636", "EPSG:4326", always_xy=True)
    
    points_utm = []
    points_wgs84 = []
    for point in points:
        lon, lat = point['lng'], point['lat']
        x, y = transformer_to_utm.transform(lon, lat)
        points_utm.append((x, y, float(point['orientation'])))
        points_wgs84.append((lat, lon))
    
    return points_utm, points_wgs84, transformer_to_wgs84

@app.route('/')
def index():
    with open("./index.html", "r", encoding="utf-8") as f:
        return f.read()

@app.route('/calculate_route', methods=['POST'])
def calculate_route():
    data = request.get_json()
    
    # Константы из запроса
    DRONE_SPEED = float(data['droneSpeed'])
    ENERGY_PER_METER = float(data['energyPerMeter'])
    TURN_ENERGY_FACTOR = float(data['turnEnergyFactor'])
    BATTERY_CAPACITY = float(data['batteryCapacity'])
    ACO_ANTS = int(data['acoAnts'])
    ACO_ITERATIONS = int(data['acoIterations'])
    ACO_ALPHA = float(data['acoAlpha'])
    ACO_BETA = float(data['acoBeta'])
    ACO_RHO = float(data['acoRho'])
    ACO_Q = float(data['acoQ'])
    
    turn_radius = 30  # Можно тоже сделать настраиваемым
    
    # Обработка точек
    points_utm, points_wgs84, transformer_to_wgs84 = process_points(data['points'])
    distances = calculate_dubins_distances(points_utm, turn_radius)
    
    # ACO
    best_path, best_length = ant_colony_optimization(len(points_utm), distances, 
                                                    ACO_ANTS, ACO_ITERATIONS, 
                                                    ACO_ALPHA, ACO_BETA, ACO_RHO, ACO_Q)
    
    # Траектория
    trajectory = generate_trajectory(points_utm, best_path, turn_radius, transformer_to_wgs84)
    
    # Энергия и время
    total_energy, segment_times = calculate_energy(points_utm, best_path, turn_radius, 
                                                  ENERGY_PER_METER, TURN_ENERGY_FACTOR, DRONE_SPEED)
    energy_remaining = BATTERY_CAPACITY - total_energy
    energy_percentage = (energy_remaining / BATTERY_CAPACITY) * 100
    total_time = sum(segment_times)  # Общее время как сумма времён сегментов

    # Преобразование NumPy-типов в стандартные Python-типы
    best_path = [int(x) for x in best_path]
    best_length = float(best_length)
    total_energy = float(total_energy)
    energy_percentage = float(energy_percentage)
    total_time = float(total_time)
    segment_times = [float(t) for t in segment_times]  # Преобразуем времена сегментов

    return jsonify({
        'best_path': best_path,
        'best_length': best_length,
        'trajectory': trajectory,
        'total_energy': total_energy,
        'energy_percentage': energy_percentage,
        'total_time': total_time,
        'segment_times': segment_times  # Добавляем времена сегментов
    })

def calculate_dubins_distances(points_utm, turn_radius):
    n = len(points_utm)
    distances = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if i != j:
                path = dubins.shortest_path(points_utm[i], points_utm[j], turn_radius)
                distances[i][j] = path.path_length()
    return distances

def ant_colony_optimization(n, distances, ants, iterations, alpha, beta, rho, q):
    pheromones = np.ones((n, n)) * 0.1
    eta = 1 / (distances + 1e-10)
    best_path = None
    best_length = float('inf')

    for _ in range(iterations):
        paths = []
        lengths = []
        for _ in range(ants):
            visited = [0]
            while len(visited) < n:
                current = visited[-1]
                unvisited = [i for i in range(n) if i not in visited]
                probs = [(pheromones[current][j]**alpha) * (eta[current][j]**beta) for j in unvisited]
                probs = probs / np.sum(probs)
                next_point = np.random.choice(unvisited, p=probs)
                visited.append(next_point)
            visited.append(0)
            length = sum(distances[visited[i]][visited[i+1]] for i in range(n))
            paths.append(visited)
            lengths.append(length)
            if length < best_length:
                best_length = length
                best_path = visited.copy()
        
        pheromones *= (1 - rho)
        for path, length in zip(paths, lengths):
            for i in range(n):
                pheromones[path[i]][path[i+1]] += q / length
    
    return best_path, best_length

def generate_trajectory(points_utm, best_path, turn_radius, transformer_to_wgs84, sample_step=0.1):
    n = len(points_utm)
    full_trajectory = []
    for i in range(n):
        path = dubins.shortest_path(points_utm[best_path[i]], points_utm[best_path[i+1]], turn_radius)
        path_points = path.sample_many(sample_step)[0]
        path_wgs84 = [[transformer_to_wgs84.transform(x, y)[1], transformer_to_wgs84.transform(x, y)[0]] 
                     for x, y, _ in path_points]
        full_trajectory.append(path_wgs84)
    return full_trajectory

def calculate_energy(points_utm, best_path, turn_radius, energy_per_meter, turn_energy_factor, drone_speed):
    total_energy = 0
    segment_times = []  # Список времён для каждого сегмента
    n = len(points_utm)
    for i in range(n):
        path = dubins.shortest_path(points_utm[best_path[i]], points_utm[best_path[i+1]], turn_radius)
        segment_length = path.path_length()
        segment_energy = segment_length * energy_per_meter * turn_energy_factor
        segment_time = segment_length / drone_speed  # Время для сегмента
        total_energy += segment_energy
        segment_times.append(segment_time)
    return total_energy, segment_times

if __name__ == '__main__':
    app.run(host='localhost', port=6501)