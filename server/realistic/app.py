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
    
    # Параметры из запроса
    DRONE_SPEED = float(data['droneSpeed'])
    ACO_ANTS = int(data['acoAnts'])
    ACO_ITERATIONS = int(data['acoIterations'])
    ACO_ALPHA = float(data['acoAlpha'])
    ACO_BETA = float(data['acoBeta'])
    ACO_RHO = float(data['acoRho'])
    ACO_Q = float(data['acoQ'])
    turn_radius = 30  # Фиксированный радиус поворота, можно сделать настраиваемым
    
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
    total_energy, segment_times = calculate_energy(points_utm, best_path, turn_radius, DRONE_SPEED, data)
    energy_consumed, energy_remaining, flight_time_possible = simulate_battery_discharge(total_energy, data, segment_times)
    energy_percentage = (energy_remaining / (float(data['battery_nominal_capacity']) * 3600 / 1000 * float(data['battery_initial_voltage']))) * 100

    # Преобразование типов
    best_path = [int(x) for x in best_path]
    best_length = float(best_length)
    total_energy = float(energy_consumed)
    energy_percentage = float(energy_percentage)
    total_time = float(flight_time_possible)
    segment_times = [float(t) for t in segment_times]

    return jsonify({
        'best_path': best_path,
        'best_length': best_length,
        'trajectory': trajectory,
        'total_energy': total_energy,
        'energy_percentage': energy_percentage,
        'total_time': total_time,
        'segment_times': segment_times
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

def calculate_energy(points_utm, best_path, turn_radius, drone_speed, data):
   # Извлечение параметров из data
    M = float(data['mass_drone'])          # Масса дрона, кг
    m = float(data['mass_payload'])        # Масса нагрузки, кг
    C_res = float(data['drag_coefficient']) # Коэффициент сопротивления
    S = float(data['cross_section_area'])  # Площадь сечения, м²
    r = float(data['propeller_radius'])    # Радиус винта, м
    n = int(data['num_propellers'])        # Количество винтов
    rho = float(data['air_density'])       # Плотность воздуха, кг/м³
    eta = float(data['efficiency'])        # КПД
    V = float(data['droneSpeed'])          # Скорость полёта, м/с
    g = 9.81                               # Ускорение свободного падения, м/с²

    total_energy = 0
    segment_times = []
    n_points = len(points_utm)

    for i in range(n_points):
        # Длина сегмента с учётом траектории Дубинса
        path = dubins.shortest_path(points_utm[best_path[i]], points_utm[best_path[i+1]], turn_radius)
        segment_length = path.path_length()
        segment_time = segment_length / V
        segment_times.append(segment_time)

        # Вес дрона
        P = g * (M + m)

        # Сила сопротивления воздуха
        A = 0.5 * rho * C_res * S * V**2

        # Тяга
        T = np.sqrt(P**2 + A**2)

        # Итеративный расчёт индуктивной скорости (упрощённо, начальное приближение)
        V_ind = T / (2 * n * np.pi * r**2 * rho * V)  # Начальное значение
        for _ in range(5):  # 5 итераций для сходимости
            denominator = 2 * n * np.pi * r**2 * rho * np.sqrt(V**2 + V_ind**2)
            V_ind_new = T / denominator
            if abs(V_ind_new - V_ind) < 1e-6:
                break
            V_ind = V_ind_new

        # Потребная мощность (Вт)
        N = (T * V_ind + A * V) / eta

        # Энергия для сегмента (Дж = Вт * с)
        segment_energy = N * segment_time
        total_energy += segment_energy

    return total_energy, segment_times

def simulate_battery_discharge(total_energy, data, segment_times):
    C_nom = float(data['battery_nominal_capacity']) * 3600 / 1000  # мА·ч в А·с
    U_init = float(data['battery_initial_voltage'])               # В, начальные
    U_nom = float(data['battery_nominal_voltage'])                # В, при нагрзуке(?)
    phi = float(data['capacity_factor'])                          # φ, 
    K = float(data['battery_type_factor'])                        # K
    t_0 = float(data['battery_discharge_time'])                   # с, время разряда (?)
    total_time = sum(segment_times)                               # Общее время, с

    C_0 = C_nom  # Начальная ёмкость
    U = U_init   # Начальное напряжение
    C_bat = C_0  # Текущая ёмкость
    delta_t = 1  # Шаг времени, с
    time_elapsed = 0
    energy_consumed = 0

    while time_elapsed < total_time and C_bat > C_0 * (1 - phi):
        # Мощность для текущего момента (предполагается равномерное потребление (одна скорость))
        N = total_energy / total_time
        I = N / U  # Ток, А
        C_bat -= I * delta_t  # Уменьшение ёмкости (равномерно)
        U = U_init - (U_init - U_nom) / (phi * C_0) * (C_0 - C_bat)  # Обновление напряжения
        energy_consumed += N * delta_t
        time_elapsed += delta_t

    energy_remaining = C_nom * U_init - energy_consumed  # Дж
    flight_time_possible = time_elapsed if C_bat > 0 else total_time
    return energy_consumed, energy_remaining, flight_time_possible

if __name__ == '__main__':
    app.run(debug=True, port=6500)