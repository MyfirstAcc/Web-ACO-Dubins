import geopandas as gpd
import pyproj
from pyproj import Transformer
import numpy as np
import folium
import dubins

# Читаем GeoJSON файл
gdf = gpd.read_file("forest_points.geojson")

# Преобразуем координаты
transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)
transformer_to_wgs84 = Transformer.from_crs("EPSG:32636", "EPSG:4326", always_xy=True)

# Извлекаем координаты и NodeID
points_utm = []  # Координаты в UTM (для расчётов)
points_wgs84 = []  # Координаты в WGS84 (для карты)
node_ids = []
for idx, row in gdf.iterrows():
    lon, lat = row.geometry.x, row.geometry.y
    x, y = transformer_to_utm.transform(lon, lat)
    points_utm.append((x, y, 0))  # Ориентация пока 0
    points_wgs84.append((lat, lon))  # (широта, долгота) для карты
    node_ids.append(row['NodeID'])

# Параметры
n = len(points_utm)
R = 2  # Радиус поворота
alpha = 1.0
beta = 2.0
rho = 0.1
Q = 100
ants = 20
iterations = 100
max_distance = 13000 # Максимальная дальность полёта дрона в метрах

# Матрица расстояний по Дубинсу
distances = np.zeros((n, n))
for i in range(n):
    for j in range(n):
        if i != j:
            q0 = points_utm[i]
            q1 = points_utm[j]
            path = dubins.shortest_path(q0, q1, R)
            distances[i][j] = path.path_length()

# ACO для порядка точек
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
        visited.append(0)  # Возвращаемся к первой точке
        length = sum(distances[visited[i]][visited[i+1]] for i in range(n))
        paths.append(visited)
        lengths.append(length)
        if length < best_length:
            best_length = length
            best_path = visited.copy()
    
    pheromones *= (1 - rho)
    for path, length in zip(paths, lengths):
        for i in range(n):
            pheromones[path[i]][path[i+1]] += Q / length

# Проверка длины маршрута и разделение на этапы
print(f"Общая длина маршрута: {best_length:.2f} метров")
if best_length <= max_distance:
    print("Маршрут подходит по дальности полёта!")
    stages = [best_path]  # Один этап
else:
    print(f"Маршрут слишком длинный ({best_length:.2f} м). Максимум: {max_distance} м.")
    print("Разделяем маршрут на этапы...")

    stages = []
    current_stage = [best_path[0]]
    current_length = 0
    for i in range(n):
        next_point = best_path[i+1]
        segment_length = distances[best_path[i]][next_point]
        if current_length + segment_length <= max_distance:
            current_stage.append(next_point)
            current_length += segment_length
        else:
            # Завершаем текущий этап и начинаем новый
            if len(current_stage) > 1:  # Убедимся, что этап содержит хотя бы 2 точки
                current_stage.append(current_stage[0])  # Возвращаемся к начальной точке этапа
                stages.append(current_stage)
            current_stage = [next_point]
            current_length = 0
    # Добавляем последний этап
    if len(current_stage) > 1:  # Убедимся, что последний этап содержит хотя бы 2 точки
        current_stage.append(current_stage[0])
        stages.append(current_stage)
    elif len(current_stage) == 1 and stages:  # Если осталась одна точка, добавим её к последнему этапу
        stages[-1].extend(current_stage)
        stages[-1][-1] = stages[-1][0]  # Обновляем возвращение

# Вывод этапов
print("\nЭтапы маршрута:")
for idx, stage in enumerate(stages):
    stage_nodes = [node_ids[i] for i in stage]
    stage_length = sum(distances[stage[i]][stage[i+1]] for i in range(len(stage)-1))
    print(f"Этап {idx+1}: {stage_nodes}, длина: {stage_length:.2f} м")

# Визуализация на карте
center_lat = sum(lat for lat, lon in points_wgs84) / len(points_wgs84)
center_lon = sum(lon for lat, lon in points_wgs84) / len(points_wgs84)
m = folium.Map(location=[center_lat, center_lon], zoom_start=15)

# Добавляем точки на карту
for (lat, lon), node_id in zip(points_wgs84, node_ids):
    folium.Marker(
        location=[lat, lon],
        popup=node_id,
        icon=folium.Icon(color="red", icon="circle")
    ).add_to(m)

# Добавляем траекторию для каждого этапа
colors = ['blue', 'green', 'purple']  # Разные цвета для этапов
for stage_idx, stage in enumerate(stages):
    stage_trajectory = []
    for i in range(len(stage)-1):
        q0 = points_utm[stage[i]]
        q1 = points_utm[stage[i+1]]
        path = dubins.shortest_path(q0, q1, R)
        path_points = path.sample_many(0.1)[0]
        if not path_points:  # Проверяем, что траектория не пустая
            print(f"Предупреждение: не удалось построить траекторию между {node_ids[stage[i]]} и {node_ids[stage[i+1]]}")
            continue
        path_wgs84 = []
        for x, y, _ in path_points:
            lon, lat = transformer_to_wgs84.transform(x, y)
            path_wgs84.append([lat, lon])
        stage_trajectory.extend(path_wgs84)
    
    # Рисуем траекторию этапа, только если она не пустая
    if stage_trajectory:
        folium.PolyLine(
            locations=stage_trajectory,
            color=colors[stage_idx % len(colors)],
            weight=2,
            popup=f"Этап {stage_idx+1}"
        ).add_to(m)
    else:
        print(f"Этап {stage_idx+1} не содержит траектории для отображения.")

# Сохраняем карту как HTML
m.save("quadcopter_route.html")

print("\nКарта сохранена как 'quadcopter_route.html'. Открой её в браузере!")