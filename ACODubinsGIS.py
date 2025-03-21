import geopandas as gpd
import pyproj
from pyproj import Transformer
import numpy as np
import folium
import dubins

# Читаем GeoJSON файл
gdf = gpd.read_file("forest_points.geojson")

# Преобразуем координаты
# WGS84 (широта/долгота) → UTM (метры) для расчётов
transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)
# UTM → WGS84 для отображения на карте
transformer_to_wgs84 = Transformer.from_crs("EPSG:32636", "EPSG:4326", always_xy=True)

# Извлекаем координаты и NodeID
points_utm = []  # Координаты в UTM (для расчётов)
points_wgs84 = []  # Координаты в WGS84 (для карты)
node_ids = []
for idx, row in gdf.iterrows():
    lon, lat = row.geometry.x, row.geometry.y
    x, y = transformer_to_utm.transform(lon, lat)
    points_utm.append((x, y, row['Orientation']))  # Ориентация пока 0 (можно добавить поле points.append((x, y, row['Orientation'])).)
    points_wgs84.append((lat, lon))  # (широта, долгота) для карты
    node_ids.append(row['NodeID'])

# Параметры
n = len(points_utm)
R = 30  # Радиус поворота
alpha = 1.0
beta = 2.0
rho = 0.1
Q = 100
ants = 20
iterations = 100

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

# Собираем траекторию для отображения на карте
full_trajectory = []
for i in range(n):
    q0 = points_utm[best_path[i]]
    q1 = points_utm[best_path[i+1]]
    path = dubins.shortest_path(q0, q1, R)
    path_points = path.sample_many(0.1)[0]  # Шаг дискретизации 0.1
    # Преобразуем точки траектории из UTM в WGS84
    path_wgs84 = []
    for x, y, _ in path_points:
        lon, lat = transformer_to_wgs84.transform(x, y)
        path_wgs84.append([lat, lon])  # [широта, долгота] для folium
    full_trajectory.append(path_wgs84)

# Создаём карту с folium
# Центр карты — средние координаты всех точек
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

# Добавляем траекторию
for path_segment in full_trajectory:
    folium.PolyLine(
        locations=path_segment,
        color="blue",
        weight=2
    ).add_to(m)

# Сохраняем карту как HTML
m.save("quadcopter_route.html")

print("Лучший порядок точек:", [node_ids[i] for i in best_path])
print("Общая длина маршрута:", best_length)
print("Карта сохранена как 'quadcopter_route.html'. Открой её в браузере!")