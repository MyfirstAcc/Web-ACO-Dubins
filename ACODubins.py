import numpy as np
import matplotlib.pyplot as plt
import dubins

# Генерация 20 случайных точек
np.random.seed(80)  # Для воспроизводимости
n = 20
points = [(np.random.uniform(0, 50), np.random.uniform(0, 50), np.random.uniform(0, 360)) for _ in range(n)]
R = 2  # Радиус поворота (в метрах)

# Матрица расстояний по Дубинсу
distances = np.zeros((n, n))
for i in range(n):
    for j in range(n):
        if i != j:
            q0 = points[i]
            q1 = points[j]
            path = dubins.shortest_path(q0, q1, R)
            distances[i][j] = path.path_length()

# Параметры ACO
alpha = 1.0  # Влияние феромонов
beta = 2.0   # Влияние эвристики
rho = 0.1    # Испарение феромонов
Q = 100      # Константа для феромонов
ants = 20    # Количество муравьёв
iterations = 100

# Инициализация феромонов и эвристики
pheromones = np.ones((n, n)) * 0.1
eta = 1 / (distances + 1e-10)  # Эвристика = 1/расстояние

# Основной цикл ACO
best_path = None
best_length = float('inf')

for _ in range(iterations):
    paths = []
    lengths = []
    for _ in range(ants):
        # Строим маршрут для муравья
        visited = [0]  # Начинаем с точки 0
        while len(visited) < n:
            current = visited[-1]
            unvisited = [i for i in range(n) if i not in visited]
            probs = [(pheromones[current][j]**alpha) * (eta[current][j]**beta) for j in unvisited]
            probs = probs / np.sum(probs)
            next_point = np.random.choice(unvisited, p=probs)
            visited.append(next_point)
        visited.append(0)  # Возвращаемся в начало
        # Считаем длину маршрута
        length = sum(distances[visited[i]][visited[i+1]] for i in range(n))
        paths.append(visited)
        lengths.append(length)
        if length < best_length:
            best_length = length
            best_path = visited.copy()
    
    # Обновляем феромоны
    pheromones *= (1 - rho)  # Испарение
    for path, length in zip(paths, lengths):
        for i in range(n):
            pheromones[path[i]][path[i+1]] += Q / length

# Визуализация лучшего пути
plt.figure(figsize=(10, 10))
for i in range(n):
    q0 = points[best_path[i]]
    q1 = points[best_path[i+1]]
    path = dubins.shortest_path(q0, q1, R)
    path_points = path.sample_many(0.1)[0]  # Шаг дискретизации 0.1
    x, y = zip(*[(p[0], p[1]) for p in path_points])
    plt.plot(x, y, 'b-', linewidth=1)
for px, py, _ in points:
    plt.plot(px, py, 'ro', markersize=5)  # Точки как красные круги
plt.grid(True)
plt.title(f"Лучший маршрут: {best_length:.2f} метров")
plt.xlabel("X (м)")
plt.ylabel("Y (м)")
plt.show()

print("Лучший порядок точек:", best_path)
print("Общая длина маршрута:", best_length)