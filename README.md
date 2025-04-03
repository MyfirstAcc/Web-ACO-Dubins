ИСПОЛЬЗОВАТЬ Python 3.7, так как библиотека поддерживается только до python 3.7:
https://piwheels.org/project/dubins/
  
### Один из рабочих способов установки на win 11

1. Откройте Visual Studio Installer.

Убедитесь, что установлен компонент "Desktop development with C++" (Разработка настольных приложений на C++). Если его нет, добавьте и установите.

Убедитесь, что установлен Windows SDK (например, версия 10.x).

2. Официальный репозиторий библиотеки dubins на GitHub: github.com/AndrewWalker/pydubins.

```powershell

git clone https://github.com/AndrewWalker/pydubins.git

cd pydubins

```

3. Добавьте определение M_PI: В начало файла (В папке с исходниками перейди в dubins/src/), после #include директив (например, после #include "dubins.h"), вставьте:

```c

#ifndef M_PI

#define M_PI 3.14159265358979323846

#endif

```

  4. Установка модифицированной версии через pip
Важно запускать pip и python с приставкой 3.7(при условии, что он установлен и активно окружение), например:
```powershell
PS C:\Test> python3.7 app.js
PS C:\Test> pip3.7 install dubins
```

Установка окружения:
```powershell
python3.7 -m venv venv
```
Активация:

```powershell
.\venv\Scripts\activate
```


Установите зависимости для компиляции:

```powershell


pip3.7 install numpy setuptools

```

Перейдите в папку с исходниками:

```powershell

cd C:\dev\python\dubins-source\pydubins

```

Выполните установку:

```powershell

pip3.7 install .

```

Проверьте установку:
После завершения установки попробуйте импортировать:

```powershell

python3.7 -c "import dubins; print(dubins.__version__)"

```

Если ошибок нет, всё сработало!

### Перед запуском
Не забыть установить другие зависимости: 
```powershell
pip install -r requirements.txt
```

### Структура проекта
  

В решении две версии.

- ACODubinsGIS.py - работает с geojson(QGIS), результат: страница html с маршрутом.
- ACODubins.py - рандомные точки


В папке server:
- /simple - простая версия сервера с простым подсчётом энергии дрона. Считает энергию как: 
$$Е_р = L * L/м * K_(поворота)$$ 
- /realistic - сложная модель подсчёта энергии дрона (экспериментально). Есть баги




