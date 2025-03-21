ИСПОЛЬЗОВАТЬ Python 3.7.
https://piwheels.org/project/dubins/

Один из рабочих способов установки на win 11:

0. Открой Visual Studio Installer.
Убедись, что установлен компонент "Desktop development with C++" (Разработка настольных приложений на C++). Если его нет, добавь и установи.
Убедись, что установлен Windows SDK (например, версия 10.x).

1. Официальный репозиторий библиотеки dubins на GitHub: github.com/AndrewWalker/pydubins.
```powershell
git clone https://github.com/AndrewWalker/pydubins.git
cd pydubins
```


2. Добавь определение M_PI:
В начало файла
(В папке с исходниками перейди в dubins/src/), после #include директив (например, после #include "dubins.h"), вставь:
```c
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
```

3. Установка модифицированной версии через pip
Установи зависимости для компиляции
```powershell
pip install numpy setuptools
```
Перейди в папку с исходниками:
```powershell
cd C:\dev\python\dubins-source\pydubins
```
Выполни установку:
```powershell
pip install .
```


Проверь установку:
После завершения установки попробуй импортировать:
```powershell
python -c "import dubins; print(dubins.__version__)"
```
Если ошибок нет, всё сработало!

