Используется более реалистичная модель расхода энергии дрона.

За онсову взято:
Электронный научный журнал "Математическое моделирование, компьютерный и натурный эксперимент в естественных науках" http://mathmod.esrae.ru/
URL статьи: mathmod.esrae.ru/35-129
Ссылка для цитирования этой статьи:
Махонин А.А., Аль-Духэйдахави М.А.Л., Аль-Карави Р.Д.С. Анализ энергопотребления беспилотного летательного аппарата малых размеров // Математическое моделирование, компьютерный и натурный эксперимент в естественных науках. 2021. № 3

  
  Сервер получает:

``` js

{

  "droneSpeed": 10.0,         /* скорость */

  "energyPerMeter": 0.1,  //энергия на метр

  "turnEnergyFactor": 1.5,  // энергия на поворот

  "batteryCapacity": 16000, //mAч

  "acoAnts": 50,    // кол-во муравьёв

  "acoIterations": 100,  // кол-во итераций

  "acoAlpha": 1.0,  // альфа

  "acoBeta": 2.0,   // бета

  "acoRho": 0.1,    // испарение

  "acoQ": 100.0,

  "points": [...],  // точки для маршурта

  "mass_drone": 10.0,          // кг

  "mass_payload": 2.0,         // кг

  "drag_coefficient": 0.96,    // C_res

  "cross_section_area": 0.83,  // S, м²

  "propeller_radius": 0.559,   // r, м

  "num_propellers": 6,         // n

  "air_density": 1.225,        // ρ, кг/м³

  "efficiency": 0.85,          // η, КПД

  "battery_nominal_capacity": 16000, // C_nom, мА·ч

  "battery_initial_voltage": 25.2,   // U_init, В

  "battery_nominal_voltage": 22.2,   // U_nom, В

  "capacity_factor": 0.7,            // φ

  "battery_type_factor": 1.05,       // K

  "battery_discharge_time": 720      // t_0, с

}

```
Отдает:

``` js
{
  "best_length": 8642.797952, // длина маршрута
  "best_path": [1,3],  // лучшие точки
  "energy_percentage": 49.11796560774794, // остаток аккамулятора
  "segment_times": [258.1259538602234, 237.81697764647743, 368.3368636932993], // время по сегментам 
  "total_energy": 46160.18160065106,
  "total_time": 854, //  в сек
  "trajectory": [] // траектория Дубинса
}
```




Примерный набор параметров для дрона DJI MAVIC AIR 2, брался с https://img.mvideo.ru/ins/10024261.pdf
  

|Параметр|Значение|Единица|
|---|---|---|
|mass_drone|0.43|кг|
|mass_payload|0.15|кг|
|drag_coefficient|0.96|-|
|cross_section_area|0.05|м²|
|propeller_radius|0.09|м|
|num_propellers|4|-|
|air_density|1.225|кг/м³|
|efficiency|0.85|-|
|droneSpeed|10|м/с|
|battery_nominal_capacity|2000|мА·ч|
|battery_initial_voltage|12.6|В|
|battery_nominal_voltage|11.1|В|
|capacity_factor|0.7|-|
|battery_type_factor|1.05|-|
|battery_discharge_time|300|с|


### Работа

Для работы запустить сначала сервер, потом в браузере перейти по адресу:
http://localhost:6500/ 


