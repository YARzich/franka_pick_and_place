# Franka Pick and Place

В этом примере Franka Panda захватывает куб и переносит его в заданную точку.
Исходную позу куба можно задать в конфигурации или определить по камере.
Репозиторий объединяет семь пакетов ROS 2. `colcon` находит их по `package.xml`.

Базовый путь: **Ubuntu 24.04, ROS 2 Jazzy, MoveIt 2 и Gazebo Harmonic**.
Начни с [установки и сборки](INSTALL.md): обычная демонстрация с настроенной
позой объекта не требует Isaac Sim, CUDA или моделей нейросетей. Для работы
по камере внешние perception-окружения устанавливаются отдельно.

Сейчас семейство имеет общий статус **candidate**. Рекомендуемый перенос объекта
использует один V2 lifecycle и один PlanningScene contract. В ROADMAP уже
зафиксированы автоматический Gazebo detected round trip и воспроизводимая
подготовка perception-окружения; границы этих результатов и остальные критерии
читать в соответствующих карточках. Они не повышают статус семейства автоматически.

Границы новых гарантий остаются пакетными: `manipulation` разрешает
catalog-declared эквивалентные ориентации, `franka_motion` задаёт 24 симметрии
куба и workflow-aligned MTC frame, а `manipulation_moveit` защищает переход
между отдельными plan и execute. Raw observation и явный absolute target
остаются исходными данными задачи.

## Карта пакетов

| Пакет | Простыми словами | Роль | Статус |
|---|---|---|---|
| [`franka_motion`](franka_motion/README.md) | Собирает конкретную систему с Franka: модель робота, конфигурацию MoveIt, симуляторы и сценарии запуска | reference-app | candidate |
| [`manipulation_msgs`](manipulation_msgs/README.md) | Задает общий язык обмена: сообщения, сервисы и действия | contract | candidate |
| [`manipulation`](manipulation/README.md) | Управляет жизненным циклом задачи, состоянием сцены, здоровьем системы и общими правилами выполнения | component | candidate |
| [`manipulation_moveit`](manipulation_moveit/README.md) | Выполняет движение через MoveIt, включая обычное планирование и реактивное управление Servo | component | candidate |
| [`manipulation_perception`](manipulation_perception/README.md) | Приводит stereo, detector и pose estimator к общим stamped ROS-интерфейсам | component | candidate |
| [`manipulation_perception_isaac_ros`](manipulation_perception_isaac_ros/README.md) | Изолирует зависимости NVIDIA Isaac ROS для GPU-конвейера восприятия | dependency bundle | candidate |
| [`manipulation_curobo`](manipulation_curobo/README.md) | Предоставляет необязательный plan-only cuRobo PlannerManager за тем же MoveIt planning port | experimental-backend | experimental |

## Откуда начинать чтение

1. Конкретная задача: [`franka_detected_cube_round_trip.launch.py`](franka_motion/launch/franka_detected_cube_round_trip.launch.py).
2. Внешний контракт задачи: [`TransferObjectSequenceV2.action`](manipulation_msgs/action/TransferObjectSequenceV2.action).
3. Общая оркестрация и состояние: `manipulation/include/manipulation/workflow`.
4. Реализация движения: `manipulation_moveit/src`.
5. Получение позы объекта: `manipulation_perception` и launch-сборка в
   `franka_motion/franka_motion_runtime/bringup`.

Направление дальнейшей работы и критерии готовности находятся в
[`ROADMAP.md`](ROADMAP.md).

## Проверенные возможности и ограничения

- Пакетные тесты и сценарии Gazebo проверяются отдельно: успешная сборка не
  подтверждает движение, восприятие или работу оборудования.
- Предыдущие Gazebo smoke, active cancel и detected round trip прошли в
  зафиксированных в ROADMAP окружениях. Это не обещание поддержки любой GPU
  или версии стороннего backend.
- Полная приёмка текущего Isaac Sim ещё не завершена. Установка SDK и принятие
  его лицензии выполняются пользователем отдельно.
- cuRobo — необязательный экспериментальный планировщик. Текущие ограничения
  версий и лицензий описаны в его [README](manipulation_curobo/README.md).
- Работа на реальном роботе этим примером не сертифицируется.

## Правила проекта

- [Изменения и совместимость](docs/CHANGE_CONTROL.md).
- [Вклад в проект и проверки](CONTRIBUTING.md).
- [Безопасность и сообщения об уязвимостях](SECURITY.md).
- [Лицензии кода, моделей и ресурсов](THIRD_PARTY_NOTICES.md).
- [Состав и происхождение репозитория](docs/PUBLICATION.md).

Код проекта распространяется под MIT с отдельно указанными исключениями.
Веса YOLO имеют собственную маркировку AGPL-3.0; MIT не распространяется
автоматически на все файлы и внешние зависимости. При распространении или
развёртывании приложения с YOLO нужно выполнять применимые условия AGPL;
MIT-уведомления отдельных файлов их не отменяют. В поставке исходников
находятся код приложения, training tools, сохранённые настройки и ссылки
на закреплённую версию Ultralytics.
Для работы используются готовые веса; повторное обучение в подготовку проекта
не входит. Проверки сборки и кода выполняются локально по INSTALL.

## Быстрые архитектурные проверки

Из корня семейства (в отдельном репозитории это корень checkout):

```bash
python3 scripts/check_architecture.py
```

После переноса checkout использовать тот же `scripts/check_architecture.py`
относительно корня семейства. Нужен Python с `pytest`; ROS overlay, сборка
`franka_motion`, GPU и симулятор для этой команды не нужны.

Команда запускает существующие проверки направлений зависимостей, generic/scenario
границ, общих владельцев policy и объявленной ответственности файлов. Она обязательна
при изменении этих границ, зависимостей, общих контрактов или самих проверок, включая
изменения только одного пакета. Отсутствующий файл, пустая коллекция любого выбранного
модуля и падение теста завершают команду с ошибкой. Пользовательские pytest plugins,
conftest и дополнительные параметры из окружения не подключаются.

Проверки установленного дерева MoveIt/cuRobo и wire-контрактов остаются в обычных
пакетных тестах после сборки. Наличие `Responsibility` подтверждает описание роли,
но не единственность ответственности; успешный быстрый запуск не доказывает
поведение ROS-графа, симулятора или оборудования.

Тесты самой команды:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -B -m pytest -q -p no:cacheprovider scripts/tests/test_check_architecture.py
```
