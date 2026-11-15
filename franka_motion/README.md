# franka_motion

`franka_motion` - учебный ROS 2-пакет с полным сценарием манипуляции для
Franka Panda в Gazebo и Isaac Sim.

В одном месте находятся модель Panda, настройки `ros2_control` и MoveIt,
симуляционные миры, камеры, конфигурация восприятия, launch-файлы и примеры
задач. Общая логика, которую можно использовать с другим роботом, вынесена в
пакеты `manipulation*`.

Проект рассчитан на ROS 2 Jazzy и Ubuntu 24.04. Это база для симуляции и
изучения архитектуры, а не готовая система безопасности реального робота.

## Что Здесь Работает

- Gazebo и Isaac Sim с одинаковым публичным сценарием.
- `ros2_control` для руки и захвата.
- MoveIt 2 как основной планировщик.
- Стол и куб в PlanningScene.
- Stereo camera, depth, point cloud, detection и оценка позы куба.
- Один `TransferObjectSequenceV2` для configured и detected переносов.
- Один lifecycle с feedback, cancel, pause, recovery и типизированным результатом.
- Необязательное реактивное движение к фиксированной позе через MoveIt Servo.
- cuRobo как необязательный planner pipeline за тем же MoveIt-контрактом.

## Как Устроен Проект

```text
franka_motion
  |-- конкретика Panda, Gazebo, Isaac и сценариев
  |-- запускает и настраивает переиспользуемые компоненты
  |
  |-- manipulation_msgs        ROS-интерфейсы
  |-- manipulation             задачи, сцена, диагностика и чистая Servo-политика
  |-- manipulation_moveit      planned-движение и Servo-адаптер через MoveIt
  |-- manipulation_perception  обработка камеры и поза объекта
  `-- manipulation_curobo      необязательный PlannerManager plugin
```

Главное правило: `franka_motion` знает, какой робот и какой мир используются.
Пакеты `manipulation*` выполняют общую работу и не зависят от Panda.

```text
configured/detected intent
  -> /transfer_object_sequence_v2
  -> franka_transfer_sequence_v2_server
     -> ObjectCatalog + ObservationProvider + resident MTC GraspProvider
     -> PickLiftPlaceStrategy
     -> common ArmPlanningPort -> MoveGroup -> OMPL | optional cuRobo
     -> common execution, gripper and scene ports

start_servo:=true
  -> moveit_servo_task_server -> /franka_moveit_servo_task_server/servo_to_pose
```

Detected provider принимает только новое по receipt наблюдение с корректной
ненулевой stamp, не из будущего и не старше `max_observation_age_sec` (профиль:
5 секунд). Этот бюджет включает GPU inference и ожидание точного TF; его можно
переопределить в `config/tasks/detected_observation.yaml`. Это ограничение возраста
отдельного измерения, а не общий timeout задачи. Повтор уже использованной stamp
в одной эпохе отклоняется; откат ROS-времени сбрасывает эпоху и требует новое
наблюдение. Configured provider по-прежнему выдаёт настроенную позу с новой stamp.

Goal V2 содержит только намерение клиента: объект, способ задания цели,
`return_home` и recovery policy. Геометрия, source pose, grasp, provider,
strategy и planner выбираются серверными профилями.

Поставляемый профиль `cube_50mm` объявляет все 24 proper rotation куба в его
локальной системе координат. Только `TARGET_INITIAL_SOURCE` использует их, чтобы
выбрать эквивалентную сохранённую ориентацию, ближайшую к текущему raw
observation; само наблюдение и явный absolute target не меняются. Resident MTC
добавляет в свою локальную копию сцены frame в центре объекта с осями workflow
frame, поэтому строит согласованный grasp при произвольной корректной detected
orientation, не переписывая authoritative pose объекта.

Все поддерживаемые Gazebo/Isaac launch-листы заменяют стандартную capability
`move_group/MoveGroupMoveAction` на
`manipulation_moveit/ExactPlanningSceneMoveGroupCapability`. Публичный
`/move_action` и выбор pipeline `ompl`/`curobo` сохраняются, но каждый запрос
планируется только по переданному полному снимку PlanningScene.

Между отдельными plan и execute стратегия ждёт свежие `/joint_states`,
подтверждающие quiescence в конечной точке предыдущей траектории. После
планирования она повторно читает полный scene robot state: изменившийся start
отбрасывает план и запускает replan до отправки ExecuteTrajectory. Для
position-only Panda gripper команда close нейтральна:
`position=0.0, max_effort=0.0`; удержание подтверждается разрешённым stall и
типизированным postcondition, а не фиктивным force control.

В Isaac Sim подтверждённый close дополнительно проходит через object-scoped
physical barrier: simulation thread проверяет близость куба, создаёт
pose-preserving `FixedJoint` и отвечает только после следующего PhysX step.
Лишь затем coordinator прикрепляет объект в PlanningScene. После подтверждённого
open тот же барьер удаляет принадлежащие runtime joint/collision filters и
подтверждает step до PlanningScene detach. Неопределённый transport или
частичный результат блокирует следующие цели. Считать quarantine снятым можно
только после полного перезапуска всего Isaac scenario/stack — симулятора, ROS
bridge и workflow server; перезапуск одной C++ server-ноды не является
безопасным recovery. Gazebo и hardware-путь остаются свободны от Isaac-specific
зависимости.

Discovery action server, ожидание ненулевого simulation clock, ответ на V2 goal
и terminal result не имеют elapsed timeout: каждое ожидание продолжается до
результата или shutdown. Короткие ограниченные transport-пробы используются
только внутри fail-closed протокола точечной отмены/reconciliation и не
завершают весь протокол сами по себе.

## Быстрый Запуск

Для чистой машины сначала выполните [установку семейства](../INSTALL.md):
там описаны выбор ROS-зависимостей и первый Gazebo-сценарий без GPU inference.
Ниже приведён отдельный путь с камерой и внешними perception-backends.

Сначала собери `franka_motion` вместе с зависимостями workspace. Используй
терминал без подключённого `install` этого workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to franka_motion \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```

После успешной сборки подключи `install/setup.bash` в терминалах запуска.

В первом терминале запусти Gazebo, Panda, MoveIt и table-camera pipeline.
Пути внешних perception-окружений и моделей зависят от машины: подготовка
каждого backend описана в `models/README.md`. Каждый такой путь — обязательный
launch argument без встроенного личного default; если его не передать явно,
он резолвится из одноимённой переменной окружения `FRANKA_GPU_*`
(`yolo_python` из `FRANKA_GPU_YOLO_PYTHON` и т.д. — полный список в описании
каждого argument через `ros2 launch franka_motion ... --show-args`). Без
явного значения и без переменной запуск не стартует и называет, чего именно
не хватает.

Один раз пропиши реальные пути в отдельном файле и подключи его в свой shell
profile:

```bash
cat > ~/.franka_gpu_env <<'EOF'
export FRANKA_GPU_YOLO_PYTHON=$HOME/venvs/yolo_full/bin/python
export FRANKA_GPU_MEGAPOSE_PYTHON=$HOME/micromamba/envs/megapose/bin/python
export FRANKA_GPU_MEGAPOSE_DATA_DIR=$HOME/megapose_data
export FRANKA_GPU_GIGAPOSE_PYTHON=$HOME/micromamba/envs/gigapose/bin/python
export FRANKA_GPU_GIGAPOSE_REPO_DIR=$HOME/gigaPose
export FRANKA_GPU_DINOV2_REPO_DIR=$HOME/dinov2
export FRANKA_GPU_GIGAPOSE_DATA_DIR=$HOME/gigapose_data
export FRANKA_GPU_ESS_ENGINE=$HOME/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx_trt10.13/ess.engine
export FRANKA_GPU_FOUNDATIONPOSE_REFINE_MODEL=$HOME/isaac_ros_assets/models/foundationpose/refine_model.onnx
export FRANKA_GPU_FOUNDATIONPOSE_REFINE_ENGINE=$HOME/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan
export FRANKA_GPU_FOUNDATIONPOSE_SCORE_MODEL=$HOME/isaac_ros_assets/models/foundationpose/score_model.onnx
export FRANKA_GPU_FOUNDATIONPOSE_SCORE_ENGINE=$HOME/isaac_ros_assets/models/foundationpose/score_trt_engine.plan
EOF
echo '[ -f ~/.franka_gpu_env ] && source ~/.franka_gpu_env' >> ~/.bashrc
```

После этого (в новом терминале) launch не требует ни одного из этих
аргументов на командной строке:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch franka_motion franka_gazebo_moveit_table_camera.launch.py
```

После готовности системы во втором терминале отправь задачу:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch franka_motion franka_detected_cube_round_trip.launch.py
```

Куб будет найден камерой, перенесён в целевую точку и возвращён обратно.
Задача не запускает симулятор повторно: она обращается к уже работающему action
server.

### Необязательный MoveIt Servo

Обычные сценарии переноса не требуют Servo и запускаются как раньше. Чтобы
поднять отдельное реактивное действие `ServoToPose`, установи пакет:

```bash
sudo apt install ros-jazzy-moveit-servo
```

и добавь аргумент к любому основному MoveIt launch, например (table-camera
листы дополнительно резолвят perception-пути из `FRANKA_GPU_*`, см. раздел
«Быстрый Запуск»):

```bash
ros2 launch franka_motion \
  franka_gazebo_moveit_table_camera.launch.py start_servo:=true
```

Тогда дополнительно появится action
`/franka_moveit_servo_task_server/servo_to_pose`. Это отдельная команда руке:
она не входит в `TransferObjectSequenceV2`, не управляет хватателем и не
заменяет планирование маршрута вокруг препятствий. Servo host не содержит
скрытого transfer action и запускается только при `start_servo:=true`.

Для Isaac Sim выбери Python окружения с установленным `isaacsim` и замени
команду первого терминала (те же переменные `FRANKA_GPU_*`):

```bash
export FRANKA_ISAAC_PYTHON="$HOME/.venvs/isaaclab/bin/python"
ros2 launch franka_motion \
  franka_isaac_moveit_table_camera.launch.py start_servo:=true
```

`isaac_python:=...` явно переопределяет интерпретатор; без аргумента используется
`FRANKA_ISAAC_PYTHON`, затем legacy `ISAAC_SIM_ROOT/python.sh` при заданном root,
затем `python3` из `PATH`. Каталог ресурсов определяется по пакету `isaacsim`
в выбранном Python, а не по расположению или symlink интерпретатора. Для ручного
запуска runner остаётся необязательный `--install_path`. Окружение удаляет
унаследованные ROS prefixes, сохраняя стандартную библиотеку и пакеты Python;
bridge использует bundled ROS из `isaacsim.ros2.core` (или прежнего bridge layout).

Offline CLI `prepare_gigapose_cube_dataset.py` требует `--gigapose-data-dir`
или `FRANKA_GPU_GIGAPOSE_DATA_DIR`; скрытого каталога пользователя по умолчанию нет.

Обычный launch сам задаёт стандартное окружение Isaac ROS-графа. Дополнительные
`export ROS_DOMAIN_ID=0` и `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` для
стандартного запуска не нужны.

### Необязательный cuRobo planner

cuRobo выбирается только на сервере и не меняет high-level action, клиента,
стратегию или физическое выполнение. После отдельной установки и сборки
`manipulation_curobo` запусти тот же leaf с другим backend profile:

```bash
ros2 launch franka_motion franka_gazebo_moveit_table.launch.py \
  planner_backend:=curobo
```

То же самое можно выбрать через общий router:

```bash
ros2 launch franka_motion franka_motion_bringup.launch.py \
  simulator:=gazebo scenario:=table planner_backend:=curobo
```

Обычное значение `planner_backend:=moveit` загружает только OMPL и не требует
пакета `manipulation_curobo`. Явный выбор `curobo` проверяет наличие package и
PlannerManager plugin до запуска процессов. Профиль `table_camera` отклоняется
сразу: текущий adapter намеренно не отбрасывает неподдерживаемый Octomap.

cuRobo строит только траекторию из полного снимка PlanningScene. Выполнение,
захват, attach/detach, cancel и recovery остаются у общего V2 workflow. Внешний
cuRobo не vendored и не скачивается этим пакетом автоматически. Условия зависят
от версии: прежний проверявшийся checkout ограничен research/evaluation,
а `nvidia-curobo==0.8.0` объявляет Apache-2.0. Совместимость нового GPU API
этим не подтверждается; точные версии и ссылки приведены в
[`cuRobo notices`](../manipulation_curobo/THIRD_PARTY_NOTICES.md).

## Какой Launch Выбрать

| Нужно | Launch-файл |
| --- | --- |
| Gazebo, пустой мир | `franka_gazebo_moveit.launch.py` |
| Gazebo, стол | `franka_gazebo_moveit_table.launch.py` |
| Gazebo, стол и камера | `franka_gazebo_moveit_table_camera.launch.py` |
| Isaac Sim, пустой мир | `franka_isaac_moveit.launch.py` |
| Isaac Sim, стол | `franka_isaac_moveit_table.launch.py` |
| Isaac Sim, стол и камера | `franka_isaac_moveit_table_camera.launch.py` |
| Перенести обнаруженный куб и вернуть | `franka_detected_cube_round_trip.launch.py` |

## Где Что Лежит

| Каталог | Назначение |
| --- | --- |
| `launch/` | Простые точки запуска системы и задач. |
| `description/` | Модульные xacro Panda, `ros2_control` и SDF камеры. |
| `config/` | MoveIt, controllers, perception, task и robot profile. |
| `worlds/` | Миры Gazebo и USD-сцены Isaac Sim. |
| `franka_motion_runtime/` | Сборка launch-графа, simulator helpers и task clients. |
| `src/control/` | Адаптер захвата Panda. |
| `src/scene/` | Добавление статической сцены через координатор. |
| `src/workflow_v2/` | Franka-композиция V2, providers, Isaac physical barrier и resident MTC grasp generation. |
| `src/mtc/` | Небольшие переиспользуемые MTC geometry/stop helpers. |
| `test/` | Быстрые unit, contract и launch tests. |

## Текущий Статус

Configured и detected клиенты переведены на один V2 action server. Старые
`TransferObject`, perception-resolved facade и standalone MTC preview удалены;
послемиграционные runtime-smoke Gazebo/Isaac фиксируются отдельно от быстрых
package tests.

Профиль `gazebo_active_cancel_v2` ждёт свежий статус `EXECUTING` у
FollowJointTrajectory controller, пока текущая фаза шага 0 — `APPROACHING`.
Goal controller должен быть принят после отправки проверяемой sequence;
его UUID и stamp выводятся в `ACTIVE_CANCEL_CONTROLLER_EXECUTING`.
Это подтверждение принятой исполняемой команды, а не измерение ненулевой скорости.
При прямом запуске `franka_transfer_sequence_active_cancel_probe` обязателен
параметр `probe.controller_action_name`; leaf
`franka_gazebo_active_cancel.launch.py` задаёт
`/panda_arm_controller/follow_joint_trajectory`. Если подтверждение не пришло
до probe deadline, клиент отменяет свою точную sequence goal и завершает проверку
с ошибкой, сохраняя обычный протокол containment до terminal result.
После подтверждённой отмены до захвата lease должен быть освобождён, объект —
оставаться в world и отсутствовать среди attached objects. Освобождение lease
не удаляет геометрию физического объекта; стол также должен остаться в сцене.

Обе фабрики MoveGroup подключают
`manipulation_moveit/CancelableExecuteTrajectoryCapability` вместо upstream
`MoveGroupExecuteTrajectoryAction`, сохраняя `/execute_trajectory`.
В [MoveIt 2.12.4](https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/move_group/src/default_capabilities/execute_trajectory_action_capability.cpp)
исходный обработчик блокирует обработку cancel ожиданием движения и не вызывает
остановку. Локальный capability владеет исполнением и подтверждает остановку
перед terminal result; системная установка MoveIt не изменяется.

MoveIt/OMPL является основным поддерживаемым путём. cuRobo доступен только как
явно выбираемый `planner_backend:=curobo` в тех же MoveIt launch-файлах; отдельного
`CuRoboPlanPose`, bridge, execution demo или параллельного lifecycle больше нет.
