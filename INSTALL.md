<!-- Responsibility: Describe a portable Jazzy installation, selected dependency closure, and package checks. -->
# Установка и проверка

Основной путь: Ubuntu 24.04, ROS 2 Jazzy, MoveIt и Gazebo Harmonic. Для сценария
с настроенной позой куба не нужны CUDA, Isaac Sim, NVIDIA Isaac ROS или веса
детектора. Perception-пакет собирается вместе с приложением, но внешние GPU
backends устанавливаются отдельно.

Сначала установите ROS 2 Jazzy по [официальной инструкции](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
После подключения ROS apt-репозитория нужны инструменты разработки:

```bash
sudo apt update
sudo apt install ros-dev-tools ninja-build
```

Если `rosdep` ещё не инициализирован, выполните один раз `sudo rosdep init`.
Перед установкой зависимостей обновите его индекс командой `rosdep update`.

## Каталоги и окружение

Клонируйте самостоятельный репозиторий и перейдите в его корень:

```bash
git clone https://github.com/YARzich/franka_pick_and_place.git
cd franka_pick_and_place
```

Начните из каталога, содержащего `franka_motion/package.xml` и остальные шесть
пакетов. В отдельном репозитории это его корень, в `ros2_examples` — каталог
`manipulation`. Сам каталог семейства не является ROS-пакетом.

```bash
export MANIPULATION_SRC="$(pwd)"
export MANIPULATION_WS="$HOME/ros2_manipulation_ws"
test -f "$MANIPULATION_SRC/franka_motion/package.xml"
mkdir -p "$MANIPULATION_WS"
```

Путь workspace можно изменить. Исходники могут находиться как внутри его `src`,
так и в отдельном checkout: следующие команды явно задают область поиска.
Для сборки используйте системный Python и чистое окружение без подключённых
`install` других workspace и без активного GPU/conda/venv-окружения. Например,
откройте из того же терминала отдельную оболочку:

```bash
env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
  MANIPULATION_SRC="$MANIPULATION_SRC" MANIPULATION_WS="$MANIPULATION_WS" \
  /bin/bash --noprofile --norc
source /opt/ros/jazzy/setup.bash
cd "$MANIPULATION_WS"
```

## Зависимости основного пути

`colcon` выбирает `franka_motion` вместе с четырьмя пакетами семейства, от которых
он зависит. В этот набор не входят `manipulation_curobo` и
`manipulation_perception_isaac_ros`; их NVIDIA-зависимости базовой установке не
нужны. Следующая команда устанавливает также зависимости пакетных тестов:

```bash
mapfile -t manipulation_sources < <(
  colcon list --base-paths "$MANIPULATION_SRC" \
    --packages-up-to franka_motion --paths-only
)
test "${#manipulation_sources[@]}" -eq 5
rosdep install --from-paths "${manipulation_sources[@]}" \
  --ignore-src --rosdistro jazzy -y
rosdep check --from-paths "${manipulation_sources[@]}" \
  --ignore-src --rosdistro jazzy
```

Все ROS-зависимости этого пути доступны через Jazzy rosdep/apt, включая
`moveit_task_constructor_core`, `moveit_kinematics` и
`topic_based_ros2_control`. Последний объявлен приложением для Isaac-адаптера;
его наличие не устанавливает сам Isaac Sim.
Не используйте `--skip-keys` для обхода неизвестных ключей: это скрывает
неполную установку. Выбор зависимостей основан на
[package.xml и rosdep](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html),
а список исходников — на [colcon list](https://colcon.readthedocs.io/en/released/reference/verb/list.html).

## Сборка и пакетные проверки

```bash
colcon build --base-paths "$MANIPULATION_SRC" \
  --packages-up-to franka_motion \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja

source "$MANIPULATION_WS/install/setup.bash"
python3 "$MANIPULATION_SRC/scripts/check_architecture.py"
colcon test --base-paths "$MANIPULATION_SRC" \
  --packages-up-to franka_motion --return-code-on-test-failure
colcon test-result --verbose
```

Архитектурная команда проверяет исходники всего семейства. Обычные пакетные
тесты не запускают Gazebo или Isaac Sim: опция
`FRANKA_MOTION_ENABLE_SIMULATOR_LAUNCH_TESTS` по умолчанию выключена. Проверки
установленных необязательных пакетов пропускаются, если эти пакеты не собраны;
проверки их исходных контрактов сохраняются. Сборка и unit-тесты сами по себе
не подтверждают движение, GPU inference или работу на реальном роботе.

В четырёх C++ пакетах `cppcheck` входит в обычный `colcon test`. Для системного
cppcheck 2.13.0 из Ubuntu 24.04 проект явно включает анализ: обёртка ROS Jazzy
иначе пропускает всю ветку 2.x из-за исторического ограничения производительности.
Разрешение `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1` задаётся только самому тесту;
экспортировать его в оболочке или менять установленный ROS не требуется.
Подключена конфигурация googletest, чтобы анализатор понимал тестовые макросы.
Успешный результат включает фактический анализ файлов, без skipped cases.

Для отдельного повторения этого анализа после сборки нужных пакетов:

```bash
colcon test --base-paths "$MANIPULATION_SRC" \
  --packages-select manipulation manipulation_moveit franka_motion \
  --ctest-args -R '^cppcheck$' --output-on-failure \
  --return-code-on-test-failure
colcon test-result --verbose
```

Если собран необязательный `manipulation_curobo`, добавьте его в
`--packages-select`. GitHub Actions для этих проверок не используется.

Перед последующей сборкой снова используйте чистый build-терминал. Не меняйте
генератор CMake в уже существующем `build` и не удаляйте workspace для обхода
ошибок конфигурации.

## Первый запуск без GPU inference

В обычном графическом терминале с доступным дисплеем задайте путь использованного
workspace. Если при сборке выбрано другое место, замените значение в обоих
следующих примерах:

```bash
export MANIPULATION_WS="$HOME/ros2_manipulation_ws"
source /opt/ros/jazzy/setup.bash
source "$MANIPULATION_WS/install/setup.bash"
ros2 launch franka_motion franka_gazebo_moveit_table.launch.py
```

После готовности симулятора и контроллеров во втором терминале подключите тот
же overlay и запустите перенос куба с настроенной исходной позой:

```bash
export MANIPULATION_WS="$HOME/ros2_manipulation_ws"
source /opt/ros/jazzy/setup.bash
source "$MANIPULATION_WS/install/setup.bash"
ros2 launch franka_motion franka_gazebo_move_cube_round_trip.launch.py
```

Для повторного переноса после ручного перемещения куба настроенная
поза должна соответствовать сцене; наблюдение камерой относится к отдельному
perception-профилю. Полные runtime-проверки выполняются проектным
`validate-franka-manipulation`; wrapper принимает явный `WS_DIR` при нестандартном
расположении checkout. Общий статус семейства — `candidate`.

## Необязательные компоненты

- **cuRobo adapter:** C++ plugin и его unit-тесты собираются без установленного
  CUDA/cuRobo Python runtime. В чистом build-терминале добавьте
  `manipulation_curobo` к `--packages-up-to franka_motion`; перед этим повторите
  выбор исходников и `rosdep` с тем же списком целей, заменив ожидаемый размер
  массива с пяти на шесть. После сборки подключите overlay и выполните
  `colcon test --base-paths "$MANIPULATION_SRC" --packages-select manipulation_curobo --return-code-on-test-failure`.
  Для выполнения планирования нужен отдельно настроенный внешний runtime;
  ограничения приведены в [README backend](manipulation_curobo/README.md).
- **Isaac Sim:** установка SDK и принятие его условий выполняются отдельно.
  Способ выбора Python описан в [README Franka](franka_motion/README.md).
  `manipulation_perception_isaac_ros` для самого симулятора не требуется.
- **Камера и GPU perception:** окружения и модели описаны в
  [руководстве ресурсов](franka_motion/models/README.md). Bundle
  `manipulation_perception_isaac_ros` предназначен только для отдельно
  установленной линии Isaac ROS `4.1.x`; его нельзя добавлять в базовый
  `rosdep --from-paths` без соответствующего NVIDIA apt-репозитория.
  [Manifest pose backends](manipulation_perception/dependencies.repos)
  закрепляет внешние исходники для отдельных Python-окружений, а не ROS workspace.

## Закреплённые версии

При подготовке самостоятельного checkout использовались следующие версии
пакетов Jazzy: MoveIt `2.12.4`, MoveIt Task Constructor `0.1.8`,
`gz_ros2_control` `1.2.20`, Panda description `3.1.0`,
`topic_based_ros2_control` `0.3.0`. Это сведения о проверяемом окружении, а не
полный lock apt-репозитория. Версии установленных зависимостей сохраняйте вместе
с результатами своих проверок; обновление apt может изменить их.

[dependencies.repos](dependencies.repos) закрепляет необязательную исходную
сборку `topic_based_ros2_control` версии `0.3.0` на commit
`6bd8d55e1c4ad3188770fe5c8b93b942bcede4a2`. Она нужна, если требуется именно этот
source snapshot вместо выпущенного бинарного пакета:

```bash
mkdir -p "$MANIPULATION_WS/src"
vcs import "$MANIPULATION_WS/src" < "$MANIPULATION_SRC/dependencies.repos"
```

В таком варианте добавьте `"$MANIPULATION_WS/src/topic_based_ros2_control"` к
`--base-paths` при выборе исходников, сборке и тестировании. Выбор
`--packages-up-to franka_motion` теперь включает шесть пакетов; проверку размера
массива замените на `-eq 6`. Повторите `rosdep install` для этого набора, включая
test-зависимости внешнего пакета. Импорт не загружает модели, datasets или SDK.
