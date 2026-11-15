---
name: validate-franka-manipulation
description: Use when running or validating the recommended franka_motion MoveIt path in Gazebo or Isaac Sim, including smoke, round-trip soak, cancel, health-fault, backend-death, runtime-evidence, and interrupted-run cleanup profiles. Do not use for perception-only diagnosis (use diagnose-franka-perception), static code review, real hardware, or the experimental cuRobo backend.
---

# Validate Franka Manipulation

Используй проектный `franka_runtime_harness`: он запускает команды в отдельных
process groups, пишет evidence и завершает только процессы с точным run token.
Не заменяй его `pkill` по именам процессов.

## Порядок работы

1. Прочитай `AGENTS.md` и `ROADMAP.md` семейства. В monorepo они находятся в
   `manipulation/`; в самостоятельном репозитории — в корне.
2. Убедись, что другой ROS/симуляционный runtime не запущен.
3. Посмотри актуальные профили:

```bash
.agents/skills/validate-franka-manipulation/scripts/run_profile.sh --list
```

Список читается из исходников и не требует ROS или собранного overlay.
Для сборки и запуска размести checkout под `<workspace>/src/` либо явно задай
`WS_DIR`. `ROS_SETUP_FILE` выбирает underlay (по умолчанию
`/opt/ros/jazzy/setup.bash`). Скрипт собирает `franka_motion` с workspace-
зависимостями из чистого окружения underlay, затем подключает выбранный
`install/setup.bash`. `--build-first false` используй после своей сборки.
Вне стандартного workspace при таком флаге можно использовать заранее
подключённый overlay, оставив `WS_DIR` незаданным; личные каталоги не угадываются.

4. Запусти минимальный подходящий профиль. Основной быстрый тест:

```bash
.agents/skills/validate-franka-manipulation/scripts/run_profile.sh \
  --profile gazebo_smoke_v2
```

Для Isaac Sim:

```bash
.agents/skills/validate-franka-manipulation/scripts/run_profile.sh \
  --profile isaac_smoke_v2
```

5. В ответе укажи профиль, `evidence_status`, путь к
`runtime-result.v2.json` и отдельно ошибки cleanup.

## Выбор проверки

- `gazebo_smoke_v2`, `isaac_smoke_v2`: один полный перенос через MoveIt.
- `gazebo_soak_v2`, `isaac_soak_v2`: многократный round trip; запускай только
  для проверки устойчивости, поскольку эти профили долгие.
- `gazebo_active_cancel_v2`: отмена во время активного движения.
- `gazebo_health_fault_v2`: отказ контроллера, joint state, TF, MoveIt и сцены.
- `gazebo_backend_death_v2`: смерть backend во время задачи.

Полный GPU perception профиль и `gazebo_detected_round_trip_v2` (полная
цепочка камера -> поза -> движение, нужны FRANKA_GPU_* биндинги) относятся к
`$diagnose-franka-perception`.
Названия с суффиксом `v2` сейчас являются актуальными profile ID внутри matrix
schema v3; не переименовывай их в skill.

## Границы результата

- Успешная сборка или readiness не равны runtime evidence.
- Успех засчитывается только при `evidence_status: passed`, завершенной стадии
  validation и чистом cleanup.
- Семейство пакетов пока имеет статус `candidate`; отдельный успешный профиль
  не повышает этот статус автоматически.
- cuRobo пока экспериментален и не входит в этот рекомендуемый validation skill.
- Skill не запускает проверки реального робота.
