---
name: diagnose-franka-perception
description: Use when diagnosing the franka_motion table-camera perception pipeline in Gazebo or Isaac Sim, including stereo images, CameraInfo, ESS/SGBM disparity, depth, point cloud, stamped Detection2DArray, MegaPose/GigaPose/FoundationPose output, object PoseStamped, frame/stamp correlation, or perception diagnostics. Do not use for general MoveIt motion validation, static architecture review, model training, or accuracy benchmarking.
---

# Diagnose Franka Perception

Сначала раздели pipeline на этапы: camera -> stereo -> detector -> pose backend
-> world pose. Не начинай с настройки последней модели, пока предыдущий этап не
выдает stamped данные правильного типа.

## Уже запущенный graph

Для обычного режима `gigaPose`:

```bash
.agents/skills/diagnose-franka-perception/scripts/inspect_graph.sh \
  --mode gigapose
```

Для других режимов доступны `stereo`, `ess`, `ecc`, `yolo`, `foundationpose`
и `megapose`. Скрипт проверяет актуальный stamped bbox
`/table_stereo/cube_detection2d`; `/table_stereo/cube_bbox` является legacy и
не считается доказательством синхронизации.

При отказе сначала назови первый неработающий этап и краткую причину. Затем
проверь его вход, тип сообщения, непустой `frame_id`, ненулевой stamp и точное
diagnostic status name.

## Полное GPU evidence

Полный ограниченный по времени Gazebo evidence-прогон с provenance моделей:

```bash
.agents/skills/diagnose-franka-perception/scripts/run_gazebo_evidence.sh
```

Он использует проектный профиль `gazebo_gpu_perception_v2` и sibling skill
`$validate-franka-manipulation`. Внешние Python-окружения, engines и каталоги
данных нужно явно задать через `FRANKA_GPU_*` по `franka_motion/models/README.md`
(в monorepo пакет находится под `manipulation/`). Отсутствующие значения
скрипт перечисляет до запуска процессов; личных defaults нет. Checkpoints
GigaPose/MegaPose по умолчанию ищутся внутри явно заданных каталогов данных,
но их можно переопределить переменными `FRANKA_GPU_*_CHECKPOINT`.
Project-owned contract, YOLO checkpoint и cube mesh берутся из текущего checkout.

Для обоих скриптов `WS_DIR` задаёт colcon workspace; автоматически определяется
только предок `src/`. `ROS_SETUP_FILE` выбирает underlay, по умолчанию Jazzy.
Диагностика использует выбранный собранный workspace или явно подключённое
окружение, если checkout расположен вне `src/` и `WS_DIR` не задан.

Полная цепочка камера -> поза -> движение проверяется профилем
`gazebo_detected_round_trip_v2` (5 подряд detected transfer-and-return):

```bash
.agents/skills/diagnose-franka-perception/scripts/run_gazebo_evidence.sh \
  --profile gazebo_detected_round_trip_v2
```

Для Isaac Sim отдельного GPU evidence-профиля пока нет. Запусти
`franka_isaac_moveit_table_camera.launch.py`, затем используй
`inspect_graph.sh`; не называй такую проверку полным evidence или оценкой
точности.

## Границы результата

- Наличие topic не доказывает, что по нему идут свежие данные.
- Успешная корреляция pipeline не доказывает точность позы или ADD-S.
- Legacy compact bbox не заменяет stamped `Detection2DArray`.
- Не уменьшай число гипотез или качество моделей ради прохождения диагностики.
