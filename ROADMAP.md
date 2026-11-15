# Manipulation roadmap

Этот файл фиксирует направление развития семейства пакетов. Статус `done`
ставится только после фактической реализации и предусмотренной проверки.

Статусы:

- `planned` — направление зафиксировано, но решение еще требует обсуждения;
- `ready` — решение согласовано и ожидает команды `Приступай`;
- `in_progress` — выполнение разрешено и уже идет;
- `blocked` — есть конкретное препятствие, без устранения которого работа не
  может продолжаться;
- `done` — реализация и предусмотренные проверки завершены.

## Текущее состояние

- **Статус семейства:** `candidate`.
- **Цель:** один понятный, расширяемый и проверенный базовый путь объектной
  манипуляции; Franka служит прикладным примером, а общая реализация не зависит
  от конкретного робота.
- **Доказательства:** автоматический Gazebo detected round trip и подготовка
  воспроизводимого perception-окружения отмечены `done` в карточках этапа 3.
  Их результаты ограничены указанными там профилями и проверками; полный
  автоматический detected round trip для Isaac из этого не следует.
  Текущие статусы и оставшиеся критерии определяются карточками задач, а не
  этой вводной сводкой. Общий статус семейства остаётся `candidate`.

## Подготовка отдельного публичного репозитория 2026-11-13

- **Статус:** `done` для локальной подготовки исходной поставки, включая
  комплект обучения и атрибуцию.
  Пользователь разрешил работу командой
  «Тогда наведи порядок, что бы все соответствовало нормам. Потом отчет что сделал».
  Дополнение разрешено командой «Тогда поправь все, в соответствии с пунктами»:
  комплект training tools/configs, уведомления о сторонних ресурсах и контакт
  для приватных сообщений, затем обновление отдельной копии и отчёта.
- **Цель:** переносимое проверенное семейство из семи пакетов для отдельного
  репозитория `franka_pick_and_place`, с явными зависимостями и сведениями о лицензиях.
- **Объём:** исправление личных путей/overlay в tools и тестах, перенос policies,
  самостоятельные INSTALL и dependency manifest, локальные проверки, проверенный состав
  отдельной копии и проверка происхождения vendored ресурсов. ROS-контракты,
  физика, алгоритмы планирования и бинарные модели не изменяются.
- **Приёмка:** чистая сборка и зарегистрированные тесты отдельной копии,
  source architecture gate, переносимость tools, проверка diff и состава.
  Новые GPU/SDK не скачиваются; Isaac/GPU runtime не является результатом
  этой упаковочной задачи. Создание публичного remote, commit и push
  не входило в этап локальной подготовки.
- **Отдельные решения:** адрес сопровождающего нельзя придумывать;
  неподтверждённую лицензию нельзя заменять удобной. Такие ограничения
  публикации перечисляются в отчёте, независимая подготовка продолжается.
- **Документация:** [INSTALL](INSTALL.md), [состав публикации](docs/PUBLICATION.md),
  [лицензии и происхождение](THIRD_PARTY_NOTICES.md).

- **Выполнено:** переносимые wrappers и тесты без личного overlay; cleanup только
  своих процессов; исправление rosdep `kdl_kinematics_plugin` → `moveit_kinematics`;
  самостоятельные инструкции, policies и dependency manifest;
  копия без Git-истории, артефактов сборки и соседних проектов. Лицензии моделей
  и внешних зависимостей уточнены по конкретным версиям, bytes весов сохранены.
- **Проверки:** сборка 7 пакетов из отдельной копии с чистым Jazzy underlay;
  288 зарегистрированных CTest commands без оставшихся failures; architecture
  gate 44 passed/2 installed checks deselected в обоих layouts; 8 тестов самой
  команды; 14 новых tooling checks включены в CTest. Baseline policy отдельно:
  8 passed/2 ожидаемых skip для отсутствующих optional packages.
- **Уточнение тестов:** discovery barrier в `test_ros_v2_scene_adapter_waits`
  иногда не укладывался в 2 секунды до начала проверяемой операции. Startup
  budget теперь 10 секунд, CTest budget 90 секунд; прежний cancel budget
  1 секунда и проверки задержанного ответа сохранены. Исходный failure
  воспроизведён отдельно, затем весь MoveIt package suite прошёл.
- **Ограничения проверки:** ament_cppcheck пропустил анализ на этой версии
  инструмента (402 файловых skip в четырёх пакетах). Прямой cppcheck изменённого
  C++ теста выдал прежний advisory `useInitializationList` в setup-helper,
  не относящийся к discovery change. Исправление cppcheck описано ниже.
  Gazebo/Isaac/GPU runtime в этой задаче не запускались.
- **Дополнение по пунктам владельца:** `yaric002@gmail.com` указан во всех семи
  package.xml и в SECURITY как приватный контакт. `training/` содержит шесть
  исходных tools, все 109 настроек каждого из шести запусков, dataset manifests,
  hashes происхождения и переносимый prepare-only helper. Исходный
  `perception_training` сохранён без изменений. MoveIt-attribution закреплена
  по 3.1.0/commit, manifest и официальной BSD-3 документации; выдуманный copyright
  заменён проверенными сведениями. MIT собственных файлов и условия AGPL для
  YOLO-приложения описаны отдельно.
- **Проверка дополнения:** 26 training tests и 8 тестов архитектурной команды
  прошли в обеих раскладках; source gate 44 passed/2 deselected в каждой.
  Повторная сборка пяти пакетов основного пути и пять затронутых CTest checks
  (licenses, responsibility, policies, flake8, xmllint) прошли. Все семь
  package.xml валидны. Доказано сохранение выходного dataset при ошибочном
  USD path с `--overwrite`; пути с YAML-спецсимволами сохраняются корректно.
  Проверки состава, ссылок, хешей и shell-синтаксиса
  прошли; эвристический поиск ключей/токенов не дал совпадений.
- **Границы дополнения:** сохранён доступный исходный snapshot, а не выдуманная
  историческая версия скриптов для каждого обучения. Изображения, промежуточные
  checkpoints и SDK не включены; обучение и генерация dataset не запускались.
  Эти ограничения не являются утверждением о запрете публикации. Настройка
  private reporting хостинга необязательна при указанном контакте.
- **Локальный результат:** `/home/yaroslav/ros2_publication/2026-11-13/` содержит
  отдельную копию `ros2_manipulation`, manifest и отчёт. Публичный remote,
  commit и push на этапе подготовки не выполнялись.

### Уточнение объёма и работающий cppcheck

- **Статус:** `done`, по прямой команде владельца.
- GitHub Actions не нужен на текущем этапе: добавленный workflow удалён,
  проверки остаются локальными. Повторное обучение и восстановление старых
  обучений не являются задачами или критериями публикации; используются
  готовые веса. Исходники при моделях сохраняются как часть их поставки.
- Причина skip установлена: `ament_cppcheck` 0.17.5 отключает всю ветку
  cppcheck 2.x без `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS`. Установленный 2.13.0
  совпадает с версией Ubuntu 24.04 apt. Замена на другую 2.x не устраняет skip.
- Исправление: штатный opt-in в окружении только CTest cppcheck, явная
  конфигурация googletest для тестовых макросов, сохранение всех исходных
  файлов и include paths. ROS и системные инструменты не подменяются.
- Приёмка: обычный `colcon test` без ручного env override действительно
  анализирует четыре C++ пакета; в XML нет skipped/failures; версия и набор
  файлов зафиксированы. Проверяются CMake/lint, архитектура и затронутый
  тест владения временным файлом; копия и отчёт обновляются.
- Результат: 402 файла (194 + 88 + 24 + 96), 0 errors/failures/skipped.
  Набор файлов совпадает с прежним набором пропущенных проверок; исключения
  исходников не добавлялись. Сборка шести пакетов прошла, 14 затронутых CTest
  checks прошли, source architecture — 44 passed/2 installed checks deselected.
  Системные пакеты и `/opt/ros` не менялись. Сведения и XML сохранены в
  `/home/yaroslav/ros2_publication/2026-11-13/evidence/cppcheck/`.
- В `TemporaryFile` явно создаётся владеющий `std::filesystem::path` перед
  присваиванием: это сохраняет поведение и устраняет ложное предупреждение
  cppcheck 2.13 о времени жизни буфера. Existing ownership/move/cleanup tests
  прошли. GoogleTest-макросы распознаются через штатную library configuration.

## Исправления полного архитектурного прохода 2026-11-12

- **Статус:** `blocked` только по оставшейся runtime-приёмке Isaac. Код F01–F15
  реализован, пакетные проверки и Gazebo завершены. Пользователь разрешил весь
  объём командой «Тогда приступай к работе». Штатный NVIDIA loader требует
  самостоятельного принятия EULA пользователем; это действие не выполнялось.
  Общая приёмка не объявляется `done` до доступного запуска Isaac.
- **Основание:** `/tmp/manipulation-skill-audit-20261112-38najg/index.html`
  и `review.json`: F01–F15, десять P2 и пять P3. Это отдельная работа после
  завершённых ниже исправлений первого ревью; прежние изменения сохраняются.
- **Цель:** исправить подтверждённые нарушения входных контрактов, отмены,
  владения ресурсами и конфигурации, выделить две изолируемые policy; общий
  статус семейства остаётся `candidate`.
- **Объём и порядок:** независимые части выполняются параллельно:
  - core/Franka providers: безопасные timestamps монитора (F03), сохранение
    локального progress (F08), приватная target geometry policy (F11),
    свежесть/replay/эпоха detected observation (F13);
  - MoveIt: отмена до создания planner context (F02), типизированный эффект
    release (F09), приватная quiescence policy (F10), timestamps облака (F15);
  - perception/cuRobo: принадлежащие экземпляру рабочие файлы (F04), отсутствие
    изменений конфигурации без ввода в YOLO GUI (F05), часы TF buffer (F06),
    отклонение неизвестного collision checker (F07);
  - Franka runtime: pip bootstrap Isaac без старого личного binary path (F01),
    явный каталог dataset (F12), удержание лидера process group до последнего
    сигнала (F14). Основной исполнитель интегрирует изменения и документацию.
- **Контракты:** имена ROS executable/endpoints и wire messages/actions
  сохраняются. Один coordinator остаётся writer-ом сцены; отмена не создаёт
  whole-goal deadline. Shared models/datasets не становятся runtime-workspace.
  Старые личные defaults заменяются явным параметром/окружением либо корректным
  обнаружением установленного runtime, а не новым личным путём. Новые пакеты,
  миграция MoveIt/cuRobo, скачивание моделей/SDK и изменение физики не входят.
- **Приёмка:** различающие regression tests исходных дефектов и прямые unit-тесты
  выделенных policy; сборка затронутых пакетов и зависимостей из чистого Jazzy
  underlay; зарегистрированные пакетные проверки, source architecture gate,
  проверка diff независимым рецензентом. Сборка/install и runtime имеют одного
  владельца. Для изменённой физической цепочки — Gazebo smoke/active cancel;
  Isaac bootstrap проверяется на установленном pip-окружении, затем доступным
  минимальным runtime-сценарием без загрузки новых моделей/SDK. Непроведённые
  проверки отмечаются отдельно и не становятся PASS.
- **Условия остановки:** новый несовместимый публичный контракт, необходимость
  установки тяжёлых зависимостей или изменения внешнего ПО требуют отдельного
  решения; независимые разрешённые исправления продолжаются. Пользователь
  отдельно разрешил разбить готовые изменения на коммиты и выполнить push
  2026-11-12. Чужие процессы и изменения не затрагиваются.
- **Артефакты:** `/tmp/manipulation-fixes-20261112-vaH2N4/` хранит исходный
  снимок, результаты отдельных частей и итоговую проверку. Итоговый
  `index.html`, `validation-summary.json`, `implementation.patch` и
  `change-manifest.json` отделяют этот этап от исходных незакоммиченных правок.
- **Выполнено:** все 15 исправлений; приватные target/quiescence policies,
  bounded process ownership, private perception files/caches, validation stamps,
  replay/age policy, lossless UI controls и pip Isaac wiring. Перекрёстное ревью
  выявило и закрыло дополнительные края: nested scene waits, `/proc` snapshot,
  compatibility imports, cache copying под lock и сторонние cache writers.
- **Проверки:** чистая сборка 8 пакетов; зарегистрированные проверки 7 пакетов
  (287 CTest commands) и повторы затронутых тестов — без оставшихся ошибок.
  Одно устаревшее YAML-ожидание обновлено под `max_observation_age_sec`.
  Architecture gate: 44 PASS, 2 installed checks выполнены в пакетных наборах.
  Четыре cppcheck wrappers пропускают версию 2.13.0; отдельный принудительный
  cppcheck по 23 изменённым C++ файлам с googletest config прошёл.
- **Runtime:** `gazebo_smoke_v2` и `gazebo_active_cancel_v2` дали
  `evidence_status: passed`, `cleanup_errors: []` в
  `runtime/runs/gazebo_smoke_v2-20261112T144053Z-377800/runtime-result.v2.json`
  и `runtime/runs/gazebo_active_cancel_v2-20261112T144214Z-379636/runtime-result.v2.json`
  внутри каталога артефактов. Тестовые runtime-процессы завершены.
- **Ограничения:** Isaac package/root/stdlib и preflight проверены; реальный
  `SimulationApp` import дошёл до EULA и отказался продолжать. GPU inference и
  GPU cuRobo не запускались. Malformed `/clock` проверен в собственном health
  callback; внешний rclcpp TimeSource при `use_sim_time=True` отдельно не
  проверялся. Новые общие проценты coverage не измерялись; прежние значения
  относятся к предыдущему снимку. Подробности — `validation-summary.json`.
- **Следующий шаг:** после самостоятельного принятия NVIDIA EULA пользователем
  повторить bootstrap и минимальную Isaac runtime-приёмку с выбранным
  `FRANKA_ISAAC_PYTHON`; до этого не повышать статус семейства `candidate`.

## Исправления по ревью 2026-11-12

- **Статус:** `done`; реализация и предусмотренные проверки завершены
  2026-11-12 по прямому разрешению пользователя.
- **Цель:** устранить подтверждённые дефекты отмены, паузы, временных меток,
  аварийных результатов и жизненного цикла backend-процессов; закрепить их
  регрессионными проверками без изменения wire-контрактов messages/actions.
- **Объём:** `manipulation` (pause acknowledgement и сохранение результатов
  завершённых шагов при исключении), `manipulation_moveit` (физическое состояние
  после cancel, timestamp terminal evidence и cancel-capable обработчик
  `ExecuteTrajectory` вместо дефектного upstream MoveIt 2.12.4), `manipulation_perception`
  (temporal confirmation, геометрия и безопасное владение worker process group),
  `manipulation_curobo` (владение process group), `franka_motion` (подтверждение
  исполняемой команды контроллера перед runtime-проверкой cancel), затронутая
  документация.
- **Порядок:** независимые пакетные исправления выполняются параллельно;
  общая сборка/install, пакетные тесты и runtime-профили координируются одним
  исполнителем. Существующие пользовательские изменения сохраняются.
- **Приёмка:** воспроизведение исходных дефектов регрессионными тестами,
  успешная сборка затронутых пакетов и зависимостей, их пакетные проверки,
  source-only архитектурная проверка семейства; минимальные Gazebo smoke/cancel
  профили для изменений физического lifecycle. GPU-проверки выполняются только
  на уже подготовленном окружении, без скачивания моделей и SDK.
- **Границы:** общий статус остаётся `candidate`; неподтвердившиеся замечания
  не становятся правками. Commit/push разрешены отдельным запросом пользователя
  2026-11-12; изменения сторонних проектов не входят в задачу. Процент покрытия
  не выводится из числа тестов.
- **Результат:** cancel сохраняет подтверждённое состояние захвата; release без
  подтверждённого detach удерживает scene lease. Пауза подтверждается в feedback
  с продолжающимся renewal, исключения сохраняют результаты завершённых шагов.
  Terminal joint evidence отклоняет zero/future stamps, rollback и replay;
  старые samples не подтверждают остановку. YOLO учитывает один hit на кадр,
  pose adapter отбрасывает невалидную геометрию. Python и cuRobo удерживают PID
  лидера до последнего group signal и завершают оставшихся потомков.
- **Runtime-дефект, найденный усиленным тестом:** стандартный ExecuteTrajectory
  в установленном MoveIt 2.12.4 не прерывал команду контроллера. Локальный
  capability сохраняет action/endpoint, обслуживает exact cancel и публикует
  terminal result после stop/join. При неопределённой остановке сохраняет
  владение goal handle и карантин. Upstream capability отключён в обеих фабриках
  MoveGroup; системная установка MoveIt не менялась.
- **Уточнение замечания по сцене:** cancel до grasp освобождает lease после
  доказанного stop, но сохраняет физический объект в world. Старое требование
  probe удалить объект исправлено на обязательное присутствие без attachment;
  проверки lease, reconciliation и quarantine сохранены.
- **Проверки:** сборка семи пакетов семейства и `topic_based_ros2_control` из
  чистого Jazzy underlay прошла. Пакетные проверки всех семи пакетов завершены
  без failures/errors; затронутые MoveIt/Franka повторно проверены после замены
  capability. Source-only architecture: **44 passed, 2 deselected**; исключённые
  installed checks выполнены пакетными наборами. Новые дефекты закреплены
  регрессионными тестами и независимым ревью трёх подагентов.
- **Покрытие:** полный Python-набор perception — **516 passed**; измерено
  **89,1% строк** (6227/6989) и **78,7% ветвей** (1820/2312) библиотечного кода.
  Это не покрытие всего семейства; C++ coverage не измерялось. Штатный
  `ament_cppcheck` пропускает установленный cppcheck 2.13.0, поэтому восемь
  изменённых C++ production-файлов отдельно проверены с разрешённым запуском
  этой версии; ошибок нет. Остальные пакетные linters прошли.
- **Gazebo:** финальные `gazebo_active_cancel_v2` и `gazebo_smoke_v2` имеют
  `evidence_status=passed`, завершённую validation и пустой `cleanup_errors`.
  В cancel-логе подтверждены controller EXECUTING → exact cancel → controller
  cancel → PREEMPTED → terminal result → проверка сцены и lease.
- **Артефакты:** `/tmp/manipulation-quality-20261112-eixQeL/` — логи сборки,
  пакетных и регрессионных тестов, coverage, runtime-result.v2.json и снимок
  изменённого рабочего дерева. Evidence относится к этому рабочему дереву,
  включая существовавшие до задачи изменения, а не к чистому commit из HEAD.
  GPU-модели, полноценный cuRobo runtime, Isaac Sim и реальное оборудование в
  этой проверке не запускались; модели/SDK не скачивались.

## Выполнено

### Разделить общую реализацию и конкретный сценарий

- **Статус:** `done`.
- **Результат:** robot-independent контракты и реализация находятся в
  `manipulation*`, а Franka, ее модели, конфигурация и задачи — в
  `franka_motion`.
- **Критерий готовности:** generic-пакеты не зависят от `franka_motion`; граница
  проверяется архитектурными тестами.

### Сгруппировать примеры манипуляции

- **Статус:** `done`.
- **Результат:** все runtime-пакеты семейства лежат в этом каталоге без
  переименования ROS-пакетов и публичных контрактов.
- **Критерий готовности:** `colcon list` находит каждый пакет по прежнему имени,
  а пакетные сборки и быстрые тесты проходят после переноса.

## Этап 1. Один рекомендуемый путь

### Перевести перенос объектов на единый расширяемый workflow

- **Статус:** `done`.
- **Фактический результат (2026-07-23):** configured и detected сценарии
  переведены на один `TransferObjectSequenceV2` и общий C++ lifecycle с
  каталогом объектов, provider/strategy границами, goal-scoped PlanningScene
  lease, типизированными physical outcomes и двумя настраиваемыми timeout
  физических команд. V1 wire types, endpoint-ы и compatibility-пути удалены с
  зафиксированными inventory и migration evidence.
- **Проверка:** пакетные тесты `manipulation_msgs`, `manipulation`,
  `manipulation_moveit`, `manipulation_perception` и `franka_motion` прошли без
  ошибок; configured и GigaPose-detected round trip прошли в Gazebo и Isaac Sim,
  включая единый `use_sim_time`, ненулевые stamps/TF, terminal robot-state
  evidence и подтвержденные физические attach/detach в Isaac.
- **Проблема:** static и perception-resolved сценарии имеют разные публичные
  actions и частично дублируют lifecycle. Нижний `TransferObject` одновременно
  задает конкретный порядок стадий и MoveIt-реализацию, текущий
  `DetectedGraspProvider` смешивает получение наблюдения, TF, сцену и построение
  хвата, а конфигурация содержит много технических timeout/poll параметров.
  Из-за этого запрос несет уже известные системе детали, код трудно расширять
  на другие объекты, а YAML выглядит как неясное описание программы движения.
- **Ожидаемый результат:** один high-level action server принимает намерение
  перенести объект, сам получает актуальное наблюдение и кандидаты хвата, затем
  выполняет выбранную C++-стратегию. Configured и perception-сценарии отличаются
  только подключенными provider-ами и используют один lifecycle, scene
  ownership, pause, cancel, recovery, feedback и terminal result.

#### Согласованный high-level контракт

Один шаг запроса содержит только:

- `object_id` — идентификатор конкретного физического экземпляра;
- способ задания цели и соответствующую target pose/translation;
- `return_home`;
- `recovery_policy`.

В запросе нет source pose, grasp pose, геометрии, размеров, detector/provider ID,
`object_profile_id`, стратегии и общего timeout. `object_profile_id` сервер
однозначно получает из каталога по `object_id`; source pose приходит от
observation provider, grasp — от grasp provider, а стратегия выбирается при
сборке процесса. Клиент не повторяет сведения, которыми уже владеет система.

Существующие `TransferObjectSequence`, `PerceptionResolvedTransferSequence` и
`TransferObject` документируют общий срок всей цели, поэтому их нельзя молча
переосмыслить. Нужно добавить новый versioned action
`TransferObjectSequenceV2` с новым endpoint, перевести на него все repository
consumers, обновить contract evidence и migration notes, после чего удалить
старые типы, endpoint-ы и compatibility adapters. Нижний `TransferObjectV2` не
создается: единый сервер вызывает C++-стратегию напрямую через backend ports и
сам владеет cancel/pause/physical-timeout lifecycle.

Публичное сопоставление endpoint-ов фиксируется заранее:

- `/transfer_object_sequence` и `/perception_resolved_transfer_sequence`
  заменяются одним `/transfer_object_sequence_v2`;
- `/set_transfer_object_sequence_paused` и
  `/set_perception_resolved_transfer_sequence_paused` заменяются одним
  `/set_transfer_object_sequence_v2_paused`;
- `/franka_moveit_transfer_object_server/transfer_object` и его
  `set_task_paused` удаляются без ROS replacement: их роль выполняют внутренние
  C++ backend ports под lifecycle верхнеуровневого сервера.

Перед удалением v1 implementation обязана зафиксировать inventory всех
repository consumers и внешних removal gates. По согласованному контексту
внешние потребители не ожидаются; если аудит их обнаружит, удаление
останавливается и возвращается на отдельное согласование. При подтвержденном
нулевом inventory bridge и длительный период совместимости не нужны.

#### Границы ответственности

```text
TransferObjectSequenceV2 server
  -> ObjectCatalog
  -> ObservationProvider
  -> GraspProvider
  -> ResolvedTransferTask
  -> TransferExecutionStrategy
  -> backend ports / physical actions
```

- `manipulation` владеет robot-independent lifecycle, внутренними типами
  `ObjectObservation`, `GraspCandidate`, `ResolvedTransferTask`, неизменяемым
  снимком PlanningScene и C++-контрактами provider/strategy. Чистые контракты не
  владеют ROS node, TF buffer, MoveIt monitor или правом записи PlanningScene.
- `manipulation_moveit` реализует MoveIt-адаптеры и поставляемую стратегию
  `PickLiftPlaceStrategy`, сохраняя единственного writer-а PlanningScene и
  существующие evidence/recovery инварианты. Стратегия получает явные backend
  ports от composition root; внутренний ROS action между sequence server и
  стратегией отсутствует.
- `franka_motion` выбирает и собирает реализации, хранит Panda profile, catalog,
  provider presets и launch-сценарии. Его внутренние headers/targets не являются
  внешним C++ API и после inventory должны стать private.
- `ConfiguredObservationProvider` сохраняет пример без камеры с фиксированным
  наблюдением. Perception adapter выдает актуальное stamped-наблюдение. Оба пути
  работают через тот же action server; отдельного static action server нет.
- `GraspProvider` — обычный тестируемый C++-интерфейс. Он получает observation,
  object profile и неизменяемый read-only снимок PlanningScene с session/revision
  автора координатора, а возвращает ранжированные кандидаты с grasp/pregrasp
  данными, направлением подхода, раскрытием/усилием захвата, score и корреляцией
  со stamped-наблюдением. Текущий resident MTC становится одной реализацией и
  планирует на локальной копии снимка, но не пишет в авторитетную сцену.
  Отдельная ROS-нода или ROS action adapter появятся только при реальном внешнем
  процессном/GPU backend и в эту задачу не входят.
- `TransferExecutionStrategy` определяет алгоритм выполнения подготовленного
  задания. В production регистрируется только `pick_lift_place`; нестандартный
  transfer реализуется новым скомпилированным C++-классом через простую фабрику.
  `pluginlib`, dynamic loading, YAML DSL стадий и режим `strategy: none` не
  добавляются.

В `PickLiftPlaceStrategy` остаются порядок стадий, pause/cancel checkpoints,
attach/detach только после подтвержденного физического результата, recovery,
best-known object disposition и формирование typed result. Параметры конкретного
кандидата подхода принадлежат `GraspCandidate`, а не общей стратегии.

#### Каталог объектов

Новая schema каталога разделяет экземпляр и профиль:

- `objects.<object_id>.profile_id` связывает физический экземпляр с типом;
- profile поддерживает primitive, mesh и compound geometry;
- profile может хранить semantic frames, симметрии, допустимые области хвата,
  ограничения захвата и `grasp_profile_id`, если эти данные использует provider;
- текущая source pose не хранится в profile. Для сценария без камеры она лежит в
  конфигурации `ConfiguredObservationProvider` конкретного мира.

Контракт должен позволять одному серверу выбирать несколько экземпляров разных
профилей. В этой задаче обязательны schema/conversion tests для primitive, mesh
и compound и текущий runtime cube; новый detector, модель или физический demo
сложного объекта не требуются.

#### Целевая конфигурация

YAML выбирает уже реализованные компоненты и задает изменяемые значения, но не
описывает стадии программы. Целевая форма task profile:

```yaml
franka_transfer_object_sequence_server:
  ros__parameters:
    workflow.strategy_id: pick_lift_place
    workflow.observation_provider_id: detected
    workflow.grasp_provider_id: mtc
    workflow.object_catalog_file: package://franka_motion/config/tasks/object_catalog.yaml
    workflow.max_steps: 32

    pick_lift_place.lift_clearance_m: 0.25
    pick_lift_place.preplace_clearance_m: 0.12
    pick_lift_place.retreat_clearance_m: 0.25
    pick_lift_place.release_settle_sec: 0.3

    physical_actions.arm_motion_timeout_sec: 30.0
    physical_actions.gripper_command_timeout_sec: 3.0
```

Robot frames, groups, links, resources and home pose остаются в robot profile;
MoveIt pipeline, planner, scaling, tolerances и Cartesian-настройки — в backend
profile; MTC sampling/ranking — в grasp-provider profile; endpoints — в
deployment profile; геометрия — в object catalog. Обязательный внутренний
planning budget конкретного API не становится параметром задачи или backend
profile и сам по себе не завершает всю задачу. Если API завершает попытку по
этому бюджету, workflow повторяет только этот retryable planning result с
внутренним bounded backoff, свежей проверкой scene revision и stop token.
Invalid request/configuration, invalid start/goal state и другой явно
non-retryable typed result завершают задачу; бесконтрольного busy retry нет.
`release_settle_sec` является явной физической выдержкой, а не timeout.

В task/provider YAML не остаются semantic timeouts ожидания observation, TF,
health readiness, ROS discovery/response, PlanningScene convergence, grasp
generation или planning, а также настраиваемые poll/retry/heartbeat periods.
Такие ожидания продолжаются до результата, типизированной окончательной ошибки,
cancel или shutdown. Короткие внутренние wait slices допустимы только для
проверки stop token и не завершают задачу сами по себе.

Timeout физического действия начинает отсчет только после принятия конкретной
команды контроллером: отдельно для движения руки и команды захвата. Планирование,
пауза и получение данных этот budget не расходуют. Component-internal safety
watchdogs, freshness limits, lease TTL/renewal и обязательные ограничения
внешнего API не считаются task timeout: они остаются приватными или выводятся из
контракта компонента и не дают пользователю еще один способ настроить ожидание.
При неограниченных ожиданиях lease должен продлеваться goal-scoped механизмом;
перед каждым scene или physical side effect его валидность проверяется заново.

`use_sim_time` не хранится в task YAML. Один launch argument задает clock domain
сценария и передает одинаковое значение всем участвующим нодам. ROS stamps и TF
используют node clock; измерение длительности ожидания, отмена и safety bounds —
steady clock. Нода не должна узнавать clock domain путем runtime-копирования
параметра другой ноды.

#### Наблюдаемость ожиданий

- Action feedback остается типизированным источником текущей фазы и причины
  ожидания.
- Сервер пишет один `INFO` при входе в длительное ожидание, throttled `INFO` с
  steady elapsed во время ожидания и один итоговый лог при ready, cancel,
  shutdown или ошибке; каждый poll/callback не логируется.
- Лог различает observation, TF, grasp generation, scene/health readiness,
  planning и ROS endpoint, содержит goal/step/object и последнюю диагностику.
  Штатное долгое ожидание само по себе не является `WARN` или `ERROR`.
- Период throttling является внутренней константой, а не новым YAML-параметром.

#### Этапы выполнения

1. Добавить `TransferObjectSequenceV2` и contract tests до изменения consumers;
   зафиксировать package-version/changelog, consumer inventory, removal gates и
   удаление v1 без bridge как явную breaking migration.
2. Выделить ObjectCatalog, ObservationProvider, GraspProvider,
   ResolvedTransferTask и TransferExecutionStrategy с fake-реализациями для
   быстрых unit-тестов.
3. Перенести текущий алгоритм в `PickLiftPlaceStrategy`, подключенную напрямую к
   backend ports без внутреннего transfer action; разделить strategy, grasp,
   robot и MoveIt config и ввести только два task-configurable timeout физических
   команд.
4. Перевести configured и perception/MTC пути на один V2 action server; передать
   grasp provider неизменяемый scene snapshot, сделать все ожидания interruptible
   и добавить lease renewal, planning retry policy и logging.
5. Перевести launch/task clients на новые endpoint-ы и единый launch-owned
   `use_sim_time`.
6. После consumer/contract gates удалить старые action facades, static-box и
   perception-specific adapters, aliases, параметры, targets и случайно
   установленную C++ поверхность.
7. Обновить README, API-схемы и launch diagrams фактическим состоянием, затем
   выполнить package-specific и runtime-проверки.

#### Не входит

- вторая production-стратегия, YAML-язык произвольных стадий и dynamic plugins;
- отдельная ROS-нода grasp provider или новый ML/GPU grasp backend;
- новый perception detector/model и runtime-demo сложного объекта;
- изменение MoveIt Servo и отдельного cuRobo reference backend;
- selectable tabletop/workspace policy, result verifier, метрики точности,
  второй робот и автоматизация полного round trip из следующих этапов roadmap.

#### Проверки и критерии готовности

- В ROS graph есть один рекомендуемый V2 high-level transfer action и один
  lifecycle; static/configured и detected clients используют его же.
- Новый goal не содержит geometry, source pose, grasp pose,
  `object_profile_id`, provider/strategy ID или whole-goal timeout.
- Все repository consumers мигрированы; consumer/external-gate inventory
  подтверждает возможность удаления, старые wire types, endpoint-ы, aliases и
  compatibility targets удалены, historical contract evidence и migration
  record сохранены.
- Неизвестный `strategy_id` дает ошибку запуска, production registry содержит
  только `pick_lift_place`, а test strategy меняет состав стадий без изменения
  action server.
- Fake providers подтверждают stamped correlation, ranking, read-only scene
  revision и отсутствие ROS у чистого GraspProvider; catalog tests покрывают
  primitive, mesh, compound и два экземпляра разных профилей.
- Каждое нефизическое ожидание в тесте остается активным дольше прежнего лимита,
  затем успешно продолжается; отдельный cancel/shutdown test быстро завершает
  ожидание и доказывает отсутствие последующего side effect.
- Retryable planning-budget result повторяется с backoff и может завершиться
  успехом; non-retryable typed planning failure завершается без повторов, а
  cancel останавливает backoff и новую попытку.
- Timeout принятой arm/gripper команды дает typed failure и подтвержденный safe
  stop либо сохраняет ресурс в fail-closed состоянии. Нефизическая стадия не
  завершается только из-за elapsed time.
- Logging test проверяет `enter -> throttled waiting -> exit` без spam, а action
  feedback сообщает конкретную waiting phase и объект.
- Parameter inventory task/provider/backend profiles не содержит
  wait/compute/planning/transport timeout и poll/heartbeat параметров; отдельная
  allowlist допускает только два timeout физических команд. Freshness и safety
  watchdogs проверяются отдельно и не смешиваются с task timeout.
- Launch contract test подтверждает единый `use_sim_time` для всех нод
  сценария; Gazebo/Isaac smoke проверяет одинаковый параметр и корректные ROS
  stamps/TF.
- Отдельно собираются и тестируются `manipulation_msgs`, `manipulation`,
  `manipulation_moveit` и `franka_motion`; configured transfer и detected cube
  round trip проходят в Gazebo, detected round trip — в Isaac Sim.
- Family остается `candidate` до выполнения отдельных задач автоматизации E2E и
  метрик; эта архитектурная миграция сама по себе статус не повышает.

### Подключить cuRobo как необязательный планировщик

- **Статус:** `done`.
- **Фактический результат (2026-07-23):** cuRobo подключен как optional
  `PlannerManager` за общим `ArmPlanningPort`; он получает полный неизменяемый
  PlanningScene snapshot и не создает отдельный action, lifecycle или physical
  path. Legacy `CuRoboPlanPose`, bridge и compatibility-поверхность удалены;
  основной MoveIt-профиль не зависит от `manipulation_curobo`.
- **Проверка:** CPU/package/architecture тесты `manipulation_curobo` прошли без
  ошибок; opt-in smoke с локальным cuRobo/CUDA и caller-owned scene request
  построил непустую траекторию для world + attached geometry. Зависимости и
  модели автоматически не скачивались.
- **Зависимость:** выполнять после задачи «Перевести перенос объектов на единый
  расширяемый workflow», когда общий planning backend port и единый lifecycle
  уже существуют.
- **Проблема:** два package-qualified `CuRoboPlanPose` action с одинаковыми
  полями требуют bridge и создают отдельный путь рядом с общим workflow.
  Текущий reference action использует собственный статический world, а
  существующий MoveIt plugin не переносит в cuRobo полную авторитетную
  PlanningScene. Поэтому cuRobo пока нельзя просто выбрать вместо основного
  планировщика, а его удаление оставляет cuRobo-specific публичный контракт в
  generic-слое.
- **Ожидаемый результат:** cuRobo является необязательной reference-реализацией
  того же planning backend port, который использует `PickLiftPlaceStrategy`.
  Высокоуровневый запрос, выполнение траектории, управление захватом,
  PlanningScene ownership, cancel, recovery и physical timeout остаются общими
  и не знают о cuRobo.

#### Согласованная граница

```text
TransferObjectSequenceV2
  -> PickLiftPlaceStrategy
     -> ArmPlanningPort
        -> основной planner backend
        -> optional cuRobo adapter
     -> общие execution/gripper/scene ports
```

- Planner выбирается при запуске через backend profile; planner ID не попадает
  в `TransferObjectSequenceV2` goal.
- Основной integration path cuRobo — канонический MoveIt `PlannerManager`
  `manipulation_curobo/CuRoboPlanner`, подключенный за общим planning port.
  Deprecated alias `franka_motion/CuRoboPlanner` удаляется.
- cuRobo только строит траекторию. Он не отправляет команды контроллеру, не
  управляет захватом и не изменяет авторитетную PlanningScene.
- Adapter получает тот же неизменяемый scene snapshot и revision, что и другие
  backend-ы: актуальное состояние робота, collision objects, attached objects,
  ACM и необходимые frame transforms. Преобразование покрывает используемые
  стратегией primitive, mesh и compound geometry; движение с прикрепленным
  объектом не использует отдельный статический world как источник истины.
- Adapter реализует общий typed planning result, retryable/non-retryable
  классификацию и interruptible cancel/shutdown. Обязательный внутренний budget
  API не становится настраиваемым planning timeout и не завершает всю transfer
  goal сам по себе.
- `manipulation_curobo` остается явно optional/reference из-за проверенной
  non-commercial research/evaluation лицензии внешнего cuRobo. Generic-пакеты и
  основной Franka-путь не имеют обязательной build/runtime-зависимости от него.
  Если выбран cuRobo profile, но backend не установлен, запуск завершается с
  понятной ошибкой до принятия цели.

#### Удаляемые публичные и compatibility-пути

- Удаляются обе action definition `CuRoboPlanPose` из `manipulation_msgs` и
  `franka_motion`, endpoint `/plan_curobo_pose`, bridge, его внутренний endpoint
  и связанные compatibility wrappers/clients/tests.
- Отдельный cuRobo-specific или generic plan-only ROS action не создается.
  Публичный topic для выдачи рассчитанных траекторий также не добавляется.
  Если такой доступ понадобится позже, он проектируется под конкретного
  потребителя отдельной задачей.
- Repository consumer inventory и removal gate выполняются до удаления wire
  types. По согласованному контексту внешние потребители не ожидаются; если они
  обнаружатся, удаление останавливается для отдельного согласования.
- Единственным task-level ROS action для этого пути остается
  `/transfer_object_sequence_v2`; cuRobo не создает параллельный lifecycle.

#### Этапы выполнения

1. После появления общего planning port зафиксировать его capability/result и
   scene-snapshot contract, затем добавить fake adapter tests для cuRobo.
2. Подключить `manipulation_curobo/CuRoboPlanner` за этим port и передавать ему
   полный immutable PlanningScene snapshot вместо отдельного статического мира.
3. Привести cancel, shutdown и typed planning outcomes к общему lifecycle и
   planning retry policy.
4. Подключить cuRobo через optional backend profile и проверить, что основной
   planner собирается и запускается без пакета `manipulation_curobo`.
5. После consumer gate удалить оба `CuRoboPlanPose`, legacy bridge, прямые
   cuRobo execution demos, aliases и оставшиеся compatibility/config следы.
6. Обновить README, API/migration notes и launch diagrams фактической схемой и
   лицензионной границей.

#### Проверки и критерии готовности

- В исходниках и ROS graph нет `CuRoboPlanPose`, `/plan_curobo_pose` и legacy
  bridge; `TransferObjectSequenceV2` не содержит cuRobo-specific полей.
- Backend profile выбирает cuRobo без изменения strategy, task client,
  execution/gripper/scene ports или high-level action endpoint.
- Автоматические CPU-тесты с fake runtime проверяют выбор backend-а,
  request/result conversion, полный scene snapshot, primitive/mesh/compound и
  attached geometry, typed failures, retry policy, cancel и shutdown.
- Автоматический composition test проходит путь
  `TransferObjectSequenceV2 -> cuRobo adapter -> fake physical ports`, не
  создавая отдельный task lifecycle.
- Отдельные opt-in GPU integration tests используют настоящий cuRobo и проверяют
  построение траектории для репрезентативных scene/attached-object запросов.
  Полный cuRobo `pick -> lift -> place` round trip и ручная проверка не являются
  критериями этой задачи.
- Package/architecture tests подтверждают отсутствие обязательной зависимости
  generic-слоя и основного Franka profile от `manipulation_curobo`; отсутствие
  optional backend-а не ломает основной путь, а его явный выбор дает раннюю
  понятную ошибку.
- Отдельно собираются и тестируются затронутые пакеты; GPU-зависимости и cuRobo
  автоматически не скачиваются.

### Удалить устаревшие runtime-профили

- **Статус:** `done`.
- **Фактический результат (2026-07-28):** удалены `runtime_matrix.v1.json`,
  `runtime_matrix.v2.json`, обе schema v1/v2 и historical-тест; fail-closed
  поведение осталось за `test_unknown_schema_version_fails_closed`.
- **Проверка:** чистая пересборка и тесты `franka_motion` прошли (90/90
  сьютов); в репозитории и installed share только v3 matrix и schema.
- **Проблема:** `runtime_matrix.v1.json` ссылается на уже удаленные пакеты
  (`franka_motion_bringup`, `franka_motion_monitoring`, message-пакет
  `franka_motion_msgs`); `runtime_matrix.v2.json` таких ссылок не содержит, но
  собирает более старый набор профилей, чем `v3`. `runtime_matrix.schema.v1.json`
  и `.schema.v2.json` не читает ни один тест и ни один код. Все версии и обе
  схемы сейчас реально попадают в installed share, потому что
  `install(DIRECTORY config ...)` ставит `config/` целиком без фильтра.
- **Зачем:** старые профили дают ложное впечатление поддерживаемого пути и
  засоряют installed share нерабочими файлами.
- **Согласованное решение:** удалить `runtime_matrix.v1.json`,
  `runtime_matrix.v2.json`, `runtime_matrix.schema.v1.json` и
  `runtime_matrix.schema.v2.json`. Тест
  `test_v3_loader_rejects_historical_v1_and_v2_matrices` убрать как
  избыточный — fail-closed на неизвестной `schema_version` уже покрыт
  `test_unknown_schema_version_fails_closed`. CMake менять не нужно: после
  удаления файлов `install(DIRECTORY config ...)` перестанет их ставить
  автоматически.
- **Критерий готовности:** в репозитории и installed share остается только
  `runtime_matrix.v3.json` и `runtime_matrix.schema.v3.json`; loader,
  документация и тесты используют одну schema; актуальная matrix проходит
  валидацию.

### Удалить legacy прямой writer PlanningScene

- **Статус:** `done`.
- **Фактический результат (2026-07-28):** удалены `LegacyStaticTableSceneNode`,
  executable `static_table_scene_legacy_direct`, route-селектор и его тест,
  запись `legacy_direct_target` из public contract. Фасад `static_table_scene`
  всегда исполняет `static_table_scene_coordinated`; явный
  `use_scene_coordinator=false` завершается ранней понятной ошибкой.
- **Проверка:** чистая пересборка и тесты `franka_motion` прошли;
  `test_canonical_static_table_bootstrap_has_no_direct_writer` и обновленный
  contract-тест подтверждают отсутствие прямого writer-а; в installed
  `libexec` только coordinated executable.
- **Проблема:** `static_table_scene_legacy_direct` (`LegacyStaticTableSceneNode`,
  `legacy_static_table_scene_node.hpp/cpp`, `legacy_static_table_scene_direct.cpp`,
  ветка выбора в `static_table_scene.cpp`/`static_table_scene_route.hpp`) пишет
  стол в PlanningScene напрямую через `/apply_planning_scene`, в обход
  coordinator'а. Единственное место, которое запускает `static_table_scene`
  (`planning_scene.py`), жестко передает `use_scene_coordinator=true` — ветка
  `false` не выбирается нигде в системе. Существование ветки защищено только
  записью `static_table_scene_routing.legacy_direct_target` в
  `public_contract.json`/`test_public_compatibility_contract.py`.
- **Зачем:** прямой writer PlanningScene в обход coordinator'а противоречит уже
  принятому правилу этого проекта (см. «PlanningScene и состояние мира» в
  `CLAUDE.md`): поддерживаемый V2-путь не содержит такого writer-а, а
  compatibility escape hatch не должен возвращаться в новый код. Мертвый код
  без единого потребителя — не задел на будущее, а лишний груз.
- **Согласованное решение:** удалить `LegacyStaticTableSceneNode`,
  `legacy_static_table_scene_direct.cpp`, ветку выбора по
  `use_scene_coordinator` в `static_table_scene.cpp`/`static_table_scene_route.hpp`,
  соответствующий executable из `CMakeLists.txt`/install и запись
  `legacy_direct_target` из `public_contract.json` и
  `test_public_compatibility_contract.py`. Параметр `use_scene_coordinator` и
  единственный путь `static_table_scene_coordinated` остаются без изменений.
- **Критерий готовности:** `static_table_scene_legacy_direct` не собирается и
  не ставится; `/apply_planning_scene` вызывается только из coordinator'а;
  package-тесты `franka_motion` проходят без упоминаний legacy-ветки.

### Убрать неактуальную incremental-install очистку из CMake

- **Статус:** `done`.
- **Фактический результат (2026-07-28):** удалены все семь `install(CODE)`
  remove-блоков для путей давно завершенных миграций и мертвые
  `PATTERN ... EXCLUDE` несуществующих заголовков; оставлены только очистка
  стейл symlink перед установкой локальных asset-файлов и живой
  resource forwarder. Тесты, которые фиксировали текст удаленных блоков,
  переведены на реальные инварианты (отсутствие retired-поверхности в
  исходниках и в установленном дереве). Разовые стейл-артефакты старых
  установок удалены из локального install-дерева вручную.
- **Проверка:** incremental и чистая сборка `franka_motion` с полными
  тестами прошли; в CMake не осталось remove-блоков для путей, отсутствующих
  в текущих исходниках.
- **Проблема:** `FrankaMotionCompatibilityInstall.cmake` чистит конкретные
  файлы прошлых версий пакета (заголовки вроде `mtc_grasp_generator.hpp`,
  `curobo_planner.hpp`, executable вроде `franka_transfer_sequence_server`,
  legacy cuRobo action/bridge модули) при инкрементальной пересборке поверх
  старого install. Проверено: 13 из 15 таких путей уже не существуют в
  исходниках вообще.
- **Зачем:** для одного разработчика на одной машине чистая пересборка ничего
  не стоит, а эта логика — чистая накладная сложность. Проект также служит
  примером хорошей практики: мертвый migration-груз в CMake учит плохому
  всякого, кто скопирует пакет как шаблон.
- **Согласованное решение:** удалить `install(CODE ...)` блоки, чистящие
  пути, отсутствующие в текущих исходниках, из
  `FrankaMotionCompatibilityInstall.cmake` и основного `CMakeLists.txt`.
  Оставить только очистку артефактов, которые реально может оставить текущая
  версия (например, стейл symlink от `franka_motion_install_resource_forwarder`).
- **Критерий готовности:** CMake не содержит `remove`-блоков для путей,
  отсутствующих в текущих исходниках; чистая и повторная (incremental) сборка
  `franka_motion` проходят без ошибок.

## Этап 2. Следующие расширения workflow

### Использовать approach_direction для lift/preplace/retreat

- **Статус:** `done`.
- **Фактический результат (2026-07-28):** `make_waypoints` строит `lift`,
  `preplace` и `retreat` вдоль обратного нормированного `approach_direction`
  выбранного кандидата; place-сторона переиспользует вектор взятия, как и
  согласовано. Аварийный `recovery_retreat` осознанно оставлен вертикальным:
  после сбоя фактическая поза может не соответствовать approach-геометрии.
- **Проверка:** пакетные тесты `manipulation_moveit` прошли (541 кейс);
  новый `ClearanceWaypointsFollowReversedApproachDirection` подтверждает
  отвод вдоль `-approach` для бокового кандидата; для top-down профиля
  `(0,0,-1)` смещение математически совпадает с прежним вертикальным.
- **Проблема:** `pick_lift_place_strategy.cpp` считает точки `lift`,
  `preplace` и `retreat` функцией `offset_z` — фиксированным смещением по Z,
  независимо от `approach_direction` кандидата хвата. Направление подхода уже
  приходит в `GraspCandidate` и уже проверяется для пути pregrasp→grasp, но не
  используется для отвода.
- **Зачем:** любой grasp-provider profile с невертикальным
  `approach_direction` (не только текущий top-down) получит физически неверный
  отвод «всегда вверх».
- **Согласованное решение:** вычислять `lift` как смещение вдоль
  `-approach_direction`, а не вдоль мировой Z. Для `preplace`/`retreat` со
  стороны постановки, где отдельного approach_direction нет ни в запросе, ни в
  каталоге объектов, по умолчанию переиспользовать тот же вектор, что и при
  взятии — явное упрощающее допущение, а не новый источник данных. Второй
  production-стратегии, нового порта или policy для этого не требуется.
- **Критерий готовности:** unit-тест `pick_lift_place_strategy` подтверждает
  отвод/подвод вдоль обратного `approach_direction` для невертикального
  кандидата; поведение для текущего top-down профиля не меняется.

### Отклонено: выбираемая policy опорной поверхности

Первоначальная идея отдельного порта `SupportContextPolicy`
(tabletop/workspace-only, переключаемого конфигом) не подтвердилась при
разборе кода: ни в generic workflow, ни в `PickLiftPlaceStrategy` нет ни
одного места, которое жестко требует горизонтальную опору — единственное
реальное несоответствие устраняет задача выше. Строить порт под гипотетические
будущие опоры (полка, стек, объект в руке другого манипулятора, движущийся
объект) без единого реального сценария — преждевременная абстракция.
Возвращаться к идее только когда появится конкретный второй сценарий.

### Направление на будущее (без даты, не запланировано): cost-aware planning request

`ArmPlanningPort`/`ArmPlanningRequest` поддерживает только запрос «точная
поза» или «точные суставы» — ни для MoveIt/OMPL, ни для уже подключенного
cuRobo нет способа спросить «путь подешевле» или «безопасный локальный отвод
с учетом цены». При этом сама смена алгоритма планирования (sampling ↔
cost-based ↔ будущий нейросетевой/гибридный) уже не требует нового кода в этом
проекте: она происходит через собственный pluginlib `PlannerManager` MoveIt
(`planning_pipeline`/`planner_id` в конфиге), как уже сделано для cuRobo
(`manipulation_curobo/CuRoboPlanner`). Новый подход к планированию — это новый
пакет с тем же MoveIt-плагином или, если подход совсем не укладывается в форму
MoveIt «сцена+цель → траектория», новая реализация `IArmPlanningBackend`; в
обоих случаях без изменения `PickLiftPlaceStrategy` или задач выше по стеку.
Если позже понадобится, чтобы сама стратегия просила у планировщика не
точную цель, а что-то вроде «отойди подальше от препятствий», в
`ArmPlanningPort` потребуется новый вид запроса рядом с `kPose`/`kJoint` —
проектировать его стоит после того, как накопится понимание оптимизационного
планирования, а не заранее.

## Этап 3. Доказательства качества

### Автоматизировать полный detected round trip

- **Статус:** `done`.
- **Фактический результат (2026-07-28):** `detected_round_trip` добавлен в
  `EvidenceKind` и `runtime_matrix.schema.v3.json`; профиль
  `gazebo_detected_round_trip_v2` добавлен в `runtime_matrix.v3.json` по
  согласованной схеме (background table-camera bringup с
  `start_transfer_server:=true`, readiness perception + transfer action +
  health, validation `franka_detected_cube_round_trip.launch.py` с
  `repetitions: 5`, requirements как у `gazebo_gpu_perception_v2`, свой
  `ROS_DOMAIN_ID: 77`); `run_gazebo_evidence.sh` получил `--profile`.
  Профиль сразу выявил три реальных дефекта рекомендуемого пути, которые
  одиночные ручные прогоны не ловили; все три устранены:
  1. OMPL проверял коллизии грубее, чем `ValidateSolution` (дефолтная
     дискретизация ~0.5% extent), и изредка возвращал план сквозь одиночный
     octomap-воксель, который валидатор законно браковал
     (`INVALID_MOTION_PLAN` на retreat). Дискретизация планировщика прижата к
     разрешению октомапа (`longest_valid_segment_fraction: 0.001`,
     `maximum_waypoint_distance: 0.01`) и закреплена contract-тестом.
  2. Stereo flying pixels по силуэту руки попадали в octomap-облако
     одиночными вокселями в воздухе и блокировали/искривляли clearance-пути
     (подтверждено `/display_contacts`: `<octomap>=panda_hand` в воздухе).
     В `build_sensor_frame_cloud` добавлен voxel-density фильтр изолированных
     точек (внутренние константы, без нового task-параметра).
  3. Миллиметровый шум detected-позы по z «топил» покоящийся куб в
     collision-box стола: у шага с прикрепленным объектом стартовое состояние
     оказывалось в контакте `table<->cube` (подтверждено
     `/check_state_validity`, depth 0.1 мм), и планирование lift/recovery
     мгновенно падало (`PLANNING_FAILED`). В detected-провайдер добавлен
     опциональный `minimum_position_z_m` (в `detected_observation.yaml`
     значение 0.027 — тот же 2-мм зазор над столом, который configured-preset
     держит by construction).
- **Проверка:** пакетные тесты `franka_motion` (1287) и
  `manipulation_perception` (763) прошли; `gazebo_detected_round_trip_v2`
  прошел два полных прогона подряд (`evidence_status: passed`, в каждом 5/5
  detected transfer-and-return, чистый cleanup); `gazebo_smoke_v2` после
  уплотнения дискретизации прошел (`passed`) — основной configured-путь не
  деградировал.
- **Проблема:** камера, оценка позы и движение проверяются по частям; ручной
  успешный запуск не защищает от следующей регрессии. Существующий
  `gazebo_gpu_perception_v2` проверяет только корреляцию perception-топиков
  (`start_transfer_server:=false`, без движения); существующий `gazebo_soak_v2`
  повторяет обычный (без камеры) round trip. Никто их не соединял.
- **Зачем:** заявленная полная возможность должна проверяться как единая цепочка,
  а не по частям.
- **Согласованное решение:** добавить в `runtime_matrix.v3.json` только для
  Gazebo (Isaac — отдельным будущим шагом по образцу, когда понадобится) новый
  профиль `gazebo_detected_round_trip_v2`:
  - background — `franka_gazebo_moveit_table_camera.launch.py` с теми же
    GPU-perception аргументами, что и `gazebo_gpu_perception_v2`, но
    `start_transfer_server:=true`;
  - validation — `franka_detected_cube_round_trip.launch.py` с
    `repetitions: 5` (тот же паттерн, что уже использует `gazebo_soak_v2`,
    только меньше повторов — цикл с камерой и GPU-моделями заметно дороже);
  - readiness — perception readiness `gazebo_gpu_perception_v2` плюс готовность
    transfer action;
  - новое значение `evidence_kind` — `detected_round_trip` (добавить в enum
    `EvidenceKind` и в `runtime_matrix.schema.v3.json`): отдельная claim, не
    покрытая ни `gpu_perception` (нет движения), ни `simulator_soak` (нет
    камеры в контуре);
  - requirements — как у `gazebo_gpu_perception_v2` (`nvidia-gpu`,
    `isaac-ros-4-1`, `franka-perception-gpu`).
- **Критерий готовности:** профиль проходит повторно на чистом commit, 5 раз
  подряд перемещает и возвращает объект по детектированной позе и фиксирует
  environment/профиль/результат тем же evidence-механизмом, что и остальные
  профили matrix, без выдачи unit-тестов за runtime-доказательство.

### Сделать окружение perception воспроизводимым

- **Статус:** `done`.
- **Фактический результат (2026-07-28):** личные default-пути удалены из
  12 launch-аргументов внешних backend'ов (`gigapose_python`,
  `gigapose_repo_dir`, `gigapose_dinov2_repo_dir`, `gigapose_data_dir`,
  `megapose_python`, `megapose_data_dir`, `ess_engine_file_path`, четыре
  `foundationpose_*_file_path`, `yolo_python`): аргументы объявлены без
  default и стали обязательными, каждый description ссылается на раздел
  `franka_motion/models/README.md`. README моделей покрывает YOLO-venv,
  MegaPose, GigaPose (checkout + DINOv2 + структура data dir) и официальные
  процедуры Isaac ROS для ESS/FoundationPose; модели `models/*.pt` остаются в
  git. Профиль `gazebo_gpu_perception_v2` передает обязательные
  FoundationPose-аргументы токенами `FRANKA_GPU_FOUNDATIONPOSE_*` (сами файлы
  открывает только `mode:=foundationPose`). Попутно исправлен дефект
  `gpu_perception_evidence_probe.py`: `DiagnosticStatus.level` приходит по
  wire как ROS `byte` (bytes), и `int(status.level)` ронял пробу на первом же
  diagnostics-сообщении.
- **Проверка:** пакетные тесты `franka_motion` (1275) и
  `manipulation_perception` (762) прошли; запуск
  `franka_gazebo_moveit_table_camera.launch.py mode:=gigaPose` без аргументов
  завершается ранней понятной ошибкой с именем недостающего аргумента и
  ссылкой на README; полный профиль `gazebo_gpu_perception_v2` прошел
  (`evidence_status: passed`, evidence `pass` без ошибок, correlated sample,
  чистый cleanup).
- **Проблема:** пять launch-аргументов (`gigapose_python`, `gigapose_repo_dir`,
  `gigapose_dinov2_repo_dir`, `gigapose_data_dir`, `megapose_python`,
  `megapose_data_dir`, пути к ESS/FoundationPose моделям, `yolo_python`) имеют
  значением по умолчанию личные пути этой машины
  (`franka_motion_runtime/bringup/launch_parts/camera/*_arguments.py`). На
  чистом окружении запуск с этими backend'ами падает без объяснения, что нужно
  установить.
- **Зачем:** пример должен явно показывать внешние зависимости и способ их
  получения, а не полагаться на конкретную личную машину.
- **Согласованное решение:**
  - в перечисленных аргументах убрать личный default, заменить на
    `default_value=None` — штатный способ ROS 2 launch сделать аргумент
    обязательным (команда переспросит его или явно откажется запускаться); в
    `description` каждого добавить ссылку на раздел README с инструкцией;
  - расширить `franka_motion/models/README.md` разделами GigaPose (чекаут
    `nv-nguyen/gigaPose` + DINOv2 с GitHub, версии torch/CUDA) и
    ESS/FoundationPose (ссылка на официальную процедуру скачивания NVIDIA
    Isaac ROS, не переизобретать её) и коротким разделом про YOLO-venv
    (`ultralytics`/`torch`); существующий раздел MegaPose обобщить, убрав
    личный путь из текста;
  - файлы моделей (`franka_motion/models/*.pt`) остаются закоммиченными как
    есть: без них ничего не работает, переобучение заняло много часов и
    переделывать его не оправдано, альтернативного источника нет, а вес
    файлов не является проблемой для этого проекта. Прежняя формулировка «без
    добавления тяжелых моделей в git» была ошибочной для этого проекта и
    снимается.
- **Критерий готовности:** запуск с `mode:=gigaPose`/`megaPose`/ESS без
  обязательных аргументов явно требует их указать, а не падает молча;
  `franka_motion/models/README.md` покрывает реальными командами все четыре
  backend'а; `gpu_perception_evidence_probe.py` проходит при обязательных
  аргументах, направленных на реальную (в том числе заново созданную)
  установку.
