<!-- Responsibility: Route each redistributed component to its actual license. -->

# Лицензии и происхождение

Авторский код проекта и его документация имеют лицензию [MIT](LICENSE), если
рядом с файлом или в уведомлениях его пакета не указаны иные условия.
Это не означает, что все ресурсы и внешние backends распространяются под MIT.

| Компонент | Условия и источник сведений |
|---|---|
| Franka USD, MoveIt-конфигурация, YOLO checkpoints | [franka_motion/THIRD_PARTY_NOTICES.md](franka_motion/THIRD_PARTY_NOTICES.md) |
| YOLO checkpoints | Встроенная маркировка AGPL-3.0; [модели и воспроизведение](franka_motion/models/README.md) |
| Исходники дообучения и dataset manifests | [Комплект обучения](training/README.md), происхождение и изменения в `training/provenance.yaml` |
| Внешние MegaPose, GigaPose, DINOv2 и Ultralytics | [perception notices](manipulation_perception/THIRD_PARTY_NOTICES.md) и закреплённые ревизии |
| Внешний cuRobo | [cuRobo notices](manipulation_curobo/THIRD_PARTY_NOTICES.md); условия зависят от версии |
| Isaac ROS и отдельно устанавливаемые NVIDIA runtimes/models | [Isaac ROS notices](manipulation_perception_isaac_ros/THIRD_PARTY_NOTICES.md) |

Для приложения, объединённого с YOLO, применяются соответствующие условия
AGPL-3.0. MIT отдельных самостоятельных файлов сохраняется и не отменяет
этих условий. Исходная поставка включает код приложения и комплект обучения;
ссылки на upstream-код и правила сохранения этого комплекта находятся в
[уведомлении моделей](franka_motion/models/THIRD_PARTY_NOTICES.md).

Версии, атрибуция и границы установленных фактов приведены в notices каждого
ресурса. Текущая готовность публикации зафиксирована в
[PUBLICATION.md](docs/PUBLICATION.md).
