# Robot Custom Behaviour Executor

Bu paket, robotun özel Behavior Tree (BT) executor'ünü içerir.

## İçerik

- **monitor_bt_node**: Basit bir BT executor node'u. BT XML dosyasını yükler ve 10 Hz'de tick eder.

## Kullanım

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select robot_custom_behaviour_executor
source install/setup.bash
```

### Çalıştırma

```bash
ros2 launch robot_custom_behaviour_executor monitor_bt_launch.py
```

veya direkt node:

```bash
ros2 run robot_custom_behaviour_executor monitor_bt_node
```

## Sonraki Adımlar

- Custom BT node'ları eklenecek
- BT XML dosyası geliştirilecek
- Farklı executor stratejileri test edilecek
