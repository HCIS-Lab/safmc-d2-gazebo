# SAFMC D2 Gazebo

Custom Gazebo systems for the SAFMC D2 competition, including a payload pickup mechanism using an electromagnet.

## Prerequisite

請先安裝以下必要的 library：

```sh
sudo apt install -y libgz-cmake4-dev libgz-plugin3-dev libgz-sim8-dev
```

## Build

```sh
colcon build
```

3. 確保在 world file 的 `<world>` 標籤下新增以下內容：

```xml
<plugin
  filename="libPayloadSystem.so"
  name="gz::sim::systems::PayloadSystem">
</plugin>
```

4. 設定環境變數

```sh
export GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/safmc-d2-gazebo/build/payload_system
```

## Run

```sh
source /workspace/safmc-d2-gazebo/install/setup.bash
gz sim <world file>
```
