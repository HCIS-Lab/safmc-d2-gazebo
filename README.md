# SAFME D2 Gazebo

## Prerequisite

請先安裝以下必要的 library：

```sh
sudo apt install -y libgz-cmake4-dev libgz-plugin3-dev libgz-sim9-dev
```

## Build

1. 建立並進入 build 目錄

```sh
mkdir build
cd build
```

2. 編譯

```sh
cmake ..
make
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
export GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/safmc-d2-gazebo/build
```

## Run

```sh
gz sim <world file>
```
