# GigE Vision 热成像相机驱动 技术变更日志

### 版本：稳定性改造 v1.0
### 日期：2025-12-17

---

## 一、问题背景

基于 Aravis 0.8 + ROS 的 GigE Vision 驱动存在以下不稳定问题：
- 异常退出后再次启动偶发无法发现相机
- 同序列号双相机场景下 device list 扫描抖动
- 快速重启触发 discovery/cache 状态脏

---

## 二、核心改动

### 1. 统一资源清理机制
**文件**: `src/genicam_node.cpp`

- 新增 `cleanup_resources()` 函数，严格按顺序执行清理：
  1. 停止相机采集 (`arv_camera_stop_acquisition`)
  2. 等待 300ms 让 UDP/stream 线程退出
  3. 释放 stream 对象
  4. 释放 camera 对象
  5. 调用 `arv_shutdown()`
  6. 关闭 OpenCV 窗口

- 使用 `std::atomic<bool>` 和 `std::mutex` 防止重复清理

### 2. 多信号处理
**文件**: `src/genicam_node.cpp`

- 注册统一信号处理函数 `signal_handler()`
- 覆盖信号：
  - `SIGINT` (Ctrl+C)
  - `SIGHUP` (终端关闭)
  - `SIGTERM` (roslaunch stop / rosnode kill)

### 3. IP 直连模式（核心改进）
**文件**: `src/genicam_node.cpp`

- 新增 `camera_ip` 参数，直接通过 IP 地址打开相机
- 使用 `arv_camera_new(ip)` 绕过 device list 扫描
- 完全避免 Aravis 0.8 的 device manager 去重/cache 问题
- 稳定支持同序列号多相机同时运行

### 4. 失败重试机制
**文件**: `src/genicam_node.cpp`

- 设备打开支持可配置次数重试
- Stream 创建支持可配置次数重试
- 重试间隔可配置（默认 500ms）

### 5. 启动稳定化
**文件**: `src/genicam_node.cpp`

- 启动前添加 200ms 延时确保之前实例完全释放
- 设备扫描模式下每次扫描前增加延时

---

## 三、新增 ROS 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `camera_ip` | string | `""` | 相机 IP 地址（推荐使用） |
| `retry_count` | int | `5` | 连接失败重试次数 |
| `retry_delay_ms` | int | `500` | 重试间隔（毫秒） |

---

## 四、Launch 文件更新
**文件**: `launch/thermal.launch`

新增参数传递：
```xml
<arg name="camera_ip" default="" />
<arg name="retry_count" default="5" />
<arg name="retry_delay_ms" default="500" />
```

---

## 五、使用方式变更

**旧方式（不推荐）**：
```bash
roslaunch genicam thermal.launch identify_str:=192.168.4.16 ...
```

**新方式（推荐）**：
```bash
roslaunch genicam thermal.launch camera_ip:=192.168.4.16 ...
```

---

## 六、兼容性说明

- ✅ 保留 `identify_str` 参数兼容旧配置
- ✅ 不依赖 Aravis ≥0.9，仍使用 0.8
- ✅ 不修改图像处理/ROS topic 逻辑
- ✅ 不修改相机固件/序列号

---

## 七、验证结果

| 测试场景 | 结果 |
|----------|------|
| Ctrl+C 退出后重启 | ✅ 立即成功 |
| 关闭终端后重启 | ✅ 立即成功 |
| roslaunch stop 后重启 | ✅ 立即成功 |
| 快速停启（2秒间隔） | ✅ 稳定连接 |
| 连续多次重启 | ✅ 无 "not found" |
