# PC 上位机 USB 部署指南

这份文档只说明当前 **Windows 上位机** 该怎么和你已经打通的 **MC02 USB CDC** 通信，并直接调用 `Isaac Lab play.py` 导出的 `policy.onnx`。

## 1. 当前上位机代码结构

当前上位机代码位于：

- `E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real\host_runtime`

其中与这次部署最相关的文件如下：

- `host_runtime\mc02_usb_protocol.py`
  作用：实现和 MC02 当前板端一致的 `A5 + CRC8 + cmd_id + payload + CRC16` 协议。
- `host_runtime\mc02_usb_transport.py`
  作用：基于 `pyserial` 封装虚拟串口读写。
- `host_runtime\mc02_bridge.py`
  作用：把板端 `0x0203 L1_SENSOR_STATE` 转成现有 `HostRuntime` 可以直接消费的观测结构。
- `host_runtime\cli.py`
  作用：命令行入口，包含监视、正弦测试、ONNX 策略闭环三种模式。

## 2. 先准备 Conda 环境

环境文件已经写好：

- `E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real\environment-host-runtime.yml`

自动安装脚本也已经写好：

- `E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real\scripts\setup_host_runtime_env.ps1`

在 PowerShell 中执行：

```powershell
cd E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real
.\scripts\setup_host_runtime_env.ps1
```

脚本会在项目目录下创建环境：

```text
E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real\.conda\cartpole-s2r-host
```

创建完成后激活：

```powershell
conda activate E:\RoboMaster\RL\isaac-lab\isaac-lab-circular-cartpole-sim2real\.conda\cartpole-s2r-host
```

## 3. 准备 ONNX 策略文件

你需要先在 Ubuntu 侧用 `play.py` 导出策略：

```bash
./isaaclab.sh -p scripts/rsl_rl/play.py \
  --task RK-Circular-Cartpole-S2R-L1-REAL-V0 \
  --checkpoint /absolute/path/to/checkpoint.pt \
  --headless
```

导出后通常会得到：

- `policy.onnx`
- `policy.pt`

把 `policy.onnx` 拷到 Windows 机器上任意你方便使用的位置即可。

## 4. 三个最常用的命令

### 4.1 只看板端有没有稳定发数据

```powershell
python -m host_runtime mc02-monitor --port COM7 --max-frames 20
```

如果链路正常，你会看到类似下面的 JSON 输出：

- `0x0201`：电机状态
- `0x0202`：磁编码器状态
- `0x0203`：L1 观测状态

如果你只想看 L1 观测帧：

```powershell
python -m host_runtime mc02-monitor --port COM7 --filter-cmd-id 0x0203
```

### 4.2 先不上策略，只做正弦位置增量测试

```powershell
python -m host_runtime mc02-sine ^
  --port COM7 ^
  --motor-id 1 ^
  --amplitude 0.08 ^
  --frequency-hz 0.5 ^
  --kp 3.0 ^
  --kd 0.2 ^
  --auto-enable ^
  --disable-on-exit
```

这一步的目的很明确：

- 先验证 `USB CDC -> PC -> MC02 -> DM4310` 整条控制链没问题
- 先验证位置增量命令的符号和量纲没问题
- 如果正弦测试都不稳定，就不要直接上 RL

### 4.3 直接运行 ONNX 策略

```powershell
python -m host_runtime mc02-policy ^
  --policy E:\RoboMaster\RL\models\policy.onnx ^
  --port COM7 ^
  --motor-id 1 ^
  --kp 3.0 ^
  --kd 0.2 ^
  --auto-enable ^
  --disable-on-exit ^
  --log E:\RoboMaster\RL\logs\mc02_l1_session.jsonl
```

含义如下：

- `mc02-policy`
  直接使用你现在板端 A5 USB 协议。
- `--policy`
  指向 Ubuntu 导出的 `policy.onnx`。
- `--motor-id`
  你当前要控制的达妙电机 ID。
- `--kp --kd`
  发送给板端位置增量控制命令的增益。
- `--auto-enable`
  启动时先发一帧电机使能命令。
- `--disable-on-exit`
  退出时自动失能，避免电机保持在危险状态。
- `--log`
  记录一份 JSONL 日志，后面可以离线回放和排查问题。

## 5. 当前协议字段是怎么映射到策略输入的

板端上报 `0x0203 L1_SENSOR_STATE` 的 payload 字段为：

- `base_pos_rad`
- `base_vel_rad_s`
- `flex_pos_rad`
- `flex_vel_rad_s`
- `motor_temp_c`
- `status_flags`
- `fault_flags`

上位机会在 `host_runtime\mc02_bridge.py` 中把它映射成 `HostRuntime` 当前使用的观测：

1. `base_pos_rad`
2. `flex1_pos_rad`
3. `base_vel_rad_s`
4. `flex1_vel_rad_s`
5. `last_action_delta_rad`

然后仍然保持 `history_length = 3` 的历史拼接方式，所以 **ONNX 模型输入格式不需要改**。

## 6. 策略输出是怎么发回板端的

`policy.onnx` 推理输出的第一个动作值会被解释成：

- `base_pos_delta_rad`

然后上位机会把它打包成：

- `0x0311 POSITION_DELTA_CONTROL`

对应 payload 字段为：

- `motor_id`
- `pos_delta_rad`
- `kp`
- `kd`

这正好对应你现在板端的位置增量控制命令格式。

## 7. 建议的联调顺序

建议严格按下面顺序来：

1. `mc02-monitor`
   先确认能稳定看到 `0x0203`
2. `mc02-sine`
   先确认非 RL 的位置增量控制链稳定
3. `mc02-policy`
   最后再接 `policy.onnx`

如果在第 2 步发现正弦测试方向是反的、增量量纲不对、或者 `kp/kd` 明显不合适，先回板端修正，不要直接继续第 3 步。

## 8. 额外说明

为了兼容之前已经存在的离线工具，这次没有删除旧的固定帧协议代码：

- `python -m host_runtime policy`
- `python -m host_runtime sine`

这两个旧命令仍然保留，但你当前这条真实硬件链路应该优先使用：

- `python -m host_runtime mc02-monitor`
- `python -m host_runtime mc02-sine`
- `python -m host_runtime mc02-policy`
