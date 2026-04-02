# Circular Cartpole：文件职责与 Windows / Ubuntu 分工

这份文档用于快速回答两个问题：

1. 关键文件各自负责什么。
2. Windows 和 Ubuntu 分别该做什么，按什么顺序做。

> 说明：这里的“每一个文件”指**当前主流程会用到的关键文件**，不是仓库所有源码文件逐个罗列。

---

## 1. 关键文件地图（按职责分组）

## 1.1 合同与任务定义（训练/部署共同基线）

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/observation_contract.py`
  - 统一定义观测合同：`history_length=3`、`input_dim=15`、term 顺序。
  - 明确 `flex_pos` 语义（当前是 raw joint-relative 语义）。
  - Host Runtime 与 Isaac Lab 训练都应遵守它。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_v1.py`
  - 观测核对基线任务（L1-V1）。
  - 常用于先确认观测语义是否正确。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_real_v0.py`
  - 面向真实部署的训练任务（L1-REAL-V0）。
  - 在观测合同一致前提下，用于最终训练与导出。

## 1.2 Ubuntu 训练与核对脚本

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/scripts/check_observation_contract.py`
  - 逐步打印观测 bucket、raw/wrap 状态，检查维度和顺序。
  - 支持 `--jsonl` 落盘，便于和 Windows 侧日志比对。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/scripts/rsl_rl/train.py`
  - Isaac Lab 训练入口（RSL-RL）。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/scripts/rsl_rl/play.py`
  - 策略验证与导出入口。
  - 会导出 `policy.onnx` / `policy.pt`。

## 1.3 Windows 主机运行时（连板部署）

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/host_runtime/mc02_usb_protocol.py`
  - 定义 MC02 USB 帧结构与 payload（如 `0x0203` 的 `L1SensorStatePayload`）。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/host_runtime/mc02_bridge.py`
  - 把板端 payload 转为 HostRuntime 消费的 `SensorFrameV1`。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/host_runtime/history.py`
  - 把单帧特征拼接为 history，并按合同 flatten 成 15 维输入。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/host_runtime/runtime.py`
  - 推理主循环（build observation → policy.predict → action）。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/host_runtime/cli.py`
  - Windows 命令入口：`mc02-monitor` / `mc02-sine` / `mc02-policy`。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/host_runtime/policy.py`
  - ONNX / Torch 策略加载与推理后端封装。

## 1.4 说明文档（操作手册）

- `docs/UBUNTU_ISAAC_LAB_NEXT_STEPS.md`
  - Ubuntu 主线执行手册（先核对、再训练、再导出）。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/docs/OBSERVATION_CONTRACT_AND_UBUNTU_CHECK.md`
  - 观测合同与 Ubuntu 核对脚本速查。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/docs/PC_HOST_RUNTIME.md`
  - Windows Host Runtime 运行方式与关键命令。

- `isaac-lab/isaac-lab-circular-cartpole-sim2real/docs/PC上位机USB部署指南.md`
  - Windows 侧 USB 部署流程细节。

---

## 2. Windows 需要做什么（部署侧）

Windows 只做“板子联调 + ONNX 实机推理”，不在这里改训练合同。

1. 准备 HostRuntime 环境（conda / 依赖）。
2. 用 `mc02-monitor` 确认板端帧在正常上报（`0x0201/0x0202/0x0203`）。
3. 用 `mc02-sine` 先做无策略动作链路验证（电机使能、通讯、回包）。
4. 拿 Ubuntu 导出的 `policy.onnx`，用 `mc02-policy` 跑闭环。
5. 记录日志（观测、动作、状态字）用于回放和排障。

Windows 侧重点：

- 不改观测维度与顺序。
- 不单边改 `flex_pos` 语义（避免与训练错位）。

---

## 3. Ubuntu 需要做什么（训练侧）

Ubuntu 只做“合同核对 + 训练 + 导出”，不直接负责上板通讯。

1. 安装扩展到 Isaac Lab 环境（`pip install -e source/circular_cartpole_sim2real`）。
2. 跑 `check_observation_contract.py` 核对 `L1-V1` 观测语义。
3. 再核对 `L1-REAL-V0`，确认与基线合同一致。
4. 先做短训 smoke test（例如 20 iter）验证链路。
5. 再做正式训练（`L1-REAL-V0`）。
6. 用 `play.py` 指定 checkpoint 导出 `policy.onnx` / `policy.pt`。
7. 带回导出文件和训练元信息给 Windows。

Ubuntu 侧重点：

- 保持合同稳定：15 维、固定顺序、history=3。
- 如需改 `flex_pos` 语义，必须训练/部署两边一起改，不可单边改。

---

## 4. 两个平台的交接件（必须对齐）

Ubuntu → Windows 至少交接以下内容：

- `policy.onnx`
- `policy.pt`
- 对应 run 目录与 checkpoint 名称
- `params/env.yaml`
- `params/agent.yaml`
- 观测核对日志（jsonl，建议）
- 一句语义结论（例如：`flex_pos kept as raw joint-relative value`）

只要交接件不全，Windows 侧就不要直接上板闭环。

---

## 5. 推荐最短主线（便于执行）

1. Ubuntu：安装扩展 + 双任务观测核对。
2. Ubuntu：短训 → 正训 → `play.py` 导出 ONNX。
3. Windows：monitor → sine → mc02-policy。
4. 若行为异常：用日志回放定位是观测错位、动作限幅还是通讯问题。

---

## 6. 一句话版本

- Ubuntu 负责“训练和导出正确策略”。
- Windows 负责“把同一份合同下的策略安全跑在 MC02 上”。
- 任何观测语义变更都必须双端同步，否则必然错位。
