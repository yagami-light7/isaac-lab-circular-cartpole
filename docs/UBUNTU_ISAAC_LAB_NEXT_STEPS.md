# Ubuntu Isaac Lab 下一步操作手册

这份文档是给你在 Ubuntu / Isaac Lab 环境里直接照着执行用的。

目标只有一个：

- 在 Ubuntu 上把观测语义彻底核对清楚
- 在不破坏当前 Windows host runtime 合同的前提下，训练并导出可部署的 `policy.onnx`
- 达到明确条件后，再回到 Windows 做 ONNX 部署

---

## 0. 先记住当前主线，不要在 Ubuntu 上随意改掉

当前已经在 Windows 侧收敛好的主线是：

- 控制频率：`250 Hz`
- 单帧策略特征顺序：`[base_pos, flex_pos, base_vel, flex_vel, last_action]`
- history 长度：`3`
- 策略输入维度：`15`
- history 展平方式：`term-major, oldest to newest`
- 当前默认合同里，`flex_pos` 仍然按“原始 joint-relative angle”走策略路径，没有正式切到 host-side unwrap
- 最终适合真实部署的训练任务：`RK-Circular-Cartpole-S2R-L1-REAL-V0`
- 观测核对基线任务：`RK-Circular-Cartpole-S2R-L1-V1`

这几点已经写进代码：

- 统一观测合同：`isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/observation_contract.py`
- Ubuntu 观测检查脚本：`isaac-lab/isaac-lab-circular-cartpole-sim2real/scripts/check_observation_contract.py`
- Ubuntu 观测检查简版说明：`isaac-lab/isaac-lab-circular-cartpole-sim2real/docs/OBSERVATION_CONTRACT_AND_UBUNTU_CHECK.md`

最重要的约束：

- 如果你在 Ubuntu 上确认 `flex_pos` 必须改成 unwrap，先不要继续训练部署版本，直接停止并回 Windows。
- 原因很简单：Windows 的 host runtime 当前也还是 raw relative 合同。Ubuntu 训练改了，Windows 部署不改，会直接错位。

---

## 1. 你在 Ubuntu 里应该先确定的 3 个路径

下面文档里会反复用到 3 个路径。

### 1.1 Isaac Lab 启动器根目录

就是那个包含 `isaaclab.sh` 的目录。

下面记作：

```bash
/home/light/workspace/IsaacLab
```

### 1.2 当前 sim2real 扩展根目录

就是这个仓库里扩展项目对应的 Ubuntu 路径：

```bash
/home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real
```

它对应 Windows 上的：

```text
E:\RoboMaster\RL\isaac-lab-circular-cartpole\isaac-lab\isaac-lab-circular-cartpole-sim2real
```

### 1.3 扩展 Python 包目录

```bash
/home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real
```

---

## 2. Ubuntu 里第一件事：把扩展装进 Isaac Lab Python 环境

进入扩展根目录：

```bash
cd /home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real
```

执行：

```bash
/home/light/workspace/IsaacLab/isaaclab.sh -p -m pip install -e source/circular_cartpole_sim2real
```

这一步的目标不是“训练”，只是保证下面这些脚本都能 import：

- `circular_cartpole_sim2real`
- `circular_cartpole_sim2real.tasks`
- `scripts/check_observation_contract.py`
- `scripts/rsl_rl/train.py`
- `scripts/rsl_rl/play.py`

### 2.1 做到什么程度算通过

只要后面的观测检查脚本能正常启动，不报下面这类错误，就算通过：

- `ModuleNotFoundError: circular_cartpole_sim2real`
- `Task ... not found`
- `ImportError` 与扩展包注册相关

如果这里失败：

- 不要改观测合同
- 先反复确认你执行的是 Isaac Lab 自己的 `isaaclab.sh`
- 再确认 `pip install -e source/circular_cartpole_sim2real` 是在 `/home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real` 下执行的

---

## 3. 第二件事：先核对观测语义，不要急着训练

先在 `L1-V1` 任务上核对，因为它是当前观测基线。

### 3.1 推荐的第一条检查命令

```bash
cd /home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real

/home/light/workspace/IsaacLab/isaaclab.sh -p scripts/check_observation_contract.py \
  --task RK-Circular-Cartpole-S2R-L1-V1 \
  --num_envs 1 \
  --steps 12 \
  --seed 42 \
  --disable-observation-corruption \
  --headless
```

如果你想看界面，可以去掉 `--headless`。

### 3.2 这条命令会打印什么

它会打印：

- 当前合同版本
- 当前 task 名
- `history_length`
- `policy_input_dim`
- 5 个 term 的 bucket
- `base_pos_raw`
- `flex_pos_raw`
- `flex_pos_wrap`
- `base_vel_raw`
- `flex_vel_raw`

### 3.3 这一步你要看什么

你只看下面几件事：

1. `obs_dim` 必须是 `15`
2. 每个 term bucket 必须正好是 `3` 个值
3. bucket 顺序必须是：
   - `base_pos`
   - `flex_pos`
   - `base_vel`
   - `flex_vel`
   - `last_action`
4. 在 `--disable-observation-corruption` 打开时，最新时刻的 bucket 数值应该和对应 raw 状态对得上
5. `flex_pos` 当前应当表现为“joint relative raw value”，不是 host-side unwrap 之后的连续量

### 3.4 如果这一步通过，下一步做什么

通过条件：

- 维度是 15
- 顺序没错
- `flex_pos` 的当前表现和“raw relative angle”一致

如果通过，继续做第 4 步。

### 3.5 如果这一步不通过，怎么处理

#### 情况 A：只是数值有轻微偏差

先确认你有没有加：

```bash
--disable-observation-corruption
```

如果没加，先重新跑。

#### 情况 B：顺序不对、维度不是 15、bucket 明显错位

不要训练。

直接停止 Ubuntu 工作，回 Windows。

#### 情况 C：你确认 `flex_pos` 必须改成 unwrap 才合理

也不要继续训练部署版。

直接停止 Ubuntu 工作，回 Windows。

原因：

- Windows host runtime 当前仍按 raw relative 合同拼 15 维输入
- 你在 Ubuntu 训练 unwrap 策略，再回 Windows 直接部署，观测一定错位

这是一条硬门槛。

---

## 4. 第三件事：在 `L1-REAL-V0` 上再核对一遍

你最终要部署的是 `RK-Circular-Cartpole-S2R-L1-REAL-V0`，所以必须确认它和 `L1-V1` 的观测合同一致，只是动作语义更贴近真实部署。

执行：

```bash
cd /home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real

/home/light/workspace/IsaacLab/isaaclab.sh -p scripts/check_observation_contract.py \
  --task RK-Circular-Cartpole-S2R-L1-REAL-V0 \
  --num_envs 1 \
  --steps 12 \
  --seed 42 \
  --disable-observation-corruption \
  --headless
```

### 4.1 你要确认什么

- 观测维度仍然是 `15`
- 观测顺序仍然完全一致
- `last_action` 仍然是第 5 个单帧特征
- 只是训练任务的动作定义变成了更适合真实部署的 relative position command

### 4.2 这一步通过以后，才能进入训练

如果 `L1-V1` 和 `L1-REAL-V0` 在观测上不一致，不要继续。

先回 Windows。

---

## 5. 第四件事：先做一个很短的训练烟雾测试

不要一上来就长训练。

先用 `L1-REAL-V0` 做一个短训练，确认整条训练链路是通的。

执行：

```bash
cd /home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real

/home/light/workspace/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task RK-Circular-Cartpole-S2R-L1-REAL-V0 \
  --num_envs 512 \
  --max_iterations 20 \
  --headless
```

### 5.1 这一步的目标

不是学出好策略。

只是确认：

- task 能启动
- RSL-RL 能训练
- 日志目录能创建
- 中间没有 task 配置 / observation / action 相关崩溃

### 5.2 日志会在哪里

默认会在：

```text
/home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/logs/rsl_rl/cartpole_s2r_l1_real_v0/<时间戳_run>
```

### 5.3 做到什么程度算通过

短训练能完整跑完，不崩。

如果 20 iteration 都跑不完：

- 不要直接改 reward / observation / action
- 先看是不是环境安装问题或显存问题
- 如果不是环境问题，再回 Windows

---

## 6. 第五件事：正式训练部署任务

确认前面都通过后，才开始正式训练。

推荐先直接训部署任务：

```bash
cd /home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real

/home/light/workspace/IsaacLab/isaaclab.sh -p scripts/rsl_rl/train.py \
  --task RK-Circular-Cartpole-S2R-L1-REAL-V0 \
  --num_envs 4096 \
  --max_iterations 2000 \
  --headless
```

如果你的 Ubuntu 机器显存不够：

- 先把 `--num_envs` 降到 `2048`
- 还不够再降到 `1024`

但是：

- 不要改 task 名
- 不要改 observation contract
- 不要在 Ubuntu 上临时改成 unwrap

### 6.1 训练期间你要记录什么

至少记录这 4 件事：

1. 最终使用的 task：
   - 必须写清楚是 `RK-Circular-Cartpole-S2R-L1-REAL-V0`
2. 最终使用的 run 目录：
   - 例如 `logs/rsl_rl/cartpole_s2r_l1_real_v0/2026-03-31_23-10-00`
3. 最后你认为最好的 checkpoint：
   - 例如 `model_1500.pt`
4. 观测语义结论：
   - 明确写一句：`flex_pos kept as raw joint-relative value, no unwrap`

### 6.2 什么叫“训练结果够资格进入导出”

至少满足下面两条：

1. 训练没有明显发散
2. 你在 play 阶段能看到策略已经有稳定的目标姿态控制趋势，或者已经能较稳定维持平衡

如果训练完全学不起来：

- 先不要导出
- 可以继续训练更久
- 但不要在 Ubuntu 上擅自改观测合同

---

## 7. 第六件事：用 `play.py` 验证并导出 ONNX

`play.py` 会自动导出：

- `policy.pt`
- `policy.onnx`

你需要显式指定 checkpoint。

### 7.1 先找到你要导出的 checkpoint

位置通常类似：

```text
/home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/logs/rsl_rl/cartpole_s2r_l1_real_v0/<run_name>/model_XXXX.pt
```

### 7.2 导出命令

```bash
cd /home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real

/home/light/workspace/IsaacLab/isaaclab.sh -p scripts/rsl_rl/play.py \
  --task RK-Circular-Cartpole-S2R-L1-REAL-V0 \
  --num_envs 1 \
   --checkpoint /home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/logs/rsl_rl/cartpole_s2r_l1_real_v0/<run_name>/model_XXXX.pt \
  --headless
```

如果你想看可视化效果，就去掉 `--headless`。

### 7.3 导出文件会在哪里

会导出到 checkpoint 所在 run 目录下的：

```text
exported/policy.pt
exported/policy.onnx
```

即类似：

```text
/home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/logs/rsl_rl/cartpole_s2r_l1_real_v0/<run_name>/exported/policy.onnx
```

### 7.4 这一步做到什么程度算通过

至少满足：

- `play.py` 能正常加载 checkpoint
- `exported/policy.onnx` 真实生成出来
- `exported/policy.pt` 也真实生成出来

如果 `play.py` 能跑但策略行为很差：

- 先不要回 Windows 部署
- 继续换 checkpoint 或继续训练

---

## 8. 第七件事：最好额外保存一份观测检查日志

建议在你最终确定的任务上，再存一份 JSONL 检查日志，后面回 Windows 对照会很方便。

### 8.1 对 `L1-V1` 存一份

```bash
cd /home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real

mkdir -p logs

/home/light/workspace/IsaacLab/isaaclab.sh -p scripts/check_observation_contract.py \
  --task RK-Circular-Cartpole-S2R-L1-V1 \
  --num_envs 1 \
  --steps 24 \
  --seed 42 \
  --disable-observation-corruption \
  --action-mode sine \
  --jsonl logs/obs_contract_l1_v1.jsonl \
  --headless
```

### 8.2 对 `L1-REAL-V0` 再存一份

```bash
cd /home/light/workspace/light_extension/isaac-lab-circular-cartpole-sim2real/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real

/home/light/workspace/IsaacLab/isaaclab.sh -p scripts/check_observation_contract.py \
  --task RK-Circular-Cartpole-S2R-L1-REAL-V0 \
  --num_envs 1 \
  --steps 24 \
  --seed 42 \
  --disable-observation-corruption \
  --action-mode sine \
  --jsonl logs/obs_contract_l1_real_v0.jsonl \
  --headless
```

---

## 9. 到什么程度，你就可以回到 Windows 做 ONNX 部署

只有下面 6 条都满足，才回 Windows：

1. 你已经确认观测维度是 `15`
2. 你已经确认观测顺序是：
   - `base_pos`
   - `flex_pos`
   - `base_vel`
   - `flex_vel`
   - `last_action`
3. 你已经明确确认：
   - `flex_pos` 仍按 raw joint-relative value 进入策略
   - 没有切到 unwrap
4. 你已经在 `RK-Circular-Cartpole-S2R-L1-REAL-V0` 上拿到一个你认可的 checkpoint
5. 你已经通过 `play.py` 生成了：
   - `policy.onnx`
   - `policy.pt`
6. 你已经记下并带回下面信息：
   - task 名
   - run 目录
   - checkpoint 文件名
   - exported 目录位置
   - 观测语义结论

只要这 6 条有任何一条不满足，就不要回 Windows 部署。

---

## 10. 回 Windows 时，你需要带回哪些文件

至少带回这几样：

1. `policy.onnx`
2. `policy.pt`
3. `params/env.yaml`
4. `params/agent.yaml`
5. `logs/obs_contract_l1_v1.jsonl`
6. `logs/obs_contract_l1_real_v0.jsonl`

其中最关键的是：

- `policy.onnx`
- 你对观测语义的最终结论

---

## 11. 明确的停止条件

出现下面任意一种情况，停止 Ubuntu 工作，回 Windows：

1. 你确认 `flex_pos` 必须改 unwrap
2. `L1-V1` 和 `L1-REAL-V0` 的观测合同不一致
3. 观测不是 15 维
4. 观测顺序不是 `[base_pos, flex_pos, base_vel, flex_vel, last_action]`
5. `play.py` 导不出 `policy.onnx`
6. 你虽然有 checkpoint，但不知道它对应哪个 task 或哪个 run

---

## 12. 最短可执行路径

如果你只想走最短主线，就按这个顺序：

1. `pip install -e source/circular_cartpole_sim2real`
2. 跑 `check_observation_contract.py` on `RK-Circular-Cartpole-S2R-L1-V1`
3. 跑 `check_observation_contract.py` on `RK-Circular-Cartpole-S2R-L1-REAL-V0`
4. 如果结论仍是 raw relative contract，不改代码
5. 短训 `RK-Circular-Cartpole-S2R-L1-REAL-V0`
6. 正式训练 `RK-Circular-Cartpole-S2R-L1-REAL-V0`
7. 用 `play.py` 指定 checkpoint 导出 `policy.onnx`
8. 带着 ONNX、checkpoint 信息、观测结论回 Windows

---

## 13. 你在 Ubuntu 上不要做的事

为了避免回 Windows 后部署直接错位，下面这些事情不要在 Ubuntu 上私自做：

- 不要把 `flex_pos` 改成 unwrap 后继续训练部署版
- 不要改 15 维输入顺序
- 不要改 history length
- 不要把部署任务切回 `L1-V1`
- 不要在没有记录 task / checkpoint / run 的情况下导出 ONNX

---

## 14. 一句话判断是否可以回 Windows

只有当你能明确说出下面这句话时，才回 Windows：

> 我已经在 Ubuntu 上确认策略输入仍是 15 维 raw-relative 合同，并在 `RK-Circular-Cartpole-S2R-L1-REAL-V0` 上拿到了可运行 checkpoint，`play.py` 已成功导出 `policy.onnx`。
