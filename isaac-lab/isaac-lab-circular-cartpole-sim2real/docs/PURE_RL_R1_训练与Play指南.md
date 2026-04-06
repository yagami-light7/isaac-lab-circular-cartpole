# PURE RL R1 训练与 Play 指南

## 1. 这份文档是干什么的

这份文档只对应当前新的纯强化学习重训任务：

- 任务名：`RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1`
- 任务定位：纯 RL 基线
- 命名含义：
  - `REAL`：面向实机部署链
  - `PURE-RL`：只做纯强化学习，不混入 LQR / Hybrid
  - `R1`：第一轮重训，不表示原项目里的姿态版本 `V1`

这份文档的目标是：

- 告诉你当前新任务是怎么注册的
- 告诉你怎么从头开始训练
- 告诉你怎么从同任务旧 checkpoint 继续训练
- 告诉你怎么 play / 导出 `policy.onnx`
- 所有命令都尽量写成可以直接复制粘贴的一行


## 2. 新任务现在是怎么注册的

当前新任务已经在代码里注册完成，不需要你手动再写额外注册命令。

关键文件如下：

- 任务注册入口：
  - [__init__.py](/E:/RoboMaster/RL/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/__init__.py)
- 任务环境配置：
  - [s2r_task_l1_real_pure_rl_r1.py](/E:/RoboMaster/RL/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_real_pure_rl_r1.py)
- PPO 配置：
  - [rsl_rl_ppo_cfg_l1_real_pure_rl_r1.py](/E:/RoboMaster/RL/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/agents/rsl_rl_ppo_cfg_l1_real_pure_rl_r1.py)

当前注册进去的 gym id 是：

```text
RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1
```


## 3. 训练前先记住两件事

### 3.1 当前这轮重训改了什么

这轮不是简单调权重，而是改了训练合同本身：

- 活动杆观测改成 `wrap` 语义
- `last_action` 改成上一拍真实执行的 `delta(rad)`
- 动作执行链显式加入：
  - `scale = 0.05`
  - `hard clip = 0.45`
  - `slew limit = 0.02`
- reward 改成更强调：
  - 近直立稳住
  - 小动作
  - 低速度
  - 持续保持
- reset curriculum 改成三阶段状态课程

### 3.2 能不能接着旧 checkpoint 训练

结论：

- **同一个新任务 `REAL-PURE-RL-R1` 的旧 checkpoint：可以继续训练**
- **旧任务 `REAL-V0`、`V0`、`V1`、以及我之前错误命名成 `REAL-V1` 的旧实验：不建议拿来正式续训**

原因很简单：观测语义、`last_action` 语义、动作执行模型都已经变了。即便形状还能对上，语义也已经不一样了。

如果你只是为了做一次热启动实验，可以试，但不要把它当正式论文结果。

### 3.3 Git 同步到 Ubuntu 的正确做法

这一步非常重要，因为你现在是：

- Windows 上改代码
- Ubuntu 上训练

所以你必须先把当前训练侧改动提交到 git，再到 Ubuntu 上拉取。

#### 3.3.1 先解释你刚才那个报错是什么意思

你刚才输入的是：

```powershell
git switch -c codex/pure-rl-r1git switch -c codex/pure-rl-r1
```

这不是两条命令，而是**两条命令粘在了一起**。

所以 git 会把它错误理解成：

- 新分支名：`codex/pure-rl-r1git`
- 起始引用：`switch`

于是就报：

```text
fatal: invalid reference: switch
```

这不代表仓库坏了，也不代表你的 git 不支持 `switch`。

当前机器的 git 版本是支持 `git switch` 的。

#### 3.3.2 这一步必须在哪个目录执行

**Git 命令要在整个仓库根目录执行，不是在 `isaac-lab-circular-cartpole-sim2real` 子目录执行。**

当前 git 根目录是：

```text
E:\RoboMaster\RL\isaac-lab-circular-cartpole
```

所以 Windows 里先执行：

```powershell
cd E:\RoboMaster\RL\isaac-lab-circular-cartpole
```

#### 3.3.3 Windows 这边创建分支并提交

下面这些命令请**一行一条执行**，不要一次性乱粘成一行。

先切到仓库根目录：

```powershell
cd E:\RoboMaster\RL\isaac-lab-circular-cartpole
```

创建并切换到新分支：

```powershell
git switch -c codex/pure-rl-r1
```

如果提示这个分支已经存在，就改用：

```powershell
git switch codex/pure-rl-r1
```

然后只添加这次 pure-RL-R1 训练需要的最小文件集：

```powershell
git add isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/__init__.py
```

```powershell
git add isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/mdp/actions.py
```

```powershell
git add isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/mdp/events.py
```

```powershell
git add isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/mdp/obversations.py
```

```powershell
git add isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/mdp/rewards.py
```

```powershell
git add isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/s2r_task_l1_real_pure_rl_r1.py
```

```powershell
git add isaac-lab/isaac-lab-circular-cartpole-sim2real/source/circular_cartpole_sim2real/circular_cartpole_sim2real/tasks/manager_based/circular_cartpole_sim2real/agents/rsl_rl_ppo_cfg_l1_real_pure_rl_r1.py
```

```powershell
git add isaac-lab/isaac-lab-circular-cartpole-sim2real/docs/PURE_RL_R1_训练与Play指南.md
```

看一下暂存区是否正确：

```powershell
git status
```

确认没有把不相关的 Windows / MC02 / host runtime 文件误加进去后，再提交：

```powershell
git commit -m "Add PURE RL R1 retraining task"
```

推送到远端：

```powershell
git push -u origin codex/pure-rl-r1
```

#### 3.3.4 Ubuntu 这边拉取分支

进入 Ubuntu 上同一个仓库根目录：

```bash
cd /path/to/isaac-lab-circular-cartpole
```

然后执行：

```bash
git fetch origin
```

如果 Ubuntu 上还没有这个分支：

```bash
git switch -c codex/pure-rl-r1 --track origin/codex/pure-rl-r1
```

如果 Ubuntu 上已经有这个分支：

```bash
git switch codex/pure-rl-r1
```

然后拉取最新提交：

```bash
git pull --ff-only
```

#### 3.3.5 Git 同步完成后，再进入训练目录

Git 拉取完以后，真正训练脚本执行目录还是：

```bash
cd /path/to/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real
```

后面的训练、继续训练、play、导出命令，都是在这个子目录执行。


## 4. 命令默认在哪个目录执行

下面所有命令默认都在 `isaac-lab-circular-cartpole-sim2real` 根目录执行。

也就是先进入这个目录：

```bash
cd /path/to/isaac-lab-circular-cartpole/isaac-lab/isaac-lab-circular-cartpole-sim2real
```

如果你已经在这个目录里，就直接执行后面的命令。


## 5. 从头开始训练

这是最推荐的第一条命令。

```bash
./isaaclab.sh -p scripts/rsl_rl/train.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless
```

如果你想指定随机种子：

```bash
./isaaclab.sh -p scripts/rsl_rl/train.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --seed 42
```

如果你想先减少环境数做一次快速试跑：

```bash
./isaaclab.sh -p scripts/rsl_rl/train.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --num_envs 512 --max_iterations 200
```


## 6. 从同任务旧 checkpoint 继续训练

### 6.1 先看有哪些 run

先列出当前实验目录：

```bash
ls -lah logs/rsl_rl/cartpole_s2r_l1_real_pure_rl_r1
```

你会看到类似这样的 run 文件夹：

```text
2026-04-04_13-20-11
2026-04-04_13-20-11_mytest
```

### 6.2 从某个 run 的指定 checkpoint 继续训练

假设你要从：

- run 文件夹：`2026-04-04_13-20-11`
- checkpoint：`model_3000.pt`

继续训练，那么直接执行：

```bash
./isaaclab.sh -p scripts/rsl_rl/train.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --resume --load_run 2026-04-04_13-20-11 --checkpoint model_3000.pt
```

### 6.3 从某个 run 的“默认最新 checkpoint”继续训练

如果你只想恢复某个 run，但不想手动写模型编号，可以先去 run 目录里看一下文件名：

```bash
ls -lah logs/rsl_rl/cartpole_s2r_l1_real_pure_rl_r1/2026-04-04_13-20-11
```

然后再把想恢复的 `model_xxxx.pt` 写进上面的命令。

说明：

- `train.py` 的恢复训练路径是按 `load_run + checkpoint文件名` 组合解析的
- 所以恢复训练时最稳妥的写法就是：
  - `--resume`
  - `--load_run <run_folder>`
  - `--checkpoint model_xxxx.pt`


## 7. Play 最新 checkpoint

### 7.1 从最新 run 自动选择 checkpoint 进行 play

如果你不写 `--load_run` 和 `--checkpoint`，脚本会去当前实验目录里找默认 checkpoint。

```bash
./isaaclab.sh -p scripts/rsl_rl/play.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --num_envs 1
```

### 7.2 从指定 run 自动选最高步数 checkpoint 进行 play

这个更推荐，因为更可控。

```bash
./isaaclab.sh -p scripts/rsl_rl/play.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --num_envs 1 --load_run 2026-04-04_13-20-11
```

说明：

- `play.py` 在没有显式指定 `--checkpoint` 时，会在对应 run 目录下优先选择最高编号的 `model_*.pt`

### 7.3 从指定 checkpoint 进行 play

如果你想明确指定某一个模型文件：

```bash
./isaaclab.sh -p scripts/rsl_rl/play.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --num_envs 1 --checkpoint logs/rsl_rl/cartpole_s2r_l1_real_pure_rl_r1/2026-04-04_13-20-11/model_3000.pt
```


## 8. 只为了导出 `policy.onnx`，怎么做

`play.py` 在加载模型后，会自动导出：

- `policy.pt`
- `policy.onnx`

导出目录是对应 run 文件夹下的：

```text
exported/
```

如果你只是想导出，不想长时间运行仿真，可以用很小的步数：

```bash
./isaaclab.sh -p scripts/rsl_rl/play.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --num_envs 1 --load_run 2026-04-04_13-20-11 --max_steps 1
```

如果你要从指定 checkpoint 导出：

```bash
./isaaclab.sh -p scripts/rsl_rl/play.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --num_envs 1 --checkpoint logs/rsl_rl/cartpole_s2r_l1_real_pure_rl_r1/2026-04-04_13-20-11/model_3000.pt --max_steps 1
```


## 9. 导出后的文件在哪

假设你 play 的模型来自：

```text
logs/rsl_rl/cartpole_s2r_l1_real_pure_rl_r1/2026-04-04_13-20-11/model_3000.pt
```

那么导出结果会在：

```text
logs/rsl_rl/cartpole_s2r_l1_real_pure_rl_r1/2026-04-04_13-20-11/exported/policy.onnx
logs/rsl_rl/cartpole_s2r_l1_real_pure_rl_r1/2026-04-04_13-20-11/exported/policy.pt
```


## 10. 当前最重要的注意事项

### 10.1 新任务和旧任务不要混用

当前这套重训任务是：

```text
RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1
```

不要把它和下面这些任务混着恢复训练：

- `RK-Circular-Cartpole-S2R-L1-V0`
- `RK-Circular-Cartpole-S2R-L1-V1`
- `RK-Circular-Cartpole-S2R-L1-REAL-V0`

### 10.2 旧 pure RL 部署基线还在

你现在 Windows 上能跑的旧 pure RL 部署链，不会因为这份训练文档自动改变。

也就是说：

- 这份文档负责的是 **Ubuntu 新训练任务**
- 不等于 Windows host runtime 已经自动切换到新训练合同

如果后面你要部署这个新训练出来的 `REAL-PURE-RL-R1` 模型，还需要再单独核对一次部署侧观测合同。


## 11. 目前最常用的四条命令

### 11.1 从头训练

```bash
./isaaclab.sh -p scripts/rsl_rl/train.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless
```

### 11.2 继续训练

```bash
./isaaclab.sh -p scripts/rsl_rl/train.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --resume --load_run 2026-04-04_13-20-11 --checkpoint model_3000.pt
```

### 11.3 Play 最新模型

```bash
./isaaclab.sh -p scripts/rsl_rl/play.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --num_envs 1 --load_run 2026-04-04_13-20-11
```

### 11.4 只导出 onnx

```bash
./isaaclab.sh -p scripts/rsl_rl/play.py --task RK-Circular-Cartpole-S2R-L1-REAL-PURE-RL-R1 --headless --num_envs 1 --load_run 2026-04-04_13-20-11 --max_steps 1
```
