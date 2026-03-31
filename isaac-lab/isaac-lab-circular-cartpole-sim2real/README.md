# 圆周倒立摆 Sim2Real

## 总览

该项目采用Isaac Lab，实现圆周倒立摆（一阶）的Sim 2 Real方案


**版本信息：**

- `Isaac Lab` 2.3.0
- `Isaac Sim` 5.1.0

## 注意事项

- 建议使用`scripts\rsl_rl`下的`play.py`以及`train.py`
- 选用RSL-RL的原因是`play.py`脚本会自动生成ONNX文件
- 核心代码位于`source\circular_cartpole_sim2real\circular_cartpole_sim2real\tasks\manager_based\circular_cartpole_sim2real`
- URDF文件位于`models\S2R_Sim_L1.SLDASM\urdf\S2R_Sim_L1.SLDASM.urdf`
- 具体讲解发布在B站：https://www.bilibili.com/video/BV1k2QDByEnv