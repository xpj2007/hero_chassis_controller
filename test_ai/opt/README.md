# test_ai/opt —— 参数寻优工作区（只调参，不改控制器代码）

> 目标：在**不修改** `src/`、`include/`、`launch/` 的前提下，只改 `controllers.yaml` 参数，
> 按 main/mode0 + 新增多速度长测工况反复跑，给出 3~4 组不同权重的最优参数，写入
> `test_ai/最优参数.md`，并明确报告可达精度（**±0.01% 不可达**，原因见最终报告）。

## 1. 测试工况（`record_scenarios.py`，test-only）

| 场景 | 内容 | 仿真时长 |
|---|---|---|
| `opt_cal` | `static_a` 30 s；vx/vy 0.1/0.5/1.0/1.5/2.0/2.5 正反各 8 s + 停 2 s；wz 0.5/1.0/1.5/2.0 正反各 8 s + 停 3 s；`static_b` 30 s | **396 s** |
| `opt_static` | `static_a` 60 s → circle 25 s → `static_b` 60 s | **153 s** |
| `opt_yaw` | `static_a` 10 s；wz 0.5/1.0/1.5/2.0 正反各 8 s + 停 5 s | **122 s** |

原有的 `main`（434 s）/`mode0`（269 s）保留，用于最终全量验证。

## 2. 脚本

```bash
T=/home/ros/catkin_ws/src/hero_chassis_controller/test_ai
$T/opt/run_case.sh CASE SCENARIO [key=value ...]     # 生成参数副本 + 跑 + 出 metrics
$T/opt/opt_metrics.py DATA.csv OUT.json              # 分段指标/r,k 估计
$T/opt/opt_score.py results/*.json                   # W1~W4 评分
```

- `run_case.sh base opt_cal` → `opt/params/base.yaml`、`data/base.csv`、`opt/results/base.json`。
- 每次只覆盖指定参数，其余从当前 `config/controllers.yaml` 继承（`make_params.py`）。
- 所有输出在 `test_ai` 下；控制器代码不动。

## 3. 指标与权重

`opt_metrics.py` 输出：

- 静置：`static_a/b` GT/odom 位移、yaw、轮速和、PID 活跃；
- 停止：横向漂移、位移、yaw、自由轮；
- 运动：vx/vy 的 odom/GT 比、r_est；wz 的 odom/GT 比、yaw 比、k_est；
- 高速：1.5/2.0/2.5 的比值、实际跟踪、轮速上限；
- `static_b` 的 GT 与 odom 分开（GT 是 plant，odom 是观测量）。

`opt_score.py` 给了四个权重（越低越好）：

| 权重 | 重点 |
|---|---|
| `W1_static` | 静置漂移、yaw、停止横向、`static_b` odom |
| `W2_yaw` | 0.5/1.0/1.5/2.0 wz 的 yaw 比、circle、`static_b` yaw |
| `W3_speed` | 1.5/2.0/2.5 vx/vy 的比值 + 实际跟踪 |
| `W4_balanced` | 以上全部归一化等权 |

这些是**加权/Pareto**，不是“所有指标同时更优”；参数之间存在 trade-off（k 的 +0.5 vs -0.5、rest 的静置 vs 旋转停止、fc 的噪声 vs 滞后）。

## 4. 流程

1. **基线重复性**：当前参数跑 `opt_cal` 2~3 次，量出 run-to-run 噪声（决定“最佳”是否显著）。
2. **多速度 r/k**：从 `opt_cal` 的 vx/vy 分段 r_est、wz 分段 k_est 求单 r/单 k 的最优折中；按 W1/W2/W3 选候选（前向/横移/低速/高速）。
3. **rest/fc OFAT**：`opt_static`/`opt_yaw` 上逐个扫
   `pid_rest_enter/exit/debounce`、`pid_rest_rot_enter`、`odom_rest_enter/exit/debounce`、`low_pass_filter_fc`；
   每参数 2~3 值，记录 W1/W2，组合最优。
4. **组合 + 全量**：3~4 组候选跑 `main`+`mode0`（必要时加 `opt_cal`），按 W1~W4 给 Pareto 表。
5. **最终**：`test_ai/最优参数.md` 写 3~4 组参数、实测表、适用场景、可达精度、复现命令。

## 5. 可达精度（先写清楚）

- 静置 `static_a` 60 s：**0.5~0.9 mm** 量级；60 s/120 s 会随接触初值波动。
- +0.5 odom/GT：**0.03~0.1%**；-0.5/复合：**1~3%**；高速：**1~3%**。
- 横移 r：**~1%**（单 r 极限）。
- `static_b` 机体 yaw：**~10°**（GT plant，不是 odom）；最终航向 ~30°。
- **0.01% 不可能**：1 m 的 0.01% 是 0.1 mm，而静置漂移就 0.8 mm；1 rad 的 0.01% 是 0.01°，单 k/接触噪声做不到。
- 要 0.01% 只能改仿真/测量（理想无滑移、IMU/mocap、更高同步），不是调参。

## 6. 状态

- 基线 `opt_cal` 运行中/已完成；后续按第 4 节推进。
- 当前控制器：HEAD + 用户 rest/fc 改动 + `odom_rest_enter=1.20`、`pid_rest_rot_enter=0.3` 两处修复；本工作区**只改参数**。
