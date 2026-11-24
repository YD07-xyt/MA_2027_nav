# mpc

## 动力学模型与离散化

状态量 $x = [x, y, v_x, v_y]^T$（4 维：世界系位置与速度）

控制量 $u = [a_x, a_y]^T$（2 维：世界系加速度）

离散方式：采用前向欧拉法（Zero-Order Hold 近似）：$$x_{k+1} = A x_k + B u_k$$其中 $A$ 在位置-速度块为 $I_2 \cdot \Delta t$， $B$ 在速度-加速度块为 $I_2 \cdot \Delta t$。

## 决策变量与约束结构（init_solver）

约束矩阵 $A_{\text{lin}}$ 共包含四类约束：初始状态等式（4 行）： 
$x_0 = x_{\text{init}}$（每帧在 build_problem 中更新为 x0）。

动力学等式（ $4N$ 行）： $x_{k+1} - A x_k - B u_k = 0$（ $k = 0 \dots N-1$）。

速度硬约束（ $2N$ 行）：仅约束 $v_x, v_y$（索引 2, 3），且仅对 $k = 1 \dots N$ 生效（位置 $x, y$ 未做硬约束）。

控制量硬约束（ $2N$ 行）：约束 $a_x, a_y$ 在 [u_min, u_max] 内（ $k = 0 \dots N-1$）。


##  代价函数与每帧更新（init_solver & build_problem）
Hessian 矩阵 $H$（常数，在 init_solver 中一次性构建）： 
 
状态跟踪项 $\sum_{k=0}^{N} (x_k - x_{k,\text{ref}})^T Q_k (x_k - x_{k,\text{ref}})$（终端 $k=N$ 用 $Q_N$，其余用 $Q$）。  

控制输入项 $\sum_{k=0}^{N-1} (u_k - u_{k,\text{ref}})^T R (u_k - u_{k,\text{ref}})$。  

控制增量项 $\sum_{k=1}^{N-1} (u_k - u_{k-1})^T R_d (u_k - u_{k-1})$。  

梯度向量 $g$（每帧在 build_problem 中更新）：填入参考轨迹对应的线性项 $-2 Q_k x_{k,\text{ref}}$ 与 $-2 R u_{k,\text{ref}}$。


## 求解与容错检查（solve）
依次校验 x0 / t_now 有限性、参考轨迹有效性、QP 维度与上下界合法性（lb_ <= ub_）。

调用 solver_.updateGradient 与 solver_.updateBounds 后求解，并检查 ErrorExitFlag 以及 OsqpEigen::Status（允许 Solved 与 SolvedInaccurate）。

提取 u_cmd、整段预测状态/控制序列，并在返回前更新 param_.u_prev。