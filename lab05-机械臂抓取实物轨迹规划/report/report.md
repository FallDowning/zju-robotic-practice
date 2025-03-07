## <center>机械臂抓取与搬运实验报告 </center>
### 实验目的
- 了解机械臂的规划规划方法
- 了解机械臂的逆运动学求解方法
- 学习对机械臂的搬运任务进行目标点的选取和轨迹规划
### 实验要求
- 编写程序控制ZJU-I型机械臂，实现木块抓取与搬运，具体流程为：
  - 机械臂从零位置启动，运行至起始区域
  - 启动真空吸爪，抓取起始区域内的木块，移动至A点（370，-90，115）
  - 移动过程中控制木块从A点沿直线路径运动至B点（288，-288，115）
  - 控制从B点到达目标区域，目标区域位置为机械臂1号关节旋转角度90°所在位置
  - 抓取第二个木块放置到目标区域，并堆叠在第一个模块上，二者姿态保持一致
- 保证机械臂和物块在运动过程中不与其他物体碰撞
- 机械臂各关节的运动速度和加速度应符合限制
- 机械臂各关节的速度和加速度应尽可能保证平滑
### 实验内容与原理
实验的总体内容为机械臂的抓取与搬运，具体实现过程为：
  - 在笛卡尔空间中规划出机械臂完成抓取与搬运任务需要达到的目标点，然后采用自己编写的逆运动学求解器将笛卡尔空间中的坐标点转换至关节空间（注：放置时如果直接到目标点，物块间、物块与地面可能会发生碰撞，因此我们规划了物块终点上方的中间点作为过渡）
  - 在抓取点和放置点，人为控制气泵或电磁阀，实现目标物块的吸取与放置
  - 对于AB两点间的规划采用直线规划，对于其他任意两点间的规划，采用五次多项式规划

**五次多项式规划：** 五次多项式共有六个参数，分别是$a_0, a_1, a_2, a_3, a_4, a_5$，通过给定的初始位置、初始速度、初始加速度、目标位置、目标速度、目标加速度，可以求解出这六个参数，从而得到一个五次多项式函数，该函数可以保证关节位置、速度、加速度的平滑性
  $$
    \begin{aligned}
        &\theta(t) = a_0 + a_1t + a_2t^2 + a_3t^3 + a_4t^4 + a_5t^5 \\
        &\dot{\theta}(t) = a_1 + 2a_2t + 3a_3t^2 + 4a_4t^3 + 5a_5t^4 \\
        &\ddot{\theta}(t) = 2a_2 + 6a_3t + 12a_4t^2 + 20a_5t^3
    \end{aligned}
  $$
约束条件：
    $$
        \begin{aligned}
            &\theta(0) = \theta_0, \dot{\theta}(0) = \dot{\theta}_0, \ddot{\theta}(0) = \ddot{\theta}_0 \\
            &\theta(T) = \theta_T, \dot{\theta}(T) = \dot{\theta}_T, \ddot{\theta}(T) = \ddot{\theta}_T
        \end{aligned}
    $$
在直线运动A点之前和B点之后的五次多项式轨迹规划，我们设定了初末速度，以保证机械臂的运动更加连续，具体措施为在笛卡尔空间中规划速度，然后用雅可比矩阵将其转化为关节空间的速度

**直线规划：** 在AB点的直线运动我们采用了直线规划的方法，其原理是通过给定的初始位置、目标位置、以及规定的时间，可以通过插值的方法，得到不同时刻物块在笛卡尔坐标系中的位置，然后再利用逆运动学求解器得到每个时刻机械臂的关节位置。
$$
    \begin{aligned}
        \mathbf{p}(t) &= \mathbf{p}_0 + \frac{\mathbf{p}_T - \mathbf{p}_0}{T}t \\
        \theta(t) &= \text{IK}(\mathbf{p}(t))
    \end{aligned}  
$$

###
### 主要仪器设备
- 仿真模型：ZJU-1机械臂
- 仿真语言：python


### 实验结果与分析
仿真运行的视频见附件