# [LiteratureReview]EAO-SLAM: Monocular Semi-Dense Object SLAM Based on Ensemble Data Association

出处：2020IROS，（截止到2022-4-4）目前 Google scholer被引15次；吴艳敏大佬的文章（东北大学硕士，现北大博士在读）；

Video：[https://www.bilibili.com/video/BV15E411u7za](https://www.bilibili.com/video/BV15E411u7za)

Code：[https://github.com/yanmin-wu/EAO-SLAM](https://github.com/yanmin-wu/EAO-SLAM)

## Introduction

如图1所示，半稠密地图中使用cubes 或者 quadrics 表示object以及其locations、poses和scales ；

![EA0SLAM_1](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_1.png)

> 图1：轻量级和面向对象的语义地图

Object SLAM 的挑战主要来自两个方面：

- 现有的数据关联方法 [5]-[7] 对于处理包含多个对象实例的复杂环境并不稳健或不准确；
- 物体poses估计不准确，尤其是对于monocular object SLAM SLAM；

EAO-SLAM主要贡献：

- 提出了一种ensemble data association strategy (集成数据关联策略)，可以有效地聚合对象的不同测量值以提高关联精度；
- 提出了一个基于iForest的object pose estimation framework，它对outliers具有鲁棒性，可以准确估计物体的位置、poses和scales ；
- 系统会构建轻量级和object-oriented（通过cubes 或者 quadrics 表示object）的半稠密语义地图；

## Related work

#### A. Data Association

Data association定义：用于判断当前帧中观察到的object是否为地图中已有的object（感觉这和描述子的意思差不多，但是这种方法是否适用于动态物体？）；

相关工作如下：

- Bowman et al. [5] ：使用概率方法对数据关联过程进行建模，并利用 EM 算法来查找观察到的地标之间的对应关系；
- 随后[7]、[11] 进一步扩展了该想法以关联动态对象或进行语义稠密重建；这些方法可以实现很高的关联精度，但只能处理有限数量的对象实例；
- Object tracking是Data association中另一种常用的方法；[13] 提出将 3D 立方体投影到图像平面，然后利用Hungarian tracking算法使用投影的 2D 边界框进行关联，基于跟踪的方法运行时效率高，但在复杂环境中很容易生成不正确的先验，从而产生不正确的关联结果；
-  Liu et al. [14]：提出了random walk descriptors 来表示对象之间的拓扑关系，将共享描述符数量最多的那些视为同一实例；
- Cube-slam[8] 直接计算检测到的object上匹配的地图点的数量作为关联标准，从而产生非常有效的性能；
-  Grinvald et al. [2] ：提出测量语义标签之间的相似性；Ok et al. [3] 提出利用色调饱和度直方图的相关性；这些方法的主要缺点是设计的特征或描述符通常不够通用或不够鲁棒，很容易导致不正确的关联；
- Weng et al. [15]：首次提出语义数据关联的非参数统计检验，可以解决统计不服从高斯分布的问题；

-  Iqbal et al. [6]：随后验证了非参数数据关联的有效性；然而，这种方法不能有效地解决遵循高斯分布的统计数据，因此不能充分利用 SLAM 中的不同测量；

总结：基于上述观察，我们将参数和非参数方法结合起来进行模型集成，在存在多类别对象的复杂场景中表现出优异的关联性能；

#### B. Object SLAM

相关工作如下：

-  [15]、[18]、[19] 将objects视为landmarks以estimate camera poses or for relocalization[13]；
- 一些工作 [20] 利用object size来约束单目 SLAM 的scale，用来提高位姿估计的精度；
- 一些工作 [7]、[21]通过去除动态object以提高位姿估计的精度；
- 最近object SLAM 与抓取的结合 [22] 也引起了很多兴趣，促进了自主移动操纵的研究；

- Object models方面：可以大致分为instance-level models, category-specific models, and general models；instance-level models [9]、[23] （6 Dof形式）依赖于记录所有相关object的完善数据库，object的先验信息为**图优化提供了重要的对象-相机约束**，由于需要提前知道模型，因此此类方法的**应用场景有限**； category-specific models侧重于描述category-level 的特征；**general models**采用简单的几何元素，例如cubes[8]、[13]、quadrics[18]和圆柱体[10]来表示object，这也是最常用的模型；

- 在相机和物体姿态的联合优化方面：Frost et al. [20] 简单地将object质心作为点云集成到相机位姿估计过程中；**Cubeslam**[8]提出了一种联合camera-object-point 优化方案来构建用于图优化的位姿和scale约束；**Quadricslam**[18] 将二次曲线投影到图像平面上，然后计算投影的二维矩形和检测到的边界框之间的比例误差；

总结：本工作同样采用联合优化策略，但采用了新颖的初始化方法，可以显着提高解的最优性；

## Methods

EAO-SLAM概述：EAO-SLAM framework如图2所示，其建立在ORB-SLAM2 的基础上，另外还集成了一个 YOLOv3 目标检测线程；ensemble data association 在tracking thread中实现，它结合了边界框、语义标签和点云的信息；之后，利用 iForest 消除异常值，为联合优化过程找到准确的初始化；然后将object pose和scale与相机pose一起优化，以构建轻量级和面向对象的地图；在semi-dense mapping thread中，object map与[25]生成的semi-dense map相结合，得到semi-dense semantic map；![EA0SLAM_2](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_2.png)

> 图2：EAO-SLAM系统的架构，这项工作的主要贡献用红色突出显示

#### ENSEMBLE DATA ASSOCIATION

A. Nonparametric Test：这里使用非高斯点云构造了一个高斯统计；



![EA0SLAM_3](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_3.png)

> 图3：用于数据关联的不同类型的统计

B. Single-sample and Double-sample T-test：这里利用双样本 t-test通过测试两个object的历史质心（图 3 (c) 中的星星）来确定是否合并两个object；

#### OBJECT SLAM

**Object Representation**：EAO-SLAM利用cubes 和quadrics来表示object；**对于具有规则形状的object**，例如书籍、键盘和椅子，使用cubes（由其顶点 Po 编码）来表示它们；**对于没有明确方向的非常规对象**，例如球、瓶子和杯子，quadrics（由其半轴 Qo 编码）表示；在全局地图上表示所需的R和T：

![EA0SLAM_3_3](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_3_3.png)

**并且假设物体与地面平行放置**，即 θr=θp=0，**只需要估计立方体的 [θy, t, s] 和二次曲面的 [t, s]；**

**Estimate t and s**：假设全局框架中有一个物体点云X，我们按照约定用t表示它的均值，据此可以计算出尺度s = (max(X) − min(X))/2；主要的问题是X通常会有很多异常值，会给 t 和 s 的估计带来很大的偏差，本文中的主要贡献之一是开发了一种基于 iForest [27] 的异常稳健质心和尺度估计算法，以提高估计精度，算法详细过程在下图算法1中所示：

![EA0SLAM_3_1](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_3_1.png)

> 算法1：基于iForest的质心和尺度估计

分析：该算法的关键思想是将数据空间递归地分离成一系列孤立的数据点，然后将容易孤立的数据点作为异常值；其原理是，正常点通常位于更近的位置，因此需要更多的步骤来隔离，而异常值通常分散稀疏，可以用更少的步骤轻松隔离；如算法所示，首先使用object的点云（第 2 行和第 14-33 行）创建 t isolated tree（iForest），然后通过计算每个点 x ∈ X（行3-9)，其中评分函数定义如下：

![EA0SLAM_3_5](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_3_5.png)

其中C是归一化参数，H是权重系数，h(x)是孤立树中点x的高度；如图 4(d)-(e) 所示，黄点经过四步被隔离，因此其路径长度为 4，而绿点的路径长度为 8。因此，黄点更可能成为异常值；在作者的实现中，得分大于 0.6 的点被删除，剩下的用于计算 t 和 s（第 10-12 行），基于 s，可以初步构造对象框架中的三次方和二次方，如图 4(a)-(c) 所示， s 将在稍后与对象和相机姿势一起进一步优化；

![EA0SLAM_4](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_4.png)

> 图4：iForest 的object表示和演示

**Estimate θy**：θy的估计分为两步，即先为θy找到一个好的初始值，然后根据初始值进行数值优化；由于姿态估计是一个非线性过程，良好的初始化对于帮助提高估计结果的最优性非常重要；位姿初始化算法的细节在算法2中所示；

![EA0SLAM_3_2](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_3_2.png)

> 算法2：object 位姿估计的初始化

分析：error定义如下所示：

![EA0SLAM_3_6](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_3_6.png)

e(θ) 的计算演示在图 5 (e)-(g) 中可视化；评分函数定义如下：

![EA0SLAM_3_7](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_3_7.png)

![EA0SLAM_5](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_5.png)

> 图5：线对齐以估计物体方向

**Joint Optimization**：在获得初始 S 和 θy 后，然后作者联合优化object和相机位姿：

![EA0SLAM_3_4](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_3_4.png)

其中第一项是 Eq.(13) 中定义的object pose error；尺度误差 e(s) 定义为立方体的投影边缘与其最近的平行 LSD 段之间的距离；第二项 e(p) 是传统 SLAM 框架中常见的重投影误差；

## Experiment

#### A. Distributions of Different Statistics

Data association部分测试采用的统计数据包括point clouds 及其object的质心；为了验证关于不同统计量分布的假设，作者分析了大量数据并将它们的分布可视化，如图6所示：

![EA0SLAM_6](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_6.png)

> 图6：数据关联中不同统计量的分布； (a) 点云在三个方向上的位置分布， (b) 质心的距离误差分布

分析：图6 (a) 显示了fr3 long office序列中data association过程中13个object的点云分布，很明显该统计数据不符合Gaussian distribution，可以看出，**分布与对象的特定特征有关，并没有表现出一致的行为**；图 6 (b) 显示了object质心在不同帧中的误差分布，通常遵循Gaussian distribution；**该结果验证了对点云应用非参 Wilcoxon Rank-Sum 检验和对object质心应用 t 检验的合理性**；

#### B. Ensemble Data Association Experiments

作者将其方法与常用的Intersection over Union(IoU) 方法、非参数检验 (NP) 和 t-test进行比较，图 7 显示了这些方法在 TUM fr3 long office sequnc 中的关联结果；

![EA0SLAM_7](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_7.png)

> 图7：数据关联结果的定性比较；(a) IoU method. (b) IoU and nonparametric test. (c) IoU and t-test. (d) our ensemble method.

分析：从图7上可以看出来，(a)-(c)中有些object没有正确关联；EAO-SLAM实现了很高的关联成功率，并且地图中的对象数量更接近GT；

![EA0SLAM_2_1](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_2_1.png)

> 表1：数据关联结果

作者还将其方法与[6]进行了比较，结果如表二所示：

![EA0SLAM_2_2](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_2_2.png)

> 表2：数据关联质量分析

分析：效果比[6]好，特别是在 TUM 数据集中，EAO-SLAM成功关联的object数量几乎是 [6] 的两倍；在 Microsoft RGBD 和 Scenes V2 中，由于object数量有限，优势并不明显；[6]的关联不准确的原因有两个：1）该方法没有利用不同的统计量，只使用了非参数统计量，从而导致许多未关联的对象； 2）利用聚类算法来解决上述问题，它删除了大部分候选对象；

#### C. Qualitative Assessment of Object Pose Estimation

作者将object的cubes和quadrics叠加在半稠密地图上以进行定性评估；图 8 是键盘的 3D 俯视图（图 5（a）），其中立方体表示其pose；图 8(a) 是具有大尺度误差的初始位姿；图8(b)是使用iForest后的结果；图 8(c) 是我们联合姿态估计后的最终位姿：

![EA0SLAM_8](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_8.png)

> 图8：位姿估计可视化

图 9 展示了三个数据集的 14 个序列中object的位姿估计结果，其中object被随机放置在不同的方向上；如图所示，EAO-SLAM使用单目相机取得了有希望的结果，这证明了EAO-SLAM位姿估计算法的有效性：

![EA0SLAM_9](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_9.png)

> 图9：object位姿估计的结果；奇数列：原始 RGB 图像；偶数列：估计的物体位姿

说明：由于数据集不是专门为object pose estimation而设计的，因此没有用于定量评估这些方法的 groundtruth；在这里，作者比较了初始化前 (BI)、初始化后 (AI) 和联合优化后 (JO) 后的 θy，如表三所示，物体原始方向与全局坐标系平行，存在较大的角度误差；位姿初始化后误差减小，联合优化后误差进一步减小，验证了作者的位姿估计算法的有效性；

![EA0SLAM_2_3](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_2_3.png)

> 表3：物体角度误差的定量分析

#### D. Object-Oriented Map Building

作者基于鲁棒的数据关联算法、精确的对象位姿估计算法和半稠密地图系统构建了object-oriented的semantic maps；图10展示了 TUM fr3_long_office 和 fr2_desk 的两个示例，其中 (d) 和 (e) 显示了由 EAO-SLAM 构建的半稠密语义地图和object-oriented的地图：

![EA0SLAM_10](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_10.png)

> 图10：不同的地图表示； (a) RGB 图像， (b) 稀疏地图， (c) 半稠密地图，(d) 作者的半稠密语义图， (e)  作者的轻量级和object-oriented的地图，(d) 和 (e) 由 EAO-SLAM 构建

EAO-SLAM的半稠密语义地图比ORB-SLAM2的稀疏地图更好的表达环境，在环境理解上比[25]中提出的半稠密地图更好；

TUM、Microsoft RGB-D 和 Scenes V2 datasets其他序列的建图结果如图11所示：

![EA0SLAM_11](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_11.png)

> 图11：EAO-SLAM 在三个数据集上的结果。顶部：原始图像；底部：simi-dense 面向对象的地图

分析：以看出，EAO-SLAM可以在复杂环境中处理多类不同尺度和方向的物体；不可避免地有一些不准确的估计，例如在fire sequence中，椅子太大而无法被快速移动的摄像机很好地观察到，从而产生不准确的估计；

作者在真实场景进行了实验，效果如图12所示：

![EA0SLAM_12](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/EA0SLAM_12.png)

> 图12：EAO-SLAM 在真实场景中的结果；左右：原始图像；中：半稠密的面向对象的地图

分析：可以看出，即使物体被遮挡，也可以准确估计，进一步验证了EAO-SLAM的鲁棒性和准确性；

## Conclusion

EAO-SLAM基于 robust ensemble data association 方法和准确的位姿估计框架，目的是构建semi-dense 且 lightweight object-oriented maps；

## 问题

1.物体描述子和Data Association和Object tracking 的比如

Object tracking是Data association中另一种常用的方法；基于跟踪的方法运行时效率高，但在复杂环境中很容易生成不正确的先验，从而产生不正确的关联结果；

2.参数和非参数方法是什么意思

该数据关联方法只适用静态场景