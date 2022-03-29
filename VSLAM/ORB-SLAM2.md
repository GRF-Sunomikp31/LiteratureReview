# [LiteratureReview]ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras、

出处：2017  IEEE Transactions on Robotics，（截止到2022-3-28）Google论文被引3500+；

说明：ORB-SLAM2是基于ORB-SLAM[1]的工作，所以论文中对单目方法做了较少的描述，而对新添加的双目和RGB-D方法做了较多描述。

## Introduction

完整SLAM系统定义：初始化、tracking、优化、map reuse（仅定位模式或者支持在该map上扩展）、loop closing、relocalization ；显然ORB-SLAM2是一个非常完整（指包含Monocular、Stereo 和 RGB-D三种模式）的SLAM系统。

Monocular SLAM的问题：

- 第一帧不能三角测量，所以需要两帧才能初始化；
- 存在尺度漂移（scale drift）；
- 在初始化中，执行纯旋转任务可能会失败；

![](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/1.png)

> 图1：由图1(b)可以看出，ORB-SLAM2的开发者在自己测试的时候做了RGB-D模式稠密地图的demo，具体方法正如图上所示“估计的关键帧位姿反投影传感器深度图得到点云，不进行融合（如KinectFusion [4）”，该点云效果好说明关键帧pose非常精确；

ORB-SLAM2的主要贡献如下：

- 1.第一个单目、双目、RGB-D开源的SLAM系统，包括：loop closing、 relocalization和map reuse；
- 2.ORB-SLAM2中使用 Bundle Adjustment的 **RGB-D模式**比基于ICP或者光度和深度最小化方法最新方法**精度高**；
- 3.通过使用远近双目点（这里指远近点信息判断利用的trick），ORB-SLAM2的**双目模式**比最新的直接双目法SLAM**更准确**；
- 4.一个轻量级的定位模式，当建图被禁用情况下可以重用地图；
- （个人）ORB-SLAM2能够在各种环境中的标准CPU上运行；

## Related Work

**A. Stereo SLAM**

- [5]：其基于条件独立分而治之的EKF-SLAM方法，在大环境下比当时其他方法好 ；它是第一个同时利用近点和远点（ 即由于立体相机中的视差很小而无法可靠估计深度的点）的双目SLAM方法，并且对远点使用逆深度参数化 [6]  ；他们凭经验表明，如果点的深度小于双目基线的 40 倍，则可以可靠地对点进行三角测量 ；ORB-SLAM2用了这种方法处理近点和远点；  
- [8]：大多数现代的双目SLAM系统都是基于关键帧的，并在局部区域执行BA优化；该工作在关键帧的内部窗口中执行 BA（point-pose constraints）和外部窗口中的pose-graph图（pose-pose constraints）的联合优化，通过限定窗口大小实现了恒定的时间复杂度，但代价是不保证全局一致性；
- RSLAM[9]：使用 landmarks和poses的相对表示，并在当前区域中执行相对 BA，该区域可以被约束为恒定时间； RSLAM 能够关闭允许在循环两侧扩展活动区域的循环，但不强制执行全局一致性；
- S-PTAM[10]：做local BA，但是缺少 large loop closing；
- Stereo LSD-SLAM[11]：一种半稠密直接法，可最大限度地减少具有高梯度的图像区域中的光度误差；该方法不依赖于特征，希望对运动模糊或纹理不良的环境更加稳健；然而，作为一种直接法，它的性能可能会因未建模的效果（如卷帘快门或非朗伯反射率）而严重降低；

ORB-SLAM2采用的方法：与上面这些方法类似，ORB-SLAM2在一组本地关键帧中执行 BA，以便复杂性与地图大小无关，并且可以在大型环境中操作; ORB-SLAM2的目标是建立一个全球一致的地图;关闭循环时，ORB-SLAM2首先对齐两侧，类似于 RSLAM，以便跟踪能够继续使用旧地图进行定位，然后执行位姿图优化以最小化循环中累积的漂移，然后是 full BA 。                                                                                 

**B. RGB-D SLAM**

- KinectFusion[4] ：最早且最出名的RGB-D SLAM方法，该方法将来自传感器的所有深度数据融合到一个体积稠密模型中，该模型用于使用 ICP 跟踪相机位姿。由于其volumetric表示和缺乏loop closing，该系统仅限于小型工作空间；
- Kintinuous[12]：该方法通过使用滚动循环缓冲区在大型环境中运行，并包括使用位置识别和pose graph优化的loop closing；
- RGB-D SLAM[13]：基于特征的系统，其前端通过特征匹配和 ICP 计算帧到帧的运动。后端使用来自启发式搜索的闭环约束执行pose-graph优化
- DVO-SLAM[14]：与[13]类似，DVO-SLAM的后端做了pose-graph优化，其中关键帧到关键帧的约束是从VO中计算出来的，进而去最小化光度和深度误差；相比位置识别，DVO-SLAM采用了heuristic fashion方式在所有先前帧上搜索回环候选关键帧；
- ElasticFusion[15]：该方法构建了一个基于面元的环境地图；这是一种以地图为中心的方法，它会忘记poses并执行loop closing ，将非刚性变形应用于地图，而不是标准的pose-graph 优化；该系统的详细重建和定位精度令人印象深刻，但当前的实现仅限于房间大小的地图，因为复杂性与地图中面元的数量成比例。

ORB-SLAM2采用的方法：参考[8]的方法，ORB-SLAM2 使用深度信息为图像上提取的特征合成立体坐标，这样就不同区分输入是RGB-D还是双目；与上面方法不同的是，ORB-SLAM2的后端基于BA构建全局一致的稀疏重建；所以ORB-SLAM2是轻量级的，并且适用于标准 CPU；ORB-SLAM2的目标是长期和globally一致的localization，而不是构建最详细的稠密重建。但是可以从高度准确的关键帧poses中融合深度图并在局部区域中即时获得准确的重建，或者在full BA 之后对来自所有关键帧的深度图进行后处理，并获得整个场景的准确 3D 模型。

## Methods

ORB-SLAM2建立在单目ORB-SLAM1的基础上，总体概述如图 2 所示，主要包括三个线程：

- tracking：tracking线程过查找与本地地图匹配的特征并最小化应用motion-only BA 的重投影误差来在每一帧中定位相机；
- local mapping：local mapping线程管理local map并通过local BA优化它；
- loop closing：loop closing线程检测大回环并通过执行pose-graph optimization纠正累积的漂移；该线程在pose-graph optimization之后启动第四个线程来执行full BA，以计算最佳结构和运动解决方案；

![2](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/2.png)

> 图2：ORB-SLAM2主要由三个并行线程组成：tracking, local mapping and loop closing；在loop closure后创建第四个线程来执行full BA；tracking线程预处理双目或 RGB-D 输入，以便系统的其余部分独立于输入传感器运行；虽然图中没有显示，但 ORB-SLAM2 也适用于 [1] 中的单目输入。

ORB-SLAM2系统还包括如下部件：

- 为了解决跟踪失败（例如遮挡）或在已映射场景中重新初始化以及回环检测的情况，嵌入了一个基于 DBoW2 [16] 的位置识别模块用于 relocalization；
- ORB-SLAM2系统维护了一个 covisibiliy 图 [8]，该图链接了观察公共点的任意两个关键帧，以及一个连接所有关键帧的最小生成树;这些图结构允许检索关键帧的本地窗口，以便跟踪和本地映射在本地操作，允许在大型环境中工作，并用作关闭循环时执行的位姿图优化的结构；
- ORB-SLAM2系统使用ORB features [17]用于tracking， mapping 和 place recognition任务，该特征对旋转和缩放鲁棒，并对相机自动增益和自动曝光以及照明变化呈现良好的不变性；此外，它们可以快速提取和匹配，允许实时操作，并在词袋位置识别中表现出良好的精度/召回性能 [18]。

**A. Monocular , Close Stereo and Far Stereo Keypoints**

ORB-SLAM2是基于特征的方法，对输入进行预处理以提取显著关键点位置的特征，然后丢弃输入图像，所有系统操作都基于这些特征，因此系统独立于双目或 RGB-D 传感器；ORB-SLAM2系统处理单目和双目关键点，进一步分类为近或远：

- Stereo keypoints：假设输入的图像立体校正，核线水平；如果双目关键点的相关深度小于立体/RGB-D 基线的 40 倍，则将其分类为近，如 [5] 中所建议的，否则将其分类为远；

- Monocular keypoints：图像二维坐标定义，包含RGB-D中没有深度以及双目立体匹配找不到深度的点，这些点仅从多个视图进行三角剖分，不提供比例信息，但有助于旋转和平移估计；

**B. System Bootstrapping**（系统引导）

双目和RGB-D相机可以通过仅从一帧获得深度信息，不需要像单目情况那样来自运动初始化的特定结构；在系统启动时，ORB-SLAM2使用第一帧创建关键帧，将其pose设置为原点，并从所有双目关键点中创建初始地图。

**C. Bundle Adjustment with Monocular and Stereo Constraints**

ORB-SLAM2的BA使用的是g2o中的Levenberg-Marquardt 方法：

- Motion-only BA（tracking线程）：只优化相机pose；通过最小化世界坐标中匹配的3D点和关键点的重投影误差去优化相机的R和T

![3_1](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/3_1.png)

- Local BA（local mapping线程）：优化关键帧和点的局部窗口；优化一组共视可见关键帧 KL 和在这些关键帧 PL 中看到的所有点。所有其他关键帧 KF ，不在 KL 中，PL 中的观察点对成本函数有贡献，但在优化中保持固定；

![3_2](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/3_2.png)

- Full BA（loop closing 线程）：闭环后优化所有关键帧和点；是local BA 的特定情况，其中地图中的所有关键帧和点都经过优化，除了固定的原点关键帧以消除规范自由度；

**D. Loop Closing and Full BA**

Loop Closing 分为两个步骤执行，首先检测和验证一个回环，然后对闭环进行校正以优化pose-graph；与单目ORB-SLAM不同，RGB-D和双目的ORB-SLAM2，立体/深度信息使尺度可观察，几何验证和位姿图优化不再需要处理尺度漂移，不是相似性而是基于刚体变换；

Full BA在pose-graph 优化之后用于实现最优解，Full BA优化需要很高的计算量所以放在单独的线程中执行，允许系统继续创建地图和检测回环；如果在Full BA优化运行时检测到新回环，ORB-SLAM2将中止优化并继续回环，这将再次启动full BA 优化;当full BA 完成后，需要将完整 BA 优化的关键帧和点的更新子集与优化运行时插入的未更新关键帧和点合并。

**E. Keyframe Insertion**

ORB-SLAM2采用了monocular ORB-SLAM[1]的关键帧插入策略，即非常频繁地插入关键帧并随后剔除冗余的关键帧；近双目点和远双目点之间的区别成为引入关键帧插入的新条件；如图3所示，在这样的环境中需要有足够数量的闭合点来准确估计平移，因此如果跟踪的闭合点数量低于 τt 并且该帧可以创建至少 τc 个新的闭合立体点，系统将插入一个新的关键帧。作者凭经验发现 τt = 100 和 τc = 70 在所有的实验中效果很好。

![3](C:\Users\LD\Desktop\科研\REVIEW\IMG\3.png)

> 图3：KITTI 数据集01序列的效果，绿色的点为近点其深度小于双目基线的40倍，蓝色点为远点；远点有助于估计方向，但为平移和缩放提供微弱的信息。

**F . Localization Mode**

ORB-SLAM2添加了一种定位模式，在该模式下local mapping 和loop closing 线程被关闭，如果需要，相机可以通过使用重定位的跟踪连续定位；在这种模式下，跟踪利用视觉里程计匹配并匹配到地图点，视觉里程计匹配是当前帧中的 ORB 与根据立体/深度信息在前一帧中创建的 3D 点之间的匹配，这些匹配使定位对未建图区域具有鲁棒性，但可以累积漂移，地图点匹配确保对现有地图的无漂移定位。

## Experiment

测试硬件平台：Intel Core i7-4790 desktop computer with 16Gb RAM；

指标：**使用原作者发表的结果**（这里如何直接使用论文的结果是不是不太公平？）和文献中的标准评估指标；

测试：为了避免多线程系统的不确定性，每个序列运行 5 次，并取中值结果；

**A. KITTI Dataset [2]**

KITTI dataset 包含从城市和高速公路环境中的汽车记录的立体序列，双目基线约为54cm，工作频率为 10Hz，校正后的分辨率为 1240 × 376 像素，序列 00、02、05、06、07 和 09 包含回环；表 1显示了 11 个训练序列的结果，与最先进的Stereo LSD-SLAM [11] 相比，据我们所知ORB-SLAM2是唯一显示所有序列详细结果的Stereo SLAM。作者使用两个不同的指标，[3] 中提出的绝对平移 RMSE 和 [2] 中提出的平均相对平移 r(rel) 和旋转 r(rel) 误差；ORB-SLAM2在大多数序列中都优于 Stereo LSD-SLAM，并且总体上实现了低于 1% 的相对误差；

![2_1](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/2_1.png)

> 表1：KITTI 数据集中的准确性比较；

图 4 显示了一些估计轨迹的例子：

![4](C:\Users\LD\Desktop\科研\REVIEW\IMG\4.png)

> 图4：KITTI 00、01、05 和 07 ；

与 [1] 中提出的单目结果相比，所提出的双目版本能够处理单目系统失败的序列 01，在这个高速公路序列中，见图 3，近点仅在几帧中可见，双目版本仅从一个立体关键帧创建点的能力，而不是延迟初始化单目，包括在两个关键帧之间寻找匹配，在这个序列中不丢失跟踪是至关重要的，此外，双目系统使用公制比例估计地图和轨迹，并且不会受到比例漂移的影响，如图 5 所示；

![5](C:\Users\LD\Desktop\科研\REVIEW\IMG\5.png)

> 图5：KITTI 08数据集，左边是monocular ORB-SLAM [1]，右边是monocular ORB-SLAM [1]；monocular  ORBSLAM 在这个序列中遭受严重的尺度漂移，**尤其是在转弯处**，相比之下，所提出的立体版本能够估计轨迹和地图的真实比例，而不会发生比例漂移；

**B. EuRoC Dataset [21]**

EuRoC Dataset包含从在两个不同房间和大型工业环境中飞行MAV记录的11个双目序列；双目基线约为11cm，并提供20HZ的WVGA图像，根据 MAV 的速度、光照和场景纹理，这些序列分为简单、中等和困难；表2显示了 ORB-SLAM2 对于所有序列的绝对平移 RMSE，与 Stereo LSDSLAM 相比（采用其论文 [11] 中提供的结果）更准确；ORB-SLAM2 实现了厘米级的定位精度，比 Stereo LSD-SLAM 更准确；由于严重的运动模糊，ORB-SLAM2的tracking在 V2 03 的某些部分难以进行；如[22]所示，这个序列可以使用 IMU 信息进行处理；

![2_2](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/2_2.png)

> 表2： EuRoC Dataset上平移 RMSE (m) 的比较；

图 6 显示了计算出轨迹与真值相比的示例：

![6](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/6.png)

> 图6：数据集为EuRoC中等难度的V1_02，V2_02，MH_03和困难的MH_05；黑色代表估计出的轨迹，红色代表groundtruth；

**C. TUM RGB-D Dataset [3]**

TUM RGB-D dataset包含来自 RGB-D 传感器的室内序列，分为几类，用于评估不同纹理、照明和结构条件下的对象重建和 SLAM/里程计方法；表3中，将ORB-SLAM2和ElasticFusion [15]、Kintinuous [12]、DVO-SLAM [14] 和 RGB-D SLAM [13]做了比较，ORB-SLAM2是唯一一种基于BA的方法，并且在大多数序列中都优于其他方法；

![2_3](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/2_3.png)

> 表3：TUM RGB-D DATASET上平移 RMSE (m) 的比较；

图 7 显示了从四个序列中计算的关键帧pose对传感器深度图进行反投影所产生的点云，桌子和海报的良好定义和笔直轮廓证明了ORB-SLAM2的高精度定位：

![7](C:\Users\LD\Desktop\科研\REVIEW\IMG\7.png)

> 图7：TUM RGB-D fr3 office, fr1 room, fr2 desk and fr3 nst数据集通过计算的关键帧pose对传感器深度图进行反投影所产生的点云；

**D. Timing Results**

在表3中对三个序列在不同传感器和不同分辨率图像进行了时间分析，显示的是每个线程任务的平均值和两个标准偏差范围。由于这些序列包含一个回环，因此full  BA 和回环线程的一些任务只执行一次，并且只报告一个时间测量值。每帧的平均跟踪时间低于每个帧速率的倒数序列，这意味着我们的系统能够实时工作。由于双目图像中的ORB提取是并行的，可以看出在V2 02的立体WVGA图像中提取1000个ORB特征类似于在fr3 office的单个VGA图像通道中提取相同数量的特征；回环中的关键帧数显示为与回环相关的时间的参考，虽然 KITTI 07 中的回环包含更多关键帧，但为室内 fr3 办公室构建的 covisibility graph 更密集，因此回环融合、pose-graph优化和full BA 更昂贵， covisibility graph 的高密度使得本地地图包含更多的关键帧和点，因此本地地图tracking和local BA 也更加昂贵。

![2_4](https://raw.githubusercontent.com/GRF-Sunomikp31/PicBed/master/2_4.png)

> 表4：以毫秒为单位的每个线程的计时结果（平均值 ± 2 标准偏差）；

## Conclusion

ORB-SLAM2是一个完整的SLAM系统，接受Monocular、Stereo and RGB-D传感器输入，同时能够在标准的CPU上实时执行 relocalization、loop closing、map reuse等功能；在已知环境下，ORB-SLAM2的仅定位模式提供了一种非常稳健、零漂移和轻量级的定位方法；ORB-SLAM2在大多情况下都表现出最高的精度，在KITTI visual odometry benchmark上ORB-SLAM2是最好的双目解决方案，至关重要的是，与其他双目SLAM方法相比，ORB-SLAM2实现在已经建图区域上实现了**零漂移定位**；ORB-SLAM2的RGB-D方法表明，在定位精度上，BA的方法优于直接法或者ICP，另外还具有计算成本低、不需要GPU就能实时等优点；

作者开源了ORB-SLAM2的源代码，该源代码还包含一个使用单目相机的[AR应用程序](https://www.youtube.com/watch?v=kPwy8yA4CKM)；

**未来的扩展可能包括**：非重叠多摄像头、鱼眼或者全景摄像头支持、大尺度的稠密融合、协同建图或者是增强运动模糊鲁棒性。
