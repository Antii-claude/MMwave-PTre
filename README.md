# MMwave-PTre
## 仿真测试
---
### 文件说明
#### **数据集**：
***dataset文件夹***:原始点云  
***normalization_dataset文件夹***:    
归一化原始点云，点全部分布在半径为1，以原点为圆心的球体内  
***physicalRE_dataset文件夹***:    
处理后残缺点云  

#### **实现代码**：
##### 功能函数  
**`generate_radar.py`**:  
包含两个函数：  
生成雷达阵列（get_sensor_array）  
生成随机扫描轨迹（get_180_degree_scan_trajectory）  
**`pointcloud_noise_add.py`**:  
包含一个函数：  
点云噪点添加（Noise_Add）  
**`specularity_aware_filter.py`**:  
包含一个函数：  
根据论文的公式，  
去掉不可见点、去掉不符合镜面反射的点、模拟材质不同造成的丢点
（specularity_aware_filter）

##### 执行程序
**`try.py`**:  
用于测试目前的仿真程序，尝试对一个点云文件处理并可视化  
**`Normalization.py`**:  
批量对点云进行归一化  
**`re_pointcloud.py`**:  
对归一化的点云进行批量仿真  
***demo文件夹***：  
里面两个文件是可视化代码，用于查看点云图像；  
其中ORIGIN头展示的是归一化原始点云，RE头展示的是处理后残缺点云，使用时可以参考这个分类
