自主泊车-障碍物堵塞

进入场景：
    前方有障碍物
    旁边有停车位，停车位上没有车
    
    确定停车预停点
    确定停车位

stage obstacle avoid 避障阶段：
    使用openspace算法规划路径，到达停车预停点


stage parking 停车阶段：
    使用openspace算法规划停车路径，到达最终停车位
