# Debug Log
"谁删了我跟谁急"

1. 绿色框位置比实际提前了（匀速小陀螺）
   首先绿色框是EKF估计出来的状态映射到图像上的结果，如果快了，说明你EKF的w和a估计不准，要么w超前，要么a偏大,去调整观测噪声
2. 别轻易改EKF的实现，NIS和NEES这样给是有道理的，改了会有别的问题，别说什么我四自由度是九点几几，济爷这样做是有深意的，如果决定修改，要知道怎么改，最好能进行数学推导
3. pre_aim_max_delta_angle是直接筛除过于偏的板，后续的leaving和comming都不参加，comming_angle / leaving_angle是在可选板中做优先选择，进入射击范围中的板优先选择comming_angle小的，离开射击范围的板优先选择leaving_angle小的
4. 马氏距离门控
5. 