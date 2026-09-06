# CHANGELOG

## TODO

- 使用CUDA并行加速距离衰减计算

## V1.0.0

feat: 使用GLSL渲染点云

- 地面全屏解析求交 + 模型光栅化渲染（Lambert模型点云强度）
- 添加雷达高斯噪声
- 锥桶mesh导出为带extras雷达属性的GLB类型（附python添加extras属性）
- 添加雷达水平偏移和距离上下限（附python生成通道纹理）
- 添加OpenGL相关API的RAII模式
