### C\+\+ 逻辑修改

- **GLB 解析辅助函数**：从 glb 二进制文件提取 JSON chunk

- 叠加 MRM 逆反射分量的扩展 Schlick‑Cook‑Torrance BRDF 模型



$\begin {aligned}  {\omega_o'} &= 2\left (n\cdot\omega_o\right) n-\omega_o \\ f_{\mathrm {schlick,retro}}(\omega_i,\omega_o') &= \frac {D_{\mathrm {schlick\_ext,retro}} \cdot F_{\mathrm {schlick,retro}} \cdot G_{\mathrm {schlick\_ggx,retro}}}{4\left (n\cdot\omega_i\right)\left (n\cdot {\omega_o'}\right)} \\ f_{\mathrm {total}} &= C_\lambda \Big ( f_{\mathrm {schlick}}(\omega_i,\omega_o)
\mathrm{enable_retro} \cdot k_{\mathrm{retro}} \cdot f_{\mathrm{schlick,retro}}(\omega_i,\omega_o') \Big) \end{aligned}$

$$\begin{aligned}
f_{\text{total}} = C_\lambda \left(
\underbrace{f_{\text{schlick}}(\omega_i, \omega_o)}_{\text{BRDF: 表层PET薄膜普通镜面反射}} + \text{enable\_retro} \cdot k_{\text{retro}} \cdot
  \underbrace{
  f_{\text{schlick}}
  \Big(
  \omega_i,\,
  \overbrace{\omega'_o}^{\text{② MRM变换后的输入}}
  \Big)
  }_{\text{④ MRM等效镜面项 = BRDF函数 + MRM向量变换}}
  \right)\nonumber\\
\end{aligned}$$

1. ${C_\lambda}$：波长相关全局缩放系数；统一控制整个材质整体亮度

2. ${f_{\mathrm{schlick}}(\omega_i,\omega_o)}$：**表层 PET 薄膜镜面反射项（普通微面元，不变换）**

  - $\omega_i$：入射方向（指向表面）

  - $\omega_o$：原始出射方向（离开表面）

  - 使用**表层材质参数**：表层粗糙度

3. ${\mathrm{enable\_retro}}$：逆反射开关，0 = 关闭 \(锥桶本体\)，1 = 开启 \(反光条\)

4. ${k_{\mathrm{retro}}}$：逆反射分量幅值缩放系数，调整体亮度

5. ${f_{\mathrm{schlick,retro}}(\omega_i,\omega_o')}$：**MRM 逆反射子项**，${\omega_o'} = 2\left (n\cdot\omega_o\right) n-\omega_o$

---

## 更新：接收器角度响应（LiDAR孔径投影修正）

单站 LiDAR 下 $\omega_i = \omega_o$，接收器孔径接收到的有效回波功率与表面法线在传感器方向的投影（即 $\cos\theta$）成正比。原始 BRDF 仅描述表面散射分布，未包含接收器孔径投影衰减。为此在最终强度公式中引入 $\cos\theta$ 修正项：

$$I = I_{\max} \cdot \frac{d_{\text{near}}^2}{d^2} \cdot f_{\text{total}} \cdot \cos\theta \cdot e^{-\sigma_{\text{atm}} d} \cdot \eta_{\text{sys}}$$

其中 $\cos\theta = \max(0,\, n \cdot (-\omega_o))$ 为有效逆反射截面校正。该修正保证：

- 正面击中表面（$\cos\theta = 1$）时，BRDF 正常贡献
- 掠射角（$\cos\theta \to 0$）时，尽管菲涅尔反射率趋近 1，但投影有效面积趋零，回波功率衰减
- 无需对 Cook‑Torrance 分母做 hack，完整保留经典 BRDF 定义的物理一致性

---

## 更新：MRM 逆反射项改用 $\cos\theta$ 线性衰减

原本 MRM 逆反射项与主 BRDF 共用 GGX 微面元分布函数：

$$f_{\text{retro}}(\omega_i, \omega_o') = \frac{D_{\text{GGX,retro}} \cdot F_{\text{retro}} \cdot G_{\text{GGX,retro}}^2}{4(n\cdot\omega_i)(n\cdot\omega_o')}$$

实际场景中，反光条的角锥反射器角度响应宽而平滑，GGX 高光瓣过窄，导致反光条在锥面只有极正对位置出现一撮高亮、偏离后骤暗，视觉效果突兀。

将 retro 项改为 $\cos\theta$ 线性衰减：

$$f_{\text{retro}}(\theta) = F_{0,\text{retro}} \cdot \cos\theta$$

$$\begin{aligned} f_{\text{total}} &= C_\lambda \cdot f_{\text{schlick}}(\omega_i, \omega_o) + k_{\text{retro}} \cdot f_{\text{retro}}(\theta) \end{aligned}$$

理由：

- 真实角锥反射器的返回能量主要受接收器投影面积（$\propto\cos\theta$）而非微面元分布约束
- $\cos\theta$ 线性衰减天然平滑，从正面到边缘无突变
- 参数 $k_{\text{retro}}$ 统一控制逆反射强度，$F_{0,\text{retro}}$ 控制基础反射率