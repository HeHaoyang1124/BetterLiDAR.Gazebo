# 优化建议计划

> 来源：`PointCloudRenderer` 中 `SetHzOffsets` 内联到 `LoadChannelTable` 之后的评审建议。
> 以下按优先级排列，均为可选改进项。

## 1. `group_index` 类型隐患

- 位置：`src/plugin/PointCloudRenderer.hpp` 的 `ChannelEntry::group_index`（`float`）。
- 问题：`LoadChannelTable` 中 `static_cast<int>(max_it->group_index + 1)` 与
  `static_cast<int>(entry.group_index)` 隐含“必须为整数”的前提。
  该字段为 float 是为了装进 RGBA32F 纹理，语义上却是整数分组号。
- 建议：取整时使用 `std::lround`（或先显式取整）再转 int，避免浮点误差导致截断偏差。

## 2. `#include <ranges>` 位置归位

- 位置：`src/plugin/PointCloudRenderer.cpp`，当前夹在 `TINYEXR_IMPLEMENTATION` 宏与 `tinyexr.h` 之间。
- 问题：属于历史遗留，与 tinyexr 无关。
- 建议：将 `#include <ranges>` 移到文件顶部其他 `#include` 一起；
  `#define TINYEXR_IMPLEMENTATION` 与 `#include "tinyexr.h"` 保持紧邻。

## 3. 隐式调用顺序依赖

- 问题：`hz_scale_` 先在 `SetParams` 按 `2/(hz_max - hz_min)` 计算，
  后在 `LoadChannelTable` 中带上 offset margin 重算。
  这要求 `SetParams` 必须早于 `LoadChannelTable` 调用。
  当前 `BetterLidar::Configure` 顺序正确，但依赖未显式约束。
- 建议：在相关函数注释中显式说明该顺序约束；
  或让 `LoadChannelTable` 内部自行接收/持有 `hz_min_`/`hz_max_`，
  减少对外部状态的依赖。

## 备注

- 空表防护已由 `LoadChannelTable` 开头的 `channel_data.empty()` 兜底，
  `std::ranges::max_element` 解引用目前是安全的，无需改动。