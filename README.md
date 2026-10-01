# robot-transform-tree-go

本地进程内的坐标变换树服务。多个来源以各自频率发布带时间戳的父子坐标系相对位姿，
下游可在任意时间点查询任意两个坐标系之间的位姿。不依赖任何外部服务。

## 快速开始

```go
tree := transformtree.New(transformtree.Config{
    MaxAge:           10 * time.Second, // 每条边保留的历史时长
    MaxEntries:       1000,             // 每条边保留的最大帧数
    MaxExtrapolation: 100 * time.Millisecond,
})

tree.Publish("world", "base", tr1, t1)
tree.Publish("base", "sensor", tr2, t2)
tree.PublishStatic("map", "world", staticTr)

// 返回 t 时刻 sensor 坐标系在 map 坐标系中的位姿（map <- sensor）
tfm, err := tree.LookupTransform("map", "sensor", t)
```

`Transform` 表示子坐标系点到父坐标系的变换；`LookupTransform(target, source, at)`
返回把 `source` 中的点变换到 `target` 的位姿，与逐级手工
`Compose`/`Inverse` 结果一致。

## 时间语义

- **动态变换**：每个 `(parent, child)` 边维护按时间戳排序的采样历史。
  查询时间落在两帧之间时，平移做 LERP、旋转做单位四元数 SLERP。
- **静态变换**：`PublishStatic` 发布后与时间无关，任意查询时间直接返回；
  重复发布以最新定义覆盖。
- **乱序与重复**：采样允许延迟、乱序到达。历史始终按时间戳有序，旧时间戳数据
  晚到不会覆盖或“回退”更新的数据；同一时间戳重复发布整帧忽略。因此同一时间点的
  查询结果只取决于已接收帧的集合，与到达顺序无关。
- **外推**：查询时间晚于最新帧时，沿最近两帧做线性外推（只有一帧时保持常值），
  超出 `MaxExtrapolation` 返回 `ErrExtrapolation`。`MaxExtrapolation = 0`
  表示完全禁止外推；等于上限边界的查询允许。
- **过期**：查询时间早于现存最早帧时返回 `ErrExpired`，与外推错误可区分，
  表示所需采样已被淘汰或从未发布。
- **空历史**：动态边从未发布时返回 `ErrNoData`。

## 有界历史与淘汰

每条动态边同时受两个条件约束，任一触发即淘汰最旧帧：

- `MaxAge`：以“已知最新帧时间戳”为基准，严格早于 `newest - MaxAge` 的帧被淘汰。
  因此晚到、且已落在窗口外的旧帧会被立即丢弃，长时间运行内存不会持续增长。
- `MaxEntries`：帧数超过上限时淘汰最旧帧。

淘汰在每次插入时同步完成；查询已淘汰时间点返回 `ErrExpired`。

## 配置项

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `MaxAge` | `10s` | 单条动态边的时间窗口；零值取默认 |
| `MaxEntries` | `1000` | 单条动态边最大采样帧数；零值取默认 |
| `MaxExtrapolation` | `0` | 允许超出最新帧的外推时长；零值禁止外推 |

配置对每条边独立生效。

## 变换图与闭环

边按无向关系参与解析。查询时先在整张图上做 DFS 森林检测：任何闭环（含自环、
同一对节点之间的平行边）都使查询以 `ErrCycle` 终止并返回，不产生不确定结果。
调用 `Remove(parent, child)` 移除闭环边后，后续查询自动恢复。
图不连通或坐标系从未发布时返回 `ErrFrameNotFound`（使用 `errors.Is` 判断）。

## 并发

`Tree` 内部使用读写锁，支持发布、删除与查询并发。每次查询在同一个读锁下完成
图快照构建、闭环检测与整条路径拼接，期间其他 goroutine 的更新不会造成部分拼接
或撕裂结果；同一时间的重复查询返回一致结果。

## 错误一览

| 错误 | 含义 |
| --- | --- |
| `ErrNoData` | 动态边没有任何历史采样 |
| `ErrExpired` | 查询时间早于现存最早帧（已淘汰/从未发布） |
| `ErrExtrapolation` | 查询时间晚于最新帧且超过外推上限 |
| `ErrFrameNotFound` | 坐标系不存在或与查询另一端不连通 |
| `ErrCycle` | 变换图存在闭环 |

## 测试

```bash
go test ./...
go test -race ./...
```

测试覆盖：乱序到达与重复发布、插值边界（命中首末帧/中间插值）、外推上限边界、
过期与空历史、闭环检测与移除恢复、自环/平行边、并发发布查询一致性以及
历史帧数的内存边界。
