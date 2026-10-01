# robot-transform-tree-go

进程内的本地坐标变换树服务。多个来源以各自频率发布带时间戳的坐标系相对位姿，
下游可在任意时间点查询任意两个坐标系之间的刚体位姿。全部计算在单机进程内完成，
不依赖任何外部服务。

## 数据模型

- 每条变换是一条有向边 `parent -> child`，携带平移与单位四元数姿态。
- 同一对 `(parent, child)`：
  - `PublishStatic` 保存唯一静态位姿，查询时不受时间影响；
  - `PublishDynamic` 保存一段按时间戳有序的有界历史。
- 查询 `Lookup(target, source, at)` 返回把 `source` 系中的点变换到 `target`
  系所需的位姿，即 `T(target, source)`。
- 图允许同一子坐标系存在多条入边。查询在无向意义下寻找两帧之间的路径并逐边拼接。

## 时间语义

- **插值**：查询时间落在两帧之间时，平移线性插值，姿态走短弧 SLERP；
  恰好等于某帧时间戳时返回该帧原值。
- **外推**：查询时间早于最早帧或晚于最晚帧时，保持最近端帧的值
  （不做外推旋转/速度推算），超出 `MaxExtrapolation` 则返回错误。
  - 早于最早帧：若历史从未被淘汰，超出窗口返回 `ErrExtrapolation`；
  - 若该边历史曾经被淘汰过，更早的时间返回 `ErrExpired`，与“从未收到过那么早的数据”区分。
- **静态边**：任何时间都返回同一位姿，不参与插值与外推。
- **乱序与重复**：只接受严格新于最新帧的时间戳。更旧时间戳晚到、相同时间戳重复
  发布一律忽略，因此查询结果与到达顺序无关，不会发生状态跳变。
- **历史淘汰**：每次写入后按 `MaxHistoryAge`（相对该边最新帧）裁剪，
  再按 `MaxSamples` 限制条数；被淘汰时间点的查询返回 `ErrExpired`。

## 闭环

- 查询前对有向图做三色 DFS。只要图中存在有向环（回边指向当前递归栈上的帧），
  任意查询都立即返回 `ErrCycleDetected`，不会沿环无限拼接。
- 自环（`parent == child`）在发布时直接拒绝。
- 用 `RemoveTransform(parent, child)` 移除环上的边后，图恢复为无环状态，
  查询自动恢复正常。

## 并发

- `Tree` 内部用单一 `sync.RWMutex` 保护。
- 一次 `Lookup` 全程持有读锁，在一致快照上完成路径解析、逐边采样与拼接，
  因此不会看到部分更新或撕裂结果；查询与发布可并发进行。

## 错误

| 错误 | 含义 |
| --- | --- |
| `ErrFrameNotFound` | 目标或来源坐标系从未注册 |
| `ErrNoConnection` | 两个坐标系分属不同连通分量 |
| `ErrEmptyHistory` | 动态边尚未收到任何帧 |
| `ErrExpired` | 查询时间点的数据已按有界历史淘汰 |
| `ErrExtrapolation` | 查询时间超出 `MaxExtrapolation` 允许的范围 |
| `ErrCycleDetected` | 变换图中存在环 |

错误均被 `fmt.Errorf("...: %w", ErrXxx)` 包装，使用 `errors.Is(err, ErrXxx)` 判断。

## 配置项

`NewTree(Options{...})`：

| 字段 | 说明 | 默认 |
| --- | --- | --- |
| `MaxHistoryAge` | 每条动态边保留的历史时长，按相对最新帧裁剪；`<=0` 不按时间淘汰 | `10s`（传 0 时） |
| `MaxExtrapolation` | 允许外推到历史范围之外的最大时长；`0` 禁止外推，负值表示不限制 | `0` |
| `MaxSamples` | 每条动态边最多保留的帧数；`0` 不限制条数 | `0` |

## 使用示例

```go
tree := tftree.NewTree(tftree.Options{
    MaxHistoryAge:    5 * time.Second,
    MaxExtrapolation: 100 * time.Millisecond,
})

tree.PublishStatic("map", "odom", tftree.Transform{
    Translation: tftree.Vector3{X: 1, Y: 2, Z: 0},
    Rotation:    tftree.Quaternion{W: 1},
})
tree.PublishDynamic("odom", "base", time.Now(), tftree.Transform{
    Translation: tftree.Vector3{X: 0.1, Y: 0, Z: 0},
    Rotation:    tftree.Quaternion{W: 1},
})

pose, err := tree.Lookup("map", "base", time.Now())
```

## 测试

```sh
go test ./...
go test -race ./...
```

覆盖乱序/重复到达、插值边界、外推上限、过期错误、静态变换、链式拼接、
分支公共祖先、闭环检测与恢复、并发读写以及时间/条数两种内存边界。
