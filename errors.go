package transformtree

import "errors"

// 变换树服务可能返回的哨兵错误。调用方应使用 errors.Is 进行判定。
var (
	// ErrFrameNotFound 表示查询涉及的坐标系从未发布过。
	ErrFrameNotFound = errors.New("transformtree: frame not found")
	// ErrNoData 表示对应边上还没有任何历史采样。
	ErrNoData = errors.New("transformtree: no transform data")
	// ErrExpired 表示查询时间早于当前保留的历史，所需采样已被淘汰。
	ErrExpired = errors.New("transformtree: transform history expired")
	// ErrExtrapolation 表示查询时间超出历史范围且超过允许的外推上限。
	ErrExtrapolation = errors.New("transformtree: extrapolation limit exceeded")
	// ErrCycle 表示变换图中存在闭环，无法得到唯一的解析路径。
	ErrCycle = errors.New("transformtree: cycle detected in transform graph")
)
