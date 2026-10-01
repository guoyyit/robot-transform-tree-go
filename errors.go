package tftree

import "errors"

// 历史为空、查询时间已被淘汰、超出外推上限、图中存在环等情况
// 分别使用不同的哨兵错误，便于调用方区分处理。
var (
	ErrFrameNotFound = errors.New("tftree: frame not found")
	ErrNoConnection  = errors.New("tftree: frames are not connected in the transform tree")
	ErrEmptyHistory  = errors.New("tftree: transform history is empty")
	ErrExpired       = errors.New("tftree: requested time is older than retained history (expired)")
	ErrExtrapolation = errors.New("tftree: requested time exceeds the configured extrapolation limit")
	ErrCycleDetected = errors.New("tftree: cycle detected in transform graph")
)
