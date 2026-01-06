#!/usr/bin/env python3
"""
CompletionStrategy - 探索完成度策略管理器

功能 / Features:
  - 智能判断探索是否应该完成
  - 支持多种完成条件（阈值/停滞/失败次数）
  - 完成度历史跟踪和停滞检测
  
Purpose:
  统一 exploration_handler.py 中10处分散的完成度判断逻辑

Author: LeKiwi Bot Development Team
Date: 2026-01-05
"""

import time
from typing import List, Tuple, Optional


class CompletionStrategy:
    """
    探索完成度策略管理器
    
    Intelligently determines when exploration should be considered complete
    based on multiple criteria:
    - Target completion threshold
    - Minimum acceptable threshold with no frontiers
    - High completion with repeated failures
    - Completion stagnation detection
    """
    
    def __init__(self, 
                 target_threshold: float = 0.90,
                 min_threshold: float = 0.75,
                 stagnation_time: float = 180.0,
                 min_progress_rate: float = 0.02,
                 logger=None):
        """
        初始化完成度策略
        
        Args:
            target_threshold: 目标完成度阈值（默认90%）
            min_threshold: 最小可接受阈值（默认75%）
            stagnation_time: 停滞检测时间窗口（秒，默认3分钟）
            min_progress_rate: 最小进展率（默认2%）
            logger: ROS logger（可选）
        """
        self._target_threshold = target_threshold
        self._min_threshold = min_threshold
        self._stagnation_time = stagnation_time
        self._min_progress_rate = min_progress_rate
        self._logger = logger
        
        # 完成度历史：[(timestamp, completion), ...]
        self._completion_history: List[Tuple[float, float]] = []
    
    def should_complete(self, 
                       current_completion: float,
                       consecutive_failures: int = 0,
                       no_frontiers: bool = False,
                       context: str = "") -> Tuple[bool, str]:
        """
        判断是否应该完成探索
        
        Args:
            current_completion: 当前完成度（0.0-1.0）
            consecutive_failures: 连续失败次数
            no_frontiers: 是否无可达边界
            context: 上下文信息（用于日志）
            
        Returns:
            (should_complete, reason)
            - should_complete: 是否应该完成
            - reason: 完成原因（如果应该完成）
        """
        # 记录完成度历史
        self._record_completion(current_completion)
        
        # 策略1: 达到目标阈值
        if current_completion >= self._target_threshold:
            reason = f"✅ Reached target completion ({current_completion*100:.1f}% >= {self._target_threshold*100:.0f}%)"
            self._log_decision(True, reason, context)
            return True, reason
        
        # 策略2: 达到最小阈值 + 无可达边界
        if current_completion >= self._min_threshold and no_frontiers:
            reason = (f"✅ Reached minimum threshold ({current_completion*100:.1f}% >= {self._min_threshold*100:.0f}%) "
                     f"with no reachable frontiers")
            self._log_decision(True, reason, context)
            return True, reason
        
        # 策略3: 高完成度 + 连续失败多次
        if current_completion >= 0.85 and consecutive_failures >= 3:
            reason = (f"✅ High completion ({current_completion*100:.1f}%) "
                     f"with {consecutive_failures} consecutive failures")
            self._log_decision(True, reason, context)
            return True, reason
        
        # 策略4: 完成度停滞检测
        if self._is_stagnant():
            reason = f"✅ Completion stagnant at {current_completion*100:.1f}% (no progress in {self._stagnation_time/60:.1f} min)"
            self._log_decision(True, reason, context)
            return True, reason
        
        # 不应该完成
        return False, ""
    
    def _is_stagnant(self) -> bool:
        """
        检测完成度是否停滞
        
        停滞定义：在指定时间窗口内，进展率低于最小值
        
        Returns:
            bool: 是否停滞
        """
        if len(self._completion_history) < 5:
            # 数据不足，不判断停滞
            return False
        
        current_time = time.time()
        cutoff_time = current_time - self._stagnation_time
        
        # 清理过旧的历史
        self._completion_history = [
            (t, c) for t, c in self._completion_history if t > cutoff_time
        ]
        
        if len(self._completion_history) < 5:
            return False
        
        # 计算进展率
        oldest_completion = self._completion_history[0][1]
        current_completion = self._completion_history[-1][1]
        progress = current_completion - oldest_completion
        
        # 判断是否停滞
        is_stagnant = progress < self._min_progress_rate
        
        if self._logger and is_stagnant:
            self._logger.debug(
                f"[CompletionStrategy] Stagnation detected: "
                f"progress {progress*100:.2f}% in {self._stagnation_time/60:.1f} min "
                f"(threshold: {self._min_progress_rate*100:.1f}%)"
            )
        
        return is_stagnant
    
    def _record_completion(self, completion: float):
        """记录完成度到历史"""
        self._completion_history.append((time.time(), completion))
        
        # 限制历史大小（保留最近10分钟）
        cutoff_time = time.time() - 600  # 10分钟
        self._completion_history = [
            (t, c) for t, c in self._completion_history if t > cutoff_time
        ]
    
    def _log_decision(self, should_complete: bool, reason: str, context: str):
        """记录决策日志"""
        if not self._logger:
            return
        
        if should_complete:
            msg = f"[CompletionStrategy] COMPLETE decision: {reason}"
            if context:
                msg += f" | Context: {context}"
            self._logger.info(msg)
    
    def reset_history(self):
        """重置完成度历史"""
        self._completion_history.clear()
    
    def get_target_threshold(self) -> float:
        """获取目标阈值"""
        return self._target_threshold
    
    def get_min_threshold(self) -> float:
        """获取最小阈值"""
        return self._min_threshold
    
    def set_target_threshold(self, threshold: float):
        """动态调整目标阈值"""
        self._target_threshold = max(0.0, min(1.0, threshold))
    
    def set_min_threshold(self, threshold: float):
        """动态调整最小阈值"""
        self._min_threshold = max(0.0, min(1.0, threshold))
    
    def get_completion_history(self) -> List[Tuple[float, float]]:
        """获取完成度历史（用于分析）"""
        return self._completion_history.copy()
