import React, { useEffect, useState } from 'react';
import { Card, Table, Tag, Button, Space, message, Progress } from 'antd';
import { 
  PlayCircleOutlined, 
  PauseOutlined, 
  CloseCircleOutlined,
  ReloadOutlined,
  HistoryOutlined 
} from '@ant-design/icons';
import * as api from '../../services/api';
import { websocketService } from '../../services/websocket';
import { useTranslation } from 'react-i18next';

interface Task {
  task_id?: string;
  request_id: string;
  action: string;
  status: string;
  progress: number;
  message: string;
  created_at: string;
  updated_at: string;
  completed_at?: string;
}

interface TaskListProps {
  onTaskChange?: (task: Task | null) => void;  // 当前活跃任务变化回调
}

const TaskList: React.FC<TaskListProps> = ({ onTaskChange }) => {
  const { t } = useTranslation();
  const [activeTasks, setActiveTasks] = useState<Task[]>([]);
  const [historyTasks, setHistoryTasks] = useState<Task[]>([]);
  const [loading, setLoading] = useState(false);

  // 通知父组件当前活跃任务
  useEffect(() => {
    if (onTaskChange) {
      const activeTask = activeTasks.length > 0 ? activeTasks[0] : null;
      onTaskChange(activeTask);
    }
  }, [activeTasks, onTaskChange]);

  // 加载任务列表
  const loadTasks = async () => {
    try {
      setLoading(true);
      const response = await fetch('/api/tasks/list');
      const data = await response.json();
      
      if (data.success) {
        setActiveTasks(data.active_tasks || []);
        setHistoryTasks(data.history_tasks || []);
      } else {
        message.error(t('taskControl.loadFailed'));
      }
    } catch (error) {
      console.error('Failed to load tasks:', error);
      message.error(t('taskControl.loadFailed'));
    } finally {
      setLoading(false);
    }
  };

  // WebSocket 监听任务更新
  useEffect(() => {
    const unsubscribe = websocketService.subscribe((data: any) => {
      if (data.type === 'task_update' && data.task) {
        // 直接使用WebSocket推送的任务数据更新UI
        console.log('[TaskList] Received task update:', data.task);
        
        const updatedTask = data.task as Task;
        
        // 更新活跃任务列表
        setActiveTasks(prev => {
          const index = prev.findIndex(t => 
            t.task_id === updatedTask.task_id || t.request_id === updatedTask.request_id
          );
          
          if (index !== -1) {
            // 任务已完成，移除
            if (['completed', 'failed', 'cancelled'].includes(updatedTask.status)) {
              return prev.filter((_, i) => i !== index);
            }
            // 更新现有任务
            const newTasks = [...prev];
            newTasks[index] = updatedTask;
            return newTasks;
          } else {
            // 新任务，添加到列表
            if (!['completed', 'failed', 'cancelled'].includes(updatedTask.status)) {
              return [...prev, updatedTask];
            }
            return prev;
          }
        });
        
        // 已完成的任务添加到历史
        if (['completed', 'failed', 'cancelled'].includes(updatedTask.status)) {
          setHistoryTasks(prev => {
            // 检查是否已存在
            const exists = prev.some(t => 
              t.task_id === updatedTask.task_id || t.request_id === updatedTask.request_id
            );
            if (!exists) {
              return [updatedTask, ...prev].slice(0, 50); // 保留最近50条
            }
            return prev;
          });
        }
      }
    });

    // 初始加载
    loadTasks();

    return () => {
      unsubscribe();
    };
  }, []);

  // 手动查询单个任务状态
  const handleQuery = async (taskId: string | undefined) => {
    if (!taskId) {
      message.warning('任务ID未分配，请稍后再试');
      return;
    }
    try {
      const response = await api.tasks.queryTaskStatus(taskId);
      if (response.success) {
        // 立即使用缓存数据更新本地状态
        if (response.task_data) {
          updateLocalTask(response.task_data);
        }
        message.success(t('taskControl.querySuccess'));
      }
    } catch (error) {
      message.error(t('taskControl.queryFailed'));
    }
  };

  // 暂停任务
  const handlePause = async (taskId: string | undefined) => {
    if (!taskId) {
      message.warning('任务ID未分配，请稍后再试');
      return;
    }
    try {
      const response = await api.tasks.pauseTask(taskId);
      if (response.success) {
        // 立即使用缓存数据更新本地状态
        if (response.task_data) {
          updateLocalTask(response.task_data);
        }
        message.success(t('taskControl.pauseSuccess'));
      }
    } catch (error) {
      message.error(t('taskControl.pauseFailed'));
    }
  };

  // 恢复任务
  const handleResume = async (taskId: string | undefined) => {
    if (!taskId) {
      message.warning('任务ID未分配，请稍后再试');
      return;
    }
    try {
      const response = await api.tasks.resumeTask(taskId);
      if (response.success) {
        // 立即使用缓存数据更新本地状态
        if (response.task_data) {
          updateLocalTask(response.task_data);
        }
        message.success(t('taskControl.resumeSuccess'));
      }
    } catch (error) {
      message.error(t('taskControl.resumeFailed'));
    }
  };

  // 取消任务
  const handleCancel = async (taskId: string | undefined) => {
    if (!taskId) {
      message.warning('任务ID未分配，请稍后再试');
      return;
    }
    try {
      const response = await api.tasks.cancelTask(taskId.toString());
      if (response.success) {
        message.success(t('taskControl.cancelSuccess'));
      }
    } catch (error) {
      message.error(t('taskControl.cancelFailed'));
    }
  };

  // 更新本地任务状态（立即反馈）
  const updateLocalTask = (taskData: Task) => {
    // 终态任务：completed, failed, cancelled
    const isFinalState = ['completed', 'failed', 'cancelled'].includes(taskData.status);
    
    if (isFinalState) {
      // 1. 从活跃任务列表移除
      setActiveTasks(prev => 
        prev.filter(task => 
          task.task_id !== taskData.task_id && task.request_id !== taskData.request_id
        )
      );
      
      // 2. 添加到历史列表（如果不存在）
      setHistoryTasks(prev => {
        const exists = prev.some(t => 
          t.task_id === taskData.task_id || t.request_id === taskData.request_id
        );
        if (!exists) {
          return [taskData, ...prev].slice(0, 50); // 保留最近50条
        }
        // 如果已存在，更新状态
        return prev.map(t => 
          (t.task_id === taskData.task_id || t.request_id === taskData.request_id) 
            ? taskData 
            : t
        );
      });
    } else {
      // 非终态任务：更新活跃任务列表
      setActiveTasks(prev => {
        const index = prev.findIndex(task => 
          task.task_id === taskData.task_id || task.request_id === taskData.request_id
        );
        
        if (index !== -1) {
          // 更新现有任务
          const newTasks = [...prev];
          newTasks[index] = taskData;
          return newTasks;
        } else {
          // 新任务，添加到列表
          return [...prev, taskData];
        }
      });
    }
  };

  // 状态标签渲染
  const renderStatusTag = (status: string) => {
    const statusMap: Record<string, { color: string; text: string }> = {
      queued: { color: 'default', text: t('taskControl.status.queued') },
      executing: { color: 'processing', text: t('taskControl.status.executing') },
      paused: { color: 'warning', text: t('taskControl.status.paused') },
      completed: { color: 'success', text: t('taskControl.status.completed') },
      failed: { color: 'error', text: t('taskControl.status.failed') },
      cancelled: { color: 'default', text: t('taskControl.status.cancelled') },
    };
    
    const statusInfo = statusMap[status] || { color: 'default', text: status };
    return <Tag color={statusInfo.color}>{statusInfo.text}</Tag>;
  };

  // 动作类型翻译
  const translateAction = (action: string) => {
    const actionMap: Record<string, string> = {
      navigate_to_pose: t('taskControl.action.navigate'),
      start_exploration: t('taskControl.action.exploration'),
      start_patrol: t('taskControl.action.patrol'),
    };
    return actionMap[action] || action;
  };

  // 活跃任务表格列定义
  const activeColumns = [
    {
      title: t('taskControl.taskId'),
      dataIndex: 'task_id',
      key: 'task_id',
      width: 80,
      render: (id: number | undefined) => id || '-',
    },
    {
      title: t('taskControl.actionLabel'),
      dataIndex: 'action',
      key: 'action',
      width: 120,
      render: (action: string) => translateAction(action),
    },
    {
      title: t('taskControl.statusLabel'),
      dataIndex: 'status',
      key: 'status',
      width: 100,
      render: (status: string) => renderStatusTag(status),
    },
    {
      title: t('taskControl.progress'),
      dataIndex: 'progress',
      key: 'progress',
      width: 150,
      render: (progress: number, record: Task) => {
        if (record.status === 'executing') {
          return (
            <Progress
              percent={Math.round(progress)}
              size="small"
              strokeColor={{
                '0%': '#108ee9',
                '100%': '#87d068',
              }}
            />
          );
        }
        return <span>{Math.round(progress)}%</span>;
      },
    },
    {
      title: t('taskControl.message'),
      dataIndex: 'message',
      key: 'message',
      ellipsis: true,
    },
    {
      title: t('taskControl.operations'),
      key: 'operations',
      width: 240,
      render: (_: any, record: Task) => (
        <Space size="small">
          <Button
            size="small"
            icon={<ReloadOutlined />}
            onClick={() => handleQuery(record.task_id)}
            disabled={!record.task_id}
          >
            {t('taskControl.query')}
          </Button>
          
          {record.status === 'executing' && (
            <Button
              size="small"
              icon={<PauseOutlined />}
              onClick={() => handlePause(record.task_id)}
              disabled={!record.task_id}
            >
              {t('taskControl.pause')}
            </Button>
          )}
          
          {record.status === 'paused' && (
            <Button
              size="small"
              icon={<PlayCircleOutlined />}
              onClick={() => handleResume(record.task_id)}
              disabled={!record.task_id}
            >
              {t('taskControl.resume')}
            </Button>
          )}
          
          {(record.status === 'executing' || record.status === 'paused' || record.status === 'queued') && (
            <Button
              size="small"
              danger
              icon={<CloseCircleOutlined />}
              onClick={() => handleCancel(record.task_id)}
              disabled={!record.task_id}
            >
              {t('taskControl.cancel')}
            </Button>
          )}
        </Space>
      ),
    },
  ];

  // 历史任务表格列定义
  const historyColumns = [
    {
      title: t('taskControl.taskId'),
      dataIndex: 'task_id',
      key: 'task_id',
      width: 80,
      render: (id: number | undefined) => id || '-',
    },
    {
      title: t('taskControl.actionLabel'),
      dataIndex: 'action',
      key: 'action',
      width: 120,
      render: (action: string) => translateAction(action),
    },
    {
      title: t('taskControl.statusLabel'),
      dataIndex: 'status',
      key: 'status',
      width: 100,
      render: (status: string) => renderStatusTag(status),
    },
    {
      title: t('taskControl.message'),
      dataIndex: 'message',
      key: 'message',
      ellipsis: true,
    },
    {
      title: t('taskControl.completedAt'),
      dataIndex: 'completed_at',
      key: 'completed_at',
      width: 180,
      render: (time: string) => {
        if (!time) return '-';
        return new Date(time).toLocaleString('zh-CN');
      },
    },
  ];

  return (
    <div>
      <Card 
        title={t('taskControl.activeTasks')}
        extra={
          <Button 
            icon={<ReloadOutlined />} 
            onClick={loadTasks}
            loading={loading}
          >
            {t('taskControl.refresh')}
          </Button>
        }
        style={{ marginBottom: 16 }}
      >
        <Table
          columns={activeColumns}
          dataSource={activeTasks}
          rowKey="request_id"
          size="middle"
          pagination={false}
          loading={loading}
          locale={{ emptyText: t('taskControl.noActiveTasks') }}
        />
      </Card>

      <Card 
        title={
          <Space>
            <HistoryOutlined />
            {t('taskControl.taskHistory')}
          </Space>
        }
      >
        <Table
          columns={historyColumns}
          dataSource={historyTasks}
          rowKey="request_id"
          size="small"
          pagination={{ pageSize: 10, showTotal: (total) => `${t('taskControl.total')} ${total}` }}
          loading={loading}
          locale={{ emptyText: t('taskControl.noHistory') }}
        />
      </Card>
    </div>
  );
};

export default TaskList;
