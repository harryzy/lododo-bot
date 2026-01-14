/**
 * TaskHistory - 任务历史组件
 * 显示最近的历史任务，支持查看详情
 * 与TaskList使用相同的数据源和逻辑
 */

import { Card, Table, Tag, Modal, Descriptions, Empty, Button, message } from 'antd'
import { ReloadOutlined, EyeOutlined } from '@ant-design/icons'
import { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { websocketService } from '../../services/websocket'
import type { ColumnsType } from 'antd/es/table'

interface Task {
  task_id?: string
  request_id: string
  action: string
  status: string  // 使用status字段（与TaskList一致）
  progress: number
  message: string
  created_at: string
  updated_at?: string
  completed_at?: string
  params?: Record<string, any>
  result?: Record<string, any>
  error?: string
}

function TaskHistory() {
  const { t } = useTranslation()
  const [tasks, setTasks] = useState<Task[]>([])
  const [totalCount, setTotalCount] = useState(0)  // 总任务数
  const [loading, setLoading] = useState(false)
  const [selectedTask, setSelectedTask] = useState<Task | null>(null)
  const [modalVisible, setModalVisible] = useState(false)

  // 从API加载历史任务（与TaskList相同的数据源）
  const loadHistoryTasks = async () => {
    setLoading(true)
    try {
      const response = await fetch('/api/tasks/list')
      const data = await response.json()
      
      if (data.success) {
        // 使用后端返回的历史任务数据
        setTasks(data.history_tasks || [])
        setTotalCount(data.total_history || 0)  // 设置总数
      } else {
        message.error(t('taskControl.loadFailed'))
      }
    } catch (error) {
      console.error('[TaskHistory] 加载历史任务失败:', error)
      message.error(t('taskControl.loadFailed'))
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    // 初始加载
    loadHistoryTasks()

    // 监听WebSocket任务更新（与TaskList相同的逻辑）
    const unsubscribe = websocketService.subscribe((data: any) => {
      if (data.type === 'task_update' && data.task) {
        const updatedTask = data.task as Task
        
        // 只处理已完成的任务（添加到历史列表）
        if (['completed', 'failed', 'cancelled'].includes(updatedTask.status)) {
          setTasks(prev => {
            // 检查是否已存在
            const exists = prev.some(t => 
              t.task_id === updatedTask.task_id || t.request_id === updatedTask.request_id
            )
            if (!exists) {
              // 新任务添加到顶部，并更新总数
              setTotalCount(prev => prev + 1)
              return [updatedTask, ...prev] // 新任务添加到顶部
            }
            // 如果已存在，更新状态
            return prev.map(t => 
              (t.task_id === updatedTask.task_id || t.request_id === updatedTask.request_id) 
                ? updatedTask 
                : t
            )
          })
        }
      }
    })

    return () => {
      unsubscribe()
    }
  }, [])

  // 状态映射（与TaskList一致）
  const getStatusTag = (status: string) => {
    const statusMap: Record<string, { color: string; text: string }> = {
      queued: { color: 'default', text: t('taskControl.status.queued') },
      executing: { color: 'processing', text: t('taskControl.status.executing') },
      paused: { color: 'warning', text: t('taskControl.status.paused') },
      completed: { color: 'success', text: t('taskControl.status.completed') },
      failed: { color: 'error', text: t('taskControl.status.failed') },
      cancelled: { color: 'default', text: t('taskControl.status.cancelled') }
    }
    const config = statusMap[status.toLowerCase()] || { color: 'default', text: status }
    return <Tag color={config.color}>{config.text}</Tag>
  }

  // 动作类型映射（与TaskList一致）
  const getActionText = (action: string) => {
    const actionMap: Record<string, string> = {
      navigate_to_pose: t('taskControl.action.navigate'),
      start_exploration: t('taskControl.action.exploration'),
      start_patrol: t('taskControl.action.patrol')
    }
    return actionMap[action] || action
  }

  // 计算任务耗时
  const getTaskDuration = (task: Task) => {
    if (!task.created_at) return '-'
    
    const start = new Date(task.created_at).getTime()
    const end = task.updated_at ? new Date(task.updated_at).getTime() : Date.now()
    const durationMs = end - start
    
    const seconds = Math.floor(durationMs / 1000) % 60
    const minutes = Math.floor(durationMs / (1000 * 60)) % 60
    const hours = Math.floor(durationMs / (1000 * 60 * 60))
    
    if (hours > 0) {
      return `${hours}h ${minutes}m ${seconds}s`
    } else if (minutes > 0) {
      return `${minutes}m ${seconds}s`
    } else {
      return `${seconds}s`
    }
  }

  // 表格列定义
  const columns: ColumnsType<Task> = [
    {
      title: t('taskControl.taskId'),
      dataIndex: 'task_id',
      key: 'task_id',
      width: 80,
      render: (id: string | undefined) => <span style={{ fontFamily: 'monospace' }}>#{id || '-'}</span>
    },
    {
      title: t('taskControl.type'),
      dataIndex: 'action',
      key: 'action',
      width: 120,
      render: (action: string) => getActionText(action)
    },
    {
      title: t('taskControl.status.label'),
      dataIndex: 'status',  // 使用status字段（与TaskList一致）
      key: 'status',
      width: 100,
      render: (status: string) => getStatusTag(status)
    },
    {
      title: t('taskControl.duration'),
      key: 'duration',
      width: 120,
      render: (_: any, record: Task) => getTaskDuration(record)
    },
    {
      title: t('taskControl.startTime'),
      dataIndex: 'created_at',
      key: 'created_at',
      width: 180,
      render: (time: string) => time ? new Date(time).toLocaleString() : '-'
    },
    {
      title: t('taskControl.actions'),
      key: 'actions',
      width: 100,
      render: (_: any, record: Task) => (
        <Button
          type="link"
          icon={<EyeOutlined />}
          onClick={() => {
            setSelectedTask(record)
            setModalVisible(true)
          }}
        >
          {t('taskControl.viewDetails')}
        </Button>
      )
    }
  ]

  return (
    <>
      <Card
        title={
          <span>
            {t('taskControl.history.title')}
            <span style={{ marginLeft: 8, fontSize: 14, fontWeight: 'normal', color: '#999' }}>
              ({t('taskControl.history.totalCount', { count: totalCount })})
            </span>
          </span>
        }
        style={{ width: '100%' }}
        extra={
          <Button
            icon={<ReloadOutlined />}
            onClick={loadHistoryTasks}
            loading={loading}
          >
            {t('common.refresh')}
          </Button>
        }
      >
        {tasks.length > 0 ? (
          <Table
            columns={columns}
            dataSource={tasks}
            rowKey="request_id"
            loading={loading}
            pagination={{
              pageSize: 10,
              showSizeChanger: true,
              showQuickJumper: true,
              pageSizeOptions: ['10', '20', '50', '100'],
              showTotal: (total, range) => 
                t('taskControl.history.pageInfo', { 
                  start: range[0], 
                  end: range[1], 
                  total 
                }),
              size: 'default'
            }}
            size="small"
          />
        ) : (
          <Empty
            description={t('taskControl.history.empty')}
            image={Empty.PRESENTED_IMAGE_SIMPLE}
          />
        )}
      </Card>

      {/* 任务详情Modal */}
      <Modal
        title={t('taskControl.taskDetails')}
        open={modalVisible}
        onCancel={() => setModalVisible(false)}
        footer={null}
        width={600}
      >
        {selectedTask && (
          <Descriptions bordered column={1} size="small">
            <Descriptions.Item label={t('taskControl.taskId')}>
              <span style={{ fontFamily: 'monospace' }}>#{selectedTask.task_id || '-'}</span>
            </Descriptions.Item>
            <Descriptions.Item label={t('taskControl.requestId')}>
              <span style={{ fontFamily: 'monospace', fontSize: '12px' }}>
                {selectedTask.request_id}
              </span>
            </Descriptions.Item>
            <Descriptions.Item label={t('taskControl.type')}>
              {getActionText(selectedTask.action)}
            </Descriptions.Item>
            <Descriptions.Item label={t('taskControl.status.label')}>
              {getStatusTag(selectedTask.status)}
            </Descriptions.Item>
            <Descriptions.Item label={t('taskControl.startTime')}>
              {selectedTask.created_at ? new Date(selectedTask.created_at).toLocaleString() : '-'}
            </Descriptions.Item>
            <Descriptions.Item label={t('taskControl.endTime')}>
              {selectedTask.updated_at ? new Date(selectedTask.updated_at).toLocaleString() : '-'}
            </Descriptions.Item>
            <Descriptions.Item label={t('taskControl.duration')}>
              {getTaskDuration(selectedTask)}
            </Descriptions.Item>
            {selectedTask.params && Object.keys(selectedTask.params).length > 0 && (
              <Descriptions.Item label={t('taskControl.parameters')}>
                <pre style={{ 
                  margin: 0, 
                  fontSize: '12px', 
                  backgroundColor: '#f5f5f5', 
                  padding: '8px',
                  borderRadius: '4px'
                }}>
                  {JSON.stringify(selectedTask.params, null, 2)}
                </pre>
              </Descriptions.Item>
            )}
            {selectedTask.result && (
              <Descriptions.Item label={t('taskControl.result')}>
                <pre style={{ 
                  margin: 0, 
                  fontSize: '12px', 
                  backgroundColor: '#f5f5f5', 
                  padding: '8px',
                  borderRadius: '4px'
                }}>
                  {JSON.stringify(selectedTask.result, null, 2)}
                </pre>
              </Descriptions.Item>
            )}
            {selectedTask.error && (
              <Descriptions.Item label={t('taskControl.error')}>
                <span style={{ color: '#ff4d4f' }}>{selectedTask.error}</span>
              </Descriptions.Item>
            )}
          </Descriptions>
        )}
      </Modal>
    </>
  )
}

export default TaskHistory
