import { Card, Progress, Space, Typography, Button, Tag, Descriptions } from 'antd'
import { PauseOutlined, PlayCircleOutlined, CloseOutlined } from '@ant-design/icons'
import { useTranslation } from 'react-i18next'

const { Text, Title } = Typography

interface TaskStatus {
  task_id: number
  action: string
  status: 'queued' | 'executing' | 'completed' | 'failed' | 'paused'
  progress?: number
  message?: string
  created_at?: string
  updated_at?: string
}

interface CurrentTaskStatusProps {
  task: TaskStatus | null
  onPause?: (taskId: number) => void
  onResume?: (taskId: number) => void
  onCancel?: (taskId: number) => void
}

const CurrentTaskStatus: React.FC<CurrentTaskStatusProps> = ({
  task,
  onPause,
  onResume,
  onCancel,
}) => {
  const { t } = useTranslation()

  if (!task) {
    return (
      <Card title={t('tasks.currentTask')}>
        <Text type="secondary">{t('tasks.noActiveTask')}</Text>
      </Card>
    )
  }

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'queued':
        return 'default'
      case 'executing':
        return 'processing'
      case 'completed':
        return 'success'
      case 'failed':
        return 'error'
      case 'paused':
        return 'warning'
      default:
        return 'default'
    }
  }

  const getActionLabel = (action: string) => {
    const actionMap: Record<string, string> = {
      navigate_to_pose: t('tasks.navigation'),
      start_exploration: t('tasks.exploration'),
      start_patrol: t('tasks.patrol'),
      emergency_stop: t('tasks.emergencyStop'),
    }
    return actionMap[action] || action
  }

  const progress = task.progress !== undefined ? Math.round(task.progress * 100) : 0

  return (
    <Card
      title={
        <Space>
          <Title level={5} style={{ margin: 0 }}>
            {t('tasks.currentTask')}
          </Title>
          <Tag color={getStatusColor(task.status)}>{task.status.toUpperCase()}</Tag>
        </Space>
      }
      extra={
        <Space>
          {task.status === 'executing' && onPause && (
            <Button
              size="small"
              icon={<PauseOutlined />}
              onClick={() => onPause(task.task_id)}
            >
              {t('tasks.pause')}
            </Button>
          )}
          {task.status === 'paused' && onResume && (
            <Button
              size="small"
              type="primary"
              icon={<PlayCircleOutlined />}
              onClick={() => onResume(task.task_id)}
            >
              {t('tasks.resume')}
            </Button>
          )}
          {(task.status === 'executing' || task.status === 'paused') && onCancel && (
            <Button
              size="small"
              danger
              icon={<CloseOutlined />}
              onClick={() => onCancel(task.task_id)}
            >
              {t('tasks.cancel')}
            </Button>
          )}
        </Space>
      }
    >
      <Space direction="vertical" style={{ width: '100%' }} size="middle">
        <Descriptions column={2} size="small">
          <Descriptions.Item label={t('tasks.taskId')}>{task.task_id}</Descriptions.Item>
          <Descriptions.Item label={t('tasks.type')}>{getActionLabel(task.action)}</Descriptions.Item>
        </Descriptions>

        {task.status === 'executing' && (
          <>
            <Text type="secondary">{t('tasks.progress')}</Text>
            <Progress
              percent={progress}
              status={'active'}
              strokeColor={{
                '0%': '#108ee9',
                '100%': '#87d068',
              }}
            />
          </>
        )}

        {task.message && (
          <Text type={task.status === 'failed' ? 'danger' : 'secondary'}>
            {task.message}
          </Text>
        )}
      </Space>
    </Card>
  )
}

export default CurrentTaskStatus
