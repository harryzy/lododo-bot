import { Card, Button, Space, Input, Form, message, Tabs } from 'antd'
import { SendOutlined, StopOutlined, UnorderedListOutlined, RocketOutlined, SwapOutlined } from '@ant-design/icons'
import { useState } from 'react'
import { useTranslation } from 'react-i18next'
import { apiService } from '../../services/api'
import TaskList from './TaskList'
import TaskHistory from './TaskHistory'
import ExplorationPanel from './ExplorationPanel'
import PatrolPanel from './PatrolPanel'

function TaskControl() {
  const { t } = useTranslation()
  const [loading, setLoading] = useState(false)
  const [form] = Form.useForm()
  const [currentTask, setCurrentTask] = useState<any>(null)

  // 监听TaskList中的活跃任务变化
  const handleTaskChange = (task: any) => {
    setCurrentTask(task)
  }

  const handleNavigate = async (values: any) => {
    setLoading(true)
    try {
      const response = await apiService.tasks.createNavigationTask({
        x: parseFloat(values.x),
        y: parseFloat(values.y),
        yaw: parseFloat(values.yaw || 0),
      })
      message.success('Navigation task created: ' + response.request_id)
      form.resetFields()
    } catch (error: any) {
      message.error('Failed to create navigation task: ' + error.message)
    } finally {
      setLoading(false)
    }
  }

  const handleEmergencyStop = async () => {
    try {
      await apiService.tasks.emergencyStop()
      message.warning('Emergency stop triggered')
    } catch (error: any) {
      message.error('Failed to stop: ' + error.message)
    }
  }

  const tabItems = [
    {
      key: 'active',
      label: (
        <span>
          <UnorderedListOutlined />
          {t('taskControl.tabs.activeTasks')}
        </span>
      ),
      children: <TaskList onTaskChange={handleTaskChange} />
    },
    {
      key: 'navigation',
      label: t('taskControl.tabs.navigation'),
      children: (
        <Card title={t('tasks.navigation')} style={{ width: '100%' }}>
          <Form form={form} layout="inline" onFinish={handleNavigate}>
            <Form.Item name="x" label="X" rules={[{ required: true, message: 'Please input X coordinate' }]}>
              <Input placeholder="0.0" style={{ width: 100 }} />
            </Form.Item>
            <Form.Item name="y" label="Y" rules={[{ required: true, message: 'Please input Y coordinate' }]}>
              <Input placeholder="0.0" style={{ width: 100 }} />
            </Form.Item>
            <Form.Item name="yaw" label="Yaw">
              <Input placeholder="0.0" style={{ width: 100 }} />
            </Form.Item>
            <Form.Item>
              <Button type="primary" htmlType="submit" icon={<SendOutlined />} loading={loading}>
                {t('tasks.start')}
              </Button>
            </Form.Item>
          </Form>
        </Card>
      )
    },
    {
      key: 'exploration',
      label: (
        <span>
          <RocketOutlined />
          {t('taskControl.tabs.exploration')}
        </span>
      ),
      children: <ExplorationPanel currentTask={currentTask} />
    },
    {
      key: 'patrol',
      label: (
        <span>
          <SwapOutlined />
          {t('taskControl.tabs.patrol')}
        </span>
      ),
      children: <PatrolPanel currentTask={currentTask} />
    },
    {
      key: 'history',
      label: t('taskControl.tabs.history'),
      children: <TaskHistory />
    }
  ]

  return (
    <Space direction="vertical" size="large" style={{ width: '100%' }}>
      <Tabs items={tabItems} defaultActiveKey="active" />

      <Card title={t('tasks.emergency')} style={{ width: '100%' }}>
        <Button danger type="primary" onClick={handleEmergencyStop} icon={<StopOutlined />} size="large">
          {t('tasks.emergencyStop')}
        </Button>
      </Card>
    </Space>
  )
}

export default TaskControl
