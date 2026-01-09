import { Card, Button, Space, Input, Form, message } from 'antd'
import { SendOutlined, StopOutlined } from '@ant-design/icons'
import { useState } from 'react'
import { useTranslation } from 'react-i18next'
import { apiService } from '../../services/api'

function TaskControl() {
  const { t } = useTranslation()
  const [loading, setLoading] = useState(false)
  const [form] = Form.useForm()

  const handleNavigate = async (values: any) => {
    setLoading(true)
    try {
      const response = await apiService.createNavigationTask({
        x: parseFloat(values.x),
        y: parseFloat(values.y),
        yaw: parseFloat(values.yaw || 0),
      })
      message.success(`Navigation task created: ${response.request_id}`)
    } catch (error: any) {
      message.error(`Failed to create navigation task: ${error.message}`)
    } finally {
      setLoading(false)
    }
  }

  const handleExploration = async () => {
    setLoading(true)
    try {
      const response = await apiService.createExplorationTask({
        map_name: 'web_exploration',
        save_on_completion: true,
      })
      message.success(`Exploration task created: ${response.request_id}`)
    } catch (error: any) {
      message.error(`Failed to create exploration task: ${error.message}`)
    } finally {
      setLoading(false)
    }
  }

  const handleEmergencyStop = async () => {
    try {
      await apiService.emergencyStop()
      message.warning('Emergency stop triggered')
    } catch (error: any) {
      message.error(`Failed to stop: ${error.message}`)
    }
  }

  return (
    <Space direction="vertical" size="large" style={{ width: '100%' }}>
      <Card title={t('tasks.navigation')} style={{ width: '100%' }}>
        <Form
          form={form}
          layout="inline"
          onFinish={handleNavigate}
        >
          <Form.Item
            name="x"
            label="X"
            rules={[{ required: true, message: 'Please input X coordinate' }]}
          >
            <Input placeholder="0.0" style={{ width: 100 }} />
          </Form.Item>
          <Form.Item
            name="y"
            label="Y"
            rules={[{ required: true, message: 'Please input Y coordinate' }]}
          >
            <Input placeholder="0.0" style={{ width: 100 }} />
          </Form.Item>
          <Form.Item name="yaw" label="Yaw">
            <Input placeholder="0.0" style={{ width: 100 }} />
          </Form.Item>
          <Form.Item>
            <Button
              type="primary"
              htmlType="submit"
              icon={<SendOutlined />}
              loading={loading}
            >
              {t('tasks.start')}
            </Button>
          </Form.Item>
        </Form>
      </Card>

      <Card title={t('tasks.exploration')} style={{ width: '100%' }}>
        <Button
          type="primary"
          onClick={handleExploration}
          loading={loading}
          icon={<SendOutlined />}
        >
          {t('tasks.startExploration')}
        </Button>
      </Card>

      <Card title={t('tasks.emergency')} style={{ width: '100%' }}>
        <Button
          danger
          type="primary"
          onClick={handleEmergencyStop}
          icon={<StopOutlined />}
        >
          {t('tasks.emergencyStop')}
        </Button>
      </Card>
    </Space>
  )
}

export default TaskControl
