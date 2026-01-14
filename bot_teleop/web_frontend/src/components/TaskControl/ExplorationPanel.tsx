/**
 * ExplorationPanel - 探索任务面板
 * 创建探索任务，输入地图名称、选择保存选项
 */

import { Card, Form, Input, Switch, Button, Space, message, Progress, Statistic } from 'antd'
import { RocketOutlined } from '@ant-design/icons'
import { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { apiService } from '../../services/api'

interface ExplorationPanelProps {
  currentTask?: any  // 当前活跃的探索任务
}

function ExplorationPanel({ currentTask }: ExplorationPanelProps) {
  const { t } = useTranslation()
  const [form] = Form.useForm()
  const [loading, setLoading] = useState(false)
  const [progress, setProgress] = useState(0)

  // 监听当前任务进度
  useEffect(() => {
    if (currentTask && currentTask.action === 'start_exploration') {
      // 从任务参数中提取进度信息
      if (currentTask.result && currentTask.result.progress !== undefined) {
        setProgress(Math.round(currentTask.result.progress * 100))
      } else if (currentTask.state === 'executing') {
        // 如果没有进度信息，显示一个估计值
        setProgress(50)
      } else if (currentTask.state === 'completed') {
        setProgress(100)
      }
    } else {
      setProgress(0)
    }
  }, [currentTask])

  const handleStartExploration = async (values: any) => {
    setLoading(true)
    try {
      const response = await apiService.tasks.createExplorationTask({
        map_name: values.map_name || 'exploration_' + Date.now(),
        save_on_completion: values.save_on_completion !== false,
      })
      
      message.success(
        t('taskControl.exploration.created', {
          requestId: response.request_id.substring(0, 8)
        })
      )
      
      // 重置表单（可选）
      // form.resetFields()
    } catch (error: any) {
      message.error(t('taskControl.exploration.createFailed') + ': ' + error.message)
    } finally {
      setLoading(false)
    }
  }

  // 判断是否有活跃的探索任务
  const hasActiveExploration = currentTask && 
    currentTask.action === 'start_exploration' && 
    ['queued', 'executing'].includes(currentTask.state)

  return (
    <Card
      title={
        <Space>
          <RocketOutlined />
          {t('taskControl.exploration.title')}
        </Space>
      }
      style={{ width: '100%' }}
    >
      <Form
        form={form}
        layout="vertical"
        onFinish={handleStartExploration}
        initialValues={{
          map_name: '',
          save_on_completion: true
        }}
      >
        <Form.Item
          name="map_name"
          label={t('taskControl.exploration.mapName')}
          tooltip={t('taskControl.exploration.mapNameTooltip')}
        >
          <Input
            placeholder={t('taskControl.exploration.mapNamePlaceholder')}
            disabled={hasActiveExploration}
          />
        </Form.Item>

        <Form.Item
          name="save_on_completion"
          label={t('taskControl.exploration.saveOption')}
          valuePropName="checked"
        >
          <Switch
            checkedChildren={t('common.yes')}
            unCheckedChildren={t('common.no')}
            disabled={hasActiveExploration}
          />
        </Form.Item>

        {/* 探索进度显示 */}
        {hasActiveExploration && (
          <div style={{ marginBottom: 16 }}>
            <div style={{ marginBottom: 8 }}>
              <strong>{t('taskControl.exploration.progress')}:</strong>
            </div>
            <Progress
              percent={progress}
              status={currentTask.state === 'failed' ? 'exception' : 'active'}
              strokeColor={{
                '0%': '#108ee9',
                '100%': '#87d068',
              }}
            />
            <div style={{ marginTop: 8, display: 'flex', justifyContent: 'space-around' }}>
              <Statistic
                title={t('taskControl.status.label')}
                value={t(`taskControl.status.${currentTask.state}`)}
                valueStyle={{ fontSize: 16 }}
              />
              <Statistic
                title={t('taskControl.taskId')}
                value={`#${currentTask.task_id}`}
                valueStyle={{ fontSize: 16, fontFamily: 'monospace' }}
              />
            </div>
          </div>
        )}

        <Form.Item>
          <Button
            type="primary"
            htmlType="submit"
            icon={<RocketOutlined />}
            loading={loading}
            disabled={hasActiveExploration}
            block
          >
            {hasActiveExploration
              ? t('taskControl.exploration.exploring')
              : t('taskControl.exploration.start')}
          </Button>
        </Form.Item>

        {hasActiveExploration && (
          <div style={{ 
            padding: '8px 12px', 
            backgroundColor: '#e6f7ff', 
            border: '1px solid #91d5ff',
            borderRadius: '4px',
            fontSize: '13px'
          }}>
            💡 {t('taskControl.exploration.hint')}
          </div>
        )}
      </Form>
    </Card>
  )
}

export default ExplorationPanel
