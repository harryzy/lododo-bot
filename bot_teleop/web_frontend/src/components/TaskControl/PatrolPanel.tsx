/**
 * PatrolPanel - 巡逻任务面板
 * 创建巡逻任务，选择路点文件和巡逻模式
 */

import { Card, Form, Select, Button, Space, message, Progress, Statistic, Alert } from 'antd'
import { SwapOutlined, ReloadOutlined } from '@ant-design/icons'
import { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { apiService } from '../../services/api'

interface PatrolPanelProps {
  currentTask?: any  // 当前活跃的巡逻任务
}

interface WaypointFile {
  name: string
  waypoint_count: number
  path?: string
  description?: string
}

function PatrolPanel({ currentTask }: PatrolPanelProps) {
  const { t } = useTranslation()
  const [form] = Form.useForm()
  const [loading, setLoading] = useState(false)
  const [waypointFiles, setWaypointFiles] = useState<WaypointFile[]>([])
  const [loadingFiles, setLoadingFiles] = useState(false)
  const [currentWaypoint, setCurrentWaypoint] = useState(0)
  const [totalWaypoints, setTotalWaypoints] = useState(0)

  // 加载路点文件列表
  const loadWaypointFiles = async () => {
    setLoadingFiles(true)
    try {
      const files = await apiService.waypoints.list()
      setWaypointFiles(files)
    } catch (error: any) {
      console.error('[PatrolPanel] 加载路点文件失败:', error)
      message.error(t('taskControl.patrol.loadFilesFailed') + ': ' + error.message)
    } finally {
      setLoadingFiles(false)
    }
  }

  useEffect(() => {
    loadWaypointFiles()
  }, [])

  // 监听当前任务进度
  useEffect(() => {
    if (currentTask && currentTask.action === 'start_patrol') {
      // 从任务结果中提取路点进度
      if (currentTask.result) {
        setCurrentWaypoint(currentTask.result.current_waypoint || 0)
        setTotalWaypoints(currentTask.result.total_waypoints || 0)
      }
    } else {
      setCurrentWaypoint(0)
      setTotalWaypoints(0)
    }
  }, [currentTask])

  const handleStartPatrol = async (values: any) => {
    setLoading(true)
    try {
      const response = await apiService.tasks.createPatrolTask({
        waypoint_file: values.waypoint_file,
        mode: values.mode,
      })
      
      message.success(
        t('taskControl.patrol.created', {
          requestId: response.request_id.substring(0, 8)
        })
      )
    } catch (error: any) {
      message.error(t('taskControl.patrol.createFailed') + ': ' + error.message)
    } finally {
      setLoading(false)
    }
  }

  // 判断是否有活跃的巡逻任务
  const hasActivePatrol = currentTask && 
    currentTask.action === 'start_patrol' && 
    ['queued', 'executing'].includes(currentTask.state)

  // 计算进度百分比
  const progressPercent = totalWaypoints > 0 
    ? Math.round((currentWaypoint / totalWaypoints) * 100)
    : 0

  return (
    <Card
      title={
        <Space>
          <SwapOutlined />
          {t('taskControl.patrol.title')}
        </Space>
      }
      style={{ width: '100%' }}
      extra={
        <Button
          icon={<ReloadOutlined />}
          size="small"
          onClick={loadWaypointFiles}
          loading={loadingFiles}
        >
          {t('common.refresh')}
        </Button>
      }
    >
      {waypointFiles.length === 0 && !loadingFiles ? (
        <Alert
          message={t('taskControl.patrol.noWaypointFiles')}
          description={t('taskControl.patrol.noWaypointFilesHint')}
          type="warning"
          showIcon
          style={{ marginBottom: 16 }}
        />
      ) : null}

      <Form
        form={form}
        layout="vertical"
        onFinish={handleStartPatrol}
        initialValues={{
          mode: 'loop'
        }}
      >
        <Form.Item
          name="waypoint_file"
          label={t('taskControl.patrol.waypointFile')}
          rules={[{ required: true, message: t('taskControl.patrol.selectFileRequired') }]}
        >
          <Select
            placeholder={t('taskControl.patrol.selectFilePlaceholder')}
            disabled={hasActivePatrol || loadingFiles}
            loading={loadingFiles}
            optionFilterProp="children"
            showSearch
          >
            {waypointFiles.map(file => (
              <Select.Option key={file.name} value={file.name}>
                {file.name} ({file.waypoint_count} {t('taskControl.patrol.waypoints')})
              </Select.Option>
            ))}
          </Select>
        </Form.Item>

        <Form.Item
          name="mode"
          label={t('taskControl.patrol.mode')}
          rules={[{ required: true }]}
        >
          <Select disabled={hasActivePatrol}>
            <Select.Option value="loop">
              🔁 {t('taskControl.patrol.modeLoop')}
            </Select.Option>
            <Select.Option value="once">
              ➡️ {t('taskControl.patrol.modeOnce')}
            </Select.Option>
            <Select.Option value="bounce">
              ↔️ {t('taskControl.patrol.modeBounce')}
            </Select.Option>
          </Select>
        </Form.Item>

        {/* 巡逻进度显示 */}
        {hasActivePatrol && totalWaypoints > 0 && (
          <div style={{ marginBottom: 16 }}>
            <div style={{ marginBottom: 8 }}>
              <strong>{t('taskControl.patrol.progress')}:</strong>
            </div>
            <Progress
              percent={progressPercent}
              status={currentTask.state === 'failed' ? 'exception' : 'active'}
              format={() => `${currentWaypoint}/${totalWaypoints}`}
            />
            <div style={{ marginTop: 8, display: 'flex', justifyContent: 'space-around' }}>
              <Statistic
                title={t('taskControl.patrol.currentWaypoint')}
                value={currentWaypoint}
                suffix={`/ ${totalWaypoints}`}
                valueStyle={{ fontSize: 16 }}
              />
              <Statistic
                title={t('taskControl.status.label')}
                value={t(`taskControl.status.${currentTask.state}`)}
                valueStyle={{ fontSize: 16 }}
              />
            </div>
          </div>
        )}

        <Form.Item>
          <Button
            type="primary"
            htmlType="submit"
            icon={<SwapOutlined />}
            loading={loading}
            disabled={hasActivePatrol || waypointFiles.length === 0}
            block
          >
            {hasActivePatrol
              ? t('taskControl.patrol.patrolling')
              : t('taskControl.patrol.start')}
          </Button>
        </Form.Item>

        {hasActivePatrol && (
          <div style={{ 
            padding: '8px 12px', 
            backgroundColor: '#e6f7ff', 
            border: '1px solid #91d5ff',
            borderRadius: '4px',
            fontSize: '13px'
          }}>
            💡 {t('taskControl.patrol.hint')}
          </div>
        )}
      </Form>
    </Card>
  )
}

export default PatrolPanel
