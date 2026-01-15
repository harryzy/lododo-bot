/**
 * Settings - 系统设置页面
 * 提供基础设置、导航参数、系统信息、高级设置四个模块
 */

import { useState, useEffect } from 'react'
import { Tabs, Card, Form, Select, Switch, Button, InputNumber, Statistic, Row, Col, message, Space } from 'antd'
import { SaveOutlined, ReloadOutlined } from '@ant-design/icons'
import { useTranslation } from 'react-i18next'
import { apiService } from '../../services/api'

const { TabPane } = Tabs
const { Option } = Select

interface SettingsData {
  basic: {
    language: string
    theme: string
    map_resolution: number
    update_rates: {
      map: number
      pose: number
      costmap: number
    }
  }
  navigation: {
    max_linear_velocity: number
    max_angular_velocity: number
    obstacle_distance: number
    charging_distance: number
    timeout: number
  }
  debug: {
    show_trajectory: boolean
    show_costmap: boolean
    show_path: boolean
    log_level: string
  }
  performance: {
    canvas_fps_limit: number
    websocket_queue_size: number
  }
}

interface SystemInfo {
  robot_name: string
  ros_version: string
  system_uptime: string
  websocket_status: string
  rosbridge_status: string
  fastapi_address: string
  maps_disk_usage: string
  waypoints_disk_usage: string
}

function Settings() {
  const { t, i18n } = useTranslation()
  const [form] = Form.useForm()
  const [loading, setLoading] = useState(false)
  const [systemInfo, setSystemInfo] = useState<SystemInfo | null>(null)
  const [activeTab, setActiveTab] = useState('basic')

  // 默认配置
  const defaultSettings: SettingsData = {
    basic: {
      language: 'zh-CN',
      theme: 'light',
      map_resolution: 0.05,
      update_rates: {
        map: 1,
        pose: 10,
        costmap: 5,
      },
    },
    navigation: {
      max_linear_velocity: 0.3,
      max_angular_velocity: 0.5,
      obstacle_distance: 0.3,
      charging_distance: 0.15,
      timeout: 300,
    },
    debug: {
      show_trajectory: true,
      show_costmap: true,
      show_path: true,
      log_level: 'INFO',
    },
    performance: {
      canvas_fps_limit: 30,
      websocket_queue_size: 100,
    },
  }

  // 加载设置
  useEffect(() => {
    loadSettings()
    loadSystemInfo()
  }, [])

  const loadSettings = async () => {
    try {
      setLoading(true)
      const response = await apiService.settings.get()
      if (response) {
        form.setFieldsValue(response)
      } else {
        // 从localStorage读取或使用默认值
        const savedSettings = localStorage.getItem('user_settings')
        if (savedSettings) {
          form.setFieldsValue(JSON.parse(savedSettings))
        } else {
          form.setFieldsValue(defaultSettings)
        }
      }
    } catch (error) {
      console.error('Load settings error:', error)
      // 加载本地缓存
      const savedSettings = localStorage.getItem('user_settings')
      if (savedSettings) {
        form.setFieldsValue(JSON.parse(savedSettings))
      } else {
        form.setFieldsValue(defaultSettings)
      }
    } finally {
      setLoading(false)
    }
  }

  const loadSystemInfo = async () => {
    try {
      const info = await apiService.settings.getSystemInfo()
      setSystemInfo(info)
    } catch (error) {
      console.error('Load system info error:', error)
    }
  }

  const handleSave = async () => {
    try {
      const values = await form.validateFields()
      setLoading(true)

      // 保存到localStorage
      localStorage.setItem('user_settings', JSON.stringify(values))

      // 保存到后端
      await apiService.settings.save(values)
      
      // 如果语言改变，立即切换
      if (values.basic.language !== i18n.language) {
        i18n.changeLanguage(values.basic.language)
        localStorage.setItem('preferred_language', values.basic.language)
      }

      message.success(t('settings.saveSuccess'))
    } catch (error: any) {
      console.error('Save settings error:', error)
      message.error(error.response?.data?.detail || t('settings.saveError'))
    } finally {
      setLoading(false)
    }
  }

  const handleReset = () => {
    form.setFieldsValue(defaultSettings)
    message.info(t('settings.resetSuccess'))
  }

  return (
    <div style={{ padding: '24px' }}>
      <Card>
        <Tabs activeKey={activeTab} onChange={setActiveTab}>
          {/* 基础设置 */}
          <TabPane tab={t('settings.tabs.basic')} key="basic">
            <Form
              form={form}
              layout="vertical"
              initialValues={defaultSettings}
            >
              <Form.Item
                name={['basic', 'language']}
                label={t('settings.basic.language')}
                tooltip={t('settings.basic.languageTooltip')}
              >
                <Select style={{ width: 200 }}>
                  <Option value="zh-CN">{t('language.zhCN')}</Option>
                  <Option value="en-US">{t('language.enUS')}</Option>
                </Select>
              </Form.Item>

              <Form.Item
                name={['basic', 'theme']}
                label={t('settings.basic.theme')}
                tooltip={t('settings.basic.themeTooltip')}
              >
                <Select style={{ width: 200 }} disabled>
                  <Option value="light">{t('settings.basic.lightTheme')}</Option>
                  <Option value="dark">{t('settings.basic.darkTheme')}</Option>
                </Select>
              </Form.Item>

              <Form.Item
                name={['basic', 'map_resolution']}
                label={t('settings.basic.mapResolution')}
                tooltip={t('settings.basic.mapResolutionTooltip')}
              >
                <InputNumber
                  min={0.01}
                  max={0.1}
                  step={0.01}
                  addonAfter="m"
                  style={{ width: 200 }}
                />
              </Form.Item>

              <h3>{t('settings.basic.updateRates')}</h3>
              <Row gutter={16}>
                <Col span={8}>
                  <Form.Item
                    name={['basic', 'update_rates', 'map']}
                    label={t('settings.basic.mapUpdateRate')}
                  >
                    <InputNumber
                      min={1}
                      max={10}
                      addonAfter="Hz"
                      style={{ width: '100%' }}
                    />
                  </Form.Item>
                </Col>
                <Col span={8}>
                  <Form.Item
                    name={['basic', 'update_rates', 'pose']}
                    label={t('settings.basic.poseUpdateRate')}
                  >
                    <InputNumber
                      min={5}
                      max={30}
                      addonAfter="Hz"
                      style={{ width: '100%' }}
                    />
                  </Form.Item>
                </Col>
                <Col span={8}>
                  <Form.Item
                    name={['basic', 'update_rates', 'costmap']}
                    label={t('settings.basic.costmapUpdateRate')}
                  >
                    <InputNumber
                      min={1}
                      max={10}
                      addonAfter="Hz"
                      style={{ width: '100%' }}
                    />
                  </Form.Item>
                </Col>
              </Row>
            </Form>
          </TabPane>

          {/* 导航参数 */}
          <TabPane tab={t('settings.tabs.navigation')} key="navigation">
            <Form form={form} layout="vertical">
              <h3>{t('settings.navigation.velocityLimits')}</h3>
              <Row gutter={16}>
                <Col span={12}>
                  <Form.Item
                    name={['navigation', 'max_linear_velocity']}
                    label={t('settings.navigation.maxLinearVelocity')}
                    tooltip={t('settings.navigation.maxLinearVelocityTooltip')}
                  >
                    <InputNumber
                      min={0.1}
                      max={1.0}
                      step={0.1}
                      addonAfter="m/s"
                      style={{ width: '100%' }}
                    />
                  </Form.Item>
                </Col>
                <Col span={12}>
                  <Form.Item
                    name={['navigation', 'max_angular_velocity']}
                    label={t('settings.navigation.maxAngularVelocity')}
                    tooltip={t('settings.navigation.maxAngularVelocityTooltip')}
                  >
                    <InputNumber
                      min={0.1}
                      max={2.0}
                      step={0.1}
                      addonAfter="rad/s"
                      style={{ width: '100%' }}
                    />
                  </Form.Item>
                </Col>
              </Row>

              <h3>{t('settings.navigation.safetyDistances')}</h3>
              <Row gutter={16}>
                <Col span={12}>
                  <Form.Item
                    name={['navigation', 'obstacle_distance']}
                    label={t('settings.navigation.obstacleDistance')}
                    tooltip={t('settings.navigation.obstacleDistanceTooltip')}
                  >
                    <InputNumber
                      min={0.1}
                      max={1.0}
                      step={0.1}
                      addonAfter="m"
                      style={{ width: '100%' }}
                    />
                  </Form.Item>
                </Col>
                <Col span={12}>
                  <Form.Item
                    name={['navigation', 'charging_distance']}
                    label={t('settings.navigation.chargingDistance')}
                    tooltip={t('settings.navigation.chargingDistanceTooltip')}
                  >
                    <InputNumber
                      min={0.05}
                      max={0.5}
                      step={0.05}
                      addonAfter="m"
                      style={{ width: '100%' }}
                    />
                  </Form.Item>
                </Col>
              </Row>

              <Form.Item
                name={['navigation', 'timeout']}
                label={t('settings.navigation.navigationTimeout')}
                tooltip={t('settings.navigation.navigationTimeoutTooltip')}
              >
                <InputNumber
                  min={30}
                  max={600}
                  step={30}
                  addonAfter={t('common.seconds')}
                  style={{ width: 200 }}
                />
              </Form.Item>
            </Form>
          </TabPane>

          {/* 系统信息 */}
          <TabPane tab={t('settings.tabs.systemInfo')} key="systemInfo">
            <Space direction="vertical" size="large" style={{ width: '100%' }}>
              <Card title={t('settings.systemInfo.robotInfo')} size="small">
                <Row gutter={16}>
                  <Col span={8}>
                    <Statistic
                      title={t('settings.systemInfo.robotName')}
                      value={systemInfo?.robot_name || 'lododo Robot'}
                    />
                  </Col>
                  <Col span={8}>
                    <Statistic
                      title={t('settings.systemInfo.rosVersion')}
                      value={systemInfo?.ros_version || 'ROS2 Humble'}
                    />
                  </Col>
                  <Col span={8}>
                    <Statistic
                      title={t('settings.systemInfo.systemUptime')}
                      value={systemInfo?.system_uptime || 'N/A'}
                    />
                  </Col>
                </Row>
              </Card>

              <Card title={t('settings.systemInfo.networkInfo')} size="small">
                <Row gutter={16}>
                  <Col span={8}>
                    <Statistic
                      title={t('settings.systemInfo.websocketStatus')}
                      value={systemInfo?.websocket_status || t('connection.disconnected')}
                      valueStyle={{ color: systemInfo?.websocket_status === 'Connected' ? '#3f8600' : '#cf1322' }}
                    />
                  </Col>
                  <Col span={8}>
                    <Statistic
                      title={t('settings.systemInfo.rosbridgeStatus')}
                      value={systemInfo?.rosbridge_status || t('connection.disconnected')}
                      valueStyle={{ color: systemInfo?.rosbridge_status === 'Connected' ? '#3f8600' : '#cf1322' }}
                    />
                  </Col>
                  <Col span={8}>
                    <Statistic
                      title={t('settings.systemInfo.fastapiAddress')}
                      value={systemInfo?.fastapi_address || 'localhost:8000'}
                    />
                  </Col>
                </Row>
              </Card>

              <Card title={t('settings.systemInfo.diskUsage')} size="small">
                <Row gutter={16}>
                  <Col span={12}>
                    <Statistic
                      title={t('settings.systemInfo.mapsDiskUsage')}
                      value={systemInfo?.maps_disk_usage || 'N/A'}
                    />
                  </Col>
                  <Col span={12}>
                    <Statistic
                      title={t('settings.systemInfo.waypointsDiskUsage')}
                      value={systemInfo?.waypoints_disk_usage || 'N/A'}
                    />
                  </Col>
                </Row>
              </Card>

              <Button icon={<ReloadOutlined />} onClick={loadSystemInfo}>
                {t('common.refresh')}
              </Button>
            </Space>
          </TabPane>

          {/* 高级设置 */}
          <TabPane tab={t('settings.tabs.advanced')} key="advanced">
            <Form form={form} layout="vertical">
              <h3>{t('settings.advanced.debugOptions')}</h3>
              <Row gutter={16}>
                <Col span={8}>
                  <Form.Item
                    name={['debug', 'show_trajectory']}
                    label={t('settings.advanced.showTrajectory')}
                    valuePropName="checked"
                  >
                    <Switch />
                  </Form.Item>
                </Col>
                <Col span={8}>
                  <Form.Item
                    name={['debug', 'show_costmap']}
                    label={t('settings.advanced.showCostmap')}
                    valuePropName="checked"
                  >
                    <Switch />
                  </Form.Item>
                </Col>
                <Col span={8}>
                  <Form.Item
                    name={['debug', 'show_path']}
                    label={t('settings.advanced.showPath')}
                    valuePropName="checked"
                  >
                    <Switch />
                  </Form.Item>
                </Col>
              </Row>

              <Form.Item
                name={['debug', 'log_level']}
                label={t('settings.advanced.logLevel')}
                tooltip={t('settings.advanced.logLevelTooltip')}
              >
                <Select style={{ width: 200 }}>
                  <Option value="DEBUG">DEBUG</Option>
                  <Option value="INFO">INFO</Option>
                  <Option value="WARNING">WARNING</Option>
                  <Option value="ERROR">ERROR</Option>
                </Select>
              </Form.Item>

              <h3>{t('settings.advanced.performance')}</h3>
              <Row gutter={16}>
                <Col span={12}>
                  <Form.Item
                    name={['performance', 'canvas_fps_limit']}
                    label={t('settings.advanced.canvasFpsLimit')}
                    tooltip={t('settings.advanced.canvasFpsLimitTooltip')}
                  >
                    <InputNumber
                      min={10}
                      max={60}
                      step={5}
                      addonAfter="FPS"
                      style={{ width: '100%' }}
                    />
                  </Form.Item>
                </Col>
                <Col span={12}>
                  <Form.Item
                    name={['performance', 'websocket_queue_size']}
                    label={t('settings.advanced.websocketQueueSize')}
                    tooltip={t('settings.advanced.websocketQueueSizeTooltip')}
                  >
                    <InputNumber
                      min={10}
                      max={1000}
                      step={10}
                      style={{ width: '100%' }}
                    />
                  </Form.Item>
                </Col>
              </Row>
            </Form>
          </TabPane>
        </Tabs>

        {/* 操作按钮 */}
        <div style={{ marginTop: 24, textAlign: 'right' }}>
          <Space>
            <Button icon={<ReloadOutlined />} onClick={handleReset}>
              {t('settings.resetToDefault')}
            </Button>
            <Button
              type="primary"
              icon={<SaveOutlined />}
              loading={loading}
              onClick={handleSave}
            >
              {t('common.save')}
            </Button>
          </Space>
        </div>
      </Card>
    </div>
  )
}

export default Settings
