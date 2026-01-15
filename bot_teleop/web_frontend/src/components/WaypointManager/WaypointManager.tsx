import React, { useEffect, useState } from 'react';
import {
  Card,
  Table,
  Button,
  Space,
  Typography,
  Statistic,
  Row,
  Col,
  message,
  Modal,
  Popconfirm,
  Tooltip,
  Tag,
  Input,
  Form,
} from 'antd';
import {
  ReloadOutlined,
  EditOutlined,
  DeleteOutlined,
  EyeOutlined,
  UploadOutlined,
  VideoCameraOutlined,
  CheckOutlined,
  StopOutlined,
  EnvironmentOutlined,
} from '@ant-design/icons';
import { useTranslation } from 'react-i18next';
import { apiService } from '../../services/api';

const { Title, Text } = Typography;

// 路点数据类型
interface Waypoint {
  name: string;
  x: number;
  y: number;
  yaw: number;
  dwell_time: number;
}

// 路点路线类型 - 与API定义保持一致
interface WaypointRoute {
  name: string;
  waypoint_count: number;
  path?: string;
  description?: string;
}

const WaypointManager: React.FC = () => {
  const { t } = useTranslation();
  const [routes, setRoutes] = useState<WaypointRoute[]>([]);
  const [loading, setLoading] = useState(false);
  const [selectedRoute, setSelectedRoute] = useState<WaypointRoute | null>(null);
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [viewModalVisible, setViewModalVisible] = useState(false);
  const [editModalVisible, setEditModalVisible] = useState(false);
  const [editingWaypoints, setEditingWaypoints] = useState<Waypoint[]>([]);
  
  // 录制相关状态
  const [recording, setRecording] = useState(false);
  const [saveModalVisible, setSaveModalVisible] = useState(false);
  const [form] = Form.useForm();
  const [editForm] = Form.useForm();

  useEffect(() => {
    loadRoutes();
    checkRecordingStatus(); // 检查初始录制状态
  }, []);

  // 加载路点列表
  const loadRoutes = async () => {
    setLoading(true);
    try {
      const response = await apiService.waypoints.list();
      if (response && Array.isArray(response)) {
        setRoutes(response);
      }
    } catch (error) {
      message.error(t('waypoints.loadError'));
      console.error('Load waypoint routes error:', error);
    } finally {
      setLoading(false);
    }
  };

  // 检查录制状态
  const checkRecordingStatus = async () => {
    try {
      const response = await apiService.waypoints.getRecordingStatus();
      if (response?.recording) {
        setRecording(true);
      }
    } catch (error) {
      console.error('Check recording status error:', error);
    }
  };

  // 开始录制
  const handleStartRecording = async () => {
    try {
      const response = await apiService.waypoints.startRecording();
      if (response?.success) {
        setRecording(true);
        message.success(t('waypoints.recordingStarted'));
      }
    } catch (error: any) {
      if (error?.response?.status === 503) {
        message.error(t('waypoints.serviceUnavailable'));
      } else {
        message.error(t('waypoints.recordingError'));
      }
      console.error('Start recording error:', error);
    }
  };

  // 标记路点
  const handleMarkWaypoint = async () => {
    try {
      const response = await apiService.waypoints.markWaypoint();
      if (response?.success) {
        message.success(t('waypoints.waypointMarked'));
      }
    } catch (error) {
      message.error(t('waypoints.recordingError'));
      console.error('Mark waypoint error:', error);
    }
  };

  // 停止录制
  const handleStopRecording = async () => {
    try {
      const response = await apiService.waypoints.stopRecording();
      if (response?.success) {
        setRecording(false);
        message.success(t('waypoints.recordingStopped'));
      }
    } catch (error) {
      message.error(t('waypoints.recordingError'));
      console.error('Stop recording error:', error);
    }
  };

  // 保存录制
  const handleSaveRecording = () => {
    form.resetFields();
    setSaveModalVisible(true);
  };

  // 确认保存
  const handleConfirmSave = async () => {
    try {
      const values = await form.validateFields();
      const response = await apiService.waypoints.saveRecording(
        values.routeName,
        values.description
      );
      
      if (response?.success) {
        setRecording(false);
        setSaveModalVisible(false);
        message.success(t('waypoints.recordingSaved'));
        loadRoutes(); // 重新加载列表
      }
    } catch (error: any) {
      if (error.errorFields) {
        // 表单验证失败
        return;
      }
      message.error(t('waypoints.recordingError'));
      console.error('Save recording error:', error);
    }
  };

  // 查看路点详情
  const handleViewRoute = async (route: WaypointRoute) => {
    try {
      const response = await apiService.waypoints.getRoute(route.name);
      if (response?.waypoints) {
        setWaypoints(response.waypoints);
        setSelectedRoute(route);
        setViewModalVisible(true);
      }
    } catch (error) {
      message.error(t('waypoints.viewError'));
      console.error('View route error:', error);
    }
  };

  // 删除路点路线
  const handleDeleteRoute = async (routeName: string) => {
    try {
      await apiService.waypoints.deleteRoute(routeName);
      message.success(t('waypoints.deleteSuccess'));
      loadRoutes();
    } catch (error) {
      message.error(t('waypoints.deleteError'));
      console.error('Delete route error:', error);
    }
  };

  // 编辑路点路线
  const handleEditRoute = async (route: WaypointRoute) => {
    try {
      const response = await apiService.waypoints.getRoute(route.name);
      if (response?.waypoints) {
        setSelectedRoute(route);
        setEditingWaypoints([...response.waypoints]); // 创建副本用于编辑
        editForm.setFieldsValue({
          description: route.description || ''
        });
        setEditModalVisible(true);
      }
    } catch (error) {
      message.error(t('waypoints.viewError'));
      console.error('Load route for editing error:', error);
    }
  };

  // 更新路点字段
  const handleUpdateWaypoint = (index: number, field: keyof Waypoint, value: any) => {
    const updated = [...editingWaypoints];
    updated[index] = { ...updated[index], [field]: value };
    setEditingWaypoints(updated);
  };

  // 删除路点
  const handleDeleteWaypoint = (index: number) => {
    const updated = editingWaypoints.filter((_, i) => i !== index);
    setEditingWaypoints(updated);
  };

  // 添加新路点
  const handleAddWaypoint = () => {
    const newWaypoint: Waypoint = {
      name: `point_${editingWaypoints.length + 1}`,
      x: 0.0,
      y: 0.0,
      yaw: 0.0,
      dwell_time: 2.0
    };
    setEditingWaypoints([...editingWaypoints, newWaypoint]);
  };

  // 保存编辑
  const handleSaveEdit = async () => {
    if (!selectedRoute) return;

    try {
      const values = await editForm.validateFields();
      
      // 验证路点数据
      if (editingWaypoints.length === 0) {
        message.error(t('waypoints.noWaypointsError'));
        return;
      }

      // 保存到后端
      await apiService.waypoints.updateRoute(
        selectedRoute.name,
        editingWaypoints,
        values.description
      );

      message.success(t('waypoints.editSuccess'));
      setEditModalVisible(false);
      loadRoutes();
    } catch (error: any) {
      if (error.errorFields) {
        // 表单验证失败
        return;
      }
      message.error(t('waypoints.editError'));
      console.error('Save edit error:', error);
    }
  };

  // 表格列定义
  const columns = [
    {
      title: t('waypoints.routeName'),
      dataIndex: 'name',
      key: 'name',
      width: 200,
      render: (name: string) => <Text strong>{name}</Text>,
    },
    {
      title: t('waypoints.waypointCount'),
      dataIndex: 'waypoint_count',
      key: 'waypoint_count',
      width: 120,
      render: (count: number) => (
        <Tag color="blue">{count} {t('waypoints.points')}</Tag>
      ),
    },
    {
      title: t('waypoints.description'),
      dataIndex: 'description',
      key: 'description',
      ellipsis: true,
      render: (desc: string) => desc || <Text type="secondary">-</Text>,
    },
    {
      title: t('common.actions'),
      key: 'actions',
      width: 200,
      fixed: 'right' as const,
      render: (_: any, record: WaypointRoute) => (
        <Space>
          <Tooltip title={t('waypoints.view')}>
            <Button
              size="small"
              icon={<EyeOutlined />}
              onClick={() => handleViewRoute(record)}
            />
          </Tooltip>
          <Tooltip title={t('waypoints.edit')}>
            <Button
              size="small"
              icon={<EditOutlined />}
              onClick={() => handleEditRoute(record)}
            />
          </Tooltip>
          <Popconfirm
            title={t('waypoints.deleteConfirm')}
            description={t('waypoints.deleteWarning', { name: record.name })}
            onConfirm={() => handleDeleteRoute(record.name)}
            okText={t('common.confirm')}
            cancelText={t('common.cancel')}
            okButtonProps={{ danger: true }}
          >
            <Button size="small" danger icon={<DeleteOutlined />} />
          </Popconfirm>
        </Space>
      ),
    },
  ];

  // 路点详情表格列
  const waypointColumns = [
    {
      title: t('waypoints.waypointName'),
      dataIndex: 'name',
      key: 'name',
      width: 150,
    },
    {
      title: 'X',
      dataIndex: 'x',
      key: 'x',
      width: 100,
      render: (val: number) => val.toFixed(2),
    },
    {
      title: 'Y',
      dataIndex: 'y',
      key: 'y',
      width: 100,
      render: (val: number) => val.toFixed(2),
    },
    {
      title: 'Yaw',
      dataIndex: 'yaw',
      key: 'yaw',
      width: 100,
      render: (val: number) => val.toFixed(2),
    },
    {
      title: t('waypoints.dwellTime'),
      dataIndex: 'dwell_time',
      key: 'dwell_time',
      width: 100,
      render: (val: number) => `${val.toFixed(1)}s`,
    },
  ];

  return (
    <div style={{ padding: '24px' }}>
      {/* 统计卡片 */}
      <Row gutter={16} style={{ marginBottom: 24 }}>
        <Col span={6}>
          <Card>
            <Statistic
              title={t('waypoints.totalRoutes')}
              value={routes.length}
              suffix={t('waypoints.routes')}
            />
          </Card>
        </Col>
        <Col span={6}>
          <Card>
            <Statistic
              title={t('waypoints.recordingStatus')}
              value={recording ? t('waypoints.recording') : t('waypoints.notRecording')}
              valueStyle={{ color: recording ? '#cf1322' : '#3f8600' }}
            />
          </Card>
        </Col>
        <Col span={12}>
          <Card>
            <Space size="middle" style={{ width: '100%', justifyContent: 'flex-end' }}>
              {!recording ? (
                <Button
                  type="primary"
                  icon={<VideoCameraOutlined />}
                  onClick={handleStartRecording}
                  size="large"
                >
                  {t('waypoints.startRecording')}
                </Button>
              ) : (
                <>
                  <Button
                    type="primary"
                    icon={<EnvironmentOutlined />}
                    onClick={handleMarkWaypoint}
                    size="large"
                  >
                    {t('waypoints.markWaypoint')}
                  </Button>
                  <Button
                    icon={<CheckOutlined />}
                    onClick={handleSaveRecording}
                    size="large"
                    style={{ backgroundColor: '#52c41a', color: 'white' }}
                  >
                    {t('waypoints.saveRecording')}
                  </Button>
                  <Button
                    danger
                    icon={<StopOutlined />}
                    onClick={handleStopRecording}
                    size="large"
                  >
                    {t('waypoints.stopRecording')}
                  </Button>
                </>
              )}
            </Space>
          </Card>
        </Col>
      </Row>

      {/* 路点列表 */}
      <Card
        title={
          <Space>
            <Title level={4} style={{ margin: 0 }}>
              {t('waypoints.routeList')}
            </Title>
          </Space>
        }
        extra={
          <Space>
            <Button icon={<UploadOutlined />} onClick={() => message.info('导入功能开发中')}>
              {t('waypoints.import')}
            </Button>
            <Button icon={<ReloadOutlined />} onClick={loadRoutes} loading={loading}>
              {t('common.refresh')}
            </Button>
          </Space>
        }
      >
        <Table
          columns={columns}
          dataSource={routes}
          loading={loading}
          rowKey="name"
          pagination={{
            pageSize: 10,
            showSizeChanger: true,
            showTotal: (total) => `${t('common.total')} ${total} ${t('waypoints.routes')}`,
          }}
        />
      </Card>

      {/* 查看路点详情对话框 */}
      <Modal
        title={t('waypoints.routeDetails', { name: selectedRoute?.name })}
        open={viewModalVisible}
        onCancel={() => {
          setViewModalVisible(false);
          setSelectedRoute(null);
          setWaypoints([]);
        }}
        footer={[
          <Button key="close" onClick={() => setViewModalVisible(false)}>
            {t('common.cancel')}
          </Button>,
        ]}
        width={800}
      >
        <div style={{ marginBottom: 16 }}>
          <Text type="secondary">
            {selectedRoute?.description || t('waypoints.noDescription')}
          </Text>
        </div>
        <Table
          columns={waypointColumns}
          dataSource={waypoints}
          rowKey="name"
          pagination={false}
          size="small"
        />
      </Modal>

      {/* 保存录制 Modal */}
      <Modal
        title={t('waypoints.saveRecordingTitle')}
        open={saveModalVisible}
        onOk={handleConfirmSave}
        onCancel={() => setSaveModalVisible(false)}
        okText={t('common.confirm')}
        cancelText={t('common.cancel')}
      >
        <Form form={form} layout="vertical">
          <Form.Item
            name="routeName"
            label={t('waypoints.routeName')}
            rules={[{ required: true, message: t('waypoints.enterRouteName') }]}
          >
            <Input placeholder={t('waypoints.routeNamePlaceholder')} />
          </Form.Item>
          <Form.Item
            name="description"
            label={t('waypoints.description')}
          >
            <Input.TextArea
              rows={3}
              placeholder={t('waypoints.routeDescriptionPlaceholder')}
            />
          </Form.Item>
        </Form>
      </Modal>

      {/* 编辑路点 Modal */}
      <Modal
        title={t('waypoints.editRoute', { name: selectedRoute?.name })}
        open={editModalVisible}
        onOk={handleSaveEdit}
        onCancel={() => {
          setEditModalVisible(false);
          setSelectedRoute(null);
          setEditingWaypoints([]);
        }}
        okText={t('common.confirm')}
        cancelText={t('common.cancel')}
        width={1000}
      >
        <Form form={editForm} layout="vertical">
          <Form.Item
            name="description"
            label={t('waypoints.description')}
          >
            <Input.TextArea
              rows={2}
              placeholder={t('waypoints.routeDescriptionPlaceholder')}
            />
          </Form.Item>
        </Form>

        <div style={{ marginBottom: 16 }}>
          <Button
            type="dashed"
            onClick={handleAddWaypoint}
            icon={<UploadOutlined />}
            style={{ width: '100%' }}
          >
            {t('waypoints.addWaypoint')}
          </Button>
        </div>

        <Table
          dataSource={editingWaypoints}
          rowKey={(record, index) => `${record.name}_${index}`}
          pagination={false}
          size="small"
          scroll={{ y: 400 }}
          columns={[
            {
              title: t('waypoints.waypointName'),
              dataIndex: 'name',
              key: 'name',
              width: 150,
              render: (value: string, _record: Waypoint, index: number) => (
                <Input
                  value={value}
                  onChange={(e) => handleUpdateWaypoint(index, 'name', e.target.value)}
                  size="small"
                />
              ),
            },
            {
              title: 'X',
              dataIndex: 'x',
              key: 'x',
              width: 100,
              render: (value: number, _record: Waypoint, index: number) => (
                <Input
                  type="number"
                  value={value}
                  onChange={(e) => handleUpdateWaypoint(index, 'x', parseFloat(e.target.value) || 0)}
                  size="small"
                  step="0.01"
                />
              ),
            },
            {
              title: 'Y',
              dataIndex: 'y',
              key: 'y',
              width: 100,
              render: (value: number, _record: Waypoint, index: number) => (
                <Input
                  type="number"
                  value={value}
                  onChange={(e) => handleUpdateWaypoint(index, 'y', parseFloat(e.target.value) || 0)}
                  size="small"
                  step="0.01"
                />
              ),
            },
            {
              title: 'Yaw',
              dataIndex: 'yaw',
              key: 'yaw',
              width: 100,
              render: (value: number, _record: Waypoint, index: number) => (
                <Input
                  type="number"
                  value={value}
                  onChange={(e) => handleUpdateWaypoint(index, 'yaw', parseFloat(e.target.value) || 0)}
                  size="small"
                  step="0.01"
                />
              ),
            },
            {
              title: t('waypoints.dwellTime'),
              dataIndex: 'dwell_time',
              key: 'dwell_time',
              width: 100,
              render: (value: number, _record: Waypoint, index: number) => (
                <Input
                  type="number"
                  value={value}
                  onChange={(e) => handleUpdateWaypoint(index, 'dwell_time', parseFloat(e.target.value) || 0)}
                  size="small"
                  step="0.1"
                  suffix="s"
                />
              ),
            },
            {
              title: t('common.actions'),
              key: 'actions',
              width: 80,
              fixed: 'right' as const,
              render: (_: any, _record: Waypoint, index: number) => (
                <Popconfirm
                  title={t('waypoints.deleteWaypointConfirm')}
                  onConfirm={() => handleDeleteWaypoint(index)}
                  okText={t('common.confirm')}
                  cancelText={t('common.cancel')}
                  okButtonProps={{ danger: true }}
                >
                  <Button size="small" danger icon={<DeleteOutlined />} />
                </Popconfirm>
              ),
            },
          ]}
        />
      </Modal>
    </div>
  );
};

export default WaypointManager;
