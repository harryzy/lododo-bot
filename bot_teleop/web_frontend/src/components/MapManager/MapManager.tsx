import React, { useEffect, useState } from 'react';
import {
  Card,
  Table,
  Button,
  Space,
  Tag,
  Image,
  Typography,
  Statistic,
  Row,
  Col,
  message,
  Modal,
  Popconfirm,
  Tooltip,
} from 'antd';
import {
  ReloadOutlined,
  DatabaseOutlined,
  FileImageOutlined,
  FileTextOutlined,
  DeleteOutlined,
  PlayCircleOutlined,
  HistoryOutlined,
  CheckCircleOutlined,
  SwapOutlined,
} from '@ant-design/icons';
import { useTranslation } from 'react-i18next';
import { apiService } from '../../services/api';

const { Title, Text } = Typography;

interface MapInfo {
  name: string;
  version: number;
  versions: number[];  // 所有可用版本
  size: string;
  resolution: number;
  created_at: string;
  updated_at?: string;
  description?: string;
  tags: string[];
  thumbnail_url?: string;
  has_rtabmap_db: boolean;
  has_pgm: boolean;
  has_yaml: boolean;
}

interface VersionInfo {
  version: number;
  is_current: boolean;
  size: string;
  has_rtabmap_db: boolean;
  has_pgm: boolean;
  has_yaml: boolean;
  created_at?: string;
}

const MapManager: React.FC = () => {
  const { t } = useTranslation();
  const [maps, setMaps] = useState<MapInfo[]>([]);
  const [loading, setLoading] = useState(false);
  const [expandedRowKeys, setExpandedRowKeys] = useState<React.Key[]>([]);
  const [versionData, setVersionData] = useState<Record<string, VersionInfo[]>>({});
  const [loadingVersions, setLoadingVersions] = useState<Record<string, boolean>>({});

  useEffect(() => {
    loadMaps();
  }, []);

  // 加载地图列表
  const loadMaps = async () => {
    setLoading(true);
    try {
      const response = await apiService.maps.list();
      if (response?.maps) {
        setMaps(response.maps);
      }
    } catch (error) {
      message.error(t('maps.loadError'));
      console.error('Load maps error:', error);
    } finally {
      setLoading(false);
    }
  };

  // 加载地图版本详情
  const loadVersions = async (mapName: string) => {
    if (versionData[mapName]) {
      return; // 已加载
    }

    setLoadingVersions(prev => ({ ...prev, [mapName]: true }));
    try {
      const response = await apiService.maps.getVersions(mapName);
      if (response?.versions) {
        setVersionData(prev => ({
          ...prev,
          [mapName]: response.versions,
        }));
      }
    } catch (error) {
      message.error(`${t('maps.version.loadError')}: ${mapName}`);
      console.error('Load versions error:', error);
    } finally {
      setLoadingVersions(prev => ({ ...prev, [mapName]: false }));
    }
  };

  // 展开行时加载版本信息
  const handleExpand = (expanded: boolean, record: MapInfo) => {
    if (expanded) {
      loadVersions(record.name);
      setExpandedRowKeys([...expandedRowKeys, record.name]);
    } else {
      setExpandedRowKeys(expandedRowKeys.filter(key => key !== record.name));
    }
  };

  // 切换版本
  const handleSwitchVersion = async (mapName: string, version: number) => {
    try {
      await apiService.maps.switchVersion(mapName, version);
      message.success(t('maps.version.switched', { version }));
      // 重新加载地图列表和版本信息
      await loadMaps();
      delete versionData[mapName]; // 清除缓存
      await loadVersions(mapName);
    } catch (error: any) {
      message.error(error.response?.data?.detail || t('maps.version.switchError'));
      console.error('Switch version error:', error);
    }
  };

  // 删除版本
  const handleDeleteVersion = async (mapName: string, version: number) => {
    try {
      await apiService.maps.deleteVersion(mapName, version);
      message.success(t('maps.version.deleted', { version }));
      // 重新加载
      await loadMaps();
      delete versionData[mapName];
      await loadVersions(mapName);
    } catch (error: any) {
      message.error(error.response?.data?.detail || t('maps.version.deleteError'));
      console.error('Delete version error:', error);
    }
  };

  // 加载地图
  const handleLoadMap = (record: MapInfo) => {
    Modal.confirm({
      title: t('maps.loadConfirm'),
      content: t('maps.loadWarning', { name: record.name }),
      okText: t('common.confirm'),
      cancelText: t('common.cancel'),
      onOk: async () => {
        try {
          await apiService.maps.load({ map_name: record.name });
          message.success(t('maps.loadSuccess'));
        } catch (error) {
          message.error(t('maps.loadError'));
          console.error('Load map error:', error);
        }
      },
    });
  };

  // 删除整个地图
  const handleDeleteMap = async (name: string) => {
    try {
      await apiService.maps.delete(name);
      message.success(t('maps.deleteSuccess'));
      loadMaps();
    } catch (error) {
      message.error(t('maps.deleteError'));
      console.error('Delete map error:', error);
    }
  };

  // 渲染版本展开内容
  const expandedRowRender = (record: MapInfo) => {
    const versions = versionData[record.name] || [];
    const isLoading = loadingVersions[record.name];

    if (isLoading) {
      return <div style={{ padding: '16px', textAlign: 'center' }}>加载版本信息...</div>;
    }

    const versionColumns = [
      {
        title: t('maps.version.number'),
        dataIndex: 'version',
        key: 'version',
        width: 100,
        render: (version: number, record: VersionInfo) => (
          <Space>
            <Text strong>v{version}</Text>
            {record.is_current && (
              <Tag color="blue" icon={<CheckCircleOutlined />}>
                {t('maps.version.current')}
              </Tag>
            )}
          </Space>
        ),
      },
      {
        title: t('maps.size'),
        dataIndex: 'size',
        key: 'size',
        width: 120,
      },
      {
        title: t('maps.files'),
        key: 'files',
        width: 150,
        render: (_: any, record: VersionInfo) => (
          <Space>
            {record.has_rtabmap_db && <Tag color="green" icon={<DatabaseOutlined />}>DB</Tag>}
            {record.has_pgm && <Tag color="blue" icon={<FileImageOutlined />}>PGM</Tag>}
            {record.has_yaml && <Tag color="orange" icon={<FileTextOutlined />}>YAML</Tag>}
          </Space>
        ),
      },
      {
        title: t('maps.actions'),
        key: 'actions',
        width: 200,
        render: (_: any, versionRecord: VersionInfo) => (
          <Space>
            {!versionRecord.is_current && (
              <Popconfirm
                title={t('maps.version.switchConfirm')}
                onConfirm={() => handleSwitchVersion(record.name, versionRecord.version)}
                okText={t('common.confirm')}
                cancelText={t('common.cancel')}
              >
                <Button
                  type="link"
                  size="small"
                  icon={<SwapOutlined />}
                >
                  {t('maps.version.switchTo')}
                </Button>
              </Popconfirm>
            )}
            {!versionRecord.is_current && (
              <Popconfirm
                title={t('maps.version.deleteConfirm', { version: versionRecord.version })}
                onConfirm={() => handleDeleteVersion(record.name, versionRecord.version)}
                okText={t('common.confirm')}
                cancelText={t('common.cancel')}
              >
                <Button
                  type="link"
                  size="small"
                  danger
                  icon={<DeleteOutlined />}
                >
                  {t('common.delete')}
                </Button>
              </Popconfirm>
            )}
          </Space>
        ),
      },
    ];

    return (
      <Table
        columns={versionColumns}
        dataSource={versions}
        rowKey="version"
        pagination={false}
        size="small"
        style={{ marginLeft: 48 }}
      />
    );
  };

  // 主表格列定义
  const columns = [
    {
      title: t('maps.thumbnail'),
      dataIndex: 'thumbnail_url',
      key: 'thumbnail',
      width: 100,
      render: (_: string, record: MapInfo) => (
        <Image
          width={60}
          height={60}
          src={apiService.maps.getThumbnailUrl(record.name)}
          fallback="/placeholder.png"
          preview={{
            src: apiService.maps.getThumbnailUrl(record.name),
          }}
        />
      ),
    },
    {
      title: t('maps.name'),
      dataIndex: 'name',
      key: 'name',
      width: 150,
      render: (name: string) => <Text strong>{name}</Text>,
    },
    {
      title: t('maps.version.title'),
      dataIndex: 'version',
      key: 'version',
      width: 120,
      render: (version: number, record: MapInfo) => (
        <Space>
          <Tag color="blue">v{version}</Tag>
          {record.versions.length > 1 && (
            <Tooltip title={t('maps.version.hasHistory', { count: record.versions.length })}>
              <HistoryOutlined style={{ color: '#1890ff' }} />
            </Tooltip>
          )}
        </Space>
      ),
    },
    {
      title: t('maps.size'),
      dataIndex: 'size',
      key: 'size',
      width: 100,
    },
    {
      title: t('maps.resolution'),
      dataIndex: 'resolution',
      key: 'resolution',
      width: 100,
      render: (res: number) => `${res} m`,
    },
    {
      title: t('maps.files'),
      key: 'files',
      width: 200,
      render: (_: any, record: MapInfo) => (
        <Space>
          {record.has_rtabmap_db && <Tag color="green" icon={<DatabaseOutlined />}>DB</Tag>}
          {record.has_pgm && <Tag color="blue" icon={<FileImageOutlined />}>PGM</Tag>}
          {record.has_yaml && <Tag color="orange" icon={<FileTextOutlined />}>YAML</Tag>}
        </Space>
      ),
    },
    {
      title: t('maps.actions'),
      key: 'actions',
      width: 200,
      fixed: 'right' as const,
      render: (_: any, record: MapInfo) => (
        <Space>
          <Button
            type="primary"
            size="small"
            icon={<PlayCircleOutlined />}
            onClick={() => handleLoadMap(record)}
          >
            {t('maps.load')}
          </Button>
          <Popconfirm
            title={t('maps.deleteConfirm')}
            description={t('maps.deleteWarning', { name: record.name })}
            onConfirm={() => handleDeleteMap(record.name)}
            okText={t('common.confirm')}
            cancelText={t('common.cancel')}
            okButtonProps={{ danger: true }}
          >
            <Button size="small" danger icon={<DeleteOutlined />}>
              {t('common.delete')}
            </Button>
          </Popconfirm>
        </Space>
      ),
    },
    {
      title: t('maps.tags'),
      dataIndex: 'tags',
      key: 'tags',
      width: 180,
      render: (tags: string[]) => (
        <>
          {tags.map((tag) => (
            <Tag key={tag} color="default">
              {tag}
            </Tag>
          ))}
        </>
      ),
    },
    {
      title: t('maps.createdAt'),
      dataIndex: 'created_at',
      key: 'created_at',
      width: 180,
      render: (time: string) => new Date(time).toLocaleString(),
    },
  ];

  // 统计信息
  const stats = {
    total: maps.length,
    rtabmap: maps.filter((m) => m.has_rtabmap_db).length,
    lastUpdated: maps.length > 0 ? maps[0].updated_at || maps[0].created_at : undefined,
  };

  return (
    <div style={{ padding: '24px' }}>
      <Title level={2}>{t('maps.title')}</Title>

      {/* 统计卡片 */}
      <Row gutter={16} style={{ marginBottom: 24 }}>
        <Col span={6}>
          <Card>
            <Statistic title={t('maps.totalMaps')} value={stats.total} prefix={<DatabaseOutlined />} />
          </Card>
        </Col>
        <Col span={6}>
          <Card>
            <Statistic
              title={t('maps.rtabmapMaps')}
              value={stats.rtabmap}
              prefix={<DatabaseOutlined />}
              valueStyle={{ color: '#3f8600' }}
            />
          </Card>
        </Col>
        <Col span={12}>
          <Card>
            <Statistic
              title={t('maps.lastUpdated')}
              value={stats.lastUpdated ? new Date(stats.lastUpdated).toLocaleString() : '-'}
            />
          </Card>
        </Col>
      </Row>

      {/* 地图表格 */}
      <Card
        title={t('maps.mapList')}
        extra={
          <Button icon={<ReloadOutlined />} onClick={loadMaps} loading={loading}>
            {t('common.refresh')}
          </Button>
        }
      >
        <Table
          columns={columns}
          dataSource={maps}
          rowKey="name"
          loading={loading}
          expandable={{
            expandedRowRender,
            onExpand: handleExpand,
            expandedRowKeys,
            expandIcon: ({ expanded, onExpand, record }) => {
              const map = record as MapInfo;
              if (map.versions.length <= 1) return null;
              return expanded ? (
                <Button
                  type="text"
                  size="small"
                  onClick={(e) => onExpand(record, e)}
                  icon={<HistoryOutlined />}
                />
              ) : (
                <Button
                  type="text"
                  size="small"
                  onClick={(e) => onExpand(record, e)}
                  icon={<HistoryOutlined />}
                />
              );
            },
          }}
          scroll={{ x: 1400 }}
          pagination={{
            pageSize: 10,
            showSizeChanger: true,
            showTotal: (total) => `${t('common.total')} ${total} ${t('maps.items')}`,
          }}
        />
      </Card>
    </div>
  );
};

export default MapManager;
