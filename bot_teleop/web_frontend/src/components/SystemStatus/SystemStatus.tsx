import React, { useEffect, useState } from 'react';
import { Card, Space, Tag, Tooltip, Button, List, Badge } from 'antd';
import {
  CheckCircleOutlined,
  WarningOutlined,
  CloseCircleOutlined,
  ReloadOutlined,
  QuestionCircleOutlined,
} from '@ant-design/icons';
import { useTranslation } from 'react-i18next';
import { apiService } from '../../services/api';
import './SystemStatus.css';

interface NodeStatus {
  name: string;
  status: 'healthy' | 'warning' | 'error' | 'unknown';
  message: string;
}

interface TopicStatus {
  name: string;
  status: 'healthy' | 'warning' | 'error' | 'unknown';
  message: string;
}

interface SystemStatusData {
  nodes: NodeStatus[];
  topics: TopicStatus[];
  overall: 'healthy' | 'degraded' | 'error' | 'unknown';
  timestamp: number;
}

const SystemStatus: React.FC = () => {
  const { t } = useTranslation();
  const [status, setStatus] = useState<SystemStatusData | null>(null);
  const [loading, setLoading] = useState(false);

  const loadStatus = async () => {
    setLoading(true);
    try {
      const data = await apiService.getSystemStatus();
      setStatus(data);
    } catch (error) {
      console.error('Failed to load system status:', error);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    loadStatus();
    // 每10秒刷新一次
    const interval = setInterval(loadStatus, 10000);
    return () => clearInterval(interval);
  }, []);

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'healthy':
        return <CheckCircleOutlined style={{ color: '#52c41a' }} />;
      case 'warning':
        return <WarningOutlined style={{ color: '#faad14' }} />;
      case 'error':
        return <CloseCircleOutlined style={{ color: '#f5222d' }} />;
      default:
        return <QuestionCircleOutlined style={{ color: '#d9d9d9' }} />;
    }
  };

  const getStatusTag = (status: string) => {
    const colors = {
      healthy: 'success',
      warning: 'warning',
      error: 'error',
      unknown: 'default',
    };
    return (
      <Tag color={colors[status as keyof typeof colors] || 'default'}>
        {status.toUpperCase()}
      </Tag>
    );
  };

  const getOverallStatusBadge = () => {
    if (!status) return <Badge status="default" />;
    
    const statusMap = {
      healthy: 'success' as const,
      degraded: 'warning' as const,
      error: 'error' as const,
      unknown: 'default' as const,
    };
    
    return <Badge status={statusMap[status.overall]} />;
  };

  return (
    <div className="system-status">
      <Card
        title={
          <Space>
            {getOverallStatusBadge()}
            {t('systemStatus.title')}
          </Space>
        }
        extra={
          <Button
            type="text"
            icon={<ReloadOutlined />}
            loading={loading}
            onClick={loadStatus}
          >
            {t('common.refresh')}
          </Button>
        }
        size="small"
      >
        {status && (
          <Space direction="vertical" size="middle" style={{ width: '100%' }}>
            {/* 总体状态 */}
            <div>
              <strong>{t('systemStatus.overall')}: </strong>
              {getStatusTag(status.overall)}
            </div>

            {/* 节点状态 */}
            <Card
              title={t('systemStatus.nodes')}
              type="inner"
              size="small"
            >
              <List
                size="small"
                dataSource={status.nodes}
                renderItem={(node) => (
                  <List.Item>
                    <Space style={{ width: '100%', justifyContent: 'space-between' }}>
                      <Space>
                        {getStatusIcon(node.status)}
                        <Tooltip title={node.name}>
                          <span className="node-name">
                            {node.name.split('/').pop()}
                          </span>
                        </Tooltip>
                      </Space>
                      <span className="status-message">{node.message}</span>
                    </Space>
                  </List.Item>
                )}
              />
            </Card>

            {/* 话题状态 */}
            <Card
              title={t('systemStatus.topics')}
              type="inner"
              size="small"
            >
              <List
                size="small"
                dataSource={status.topics}
                renderItem={(topic) => (
                  <List.Item>
                    <Space style={{ width: '100%', justifyContent: 'space-between' }}>
                      <Space>
                        {getStatusIcon(topic.status)}
                        <Tooltip title={topic.name}>
                          <span className="topic-name">
                            {topic.name.split('/').pop()}
                          </span>
                        </Tooltip>
                      </Space>
                      <span className="status-message">{topic.message}</span>
                    </Space>
                  </List.Item>
                )}
              />
            </Card>
          </Space>
        )}
      </Card>
    </div>
  );
};

export default SystemStatus;
