import React, { useEffect, useState } from 'react';
import { Card, Row, Col, Statistic, Badge, Space, Divider } from 'antd';
import {
  EnvironmentOutlined,
  DashboardOutlined,
  ThunderboltOutlined,
  CompassOutlined,
} from '@ant-design/icons';
import { useTranslation } from 'react-i18next';
import { configService } from '../../services/config';
import SystemStatus from '../SystemStatus';
import './StatusMonitor.css';

interface RobotStatus {
  position: {
    x: number;
    y: number;
    yaw: number;
  };
  velocity: {
    vx: number;
    vy: number;
    vyaw: number;
  };
  battery?: number;
  connected: boolean;
  timestamp: number;
}

const StatusMonitor: React.FC = () => {
  const { t } = useTranslation();
  const [status, setStatus] = useState<RobotStatus>({
    position: { x: 0, y: 0, yaw: 0 },
    velocity: { vx: 0, vy: 0, vyaw: 0 },
    connected: false,
    timestamp: Date.now(),
  });

  useEffect(() => {
    let ws: WebSocket | null = null;
    let reconnectTimer: number | null = null;
    let isUnmounted = false;

    const connectWebSocket = async () => {
      if (isUnmounted) return;

      try {
        // Get WebSocket URL from config service
        const wsUrl = await configService.getWebSocketUrl();
        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
          console.log(`[StatusMonitor] WebSocket connected to ${wsUrl}`);
          setStatus(prev => ({ ...prev, connected: true }));
        };

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            // Listen for robot_status messages
            if (data.type === 'robot_status') {
              setStatus({
                position: data.position || { x: 0, y: 0, yaw: 0 },
                velocity: data.velocity || { vx: 0, vy: 0, vyaw: 0 },
                battery: data.battery,
                connected: true,
                timestamp: Date.now(),
              });
            }
          } catch (error) {
            console.error('[StatusMonitor] Failed to parse message:', error);
          }
        };

        ws.onerror = (error) => {
          console.error('[StatusMonitor] WebSocket error:', error);
          setStatus(prev => ({ ...prev, connected: false }));
        };

        ws.onclose = () => {
          console.log('[StatusMonitor] WebSocket disconnected, reconnecting in 3s...');
          setStatus(prev => ({ ...prev, connected: false }));
          
          // Auto-reconnect after 3 seconds
          if (!isUnmounted) {
            reconnectTimer = setTimeout(() => {
              connectWebSocket();
            }, 3000);
          }
        };
      } catch (error) {
        console.error('[StatusMonitor] Failed to get WebSocket config:', error);
        setStatus(prev => ({ ...prev, connected: false }));
        
        // Retry connection after 3 seconds
        if (!isUnmounted) {
          reconnectTimer = setTimeout(() => {
            connectWebSocket();
          }, 3000);
        }
      }
    };

    connectWebSocket();

    return () => {
      isUnmounted = true;
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
      }
      if (ws) {
        ws.close();
      }
    };
  }, []);

  // 格式化数值显示（保留2位小数）
  const formatValue = (value: number): string => {
    return value.toFixed(2);
  };

  // 格式化角度显示（弧度转角度）
  const formatAngle = (radians: number): string => {
    const degrees = (radians * 180 / Math.PI).toFixed(1);
    return `${degrees}°`;
  };

  return (
    <div className="status-monitor">
      <Space direction="vertical" size="middle" style={{ width: '100%' }}>
        {/* 连接状态 */}
        <Card size="small">
          <Space>
            <Badge
              status={status.connected ? 'success' : 'error'}
              text={status.connected ? t('robotConnected') : t('robotDisconnected')}
            />
          </Space>
        </Card>

        {/* 位置信息 */}
        <Card
          title={
            <Space>
              <EnvironmentOutlined />
              {t('position')}
            </Space>
          }
          size="small"
        >
          <Row gutter={[16, 16]}>
            <Col span={8}>
              <Statistic
                title="X"
                value={formatValue(status.position.x)}
                suffix="m"
                valueStyle={{ fontSize: '18px' }}
              />
            </Col>
            <Col span={8}>
              <Statistic
                title="Y"
                value={formatValue(status.position.y)}
                suffix="m"
                valueStyle={{ fontSize: '18px' }}
              />
            </Col>
            <Col span={8}>
              <Statistic
                title={t('yaw')}
                value={formatAngle(status.position.yaw)}
                valueStyle={{ fontSize: '18px' }}
              />
            </Col>
          </Row>
        </Card>

        {/* 速度信息 */}
        <Card
          title={
            <Space>
              <DashboardOutlined />
              {t('velocity')}
            </Space>
          }
          size="small"
        >
          <Row gutter={[16, 16]}>
            <Col span={8}>
              <Statistic
                title="Vx"
                value={formatValue(status.velocity.vx)}
                suffix="m/s"
                valueStyle={{ fontSize: '18px' }}
              />
            </Col>
            <Col span={8}>
              <Statistic
                title="Vy"
                value={formatValue(status.velocity.vy)}
                suffix="m/s"
                valueStyle={{ fontSize: '18px' }}
              />
            </Col>
            <Col span={8}>
              <Statistic
                title={t('angularVelocity')}
                value={formatValue(status.velocity.vyaw)}
                suffix="rad/s"
                valueStyle={{ fontSize: '18px' }}
              />
            </Col>
          </Row>
        </Card>

        {/* 线速度和角速度 */}
        <Card
          title={
            <Space>
              <CompassOutlined />
              {t('motion')}
            </Space>
          }
          size="small"
        >
          <Row gutter={[16, 16]}>
            <Col span={12}>
              <Statistic
                title={t('linearSpeed')}
                value={formatValue(
                  Math.sqrt(
                    status.velocity.vx ** 2 + status.velocity.vy ** 2
                  )
                )}
                suffix="m/s"
                valueStyle={{ fontSize: '18px' }}
              />
            </Col>
            <Col span={12}>
              <Statistic
                title={t('angularSpeed')}
                value={formatValue(Math.abs(status.velocity.vyaw))}
                suffix="rad/s"
                valueStyle={{ fontSize: '18px' }}
              />
            </Col>
          </Row>
        </Card>

        {/* 电池电量（如果有） */}
        {status.battery !== undefined && (
          <Card
            title={
              <Space>
                <ThunderboltOutlined />
                {t('battery')}
              </Space>
            }
            size="small"
          >
            <Statistic
              value={status.battery}
              suffix="%"
              valueStyle={{
                fontSize: '24px',
                color: status.battery > 20 ? '#3f8600' : '#cf1322',
              }}
            />
          </Card>
        )}
        
        {/* 分隔线 */}
        <Divider style={{ margin: '12px 0' }} />
        
        {/* 系统状态 */}
        <SystemStatus />
      </Space>
    </div>
  );
};

export default StatusMonitor;
