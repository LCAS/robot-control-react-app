import React from 'react';
import { useAppSelector } from '../store/hooks';

const StatusDisplay = () => {
    const { isConnected } = useAppSelector(state => state.connection);
    const { status, realTimeData } = useAppSelector(state => state.robot);
    const lastUpdate = realTimeData?.lastUpdate;

    const getBatteryColor = (percentage) => {
        if (percentage > 50) return '#28a745';
        if (percentage > 20) return '#ffc107';
        return '#dc3545';
    };

    const getDataAge = () => {
        if (!lastUpdate) return '';
        const age = Math.floor((Date.now() - new Date(lastUpdate)) / 1000);
        if (age < 2) return '(live)';
        if (age < 60) return `(${age}s ago)`;
        return '(stale)';
    };

    return (
        <div className="panel">
            <h3>🤖 Robot Status</h3>
            <div className="status-grid">
                <div className="status-item">
                    <div className="status-value">{status.mode}</div>
                        <div>State: <strong style={{ 
                            color: status.emergencyStop ? '#dc3545' : status.enabled ? '#28a745' : '#ffc107'
                        }}>
                            {status.emergencyStop ? 'E-STOP' : status.enabled ? 'Enabled' : 'Disabled'}
                        </strong></div>
                </div>
                <div className="status-item">
                    <div 
                        className="status-value" 
                        style={{ 
                            color: getBatteryColor(realTimeData?.battery?.percentage || 50) 
                        }}
                    >
                        {status.battery}
                        {' Voltage:'}
                        {realTimeData?.battery?.voltage && (
                            <small style={{ display: 'block', fontSize: '0.7em' }}>
                                {realTimeData.battery.voltage.toFixed(1)}V
                            </small>
                        )}
                    </div>
                    <div className="status-label">Battery</div>
                </div>
                <div className="status-item">
                    <div className="status-value">
                        {status.position}
                        {realTimeData?.odometry?.position?.z !== undefined && (
                            <small style={{ display: 'block', fontSize: '0.7em' }}>
                                Z: {realTimeData.odometry.position.z.toFixed(2)}
                            </small>
                        )}
                    </div>
                    <div className="status-label">Position</div>
                </div>
                <div className="status-item">
                    <div className="status-value">
                        {status.velocity}
                        {realTimeData?.odometry?.velocity && (
                            <small style={{ display: 'block', fontSize: '0.7em' }}>
                                ω: {realTimeData.odometry.velocity.angular.toFixed(2)} rad/s
                            </small>
                        )}
                    </div>
                    <div className="status-label">Velocity</div>
                </div>
            </div>
            
            {lastUpdate && (
                <div style={{ 
                    marginTop: '10px', 
                    fontSize: '0.8em', 
                    color: '#666',
                    textAlign: 'center'
                }}>
                    Last telemetry update: {getDataAge()}
                </div>
            )}

            {!isConnected && (
                <div style={{
                    background: '#fff3cd',
                    color: '#856404',
                    padding: '15px',
                    borderRadius: '8px',
                    marginTop: '15px'
                }}>
                    <p>⚠️ Robot status unavailable - not connected to ROS bridge</p>
                    <p style={{ fontSize: '0.9em', marginTop: '5px' }}>
                        Run: <code>ros2 run rosbridge_server rosbridge_websocket</code>
                    </p>
                </div>
            )}
        </div>
    );
};

export default StatusDisplay;