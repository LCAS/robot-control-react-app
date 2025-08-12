import React, { useEffect } from 'react';
import { useAppDispatch, useAppSelector } from '../store/hooks';
import { connectToRos, disconnectFromRos } from '../store/thunks/connectionThunks';
import { addLogEntry, clearLogs } from '../store/slices/logsSlice';
import { updateOdometry, updateBattery, updatePose, setRobotMode, setRobotEnabled, emergencyStop } from '../store/slices/robotSlice';
import { ROS_CONFIG, LOG_CONFIG, ROBOT_MODES } from '../utils/constants';
import rosService from '../services/rosService';
import JoystickControl from './JoystickControl';
import StatusDisplay from './StatusDisplay';
import NavigationPanel from './NavigationPanel';
import MessageLog from './MessageLog';

const RobotController = () => {
    const dispatch = useAppDispatch();
    
    // Safe selectors with fallbacks
    const connection = useAppSelector(state => state.connection) || {};
    const robot = useAppSelector(state => state.robot) || {};
    const { isConnected = false, connectionStatus = 'Disconnected', bridgeUrl = ROS_CONFIG.BRIDGE_URL } = connection;
    const { status = {} } = robot;

    useEffect(() => {
        dispatch(addLogEntry('Dashboard initialized. Ready to connect...'));
        dispatch(connectToRos(bridgeUrl));
    }, [dispatch, bridgeUrl]);

    // Setup ROS subscriptions when connected
    useEffect(() => {
        if (isConnected) {
            // Setup odometry callback
            rosService.setOdometryCallback((data) => {
                dispatch(updateOdometry(data));
            });

            // Setup battery callback
            rosService.setBatteryCallback((data) => {
                dispatch(updateBattery(data));
            });

            // Setup pose callback
            rosService.setPoseCallback((data) => {
                dispatch(updatePose(data));
            });

            dispatch(addLogEntry({
                message: '📡 Subscribed to robot telemetry topics',
                level: LOG_CONFIG.LEVELS.SUCCESS,
                category: 'telemetry'
            }));
        }
    }, [isConnected, dispatch]);

    const handleReconnect = () => {
        dispatch(connectToRos(bridgeUrl));
    };

    const handleDisconnect = () => {
        dispatch(disconnectFromRos());
    };

    const handleModeChange = (mode) => {
        try {
            rosService.publishRobotMode(mode);
            dispatch(setRobotMode(mode));
            dispatch(addLogEntry({
                message: `🔄 Robot mode changed to: ${mode}`,
                level: LOG_CONFIG.LEVELS.INFO,
                category: 'control'
            }));
        } catch (error) {
            dispatch(addLogEntry({
                message: `❌ Failed to change robot mode: ${error.message}`,
                level: LOG_CONFIG.LEVELS.ERROR,
                category: 'control'
            }));
        }
    };

    const handleRobotEnable = (enabled) => {
        try {
            rosService.publishRobotEnable(enabled);
            dispatch(setRobotEnabled(enabled));
            dispatch(addLogEntry({
                message: `🤖 Robot ${enabled ? 'enabled' : 'disabled'}`,
                level: enabled ? LOG_CONFIG.LEVELS.SUCCESS : LOG_CONFIG.LEVELS.WARNING,
                category: 'control'
            }));
        } catch (error) {
            dispatch(addLogEntry({
                message: `❌ Failed to ${enabled ? 'enable' : 'disable'} robot: ${error.message}`,
                level: LOG_CONFIG.LEVELS.ERROR,
                category: 'control'
            }));
        }
    };

    const handleEmergencyStop = () => {
        try {
            rosService.publishEmergencyStop(true);
            dispatch(emergencyStop());
            dispatch(addLogEntry({
                message: '🚨 EMERGENCY STOP ACTIVATED',
                level: LOG_CONFIG.LEVELS.ERROR,
                category: 'safety'
            }));
        } catch (error) {
            // Even if ROS publish fails, still stop locally
            dispatch(emergencyStop());
            dispatch(addLogEntry({
                message: '🚨 EMERGENCY STOP (Local only - ROS not available)',
                level: LOG_CONFIG.LEVELS.ERROR,
                category: 'safety'
            }));
        }
    };

    const handleClearLog = () => {
        dispatch(clearLogs());
    };

    return (
        <>
            <div className="header">
                <h1>🤖 LCAS Robot Control Dashboard</h1>
                <p>Web-based ROS client for robot control via WebSocket bridge</p>
                <div style={{ marginTop: '15px' }}>
                    <span className={`connection-status ${isConnected ? 'connected' : connectionStatus === 'Connecting...' ? 'connecting' : 'disconnected'}`}></span>
                    <span>{connectionStatus}</span>
                </div>
            </div>

            <div className="dashboard">
                {/* Connection Panel */}
                <div className="panel">
                    <h3>📡 ROS Connection</h3>
                    <div style={{ marginBottom: '15px' }}>
                        <label style={{ display: 'block', marginBottom: '5px' }}>ROS Bridge URL:</label>
                        <input 
                            type="text" 
                            value={bridgeUrl}
                            readOnly
                            style={{
                                width: '100%', 
                                padding: '8px', 
                                border: '1px solid #ddd', 
                                borderRadius: '4px',
                                backgroundColor: '#f8f9fa'
                            }}
                        />
                    </div>
                    <div className="controls">
                        <button className="btn btn-success" onClick={handleReconnect}>
                            {isConnected ? 'Reconnect' : 'Connect'}
                        </button>
                        <button className="btn btn-warning" onClick={handleDisconnect}>
                            Disconnect
                        </button>
                    </div>
                </div>

                {/* Robot Status Panel */}
                <StatusDisplay />

                {/* Robot Control Panel */}
                <div className="panel">
                    <h3>🎮 Robot Control</h3>
                    
                    {/* Mode Control */}
                    <div style={{ marginBottom: '15px' }}>
                        <h4>Control Mode:</h4>
                        <div className="controls">
                            <button 
                                className={`btn ${status.mode === ROBOT_MODES.MANUAL ? 'btn-primary' : 'btn-info'}`}
                                onClick={() => handleModeChange(ROBOT_MODES.MANUAL)}
                                disabled={!isConnected}
                            >
                                Manual
                            </button>
                            <button 
                                className={`btn ${status.mode === ROBOT_MODES.AUTONOMOUS ? 'btn-primary' : 'btn-info'}`}
                                onClick={() => handleModeChange(ROBOT_MODES.AUTONOMOUS)}
                                disabled={!isConnected}
                            >
                                Autonomous
                            </button>
                            <button 
                                className={`btn ${status.mode === ROBOT_MODES.TELEOP ? 'btn-primary' : 'btn-info'}`}
                                onClick={() => handleModeChange(ROBOT_MODES.TELEOP)}
                                disabled={!isConnected}
                            >
                                Teleop
                            </button>
                        </div>
                    </div>

                    {/* Robot State Control */}
                    <div style={{ marginBottom: '15px' }}>
                        <h4>Robot State:</h4>
                        <div className="controls">
                            <button 
                                className="btn btn-success" 
                                onClick={() => handleRobotEnable(true)}
                                disabled={!isConnected || status.emergencyStop}
                            >
                                Enable
                            </button>
                            <button 
                                className="btn btn-warning" 
                                onClick={() => handleRobotEnable(false)}
                                disabled={!isConnected}
                            >
                                Disable
                            </button>
                            <button 
                                className="btn btn-danger" 
                                onClick={handleEmergencyStop}
                                style={{ marginLeft: '10px' }}
                            >
                                🚨 E-STOP
                            </button>
                        </div>
                    </div>

                    {/* Status Indicators */}
                    <div style={{ 
                        fontSize: '0.9em', 
                        padding: '10px', 
                        background: '#f8f9fa', 
                        borderRadius: '4px',
                        marginBottom: '15px'
                    }}>
                        
                        <div style={{ fontSize: '0.8em', marginTop: '5px', color: '#666' }}>
                            {status.mode === ROBOT_MODES.TELEOP && status.enabled && !status.emergencyStop 
                                ? '✅ Joystick & Navigation Active' 
                                : '❌ Joystick & Navigation Disabled'
                            }
                        </div>
                    </div>

                    <JoystickControl />
                </div>

                {/* Navigation Panel */}
                <NavigationPanel />

                {/* Message Log */}
                <MessageLog onClearLog={handleClearLog} />
            </div>
        </>
    );
};

export default RobotController;