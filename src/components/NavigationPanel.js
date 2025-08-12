import React from 'react';
import { useAppDispatch, useAppSelector } from '../store/hooks';
import { 
    setCurrentGoal, 
    updateCustomGoalX, 
    updateCustomGoalY, 
    clearCustomGoal 
} from '../store/slices/navigationSlice';
import { addLogEntry } from '../store/slices/logsSlice';
import { LOG_CONFIG, ROS_CONFIG, ROBOT_MODES } from '../utils/constants';
import rosService from '../services/rosService';

const NavigationPanel = () => {
    const dispatch = useAppDispatch();
    const { quickGoals, customGoal } = useAppSelector(state => state.navigation || {});
    const { x: goalX = '', y: goalY = '' } = customGoal || {};
    const { isConnected } = useAppSelector(state => state.connection);
    const { status } = useAppSelector(state => state.robot);

    // Check if navigation should be enabled
    const isTeleopEnabled = status.mode === ROBOT_MODES.TELEOP && status.enabled && !status.emergencyStop;
    const isNavigationEnabled = isConnected && isTeleopEnabled;

    const sendGoal = (x, y, name) => {
        if (!isNavigationEnabled) {
            dispatch(addLogEntry({
                message: '⚠️ Navigation disabled - Requires TELEOP mode and ENABLED state',
                level: LOG_CONFIG.LEVELS.WARNING,
                category: 'navigation'
            }));
            return;
        }

        const goal = { x, y, name };
        dispatch(setCurrentGoal(goal));
        
        console.log('🎯 NavigationPanel sendGoal called:', { x, y, name, isNavigationEnabled });
        
        try {
            console.log('🚀 Attempting to publish goal via rosService...');
            rosService.publishNavigationGoal(x, y);
            dispatch(addLogEntry({ 
                message: `🎯 Navigation goal published to ${ROS_CONFIG.GOAL_TOPIC}: ${name} (${x}, ${y})`,
                level: LOG_CONFIG.LEVELS.SUCCESS,
                category: 'navigation'
            }));
        } catch (error) {
            console.error('❌ Failed to publish goal:', error);
            dispatch(addLogEntry({ 
                message: `❌ Failed to send navigation goal: ${error.message}`,
                level: LOG_CONFIG.LEVELS.ERROR,
                category: 'navigation'
            }));
        }
    };

    const testConnection = () => {
        console.log('🔍 === CONNECTION DEBUG ===');
        console.log('Redux isConnected:', isConnected);
        console.log('rosService.isConnected:', rosService.isConnected);
        console.log('rosService.ros exists:', !!rosService.ros);
        console.log('rosService.ros readyState:', rosService.ros?.socket?.readyState);
        console.log('rosService.cmdVelPublisher:', !!rosService.cmdVelPublisher);
        console.log('rosService.goalPublisher:', !!rosService.goalPublisher);
        console.log('Robot status:', status);
        console.log('isTeleopEnabled:', isTeleopEnabled);
        console.log('isNavigationEnabled:', isNavigationEnabled);
        
        // Test velocity first (we know this works)
        try {
            console.log('Testing velocity publish...');
            rosService.publishVelocity(0.1, 0.0);
            dispatch(addLogEntry({ 
                message: '✅ Velocity test successful',
                level: LOG_CONFIG.LEVELS.SUCCESS,
                category: 'debug'
            }));
        } catch (error) {
            console.error('Velocity test failed:', error);
            dispatch(addLogEntry({ 
                message: `❌ Velocity test failed: ${error.message}`,
                level: LOG_CONFIG.LEVELS.ERROR,
                category: 'debug'
            }));
        }
        
        // Force check rosService state before goal test
        if (rosService.ros && rosService.ros.socket && rosService.ros.socket.readyState === 1) {
            rosService.isConnected = true;
            if (!rosService.goalPublisher) {
                console.log('🔧 Goal publisher missing, setting up publishers...');
                rosService.setupPublishers();
            }
        }
        
        // Then test goal with direct rosService check
        try {
            console.log('Testing goal publish...');
            if (rosService.goalPublisher) {
                rosService.publishNavigationGoal(0, 0);
                dispatch(addLogEntry({ 
                    message: '✅ Goal test successful',
                    level: LOG_CONFIG.LEVELS.SUCCESS,
                    category: 'debug'
                }));
            } else {
                throw new Error('Goal publisher not available');
            }
        } catch (error) {
            console.error('Goal test failed:', error);
            dispatch(addLogEntry({ 
                message: `❌ Goal test failed: ${error.message}`,
                level: LOG_CONFIG.LEVELS.ERROR,
                category: 'debug'
            }));
        }
    };

    const sendCustomGoal = () => {
        const x = parseFloat(goalX) || 0;
        const y = parseFloat(goalY) || 0;
        sendGoal(x, y, 'Custom');
        dispatch(clearCustomGoal());
    };

    const handleGoalXChange = (e) => {
        dispatch(updateCustomGoalX(e.target.value));
    };

    const handleGoalYChange = (e) => {
        dispatch(updateCustomGoalY(e.target.value));
    };

    return (
        <div className="panel">
            <h3>🎯 Navigation</h3>
            
            <div style={{ marginBottom: '10px', fontSize: '0.8em', color: '#666' }}>
                Publishing to: {ROS_CONFIG.GOAL_TOPIC} ({ROS_CONFIG.GOAL_MESSAGE_TYPE})
            </div>
            
            {/* Status Indicator */}
            <div style={{ 
                marginBottom: '15px', 
                fontSize: '0.8em', 
                padding: '8px', 
                background: isNavigationEnabled ? '#d4edda' : '#f8d7da', 
                color: isNavigationEnabled ? '#155724' : '#721c24',
                borderRadius: '4px',
                border: `1px solid ${isNavigationEnabled ? '#c3e6cb' : '#f5c6cb'}`
            }}>
                <div style={{ fontWeight: 'bold' }}>
                    Navigation: {isNavigationEnabled ? '✅ ENABLED' : '❌ DISABLED'}
                </div>
                {!isNavigationEnabled && (
                    <div style={{ fontSize: '0.7em', marginTop: '4px' }}>
                        {!isConnected && '• Not connected to ROS'}
                        {isConnected && status.mode !== ROBOT_MODES.TELEOP && '• Requires TELEOP mode'}
                        {isConnected && status.mode === ROBOT_MODES.TELEOP && !status.enabled && '• Robot not enabled'}
                        {isConnected && status.emergencyStop && '• Emergency stop active'}
                    </div>
                )}
            </div>
            
            {/* Debug Section */}
            <div style={{ marginBottom: '15px', padding: '10px', background: '#f8f9fa', borderRadius: '4px' }}>
                <h4 style={{ margin: '0 0 10px 0', fontSize: '0.9em' }}>Debug:</h4>
                <button 
                    className="btn btn-info" 
                    onClick={testConnection}
                    style={{ fontSize: '0.8em', padding: '5px 10px' }}
                >
                    Test ROS Connection
                </button>
            </div>
            
            <div style={{ marginBottom: '15px' }}>
                <h4>Quick Goals:</h4>
                <div className="controls">
                    {(quickGoals || []).map((goal, index) => (
                        <button 
                            key={index}
                            className={`btn ${isNavigationEnabled ? 'btn-primary' : 'btn-secondary'}`}
                            onClick={() => sendGoal(goal.x, goal.y, goal.name)}
                            disabled={!isNavigationEnabled}
                            title={!isNavigationEnabled ? 'Requires TELEOP mode and ENABLED state' : ''}
                        >
                            {goal.name}
                        </button>
                    ))}
                </div>
            </div>

            <div style={{ marginBottom: '15px' }}>
                <h4>Custom Goal:</h4>
                <div style={{ display: 'flex', gap: '10px', alignItems: 'center' }}>
                    <input 
                        type="number" 
                        placeholder="X" 
                        step="0.1"
                        value={goalX}
                        onChange={handleGoalXChange}
                        disabled={!isNavigationEnabled}
                        style={{
                            width: '80px', 
                            padding: '8px', 
                            border: '1px solid #ddd', 
                            borderRadius: '4px',
                            backgroundColor: !isNavigationEnabled ? '#e9ecef' : 'white'
                        }}
                    />
                    <input 
                        type="number" 
                        placeholder="Y" 
                        step="0.1"
                        value={goalY}
                        onChange={handleGoalYChange}
                        disabled={!isNavigationEnabled}
                        style={{
                            width: '80px', 
                            padding: '8px', 
                            border: '1px solid #ddd', 
                            borderRadius: '4px',
                            backgroundColor: !isNavigationEnabled ? '#e9ecef' : 'white'
                        }}
                    />
                    <button 
                        className={`btn ${isNavigationEnabled ? 'btn-primary' : 'btn-secondary'}`}
                        onClick={sendCustomGoal}
                        disabled={!isNavigationEnabled}
                        title={!isNavigationEnabled ? 'Requires TELEOP mode and ENABLED state' : ''}
                    >
                        Send Goal
                    </button>
                </div>
            </div>
        </div>
    );
};

export default NavigationPanel;