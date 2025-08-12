import React, { useRef, useEffect } from 'react';
import { useAppDispatch, useAppSelector } from '../store/hooks';
import { 
    setJoystickDragging, 
    setJoystickPosition, 
    resetJoystick,
    updateVelocity 
} from '../store/slices/robotSlice';
import { addLogEntry } from '../store/slices/logsSlice';
import { ROBOT_CONFIG, VELOCITY_LIMITS, LOG_CONFIG, ROBOT_MODES } from '../utils/constants';
import rosService from '../services/rosService';

const JoystickControl = () => {
    const dispatch = useAppDispatch();
    const canvasRef = useRef(null);
    
    const { joystick, status } = useAppSelector(state => state.robot);
    const { isConnected } = useAppSelector(state => state.connection);
    
    // Check if joystick should be enabled
    const isTeleopEnabled = status.mode === ROBOT_MODES.TELEOP && status.enabled && !status.emergencyStop;
    const isJoystickEnabled = isConnected && isTeleopEnabled;

    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const maxRadius = joystick.maxRadius;

        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Draw outer circle
        ctx.beginPath();
        ctx.arc(centerX, centerY, maxRadius, 0, 2 * Math.PI);
        ctx.strokeStyle = isJoystickEnabled ? '#007bff' : '#6c757d';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Draw inner knob
        const knobX = centerX + joystick.position.x;
        const knobY = centerY + joystick.position.y;
        
        ctx.beginPath();
        ctx.arc(knobX, knobY, 15, 0, 2 * Math.PI);
        ctx.fillStyle = isJoystickEnabled 
            ? (joystick.isDragging ? '#0056b3' : '#007bff')
            : '#6c757d';
        ctx.fill();

        // Draw center dot
        ctx.beginPath();
        ctx.arc(centerX, centerY, 3, 0, 2 * Math.PI);
        ctx.fillStyle = '#dc3545';
        ctx.fill();

        // Draw disabled overlay
        if (!isJoystickEnabled) {
            ctx.fillStyle = 'rgba(108, 117, 125, 0.3)';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
        }

    }, [joystick.position, joystick.isDragging, isJoystickEnabled]);

    const getMousePos = (canvas, clientX, clientY) => {
        const rect = canvas.getBoundingClientRect();
        return {
            x: clientX - rect.left,
            y: clientY - rect.top
        };
    };

    const handleMouseDown = (e) => {
        if (!isJoystickEnabled) {
            dispatch(addLogEntry({
                message: '⚠️ Joystick disabled - Requires TELEOP mode and ENABLED state',
                level: LOG_CONFIG.LEVELS.WARNING,
                category: 'control'
            }));
            return;
        }

        const canvas = canvasRef.current;
        const mousePos = getMousePos(canvas, e.clientX, e.clientY);
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        
        const distance = Math.sqrt(
            Math.pow(mousePos.x - centerX, 2) + Math.pow(mousePos.y - centerY, 2)
        );
        
        if (distance <= joystick.maxRadius) {
            dispatch(setJoystickDragging(true));
        }
    };

    const handleMouseMove = (e) => {
        if (!joystick.isDragging || !isJoystickEnabled) return;

        const canvas = canvasRef.current;
        const mousePos = getMousePos(canvas, e.clientX, e.clientY);
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        
        let newX = mousePos.x - centerX;
        let newY = mousePos.y - centerY;
        
        const distance = Math.sqrt(newX * newX + newY * newY);
        
        if (distance > joystick.maxRadius) {
            newX = (newX / distance) * joystick.maxRadius;
            newY = (newY / distance) * joystick.maxRadius;
        }
        
        dispatch(setJoystickPosition({ x: newX, y: newY }));
        
        // Calculate and publish velocity
        const linearVelocity = (-newY / joystick.maxRadius) * VELOCITY_LIMITS.LINEAR_MAX;
        const angularVelocity = (-newX / joystick.maxRadius) * VELOCITY_LIMITS.ANGULAR_MAX;
        
        // Only publish if velocity is significant enough
        if (Math.abs(linearVelocity) > ROBOT_CONFIG.VELOCITY_THRESHOLD || 
            Math.abs(angularVelocity) > ROBOT_CONFIG.VELOCITY_THRESHOLD) {
            
            dispatch(updateVelocity({ linear: linearVelocity, angular: angularVelocity }));
            
            try {
                rosService.publishVelocity(linearVelocity, angularVelocity);
            } catch (error) {
                console.error('Failed to publish velocity:', error);
            }
        }
    };

    const handleMouseUp = () => {
        if (!joystick.isDragging) return;
        
        dispatch(resetJoystick());
        dispatch(updateVelocity({ linear: 0, angular: 0 }));
        
        if (isJoystickEnabled) {
            try {
                rosService.publishVelocity(0, 0);
            } catch (error) {
                console.error('Failed to stop robot:', error);
            }
        }
    };

    // Add global mouse event listeners
    useEffect(() => {
        const handleGlobalMouseMove = (e) => handleMouseMove(e);
        const handleGlobalMouseUp = () => handleMouseUp();

        if (joystick.isDragging) {
            document.addEventListener('mousemove', handleGlobalMouseMove);
            document.addEventListener('mouseup', handleGlobalMouseUp);
        }

        return () => {
            document.removeEventListener('mousemove', handleGlobalMouseMove);
            document.removeEventListener('mouseup', handleGlobalMouseUp);
        };
    }, [joystick.isDragging, isJoystickEnabled]);

    return (
        <div style={{ marginTop: '20px' }}>
            {/* <h4>Virtual Joystick</h4> */}
            <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
                <canvas
                    ref={canvasRef}
                    width={130}
                    height={130}
                    onMouseDown={handleMouseDown}
                    style={{
                        border: '1px solid #ddd',
                        borderRadius: '8px',
                        cursor: isJoystickEnabled ? 'pointer' : 'not-allowed',
                        backgroundColor: '#f8f9fa'
                    }}
                />
                
                {/* Status and instructions */}
                <div style={{ marginTop: '10px', textAlign: 'center' }}>
                    <div style={{ 
                        fontSize: '0.8em', 
                        color: isJoystickEnabled ? '#28a745' : '#dc3545',
                        fontWeight: 'bold'
                    }}>
                        {isJoystickEnabled ? '✅ Active' : '❌ Disabled'}
                    </div>
                    
                    {!isJoystickEnabled && (
                        <div style={{ fontSize: '0.7em', color: '#666', marginTop: '5px' }}>
                            {!isConnected && 'Not connected to ROS'}
                            {isConnected && status.mode !== ROBOT_MODES.TELEOP && 'Requires TELEOP mode'}
                            {isConnected && status.mode === ROBOT_MODES.TELEOP && !status.enabled && 'Robot not enabled'}
                            {isConnected && status.emergencyStop && 'Emergency stop active'}
                        </div>
                    )}
                    
                    <div style={{ fontSize: '0.7em', color: '#666', marginTop: '5px' }}>
                        Linear: {joystick.position.y ? ((-joystick.position.y / joystick.maxRadius) * VELOCITY_LIMITS.LINEAR_MAX).toFixed(2) : '0.00'} m/s
                        <br />
                        Angular: {joystick.position.x ? ((-joystick.position.x / joystick.maxRadius) * VELOCITY_LIMITS.ANGULAR_MAX).toFixed(2) : '0.00'} rad/s
                    </div>
                </div>
            </div>
        </div>
    );
};

export default JoystickControl;