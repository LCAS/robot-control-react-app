import { createAsyncThunk } from '@reduxjs/toolkit';
import rosService from '../../services/rosService';
import { VELOCITY_LIMITS, LOG_CONFIG, ROBOT_CONFIG } from '../../utils/constants';
import { updateVelocity, setLastCommand, emergencyStop, setRobotMode } from '../slices/robotSlice';
import { addLogEntry } from '../slices/logsSlice';

export const publishVelocity = createAsyncThunk(
  'robot/publishVelocity',
  async ({ linear, angular }, { dispatch, getState }) => {
    try {
      const { connection } = getState();
      
      if (!connection.isConnected) {
        throw new Error('Not connected to ROS bridge');
      }

      // Apply velocity limits from config
      const clampedLinear = Math.max(
        VELOCITY_LIMITS.LINEAR_MIN, 
        Math.min(VELOCITY_LIMITS.LINEAR_MAX, linear)
      );
      const clampedAngular = Math.max(
        VELOCITY_LIMITS.ANGULAR_MIN, 
        Math.min(VELOCITY_LIMITS.ANGULAR_MAX, angular)
      );

      // Only publish if above threshold
      if (Math.abs(clampedLinear) > ROBOT_CONFIG.VELOCITY_THRESHOLD || 
          Math.abs(clampedAngular) > ROBOT_CONFIG.VELOCITY_THRESHOLD) {
        rosService.publishVelocity(clampedLinear, clampedAngular);
      }

      dispatch(updateVelocity({ linear: clampedLinear, angular: clampedAngular }));
      dispatch(setLastCommand(`vel: ${clampedLinear.toFixed(2)}, ${clampedAngular.toFixed(2)}`));

      return { success: true };
    } catch (error) {
      dispatch(addLogEntry({ 
        message: `❌ Failed to publish velocity: ${error.message}`, 
        level: LOG_CONFIG.LEVELS.ERROR,
        category: 'control' 
      }));
      throw error;
    }
  }
);

export const stopRobot = createAsyncThunk(
  'robot/stopRobot',
  async (_, { dispatch }) => {
    try {
      rosService.stopRobot();
      dispatch(emergencyStop());
      dispatch(addLogEntry({ 
        message: '🆘 EMERGENCY STOP SENT!', 
        level: LOG_CONFIG.LEVELS.WARNING,
        category: 'control' 
      }));
      return { success: true };
    } catch (error) {
      dispatch(addLogEntry({ 
        message: `❌ Emergency stop failed: ${error.message}`, 
        level: LOG_CONFIG.LEVELS.ERROR,
        category: 'control' 
      }));
      throw error;
    }
  }
);

export const changeRobotMode = createAsyncThunk(
  'robot/changeMode',
  async (mode, { dispatch }) => {
    try {
      dispatch(setRobotMode(mode));
      dispatch(addLogEntry({ 
        message: `🎮 Robot mode set to: ${mode}`, 
        level: LOG_CONFIG.LEVELS.INFO,
        category: 'control' 
      }));
      return { success: true };
    } catch (error) {
      dispatch(addLogEntry({ 
        message: `❌ Failed to change mode: ${error.message}`, 
        level: LOG_CONFIG.LEVELS.ERROR,
        category: 'control' 
      }));
      throw error;
    }
  }
);