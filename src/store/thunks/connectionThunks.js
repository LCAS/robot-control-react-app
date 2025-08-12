import { createAsyncThunk } from '@reduxjs/toolkit';
import rosService from '../../services/rosService';
import { CONNECTION_CONFIG, LOG_CONFIG } from '../../utils/constants';
import { 
  setConnectionStatus, 
  setIsConnected, 
  setConnectionAttempt, 
  setConnectionError 
} from '../slices/connectionSlice';
import { addLogEntry } from '../slices/logsSlice';

export const connectToRos = createAsyncThunk(
  'connection/connectToRos',
  async (bridgeUrl, { dispatch, rejectWithValue }) => {
    try {
      dispatch(setConnectionAttempt());
      dispatch(setConnectionStatus('Connecting...'));
      dispatch(addLogEntry('Connecting to ROS bridge...'));

      rosService.connect(bridgeUrl);

      // Wait for connection with timeout from config
      const connectionPromise = new Promise((resolve, reject) => {
        const checkConnection = setInterval(() => {
          if (rosService.isConnected) {
            clearInterval(checkConnection);
            resolve(true);
          }
        }, CONNECTION_CONFIG.RETRY_INTERVAL);

        setTimeout(() => {
          clearInterval(checkConnection);
          if (!rosService.isConnected) {
            reject(new Error('Connection timeout'));
          }
        }, CONNECTION_CONFIG.MAX_RETRY_TIME);
      });

      await connectionPromise;

      dispatch(setIsConnected(true));
      dispatch(setConnectionStatus('Connected'));
      dispatch(addLogEntry({ 
        message: '✅ Connected to ROS bridge', 
        level: LOG_CONFIG.LEVELS.SUCCESS,
        category: 'connection' 
      }));

      return { success: true };
    } catch (error) {
      dispatch(setConnectionError(error.message));
      dispatch(addLogEntry({ 
        message: `❌ Connection failed: ${error.message}`, 
        level: LOG_CONFIG.LEVELS.ERROR,
        category: 'connection' 
      }));
      return rejectWithValue(error.message);
    }
  }
);

export const disconnectFromRos = createAsyncThunk(
  'connection/disconnectFromRos',
  async (_, { dispatch }) => {
    try {
      rosService.disconnect();
      dispatch(setIsConnected(false));
      dispatch(setConnectionStatus('Disconnected'));
      dispatch(addLogEntry({ 
        message: '👋 Disconnected from ROS bridge', 
        level: LOG_CONFIG.LEVELS.INFO,
        category: 'connection' 
      }));
      return { success: true };
    } catch (error) {
      dispatch(addLogEntry({ 
        message: `❌ Disconnect error: ${error.message}`, 
        level: LOG_CONFIG.LEVELS.ERROR,
        category: 'connection' 
      }));
      throw error;
    }
  }
);
