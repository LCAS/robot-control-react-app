import { createSlice } from '@reduxjs/toolkit';
import { ROS_CONFIG } from '../../utils/constants';

const initialState = {
  isConnected: false,
  connectionStatus: 'Disconnected',
  rosService: null,
  bridgeUrl: ROS_CONFIG.BRIDGE_URL,
  lastConnectionAttempt: null,
  connectionError: null,
};

const connectionSlice = createSlice({
  name: 'connection',
  initialState,
  reducers: {
    setConnectionStatus: (state, action) => {
      state.connectionStatus = action.payload;
    },
    setIsConnected: (state, action) => {
      state.isConnected = action.payload;
      if (action.payload) {
        state.connectionError = null;
      }
    },
    setRosService: (state, action) => {
      state.rosService = action.payload;
    },
    setBridgeUrl: (state, action) => {
      state.bridgeUrl = action.payload;
    },
    setConnectionAttempt: (state) => {
      state.lastConnectionAttempt = new Date().toISOString();
    },
    setConnectionError: (state, action) => {
      state.connectionError = action.payload;
      state.isConnected = false;
      state.connectionStatus = 'Error';
    },
    resetConnection: (state) => {
      state.isConnected = false;
      state.connectionStatus = 'Disconnected';
      state.connectionError = null;
    },
  },
});

export const {
  setConnectionStatus,
  setIsConnected,
  setRosService,
  setBridgeUrl,
  setConnectionAttempt,
  setConnectionError,
  resetConnection,
} = connectionSlice.actions;

export default connectionSlice.reducer;