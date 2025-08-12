import { createSlice } from '@reduxjs/toolkit';
import { LOG_CONFIG } from '../../utils/constants';

const initialState = {
  entries: [],
  maxEntries: LOG_CONFIG.MAX_ENTRIES,
  filter: LOG_CONFIG.DEFAULT_FILTER,
  levels: LOG_CONFIG.LEVELS,
};

const logsSlice = createSlice({
  name: 'logs',
  initialState,
  reducers: {
    addLogEntry: (state, action) => {
      const timestamp = new Date().toLocaleTimeString();
      const entry = {
        id: Date.now() + Math.random(),
        timestamp,
        message: action.payload.message || action.payload,
        level: action.payload.level || LOG_CONFIG.LEVELS.INFO,
        category: action.payload.category || 'general',
      };
      
      state.entries.push(entry);
      
      // Keep only the last maxEntries
      if (state.entries.length > state.maxEntries) {
        state.entries = state.entries.slice(-state.maxEntries);
      }
    },
    clearLogs: (state) => {
      state.entries = [];
    },
    setLogFilter: (state, action) => {
      state.filter = action.payload;
    },
    setMaxEntries: (state, action) => {
      state.maxEntries = action.payload;
      if (state.entries.length > state.maxEntries) {
        state.entries = state.entries.slice(-state.maxEntries);
      }
    },
  },
});

export const {
  addLogEntry,
  clearLogs,
  setLogFilter,
  setMaxEntries,
} = logsSlice.actions;

export default logsSlice.reducer;