import { createSlice } from '@reduxjs/toolkit';
import { NAVIGATION_CONFIG } from '../../utils/constants';

const initialState = {
  currentGoal: null,
  goalHistory: [],
  quickGoals: NAVIGATION_CONFIG.QUICK_GOALS,
  customGoal: { x: '', y: '' },
  navigationStatus: NAVIGATION_CONFIG.NAVIGATION_STATUSES.IDLE,
  maxGoalHistory: NAVIGATION_CONFIG.MAX_GOAL_HISTORY,
};

const navigationSlice = createSlice({
  name: 'navigation',
  initialState,
  reducers: {
    setCurrentGoal: (state, action) => {
      state.currentGoal = action.payload;
      state.goalHistory.unshift({
        ...action.payload,
        timestamp: new Date().toISOString(),
      });
      
      // Keep only last maxGoalHistory goals in history
      if (state.goalHistory.length > state.maxGoalHistory) {
        state.goalHistory = state.goalHistory.slice(0, state.maxGoalHistory);
      }
    },
    setCustomGoal: (state, action) => {
      state.customGoal = action.payload;
    },
    updateCustomGoalX: (state, action) => {
      state.customGoal.x = action.payload;
    },
    updateCustomGoalY: (state, action) => {
      state.customGoal.y = action.payload;
    },
    clearCustomGoal: (state) => {
      state.customGoal = { x: '', y: '' };
    },
    setNavigationStatus: (state, action) => {
      state.navigationStatus = action.payload;
    },
    addQuickGoal: (state, action) => {
      state.quickGoals.push(action.payload);
    },
    removeQuickGoal: (state, action) => {
      state.quickGoals = state.quickGoals.filter((_, index) => index !== action.payload);
    },
    cancelGoal: (state) => {
      state.currentGoal = null;
      state.navigationStatus = NAVIGATION_CONFIG.NAVIGATION_STATUSES.IDLE;
    },
  },
});

export const {
  setCurrentGoal,
  setCustomGoal,
  updateCustomGoalX,
  updateCustomGoalY,
  clearCustomGoal,
  setNavigationStatus,
  addQuickGoal,
  removeQuickGoal,
  cancelGoal,
} = navigationSlice.actions;

export default navigationSlice.reducer;