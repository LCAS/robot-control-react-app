import { createSlice } from '@reduxjs/toolkit';
import { ROBOT_CONFIG, VELOCITY_LIMITS, ROBOT_MODES, ROBOT_STATES } from '../../utils/constants';

const initialState = {
  status: {
    mode: ROBOT_MODES.MANUAL,
    state: ROBOT_STATES.DISABLED,
    battery: ROBOT_CONFIG.DEFAULT_BATTERY,
    position: ROBOT_CONFIG.DEFAULT_POSITION,
    velocity: ROBOT_CONFIG.DEFAULT_VELOCITY,
    enabled: false,
    emergencyStop: false,
  },
  currentVelocity: {
    linear: 0,
    angular: 0,
  },
  realTimeData: {
    odometry: {
      velocity: { linear: 0, angular: 0 },
      position: { x: 0, y: 0, z: 0 }
    },
    battery: {
      percentage: 50,
      voltage: 0,
      status: 'Unknown'
    },
    pose: {
      position: { x: 0, y: 0, z: 0 }
    },
    lastUpdate: null
  },
  joystick: {
    isDragging: false,
    position: { x: 0, y: 0 },
    maxRadius: ROBOT_CONFIG.JOYSTICK_MAX_RADIUS,
  },
  sensors: {
    laser: null,
    camera: null,
    imu: null,
  },
  lastCommand: null,
  velocityLimits: VELOCITY_LIMITS,
};

const robotSlice = createSlice({
  name: 'robot',
  initialState,
  reducers: {
    updateRobotStatus: (state, action) => {
      state.status = { ...state.status, ...action.payload };
    },
    setRobotMode: (state, action) => {
      state.status.mode = action.payload;
    },
    setRobotEnabled: (state, action) => {
      state.status.enabled = action.payload;
      state.status.state = action.payload ? ROBOT_STATES.ENABLED : ROBOT_STATES.DISABLED;
    },
    setEmergencyStop: (state, action) => {
      state.status.emergencyStop = action.payload;
      if (action.payload) {
        state.status.state = ROBOT_STATES.EMERGENCY_STOP;
        state.status.enabled = false;
        state.currentVelocity = { linear: 0, angular: 0 };
        state.joystick.isDragging = false;
        state.joystick.position = { x: 0, y: 0 };
      }
    },
    updateVelocity: (state, action) => {
      const { linear, angular } = action.payload;
      
      // Apply velocity limits from config
      const clampedLinear = Math.max(
        VELOCITY_LIMITS.LINEAR_MIN, 
        Math.min(VELOCITY_LIMITS.LINEAR_MAX, linear)
      );
      const clampedAngular = Math.max(
        VELOCITY_LIMITS.ANGULAR_MIN, 
        Math.min(VELOCITY_LIMITS.ANGULAR_MAX, angular)
      );
      
      state.currentVelocity = { linear: clampedLinear, angular: clampedAngular };
      state.status.velocity = `${Math.abs(clampedLinear || clampedAngular).toFixed(2)} m/s`;
    },
    updateOdometry: (state, action) => {
      const { velocity, position } = action.payload;
      state.realTimeData.odometry = { velocity, position };
      state.realTimeData.lastUpdate = new Date().toISOString();
      
      // Update status display with real-time data
      state.status.velocity = `${velocity.linear.toFixed(2)} m/s`;
      state.status.position = `${position.x.toFixed(2)}, ${position.y.toFixed(2)}`;
    },
    updateBattery: (state, action) => {
      const { percentage, voltage, status } = action.payload;
      state.realTimeData.battery = { percentage, voltage, status };
      
      // Update status display with real-time data
      state.status.battery = `${percentage}%`;
    },
    updatePose: (state, action) => {
      const { position } = action.payload;
      state.realTimeData.pose = { position };
      
      // Update status display with pose data if odometry not available
      if (!state.realTimeData.odometry.position.x && !state.realTimeData.odometry.position.y) {
        state.status.position = `${position.x.toFixed(2)}, ${position.y.toFixed(2)}`;
      }
    },
    setJoystickDragging: (state, action) => {
      state.joystick.isDragging = action.payload;
    },
    setJoystickPosition: (state, action) => {
      state.joystick.position = action.payload;
    },
    resetJoystick: (state) => {
      state.joystick.isDragging = false;
      state.joystick.position = { x: 0, y: 0 };
    },
    updateSensorData: (state, action) => {
      const { sensor, data } = action.payload;
      state.sensors[sensor] = data;
    },
    setLastCommand: (state, action) => {
      state.lastCommand = {
        command: action.payload,
        timestamp: new Date().toISOString(),
      };
    },
    emergencyStop: (state) => {
      state.status.emergencyStop = true;
      state.status.state = ROBOT_STATES.EMERGENCY_STOP;
      state.status.enabled = false;
      state.currentVelocity = { linear: 0, angular: 0 };
      state.status.velocity = ROBOT_CONFIG.DEFAULT_VELOCITY;
      state.joystick.isDragging = false;
      state.joystick.position = { x: 0, y: 0 };
      state.lastCommand = {
        command: 'EMERGENCY_STOP',
        timestamp: new Date().toISOString(),
      };
    },
  },
});

export const {
  updateRobotStatus,
  setRobotMode,
  setRobotEnabled,
  setEmergencyStop,
  updateVelocity,
  updateOdometry,
  updateBattery,
  updatePose,
  setJoystickDragging,
  setJoystickPosition,
  resetJoystick,
  updateSensorData,
  setLastCommand,
  emergencyStop,
} = robotSlice.actions;

export default robotSlice.reducer;