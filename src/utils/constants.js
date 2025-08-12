export const ROS_CONFIG = {
    BRIDGE_URL: process.env.REACT_APP_ROSBRIDGE_URL || 'ws://localhost:9090',
    CMD_VEL_TOPIC: '/cmd_vel',
    MESSAGE_TYPE: 'geometry_msgs/Twist',
    GOAL_TOPIC: '/goal_pose',
    GOAL_MESSAGE_TYPE: 'geometry_msgs/PoseStamped',
    
    // Control topics
    ROBOT_MODE_TOPIC: '/robot_mode',
    ROBOT_MODE_MESSAGE_TYPE: 'std_msgs/String',
    ROBOT_ENABLE_TOPIC: '/robot_enable',
    ROBOT_ENABLE_MESSAGE_TYPE: 'std_msgs/Bool',
    EMERGENCY_STOP_TOPIC: '/emergency_stop',
    EMERGENCY_STOP_MESSAGE_TYPE: 'std_msgs/Bool',
    
    // Subscription topics
    ODOM_TOPIC: '/odom',
    ODOM_MESSAGE_TYPE: 'nav_msgs/Odometry',
    BATTERY_TOPIC: '/battery_state',
    BATTERY_MESSAGE_TYPE: 'sensor_msgs/BatteryState',
    POSE_TOPIC: '/amcl_pose',
    POSE_MESSAGE_TYPE: 'geometry_msgs/PoseWithCovarianceStamped',
};

export const VELOCITY_LIMITS = {
    LINEAR_MAX: 2.0,
    LINEAR_MIN: -2.0,
    ANGULAR_MAX: 1.2,
    ANGULAR_MIN: -1.2,
    STEP: 0.2
};

export const CONNECTION_CONFIG = {
    RETRY_INTERVAL: 1000,
    MAX_RETRY_TIME: 10000,
    RECONNECT_DELAY: 2000
};

export const ROBOT_CONFIG = {
    DEFAULT_MODE: 'Manual',
    DEFAULT_BATTERY: '50%',
    DEFAULT_POSITION: '0.0, 0.0',
    DEFAULT_VELOCITY: '0.0 m/s',
    JOYSTICK_MAX_RADIUS: 55,
    VELOCITY_THRESHOLD: 0.05,
};

export const LOG_CONFIG = {
    MAX_ENTRIES: 100,
    DEFAULT_FILTER: 'all',
    LEVELS: {
        INFO: 'info',
        WARNING: 'warning',
        ERROR: 'error',
        SUCCESS: 'success'
    }
};

export const NAVIGATION_CONFIG = {
    MAX_GOAL_HISTORY: 10,
    QUICK_GOALS: [
        { name: 'Farm Tunnel', x: -3, y: -2 },
        { name: 'Home Base', x: 0, y: 0 },
    ],
    NAVIGATION_STATUSES: {
        IDLE: 'idle',
        PLANNING: 'planning',
        MOVING: 'moving',
        REACHED: 'reached',
        FAILED: 'failed'
    }
};

export const ROBOT_MODES = {
    MANUAL: 'manual',
    AUTONOMOUS: 'autonomous',
    TELEOP: 'teleop',
    IDLE: 'idle'
};

export const ROBOT_STATES = {
    ENABLED: 'enabled',
    DISABLED: 'disabled',
    EMERGENCY_STOP: 'emergency_stop',
    ERROR: 'error'
};