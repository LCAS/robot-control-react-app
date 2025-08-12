import ROSLIB from 'roslib';
import { ROS_CONFIG, ROBOT_MODES, ROBOT_STATES } from '../utils/constants';

class RosService {
    constructor() {
        this.ros = null;
        this.cmdVelPublisher = null;
        this.goalPublisher = null;
        this.robotModePublisher = null;
        this.robotEnablePublisher = null;
        this.emergencyStopPublisher = null;
        this.odomSubscriber = null;
        this.batterySubscriber = null;
        this.poseSubscriber = null;
        this.isConnected = false;
        
        // Callbacks for data updates
        this.onOdometryUpdate = null;
        this.onBatteryUpdate = null;
        this.onPoseUpdate = null;
    }

    connect(url = ROS_CONFIG.BRIDGE_URL) {
        try {
            console.log('🔌 Connecting to ROS bridge:', url);
            this.ros = new ROSLIB.Ros({
                url: url
            });

            this.ros.on('connection', () => {
                console.log('✅ Connected to ROS bridge');
                this.isConnected = true;
                setTimeout(() => {
                    this.setupPublishers();
                    this.setupSubscribers();
                }, 100);
            });

            this.ros.on('error', (error) => {
                console.error('❌ Error connecting to ROS bridge:', error);
                this.isConnected = false;
            });

            this.ros.on('close', () => {
                console.log('🔌 Connection to ROS bridge closed');
                this.isConnected = false;
                this.cleanupSubscribers();
            });

        } catch (error) {
            console.error('❌ Failed to create ROS connection:', error);
            this.isConnected = false;
        }
    }

    setupPublishers() {
        console.log('🚀 Setting up publishers...');
        
        // Setup velocity publisher
        this.cmdVelPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: ROS_CONFIG.CMD_VEL_TOPIC,
            messageType: ROS_CONFIG.MESSAGE_TYPE
        });

        // Setup navigation goal publisher
        this.goalPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: ROS_CONFIG.GOAL_TOPIC,
            messageType: ROS_CONFIG.GOAL_MESSAGE_TYPE
        });

        // Setup robot control publishers
        this.robotModePublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: ROS_CONFIG.ROBOT_MODE_TOPIC,
            messageType: ROS_CONFIG.ROBOT_MODE_MESSAGE_TYPE
        });

        this.robotEnablePublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: ROS_CONFIG.ROBOT_ENABLE_TOPIC,
            messageType: ROS_CONFIG.ROBOT_ENABLE_MESSAGE_TYPE
        });

        this.emergencyStopPublisher = new ROSLIB.Topic({
            ros: this.ros,
            name: ROS_CONFIG.EMERGENCY_STOP_TOPIC,
            messageType: ROS_CONFIG.EMERGENCY_STOP_MESSAGE_TYPE
        });

        console.log('✅ Publishers setup completed:');
        console.log('- CMD_VEL:', ROS_CONFIG.CMD_VEL_TOPIC);
        console.log('- GOAL:', ROS_CONFIG.GOAL_TOPIC);
        console.log('- ROBOT_MODE:', ROS_CONFIG.ROBOT_MODE_TOPIC);
        console.log('- ROBOT_ENABLE:', ROS_CONFIG.ROBOT_ENABLE_TOPIC);
        console.log('- EMERGENCY_STOP:', ROS_CONFIG.EMERGENCY_STOP_TOPIC);
    }

    setupSubscribers() {
        // Setup odometry subscriber
        this.odomSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: ROS_CONFIG.ODOM_TOPIC,
            messageType: ROS_CONFIG.ODOM_MESSAGE_TYPE
        });

        this.odomSubscriber.subscribe((message) => {
            if (this.onOdometryUpdate) {
                const velocity = {
                    linear: Math.sqrt(
                        Math.pow(message.twist.twist.linear.x, 2) +
                        Math.pow(message.twist.twist.linear.y, 2)
                    ),
                    angular: message.twist.twist.angular.z
                };

                const position = {
                    x: message.pose.pose.position.x,
                    y: message.pose.pose.position.y,
                    z: message.pose.pose.position.z
                };

                this.onOdometryUpdate({ velocity, position });
            }
        });

        // Setup battery subscriber
        this.batterySubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: ROS_CONFIG.BATTERY_TOPIC,
            messageType: ROS_CONFIG.BATTERY_MESSAGE_TYPE
        });

        this.batterySubscriber.subscribe((message) => {
            if (this.onBatteryUpdate) {
                const batteryPercentage = Math.round(message.percentage * 100);
                const batteryVoltage = message.voltage;
                const batteryLevel = message.power_supply_status;
                
                this.onBatteryUpdate({
                    percentage: batteryPercentage,
                    voltage: batteryVoltage,
                    status: batteryLevel
                });
            }
        });

        // Setup pose subscriber (alternative to odometry for position)
        this.poseSubscriber = new ROSLIB.Topic({
            ros: this.ros,
            name: ROS_CONFIG.POSE_TOPIC,
            messageType: ROS_CONFIG.POSE_MESSAGE_TYPE
        });

        this.poseSubscriber.subscribe((message) => {
            if (this.onPoseUpdate) {
                const position = {
                    x: message.pose.pose.position.x,
                    y: message.pose.pose.position.y,
                    z: message.pose.pose.position.z
                };

                this.onPoseUpdate({ position });
            }
        });

        console.log('Subscribers setup:', {
            odometry: ROS_CONFIG.ODOM_TOPIC,
            battery: ROS_CONFIG.BATTERY_TOPIC,
            pose: ROS_CONFIG.POSE_TOPIC
        });
    }

    cleanupSubscribers() {
        if (this.odomSubscriber) {
            this.odomSubscriber.unsubscribe();
            this.odomSubscriber = null;
        }
        if (this.batterySubscriber) {
            this.batterySubscriber.unsubscribe();
            this.batterySubscriber = null;
        }
        if (this.poseSubscriber) {
            this.poseSubscriber.unsubscribe();
            this.poseSubscriber = null;
        }
    }

    // Callback setters
    setOdometryCallback(callback) {
        this.onOdometryUpdate = callback;
    }

    setBatteryCallback(callback) {
        this.onBatteryUpdate = callback;
    }

    setPoseCallback(callback) {
        this.onPoseUpdate = callback;
    }

    publishVelocity(linear, angular) {
        if (this.cmdVelPublisher && this.isConnected) {
            const twist = new ROSLIB.Message({
                linear: {
                    x: linear,
                    y: 0,
                    z: 0
                },
                angular: {
                    x: 0,
                    y: 0,
                    z: angular
                }
            });
            this.cmdVelPublisher.publish(twist);
            console.log('📤 Published velocity:', { linear, angular });
        } else {
            console.error('❌ Cannot publish velocity - publisher not ready');
        }
    }

    publishRobotMode(mode) {
        if (this.robotModePublisher && this.isConnected) {
            const modeMessage = new ROSLIB.Message({
                data: mode
            });
            this.robotModePublisher.publish(modeMessage);
            console.log('📤 Published robot mode:', mode);
        } else {
            throw new Error('Robot mode publisher not ready or not connected');
        }
    }

    publishRobotEnable(enabled) {
        if (this.robotEnablePublisher && this.isConnected) {
            const enableMessage = new ROSLIB.Message({
                data: enabled
            });
            this.robotEnablePublisher.publish(enableMessage);
            console.log('📤 Published robot enable:', enabled);
        } else {
            throw new Error('Robot enable publisher not ready or not connected');
        }
    }

    publishEmergencyStop(stop = true) {
        if (this.emergencyStopPublisher && this.isConnected) {
            const stopMessage = new ROSLIB.Message({
                data: stop
            });
            this.emergencyStopPublisher.publish(stopMessage);
            console.log('📤 Published emergency stop:', stop);
            
            // Also immediately stop all motion
            if (stop) {
                this.publishVelocity(0, 0);
            }
        } else {
            throw new Error('Emergency stop publisher not ready or not connected');
        }
    }

    publishNavigationGoal(x, y, orientation = 0) {
        console.log('=== publishNavigationGoal DEBUG ===');
        console.log('Parameters:', { x, y, orientation });
        console.log('ROS connected:', this.isConnected);
        console.log('Goal publisher exists:', !!this.goalPublisher);
        console.log('Goal topic:', ROS_CONFIG.GOAL_TOPIC);
        console.log('Goal message type:', ROS_CONFIG.GOAL_MESSAGE_TYPE);

        if (!this.isConnected) {
            const error = 'Not connected to ROS bridge';
            console.error('ERROR:', error);
            throw new Error(error);
        }

        if (!this.goalPublisher) {
            const error = 'Goal publisher not initialized';
            console.error('ERROR:', error);
            throw new Error(error);
        }

        try {
            // Create timestamp
            const now = Date.now();
            const sec = Math.floor(now / 1000);
            const nanosec = (now % 1000) * 1000000;

            const goalMessage = new ROSLIB.Message({
                header: {
                    stamp: {
                        sec: sec,
                        nanosec: nanosec
                    },
                    frame_id: 'map'
                },
                pose: {
                    position: {
                        x: parseFloat(x),
                        y: parseFloat(y),
                        z: 0.0
                    },
                    orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: Math.sin(orientation / 2),
                        w: Math.cos(orientation / 2)
                    }
                }
            });

            console.log('Goal message created:', JSON.stringify(goalMessage, null, 2));
            
            // Attempt to publish
            console.log('Attempting to publish...');
            this.goalPublisher.publish(goalMessage);
            console.log('✅ Goal message published successfully!');
            
        } catch (error) {
            console.error('❌ Error publishing goal:', error);
            console.error('Error details:', error.message);
            console.error('Error stack:', error.stack);
            throw error;
        }
    }

    stopRobot() {
        this.publishVelocity(0, 0);
        this.publishEmergencyStop(true);
    }

    disconnect() {
        this.cleanupSubscribers();
        if (this.ros) {
            this.ros.close();
        }
        this.isConnected = false;
        this.cmdVelPublisher = null;
        this.goalPublisher = null;
        this.robotModePublisher = null;
        this.robotEnablePublisher = null;
        this.emergencyStopPublisher = null;
    }
}

const rosService = new RosService();
export default rosService;