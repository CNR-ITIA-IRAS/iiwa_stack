/**
* Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
* Technische Universität München
* Chair for Computer Aided Medical Procedures and Augmented Reality
* Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
* http://campar.in.tum.de
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

package de.tum.in.camp.kuka.ros.app;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import de.tum.in.camp.kuka.ros.*;
import de.tum.in.camp.kuka.ros.iiwaSubscriber.CommandType;
import geometry_msgs.PoseStamped;
import iiwa_msgs.*;
import org.ros.address.BindAddress;
import org.ros.address.PublicAdvertiseAddressFactory;
import org.ros.exception.ServiceException;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.time.NtpTimeProvider;

import java.net.URI;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/*
* This application allows to command the robot using SmartServo motions.
*/
public class ROSSmartServo extends ROSBaseApplication {

	private ControlModeHandler.ControlMode act_control_mode = null;
	private IMotionContainer pos_hold_container = null ;

	private Lock configureSmartServoLock = new ReentrantLock();

	private iiwaSubscriber subscriber; // IIWARos Subscriber.
	private NodeConfiguration nodeConfSubscriber;   // Configuration of the subscriber ROS node.

	private CommandType lastCommandType = CommandType.JOINT_POSITION;
	private Motions motions;

	@Override
	protected void configureNodes(URI uri) {
		// Configuration for the Subscriber.
		nodeConfSubscriber = NodeConfiguration.newPublic(configuration.getRobotIp());
		nodeConfSubscriber.setTimeProvider(configuration.getTimeProvider());
		nodeConfSubscriber.setNodeName(configuration.getRobotName() + "/iiwa_subscriber2");
		nodeConfSubscriber.setXmlRpcBindAddress(BindAddress.newPublic(30005));
		nodeConfSubscriber.setXmlRpcAdvertiseAddressFactory(new PublicAdvertiseAddressFactory(configuration.getRobotIp()) );
		nodeConfSubscriber.setTcpRosBindAddress(BindAddress.newPublic(30004));
		nodeConfSubscriber.setTcpRosAdvertiseAddressFactory(new PublicAdvertiseAddressFactory(configuration.getRobotIp()) );
		nodeConfSubscriber.setMasterUri(uri);
	}

	@Override
	protected void addNodesToExecutor(NodeMainExecutor nodeMainExecutor) {
		subscriber = new iiwaSubscriber(robot, configuration.getRobotName(), configuration);

		// Configure the callback for the SmartServo service inside the subscriber class.
		subscriber.setConfigureSmartServoCallback(new ServiceResponseBuilder<iiwa_msgs.ConfigureSmartServoRequest, iiwa_msgs.ConfigureSmartServoResponse>() {
			@Override
			public void build(ConfigureSmartServoRequest req, ConfigureSmartServoResponse res) throws ServiceException {
				
				int trials = 0;
				while (trials < 3)
				{
					configureSmartServoLock.lock();
					
					try {
						trials++;
						act_control_mode = ControlModeHandler.ToControlMode( req.getControlMode( ) );
						Logger.info(Integer.toString(req.getControlMode( )));

						if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) {
							if (controlModeHandler.isSameControlMode(linearMotion.getMode(), act_control_mode.getValue() )) { // We can just change the parameters if the control strategy is the same.
								if (!(linearMotion.getMode() instanceof PositionControlMode)) { // We are in PositioControlMode and the request was for the same mode, there are no parameters to change.
									linearMotion.getRuntime().changeControlModeSettings(controlModeHandler.buildMotionControlMode(req));
								}
							} else {
								linearMotion = controlModeHandler.switchSmartServoMotion(linearMotion, req);
							}
						}
						else {
							//						Logger.info("inside else");
							//						Logger.info(Integer.toString(act_control_mode.getValue()));
							//						Logger.info( smartMotion.getMode().getClass().getSimpleName() );
							//						if (controlModeHandler.isSameControlMode(smartMotion.getMode(), act_control_mode.getValue() )) { // We can just change the parameters if the control strategy is the same.
							//							if (!(smartMotion.getMode() instanceof PositionControlMode)) { // We are in PositioControlMode and the request was for the same mode, there are no parameters to change.
							//								Logger.info("ifif");
							//								smartMotion.getRuntime().changeControlModeSettings(controlModeHandler.buildMotionControlMode(req));
							//							}
							//							else
							//								Logger.info("uffa");
							//						} else {
							//							Logger.info("else");
							smartMotion = controlModeHandler.switchSmartServoMotion(smartMotion, req);
							//						}
						}

						res.setSuccess(true);
						controlModeHandler.setLastSmartServoRequest(req);
						break;
					} catch (Exception e) {
						ControlModeHandler.cleanup(robot);
						res.setSuccess(false);
						if (e.getMessage() != null) {
							res.setError(e.getClass().getName() + ": " + e.getMessage());
							Logger.error(e.getClass().getName() + ": " + e.getMessage());
						} else {
							res.setError("because I hate you :)");
						}
					}
					finally {
						configureSmartServoLock.unlock();
					}
				}
			}
		});

		// TODO: doc
		subscriber.setTimeToDestinationCallback(new ServiceResponseBuilder<iiwa_msgs.TimeToDestinationRequest, iiwa_msgs.TimeToDestinationResponse>() {

			@Override
			public void build(TimeToDestinationRequest req, TimeToDestinationResponse res) {
				try {
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) {
						linearMotion.getRuntime().updateWithRealtimeSystem();
						res.setRemainingTime(linearMotion.getRuntime().getRemainingTime());
					}
					else {
						smartMotion.getRuntime().updateWithRealtimeSystem();
						res.setRemainingTime(smartMotion.getRuntime().getRemainingTime());
					}
				}
				catch(Exception e) {
					// An exception should be thrown only if a smartMotion/runtime is not available.
					res.setRemainingTime(-999);
				}
			}
		});

		// TODO: doc
		subscriber.setPathParametersCallback(new ServiceResponseBuilder<iiwa_msgs.SetPathParametersRequest, iiwa_msgs.SetPathParametersResponse>() {
			@Override
			public void build(iiwa_msgs.SetPathParametersRequest req, iiwa_msgs.SetPathParametersResponse res) throws ServiceException {
				configureSmartServoLock.lock();
				try {
					if (req.getJointRelativeVelocity() >= 0) {
						controlModeHandler.jointVelocity = req.getJointRelativeVelocity();
					}
					if (req.getJointRelativeAcceleration() >= 0) {
						controlModeHandler.jointAcceleration = req.getJointRelativeAcceleration();
					}
					if (req.getOverrideJointAcceleration() >= 0) {
						controlModeHandler.overrideJointAcceleration = req.getOverrideJointAcceleration();
					}
					if (lastCommandType != CommandType.CARTESIAN_POSE_LIN) {
						iiwa_msgs.ConfigureSmartServoRequest request = null;
						smartMotion = controlModeHandler.switchSmartServoMotion(smartMotion, request);
					}

					res.setSuccess(true);
				}
				catch(Exception e) {
					res.setError(e.getClass().getName() + ": " + e.getMessage());
					res.setSuccess(false);
				}
				finally {
					configureSmartServoLock.unlock();
				}
			}
		});

		subscriber.setPathParametersLinCallback(new ServiceResponseBuilder<iiwa_msgs.SetPathParametersLinRequest, iiwa_msgs.SetPathParametersLinResponse>() {
			@Override
			public void build(SetPathParametersLinRequest req, SetPathParametersLinResponse res) {
				configureSmartServoLock.lock();
				try {
					if (isVector3GreaterThan(req.getMaxCartesianVelocity().getLinear(), 0)) { // TODO: this just works for linear velocity atm
						controlModeHandler.maxTranslationlVelocity = Conversions.rosVectorToArray(req.getMaxCartesianVelocity().getLinear());
					}
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) {
						iiwa_msgs.ConfigureSmartServoRequest request = null;
						linearMotion = controlModeHandler.switchSmartServoMotion(linearMotion, request);
					}
					res.setSuccess(true);
				}
				catch (Exception e) {
					res.setError(e.getClass().getName() + ": " + e.getMessage());
					res.setSuccess(false);
				}
				finally {
					configureSmartServoLock.unlock();
				}
			}
		});

		// Execute the subscriber node.
		nodeMainExecutor.execute(subscriber, nodeConfSubscriber);
	}

	// TODO move this somewhere else.
	private boolean isTwistGreaterThan(geometry_msgs.Twist twist, double value) {
		return (twist.getLinear().getX() > value &&
			twist.getLinear().getY() > value &&
			twist.getLinear().getZ() > value &&
			twist.getAngular().getX() > value &&
			twist.getAngular().getY() > value &&
			twist.getAngular().getZ() > value);
	}

	private boolean isVector3GreaterThan(geometry_msgs.Vector3 vector, double value) {
		return ((vector.getX() > value) &&
                (vector.getY() > value) &&
                (vector.getZ() > value) );
	}




	@Override
	protected void initializeApp() {

	 }

    @Override
    protected void beforeControlLoop() {

    	motions = new Motions(robot, smartMotion);

    	// Initialize smartMotion.
    	toolFrame.moveAsync(smartMotion);
    	// Hook the GoalReachedEventHandler
    	smartMotion.getRuntime().setGoalReachedEventHandler(handler);

    	if (configuration.getTimeProvider() instanceof org.ros.time.NtpTimeProvider) {
    		((NtpTimeProvider) configuration.getTimeProvider()).startPeriodicUpdates(100, TimeUnit.MILLISECONDS); // TODO: update time as param
    	}


    }

    /**
    * TODO: doc, take something from
    * This will acquire the last received CartesianPose command from the commanding ROS node, if there is any available.
    * If the robot can move, then it will move to this new position.
    */
    private void moveRobot() {

    	if (act_control_mode == ControlModeHandler.ControlMode.FRI_JOINT_POS_CONTROL ) {
    		
    		if( pos_hold_container == null)
    		{
    			getLogger().info("set pos hold...");
    			
    			robot.getController().getExecutionService().cancelAll();
    			
				PositionControlMode pos_control_mode = null;
				PositionHold pos_hold = null;

				pos_control_mode = new PositionControlMode();
				pos_hold = new PositionHold(pos_control_mode, -1, null);
				robot.move(ptp(robot.getCurrentJointPosition()));
				pos_hold_container = robot.moveAsync(pos_hold.addMotionOverlay(ControlModeHandler.Overlay));
			
				getLogger().info("pos hold ok!");
				
				ThreadUtil.milliSleep(500);
    		}
    		
//    		getLogger().error(robot.getCurrentJointPosition().toString());
//    		getLogger().error(robot.getCommandedJointPosition().toString());
//    		getLogger().error("------------------------");
//    		
//    		ThreadUtil.milliSleep(500);
			
		}
    	else if ( act_control_mode == ControlModeHandler.ControlMode.FRI_JOINT_TORQUE_CONTROL || act_control_mode == ControlModeHandler.ControlMode.FRI_JOINT_IMP_CONTROL ) {
    		
    		if( pos_hold_container == null)
    		{
    			getLogger().info("set pos hold...");
    			
    			robot.getController().getExecutionService().cancelAll();
    			
				PositionHold pos_hold = null;

				pos_hold = new PositionHold(smartMotion.getMode(), -1, null);
				robot.move(ptp(robot.getCurrentJointPosition()));
				pos_hold_container = robot.moveAsync(pos_hold.addMotionOverlay(ControlModeHandler.Overlay));
			
				getLogger().info("pos hold ok!");
				
				ThreadUtil.milliSleep(500);
    		}
    		
//    		getLogger().error(robot.getCurrentJointPosition().toString());
//    		getLogger().error(robot.getCommandedJointPosition().toString());
//    		getLogger().error("------------------------");
//    		
//    		ThreadUtil.milliSleep(500);
			
		}
		else if (subscriber.currentCommandType != null) {
			if( pos_hold_container != null )
			{
				pos_hold_container.cancel();
				pos_hold_container = null;
			}
    		try {
				switch (subscriber.currentCommandType) {
				case CARTESIAN_POSE: {
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) {
						smartMotion = controlModeHandler.switchToSmartServo(smartMotion, linearMotion);
					}
					PoseStamped commandPosition = subscriber.getCartesianPose();
					motions.cartesianPositionMotion(smartMotion, commandPosition);
					break;
					}
				case CARTESIAN_POSE_LIN: {
					if (lastCommandType != CommandType.CARTESIAN_POSE_LIN) {
						linearMotion = controlModeHandler.switchToSmartServoLIN(smartMotion, linearMotion);
					}
					PoseStamped commandPosition = subscriber.getCartesianPoseLin();
					motions.cartesianPositionLinMotion(linearMotion, commandPosition);
					break;
				}
				case CARTESIAN_VELOCITY: {
					geometry_msgs.TwistStamped commandVelocity = subscriber.getCartesianVelocity();
					motions.cartesianVelocityMotion(smartMotion, commandVelocity, toolFrame);
					break;
				}
				case JOINT_POSITION: {
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) {
						smartMotion = controlModeHandler.switchToSmartServo(smartMotion, linearMotion);
					}
					iiwa_msgs.JointPosition commandPosition = subscriber.getJointPosition();
					motions.jointPositionMotion(smartMotion, commandPosition);
					break;
				}
				case JOINT_POSITION_VELOCITY: {
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) {
						smartMotion = controlModeHandler.switchToSmartServo(smartMotion, linearMotion);
					}
					iiwa_msgs.JointPositionVelocity commandPositionVelocity = subscriber.getJointPositionVelocity();
					motions.jointPositionVelocityMotion(smartMotion, commandPositionVelocity);
					break;
				}
				case JOINT_VELOCITY: {
					if (lastCommandType == CommandType.CARTESIAN_POSE_LIN) {
						smartMotion = controlModeHandler.switchToSmartServo(smartMotion, linearMotion);
					}
					/* This will acquire the last received JointVelocity command from the commanding ROS node, if there is any available.
					* If the robot can move, then it will move to this new position accordingly to the given joint velocity. */
					smartMotion.getRuntime().activateVelocityPlanning(true);
					smartMotion.setSpeedTimeoutAfterGoalReach(0.1);
					iiwa_msgs.JointVelocity commandVelocity = subscriber.getJointVelocity();
					motions.jointVelocityMotion(smartMotion, commandVelocity);
					break;
				}
				default:
					throw new UnsupportedControlModeException();
				}
			}
			catch (Exception e) {
				Logger.error(e.getClass().getName() + ": " + e.getMessage());
			}
		}
		lastCommandType = subscriber.currentCommandType;


	}

	@Override
	protected void controlLoop() {
		configureSmartServoLock.lock();
		moveRobot();
        configureSmartServoLock.unlock();
    }

    @Override
    protected boolean cleanup()
    {
    	ControlModeHandler.cleanup(robot);
    	return super.cleanup();
    }

}