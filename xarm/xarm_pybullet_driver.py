from ark.system.pybullet.pybullet_robot_driver import BulletRobotDriver

class XarmPyBulletDriver(BulletRobotDriver):

    def __init__(self, component_name: str, component_config: dict[str, any] = None, client: bool = True) -> None:
        super().__init__(component_name, component_config, client)
    
    def pass_cartesian_control_cmd(self, control_mode: str, position, quaternion, **kwargs) -> None:
        """!Send a Cartesian control command by computing inverse kinematics.

        @param control_mode One of ``position``, ``velocity`` or ``torque``.
        @param position List of 3 floats representing the desired XYZ position.
        @param quaternion List of 4 floats representing the desired orientation as a quaternion.
        @param kwargs Additional keyword arguments passed to joint control.
        @return ``None``
        """
        print(f"Passing Cartesian control command with position: {position} and quaternion: {quaternion}")
        if not (len(position) == 3 and len(quaternion) == 4):
            raise ValueError("Position must be 3 elements and quaternion must be 4 elements.")

        end_effector_idx = kwargs.get("end_effector_idx")

        # Compute IK solution
        joint_angles = self.client.calculateInverseKinematics(
            bodyUniqueId=self.ref_body_id,
            endEffectorLinkIndex=end_effector_idx,
            targetPosition=position,
            targetOrientation=quaternion
        )

        if joint_angles is None:
            raise ValueError("Inverse kinematics failed to compute joint angles.")
        
        self.client.setJointMotorControlArray(
            bodyUniqueId=self.ref_body_id,
            jointIndices=list(range(7)),  # Assuming 7 joints
            controlMode=self.client.POSITION_CONTROL,
            targetPositions=joint_angles
        )