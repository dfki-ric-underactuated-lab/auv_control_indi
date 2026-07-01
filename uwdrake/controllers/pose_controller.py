##
# @file
# @brief Pose (position + orientation) controller producing a body-twist command.

from pydrake.systems.framework import BasicVector, LeafSystem, PortDataType

import numpy as np

from ..common import RigidBodyState, quat2rot

class PoseController(LeafSystem):
    ## @brief Proportional pose controller outputting a velocity-limited body twist.
    #  Ports: in "state"(13), "pos_ref"(3), "ori_ref"(4) -> out "control_twist"(6).
    def __init__(self, Kp, vlin_max, vang_max):
        '''@brief Declare ports and store the gain matrix and velocity limits.'''
        super().__init__()
        
        self.twist_output_port = self.DeclareVectorOutputPort(
            "control_twist",
            BasicVector(6),
            self.Update)
        
        self.state_input_port = self.DeclareInputPort("state",
            PortDataType.kVectorValued,
            size=RigidBodyState.SIZE)
        

        self.position_ref_input_port = self.DeclareInputPort("pos_ref",
                                        PortDataType.kVectorValued,
                                        size=3)
        

        self.orientation_ref_input_port = self.DeclareInputPort("ori_ref",
                                        PortDataType.kVectorValued,
                                        size=4)
        
        self.gains = Kp
        self.max_angular_velocity = vang_max
        self.max_linear_velocity  = vlin_max
        
    def Update(self, context, output):
        '''@brief Output callback: P control on orientation and body-frame position error.'''
        x = self.state_input_port.Eval(context)
        pos_ref = self.position_ref_input_port.Eval(context)
        ori_ref = self.orientation_ref_input_port.Eval(context)

        rot_ref = quat2rot(ori_ref)
        rot_cur = quat2rot(x[3:7])

        e1 = np.array([1, 0, 0])
        e2 = np.array([0, 1, 0])
        e3 = np.array([0, 0, 1])

        # Attitude error (Chaturvedi et al. 2011, singularity-free rotation-matrix law).
        rot_err = np.cross(e1, rot_ref.T.dot(rot_cur.dot(e1)))\
                    + np.cross(e2, rot_ref.T.dot(rot_cur.dot(e2)))\
                    + np.cross(e3, rot_ref.T.dot(rot_cur.dot(e3)))

        vang_cmd = - self.gains[3:6,3:6].dot(rot_err)
        vang_cmd = np.clip(vang_cmd, -self.max_angular_velocity, +self.max_angular_velocity)

        pos_err = pos_ref - x[0:3]
        pos_err_body = rot_cur.T.dot(pos_err)

        vlin_cmd = np.clip(self.gains[0:3,0:3].dot(pos_err_body),
                           -self.max_linear_velocity,
                           +self.max_linear_velocity)

        output.SetFromVector(np.concatenate((vlin_cmd, vang_cmd)))
