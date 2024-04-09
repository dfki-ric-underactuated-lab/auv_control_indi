import numpy as np

class DragModel:
    def __init__(self, params:dict):
        self.UpdateParams(params)

    def UpdateParams(self, params:dict):
        pass
    
    def CalculateWrench(self, twist):
        return self.CalculateWrenchLin(twist) + self.CalculateWrenchNonlin(twist)

    def CalculateWrenchLin(self, twist):
        pass

    def CalculateWrenchNonlin(self, twist):
        return np.zeros(6)

class LinearDragModel(DragModel):
    def __init__(self, params:dict):
        self.UpdateParams(params)

    def UpdateParams(self, params:dict):
        self.linear_damping_matrix = np.asarray(params['Dl'])
    
    def CalculateWrenchLin(self, twist):
        return self.linear_damping_matrix.dot(twist)

class QuadraticDragModel(DragModel):
    def __init__(self, params:dict):
        self.UpdateParams(params)

    def UpdateParams(self, params:dict):
        self.linear_damping_matrix = np.asarray(params['Dl'])
        self.quad_damping_matrix = np.asarray(params['Dq'])
    
    def CalculateWrenchLin(self, twist):
        return self.linear_damping_matrix.dot(twist)

    def CalculateWrenchNonlin(self, twist):
        return self.quad_damping_matrix.dot(np.abs(twist) * twist)