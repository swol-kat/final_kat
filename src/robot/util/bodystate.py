import numpy as np


class BodyState:

    def __init__(self, **kwargs):
        """
        look at update state documentaions
        """
        self.x = 0.
        self.y = 0.
        self.z = 0.
        self.gamma = 0.
        self.beta = 0.
        self.alpha = 0.
        self.update_state(**kwargs)

    def update_state(self, **kwargs):
        """
        pass in either
        matrix: 6, matrix assumes order x,y,z,gamma,beta,alpha
        x,y,z,alpha,beta,gamma: as floats
        :param kwargs:
        """
        if 'matrix' in kwargs:
            try:
                m = np.array(kwargs['matrix'])
                args = m.reshape(6)
                attributes = self.keys()
                kwargs = {attributes[i]: k for i, k in enumerate(args)}
            except Exception as e:
                raise Exception(f"{type(e).__name__}: matrix must have at least 6 elements and contain floats")

        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)

    def __str__(self):
        return f'x = {self.x}\ny = {self.y}\nz = {self.z}\ngamma = {self.gamma}\nbeta = {self.beta}\nalpha = {self.alpha}'

    def keys(self):
        return ['x', 'y', 'z', 'gamma', 'beta', 'alpha']

    def __getitem__(self, key):
        return getattr(self, key)

    def __iter__(self):
        return iter([self.x, self.y, self.z, self.gamma, self.beta, self.alpha])

    def move(self, **kwargs):
        cur_state = dict(self)
        for key, value in kwargs.items():
            cur_state[key] += value
        self.update_state(**cur_state)
