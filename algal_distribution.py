import numpy as np
import numpy.typing as npt

from scipy.stats import multivariate_normal


# This currently uses the probability density function (PDF) of a multivariate gaussian, which isn't entirely correct.

class AlgalDistribution:
    def __init__(self, meanX: float, meanY: float, covariance: npt.ArrayLike):
        self.mean = [meanX, meanY]
        self.covariance = covariance

        self.x,  self.y = np.mgrid[-20:20:.1, -20:20:.1]
        pos = np.dstack((self.x, self.y))
        rv1 = multivariate_normal(self.mean, self.covariance)
        rv2 = multivariate_normal([-10, -15], [[20, 0], [0, 5]])
        self.z = -rv1.pdf(pos) - rv2.pdf(pos)
        


    def SetParameters(self, meanX: float, meanY: float, covariance: npt.ArrayLike):
        self.mean = [meanX, meanY]
        self.covariance = covariance

    def GetConcentration(self, x: float, y: float) -> float:
        concentration = 2.0 - multivariate_normal.pdf([x,y], self.mean, self.covariance) - multivariate_normal.pdf([x, y], [-10, -15], [[20, 0], [0, 5]])
        return concentration
