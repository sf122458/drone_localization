import numpy as np

arr = np.ones((3,1))
arr = arr.squeeze()
print(arr.shape)

from pycolmap import Rigid3d

# rigid = Rigid3d()

# print(rigid.rotation.quat[0])