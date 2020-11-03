import numpy as np
from typing import Union, Sequence, Any


class CatSlice(np.ndarray):
    def __new__(
        cls,
        start: int = 0,
        stop: Union[int, None] = None,
        step: int = 1,
        input_array: Sequence = [Any],
    ):
        if stop is not None:
            obj = np.arange(start=start, stop=stop, step=step).view(cls)
        else:
            obj = np.asarray(input_array).view(cls)
        return obj

    def __array_finalize__(self, obj):
        if obj is None:
            return

    def __add__(self, s):
        result = CatSlice(input_array=np.unique(np.concatenate((self, s))))
        return result

    def __mul__(self, s):
        return np.ix_(self, s)

    def __pow__(self, value):
        assert value > 0, f"CatSlice.__pow__: pow value must positive integer: {value}"
        prod_list = [self.copy() for i in range(value)]
        return np.ix_(*prod_list)

eta = np.array((1,2,3,4,5,6,7,8,9,45,4,3,2))
m = eta[3:].reshape((-1, 2)).T
x = np.array([[1],[2],[3]])
vec = np.array([[1],[2]])

delta_m = m - (x[:2,None])
#print(m)
print(delta_m)
