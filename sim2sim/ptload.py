"""Read the deploy folder's torch files without torch.

`torch.save` and TorchScript files are zip archives: `data.pkl` is a pickle whose tensors point at raw
little-endian storages in `data/<key>`. This module unpickles them with numpy arrays in place of tensors,
which is all the chain needs (the lookup tables are plain tensors, the policy is three Linear layers).
"""

import pickle
import zipfile

import numpy as np

_DTYPES = {
    "FloatStorage": np.float32, "DoubleStorage": np.float64, "HalfStorage": np.float16,
    "LongStorage": np.int64, "IntStorage": np.int32, "ShortStorage": np.int16,
    "CharStorage": np.int8, "ByteStorage": np.uint8, "BoolStorage": np.bool_,
}


class _StorageType:
    def __init__(self, name):
        self.dtype = _DTYPES[name]


class _Obj:
    """Stand-in for a TorchScript module or any other class: keeps its pickled state."""

    def __init__(self, *args, **kwargs):
        self.args = args

    def __setstate__(self, state):
        self.state = state


def _rebuild_tensor(storage, offset, size, stride, *_):
    item = storage.dtype.itemsize
    return np.lib.stride_tricks.as_strided(
        storage[offset:], shape=tuple(size), strides=tuple(s * item for s in stride)).copy()


def _rebuild_parameter(data, *_):
    return data


class _Unpickler(pickle.Unpickler):
    def __init__(self, file, archive, prefix):
        super().__init__(file)
        self.archive, self.prefix = archive, prefix

    def find_class(self, module, name):
        if module == "torch._utils" and name == "_rebuild_tensor_v2":
            return _rebuild_tensor
        if module == "torch._utils" and name == "_rebuild_parameter":
            return _rebuild_parameter
        if module == "torch" and name in _DTYPES:
            return _StorageType(name)
        if module == "collections" and name == "OrderedDict":
            import collections
            return collections.OrderedDict
        return _Obj

    def persistent_load(self, pid):
        kind, storage_type, key, _location, numel = pid[:5]
        assert kind == "storage", pid
        raw = self.archive.read(f"{self.prefix}/data/{key}")
        return np.frombuffer(raw, dtype=storage_type.dtype, count=numel)


def load(path):
    """Return the unpickled object of a torch zip file, tensors as numpy arrays."""
    with zipfile.ZipFile(path) as archive:
        pkl = next(n for n in archive.namelist() if n.endswith("/data.pkl"))
        prefix = pkl[: -len("/data.pkl")]
        with archive.open(pkl) as f:
            return _Unpickler(f, archive, prefix).load()


def load_actor(path):
    """The grasp_lift TorchScript actor as numpy: mean, std and the three (weight, bias) layers."""
    root = load(path).state
    mlp = root["mlp"].state
    layers = [(mlp[k].state["weight"].astype(np.float64), mlp[k].state["bias"].astype(np.float64))
              for k in ("0", "2", "4")]
    return {"mean": root["mean"].astype(np.float64).reshape(-1), "std": root["std"].astype(np.float64).reshape(-1),
            "layers": layers}
