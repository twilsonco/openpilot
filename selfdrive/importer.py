from importlib import import_module
import pathlib
from os import listdir
from typing import Any, Dict
from common.params import Params, put_nonblocking, put_bool_nonblocking

SELF_DRIVE_PATH = pathlib.Path(__file__).parent.resolve()

class Modules:
  def __init__(self):
    self.modules: Dict[str, Any] = {}
    paths = listdir(SELF_DRIVE_PATH)
    c_module_paths = [path for path in paths if path.startswith('_c_') and path.endswith(".py") and path != "__init__.py"]
    c_modules = [name.replace(".py", "") for name in c_module_paths]
    for module in c_modules:
      full_module_name = "selfdrive." + module
      self.modules[module[3:]] = import_module(full_module_name)

    # Some additional common imports that may not already be in various files
    self.modules["params"] = Params()
    self.modules["put_bool_nonblocking"] = put_bool_nonblocking
    self.modules["put_nonblocking"] = put_nonblocking

m = Modules().modules
