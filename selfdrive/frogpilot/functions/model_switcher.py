import hashlib
import os
import shutil
from openpilot.common.basedir import BASEDIR
from openpilot.system.hardware import HARDWARE

DESTINATION_PATH = os.path.join(BASEDIR, "selfdrive/modeld/models")
MODELS_SOURCE = os.path.join(DESTINATION_PATH, "models")
THNEED_FILE = os.path.join(DESTINATION_PATH, "supercombo.thneed")

MODEL_NAMES = {
  0: "los-angeles",
  1: "certified-herbalist",
}

def set_model_list(params):
  """Create a string of all the model names for future comparisons."""
  # Retrieve the previous model list
  previous_model_list = params.get("ModelList", encoding='utf-8')

  # Create a new model list
  model_list = "".join(MODEL_NAMES.values())

  if previous_model_list != model_list:
    # Reset the selected model if the model list changed
    params.put_int("Model", 0)
    params.put("ModelList", model_list)

def onnx_already_set(path1, path2):
  """Check if the two files are identical by comparing their SHA-256 hashes."""
  with open(path1, 'rb') as f1, open(path2, 'rb') as f2:
    return hashlib.sha256(f1.read()).hexdigest() == hashlib.sha256(f2.read()).hexdigest()

def set_model(params):
  # Set the model list
  set_model_list(params)

  # Get the corresponding supercombo variant name
  selected_model = params.get_int("Model")
  variant = MODEL_NAMES.get(selected_model)

  # Copy the selected model to the models folder
  onnx_path = os.path.join(MODELS_SOURCE, f"{variant}.onnx")
  destination = os.path.join(DESTINATION_PATH, "supercombo.onnx")

  if not onnx_already_set(onnx_path, destination):
    # Remove the calibration from the old model
    params.remove("CalibrationParams")
    params.remove("LiveTorqueParameters")

    # Delete the thneed file
    if os.path.exists(THNEED_FILE):
      os.remove(THNEED_FILE)

    # Copy over the onnx file
    shutil.copy(onnx_path, destination)

    # Reboot
    HARDWARE.reboot()
