import http.client
import os
import socket
import time
import urllib.error
import urllib.request

from openpilot.common.params import Params
from openpilot.system.version import get_build_metadata

VERSION = 'v3' if get_build_metadata().channel == "FrogPilot" else 'v4'

GITHUB_REPOSITORY_URL = 'https://raw.githubusercontent.com/FrogAi/FrogPilot-Resources/'
GITLAB_REPOSITORY_URL = 'https://gitlab.com/FrogAi/FrogPilot-Resources/-/raw/'

DEFAULT_MODEL = "north-dakota-v2"
DEFAULT_MODEL_NAME = "North Dakota V2 (Default)"
MODELS_PATH = '/data/models'

NAVIGATION_MODELS = {"certified-herbalist", "duck-amigo", "los-angeles", "recertified-herbalist"}
RADARLESS_MODELS = {"radical-turtle"}

params = Params()
params_memory = Params("/dev/shm/params")

def ping_url(url, timeout=5):
  try:
    urllib.request.urlopen(url, timeout=timeout)
    return True
  except (urllib.error.URLError, socket.timeout, http.client.RemoteDisconnected):
    return False

def determine_url(model, file_type):
  if ping_url(GITHUB_REPOSITORY_URL):
    return f"{GITHUB_REPOSITORY_URL}/Models/{model}{file_type}"
  else:
    return f"{GITLAB_REPOSITORY_URL}/Models/{model}{file_type}"

def delete_deprecated_models():
  populate_models()

  model_name = params.get("ModelName", encoding='utf-8')
  if model_name and "(Default)" in model_name and model_name != DEFAULT_MODEL_NAME:
    params.put("ModelName", model_name.replace(" (Default)", ""))

  current_model = params.get("Model", encoding='utf-8')
  current_model_path = os.path.join(MODELS_PATH, f"{current_model}.thneed")
  if not os.path.exists(current_model_path):
    params_memory.put("ModelToDownload", current_model)

  available_models = params.get("AvailableModels", encoding='utf-8').split(',')
  if current_model not in available_models or not os.path.exists(current_model_path):
    params.put("Model", DEFAULT_MODEL)
    params.put("ModelName", DEFAULT_MODEL_NAME)

  for model_file in os.listdir(MODELS_PATH):
    if model_file.endswith('.thneed') and model_file[:-7] not in available_models:
      os.remove(os.path.join(MODELS_PATH, model_file))

def download_model():
  model = params_memory.get("ModelToDownload", encoding='utf-8')
  model_path = os.path.join(MODELS_PATH, f"{model}.thneed")

  if os.path.exists(model_path):
    print(f"Model {model} already exists, skipping download.")
    params_memory.remove("ModelToDownload")
    return

  url = determine_url(model, '.thneed')

  for attempt in range(3):
    try:
      total_size = get_total_size(url)
      download_file(url, model_path, total_size)
      verify_download(model, model_path)
      return

    except Exception as e:
      handle_download_error(model_path, attempt, e)
      time.sleep(2**attempt)

def get_total_size(url):
  try:
    return int(urllib.request.urlopen(url).getheader('Content-Length'))
  except Exception as e:
    print(f"Failed to get total size. Error: {e}")
    raise

def download_file(url, path, total_size):
  try:
    with urllib.request.urlopen(url) as response:
      with open(path, 'wb') as output:
        for chunk in iter(lambda: response.read(8192), b''):
          output.write(chunk)
          progress = output.tell()
          params_memory.put_int("ModelDownloadProgress", int((progress / total_size) * 100))
        os.fsync(output)

  except Exception as e:
    print(f"Error downloading file: {e}")
    raise

def verify_download(model, model_path):
  expected_size = os.path.getsize(model_path)
  actual_size = os.path.getsize(model_path)

  if expected_size == actual_size:
    print(f"Successfully downloaded the {model} model!")
  else:
    raise Exception("Downloaded file size does not match expected size.")

def handle_download_error(model_path, attempt, exception):
  print(f"Attempt {attempt + 1} failed with error: {exception}. Retrying...")

  if os.path.exists(model_path):
    os.remove(model_path)

  if attempt == 2:
    print(f"Failed to download the model after 3 attempts.")

def populate_models():
  url = f"{GITHUB_REPOSITORY_URL}Versions/model_names_{VERSION}.txt" if ping_url(GITHUB_REPOSITORY_URL) else f"{GITLAB_REPOSITORY_URL}Versions/model_names_{VERSION}.txt"
  try:
    with urllib.request.urlopen(url) as response:
      model_info = [line.decode('utf-8').strip().split(' - ') for line in response.readlines()]
    update_params(model_info)
  except Exception as e:
    print(f"Failed to update models list. Error: {e}")

def update_params(model_info):
  params.put("AvailableModels", ','.join(model[0] for model in model_info))
  params.put("AvailableModelsNames", ','.join(model[1] for model in model_info))
  print("Models list updated successfully.")
