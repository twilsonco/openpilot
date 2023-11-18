# PFEIFER - MAPD
import glob
import os
import shutil
import subprocess
import urllib.request

from openpilot.common.realtime import Ratekeeper
import stat

VERSION = 'v1.5.6'
URL = f"https://github.com/pfeiferj/openpilot-mapd/releases/download/{VERSION}/mapd"
COMMON_DIR = '/data/media/0/osm'
MAPD_PATH = os.path.join(COMMON_DIR, 'mapd')
VERSION_PATH = os.path.join(COMMON_DIR, 'mapd_version')


def download():
  if not os.path.exists(COMMON_DIR):
    os.makedirs(COMMON_DIR)
  with urllib.request.urlopen(URL) as f:
    with open(MAPD_PATH, 'wb') as output:
      output.write(f.read())
      os.fsync(output)
      current_permissions = stat.S_IMODE(os.lstat(MAPD_PATH).st_mode)
      os.chmod(MAPD_PATH, current_permissions | stat.S_IEXEC)
    with open(VERSION_PATH, 'w') as output:
      output.write(VERSION)
      os.fsync(output)


def cleanup_OSM_data():
  paths = [
    "/data/media/0/osm/db",
    "/data/media/0/osm/v*"
  ]
  for path in paths:
    files = glob.glob(path + '/**', recursive=True)
    for file in files:
      try:
        shutil.rmtree(file, ignore_errors=True)
      except NotADirectoryError:
        os.remove(file)
  for path in paths:
    shutil.rmtree(path, ignore_errors=True)

  if not os.path.isfile(VERSION_PATH):
    shutil.rmtree(VERSION_PATH, ignore_errors=True)

  if not os.path.isfile(MAPD_PATH):
    shutil.rmtree(MAPD_PATH, ignore_errors=True)


def mapd_thread():
  rk = Ratekeeper(0.05, print_delay_threshold=None)
  cleanup_OSM_data()

  while True:
    try:
      if not os.path.exists(MAPD_PATH):
        download()
        continue
      if not os.path.exists(VERSION_PATH):
        download()
        continue
      with open(VERSION_PATH) as f:
        content = f.read()
        if content != VERSION:
          download()
          continue

      process = subprocess.Popen(MAPD_PATH, stdout=subprocess.PIPE)
      process.wait()
    except Exception as e:
      print(e)

    rk.keep_time()


def main():
  mapd_thread()


if __name__ == "__main__":
  main()
