# PFEIFER - MAPD
import os
import subprocess
import urllib.request
from openpilot.common.realtime import Ratekeeper
import stat

VERSION = 'v1.1.0'
URL = f"https://github.com/pfeiferj/openpilot-mapd/releases/download/{VERSION}/mapd"
MAPD_PATH = '/data/openpilot/mapd'
VERSION_PATH = '/data/openpilot/mapd_version'

def download():
  with urllib.request.urlopen(URL) as f:
    with open(MAPD_PATH, 'wb') as output:
      output.write(f.read())
      os.fsync(output)
      os.chmod(MAPD_PATH, stat.S_IEXEC)
    with open(VERSION_PATH, 'w') as output:
      output.write(VERSION)
      os.fsync(output)

def mapd_thread(sm=None, pm=None):
  rk = Ratekeeper(0.05, print_delay_threshold=None)

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

      process = subprocess.Popen('/data/openpilot/mapd', stdout=subprocess.PIPE)
      process.wait()
    except Exception as e:
      print(e)

    rk.keep_time()


def main(sm=None, pm=None):
  mapd_thread(sm, pm)

if __name__ == "__main__":
  main()
