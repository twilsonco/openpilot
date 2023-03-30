from io import TextIOWrapper
import json
from typing import Any
from time import sleep, time
from os import remove, mkdir
from os.path import getmtime, exists

LOCK_TIMEOUT = 0.1 # seconds. Time before forceably removing lock file

def local_persist(path):
  try: mkdir(path)
  except: pass
  return path + "/"

MEM_PATH = "/dev/shm/"
MEM_PREFIX = "_c_mem_"
PERSIST_PATH = "/data/params/" if exists("/data/params/") else local_persist(".params")

persist_loaded = {}

watchers = {}
prev_values = {}
cache = {}



def check_lock(path: str):
  try:
    now = time()
    mtime = getmtime(path)
    if now - mtime > LOCK_TIMEOUT:
      remove(path)
  except: pass

def lock(path: str) -> TextIOWrapper:
  mem_lock = None
  while mem_lock is None:
    try:
      mem_lock = open(path, "x")
    except:
      check_lock(path)
      sleep(0.001)
  return mem_lock

def unlock(path: str, file: TextIOWrapper):
  file.close()
  try:
    remove(path)
  except: pass


def watch(service_name: str, mem_name: str, callback):
  """
  Register a callback to watch a mem value
  """
  if mem_name not in watchers:
      watchers[mem_name] = {}
  watchers[mem_name][service_name] = callback

def check(clear_cache = False):
  """
  Checks all values that have a watcher and calls the watchers if the value changes.
  """
  if clear_cache:
    cache = {}
  for mem_name in watchers:
    check_single(mem_name)

def check_single(name):
  if name in watchers:
    default = None

    if name in prev_values:
      default = prev_values[name]

    val = get(name, default, True)
    prev_values[name] = val

    if val != default:
      for cb in watchers[name].values():
        cb(val)



def get(name: str, default = None, persistant = False) -> Any:
  path_tail = MEM_PREFIX + name
  path = MEM_PATH + path_tail
  persist_path = PERSIST_PATH + path_tail
  lock_path = MEM_PATH + path_tail + ".lock"
  mem_lock = None
  if name in cache:
    return cache[name]

  try:
    mem_lock = lock(lock_path)
    data = default

    if persistant and name not in persist_loaded and not exists(persist_path):
      persist_loaded[name] = True

    if persistant and name not in persist_loaded:
      persist_f = open(persist_path, "r+")
      data = json.loads(persist_f.read())
      persist_f.close()
      mem_f = open(path, "w+")
      mem_f.write(json.dumps(data))
      mem_f.close()
      persist_loaded[name] = True
    else:
      mem_f = open(path, "r+")
      data = json.loads(mem_f.read())
      mem_f.close()
    unlock(lock_path, mem_lock)
    return data[0]
  except:
    if mem_lock is not None:
      unlock(lock_path, mem_lock)
    return default


def update(name: str, data: Any, persistant = False):
  path_tail = MEM_PREFIX + name
  path = MEM_PATH + path_tail
  persist_path = PERSIST_PATH + path_tail
  lock_path = MEM_PATH + path_tail + ".lock"
  mem_lock = None
  cache[name] = data
  try:
    mem_lock = lock(lock_path)
    if persistant:
      persist_f = open(persist_path, "w+")
      persist_f.write(json.dumps([data]))
      persist_f.close()
    mem_f = open(path, "w+")
    mem_f.write(json.dumps([data]))
    mem_f.close()
    unlock(lock_path, mem_lock)
  except:
    if mem_lock is not None:
      unlock(lock_path, mem_lock)
  check_single(name)

