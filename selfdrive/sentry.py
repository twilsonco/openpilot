"""Install exception handler for process crash."""
import os
import sentry_sdk
import traceback
from enum import Enum
from sentry_sdk.integrations.threading import ThreadingIntegration

from cereal import car
from openpilot.common.params import Params
from datetime import datetime
from openpilot.selfdrive.athena.registration import is_registered_device
from openpilot.system.hardware import HARDWARE, PC
from openpilot.system.swaglog import cloudlog
from openpilot.system.version import get_branch, get_commit, get_origin, get_version, \
                              is_comma_remote, is_dirty, is_tested_branch


class SentryProject(Enum):
  # python project
  SELFDRIVE = "https://5ad1714d27324c74a30f9c538bff3b8d@o4505034923769856.ingest.sentry.io/4505034930651136"
  # native project
  SELFDRIVE_NATIVE = "https://5ad1714d27324c74a30f9c538bff3b8d@o4505034923769856.ingest.sentry.io/4505034930651136"

CRASHES_DIR = '/data/community/crashes/'
ret = car.CarParams.new_message()
candidate = ret.carFingerprint
params = Params()
dongle_id = params.get("DongleId")
installed = params.get("InstallDate")
updated = params.get("Updated")
error_tags = {'dongle_id': dongle_id, 'branch': get_branch(), 'fingerprintedAs': candidate, 'updated': updated}


def report_tombstone(fn: str, message: str, contents: str) -> None:
  cloudlog.error({'tombstone': message})

  with sentry_sdk.configure_scope() as scope:
    scope.set_extra("tombstone_fn", fn)
    scope.set_extra("tombstone", contents)
    sentry_sdk.capture_message(message=message)
    sentry_sdk.flush()


def capture_exception(*args, **kwargs) -> None:
  save_exception(traceback.format_exc())
  cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  try:
    sentry_sdk.capture_exception(*args, **kwargs)
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")

def save_exception(exc_text):
  os.makedirs(CRASHES_DIR, exist_ok=True)

  files = [os.path.join(CRASHES_DIR, datetime.now().strftime('%Y-%m-%d--%H-%M-%S.log')),
           os.path.join(CRASHES_DIR, 'error.txt')]

  for file in files:
    with open(file, 'w') as f:
      f.write(exc_text)

  print(f'Logged current crash to {files}')

def bind_user(**kwargs) -> None:
  sentry_sdk.set_user(kwargs)
  sentry_sdk.flush()

def capture_warning(warning_string):
  with sentry_sdk.configure_scope() as scope:
    scope.fingerprint = [warning_string, dongle_id]
  bind_user(id=dongle_id)
  sentry_sdk.capture_message(warning_string, level='warning')
  sentry_sdk.flush()

def set_tag(key: str, value: str) -> None:
  sentry_sdk.set_tag(key, value)
  sentry_sdk.flush()


def init(project: SentryProject) -> None:
  # forks like to mess with this, so double check
  comma_remote = is_comma_remote() and "FrogAi" in get_origin(default="")
  if not comma_remote or not is_registered_device() or PC:
    return

  env = "release" if is_tested_branch() else "master"

  integrations = []
  if project == SentryProject.SELFDRIVE:
    integrations.append(ThreadingIntegration(propagate_hub=True))
  else:
    sentry_sdk.utils.MAX_STRING_LENGTH = 8192

  sentry_sdk.init(project.value,
                  default_integrations=False,
                  release=get_version(),
                  integrations=integrations,
                  traces_sample_rate=1.0,
                  environment=env)

  sentry_sdk.set_user({"id": dongle_id})
  sentry_sdk.set_tag("branch", get_branch())
  sentry_sdk.set_tag("commit", get_commit())
  sentry_sdk.set_tag("updated", updated)
  sentry_sdk.set_tag("installed", installed)

  if project == SentryProject.SELFDRIVE:
    sentry_sdk.Hub.current.start_session()
