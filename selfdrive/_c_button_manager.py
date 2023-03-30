# PFEIFER - BM

from enum import IntEnum
from time import time
from typing import Optional
from selfdrive._c_mem import get, update, watch
from uuid import uuid4

PRESS_INTERVAL = 0.300 # s. The time it takes between presses to be counted as a separate press vs double/triple press
DEBOUNCE_TIMEOUT = 0.020 # s. The minimum amount of time before rechecking button state
LONG_PRESS_LENGTH = 0.5 # s. The amount of time to wait before a press is considered a long press

PROCESS_ID = str(uuid4())


class ButtonState(IntEnum):
  NONE = 1
  PENDING_LONG = 2
  PENDING_LONG_RELEASED = 3
  PENDING_DOUBLE = 4
  PENDING_DOUBLE_RELEASED = 5
  PENDING_TRIPLE = 6
  PENDING_TRIPLE_RELEASED = 7
  SINGLE_PRESS = 8
  DOUBLE_PRESS = 9
  TRIPLE_PRESS = 10
  LONG_PRESS = 11
  ERROR = 12

def stateTimeout(state: ButtonState):
  if state == ButtonState.PENDING_DOUBLE:
    return ButtonState.SINGLE_PRESS
  if state == ButtonState.PENDING_TRIPLE:
    return ButtonState.DOUBLE_PRESS

def nextState(state : ButtonState):
  if state == ButtonState.PENDING_DOUBLE:
    return ButtonState.PENDING_TRIPLE
  if state == ButtonState.PENDING_TRIPLE:
    return ButtonState.TRIPLE_PRESS

ACTIVE_STATES = [ButtonState.PENDING_LONG_RELEASED, ButtonState.PENDING_DOUBLE, ButtonState.PENDING_TRIPLE, ButtonState.PENDING_LONG, ButtonState.PENDING_TRIPLE_RELEASED, ButtonState.PENDING_DOUBLE_RELEASED]

PENDING_RELEASE_STATES = [ButtonState.PENDING_LONG_RELEASED, ButtonState.PENDING_DOUBLE_RELEASED, ButtonState.PENDING_TRIPLE_RELEASED]

class ButtonStatus:
  def __init__(self, id: int, state: ButtonState, timestamp: Optional[float] = None):

    self.state: ButtonState = state
    if timestamp is None:
      self.timestamp: float = time()
    else:
      self.timestamp: float = timestamp
    self.id = id

  def raw(self):
    return {"state": self.state, "timestamp": self.timestamp, "id": self.id}

class ButtonManager:
  def __init__(self):
    self.registered_callbacks = {}

  def getObserver(self, service_name: str, button: str):
    return ButtonObserver(self, service_name, button)

  def registerCallback(self, service_name: str, button: str, state: ButtonState, callback):
    if button not in self.registered_callbacks:
      self.registered_callbacks[button] = { "observer": self.getObserver(PROCESS_ID, button), "callbacks": {} }

    if state not in self.registered_callbacks[button]["callbacks"]:
      self.registered_callbacks[button]["callbacks"][state] = {}

    self.registered_callbacks[button]["callbacks"][state][service_name] = callback

    watch(PROCESS_ID, self.mem_name(button), lambda _ : self.handle_callbacks())

  def mem_name(self, button: str):
    return f'button_{button}'

  def get(self, button: str) -> ButtonStatus:
    b = get(self.mem_name(button))
    if b is None:
      return ButtonStatus(0, ButtonState.NONE)

    return ButtonStatus(b['id'], b['state'],b['timestamp'])

  def set(self, button: str, state: ButtonState):
    b = self.get(button)
    id = b.id
    if state == ButtonState.PENDING_LONG: # we have started a new loop for the button machine if the state is PENDING_LONG
      id += 1
    b = ButtonStatus(id, state)
    update(self.mem_name(button), b.raw())

    self.handle_callbacks()

  def handle_callbacks(self):
    for button in self.registered_callbacks:
      status: ButtonStatus = self.registered_callbacks[button]["observer"].simple_status()
      if status is not None and status.state in self.registered_callbacks[button]["callbacks"]:
        callbacks = self.registered_callbacks[button]["callbacks"][status.state]
        for callback in callbacks.values():
          callback()

  def update(self, button: str, active: bool):
    now = time()


    b = self.get(button)

    # If button is new store for future
    if b.state == ButtonState.NONE and b.timestamp > now:
      self.set(button, b.state)

    # calculate time since last state change
    time_diff = now - b.timestamp

    ##### Button State Machine #####

    # debounce button presses
    if time_diff < DEBOUNCE_TIMEOUT:
      return

    if b.state not in ACTIVE_STATES:
      # The only possible update to state if in inactive state is to pending long press
      if active:
        self.set(button, ButtonState.PENDING_LONG)
    elif active: # In an active state with active button press
      if b.state in PENDING_RELEASE_STATES:
        return # Waiting for button to be released, don't update state
      if b.state == ButtonState.PENDING_LONG:
        if time_diff > LONG_PRESS_LENGTH: # only update if active was held longer than long press duration
          self.set(button, ButtonState.PENDING_LONG_RELEASED)
      elif b.state == ButtonState.PENDING_DOUBLE:
        self.set(button, ButtonState.PENDING_DOUBLE_RELEASED)
      elif b.state == ButtonState.PENDING_TRIPLE:
        self.set(button, ButtonState.PENDING_TRIPLE_RELEASED)
      else: # Unknown state transition
        self.set(button, ButtonState.ERROR)
    elif b.state == ButtonState.PENDING_LONG: # handle a single press release
      self.set(button, ButtonState.PENDING_DOUBLE) # needs to wait to see if this is a double press
    elif b.state in PENDING_RELEASE_STATES: # handle the rest of the release cases
      if b.state == ButtonState.PENDING_LONG_RELEASED: # Can only transition to a final state
        self.set(button, ButtonState.LONG_PRESS)
      elif b.state == ButtonState.PENDING_DOUBLE_RELEASED: # needs to wait to see if this is a triple press
        self.set(button, ButtonState.PENDING_TRIPLE)
      elif b.state == ButtonState.PENDING_TRIPLE_RELEASED: # can only transition to a final state
        self.set(button, ButtonState.TRIPLE_PRESS)
      else: # Unknown state transition
        self.set(button, ButtonState.ERROR)
    elif time_diff > PRESS_INTERVAL: # In an active state with inactive button press after press interval
      if b.state == ButtonState.PENDING_DOUBLE: # Waiting for a double press timed out, must be single press
        self.set(button, ButtonState.SINGLE_PRESS)
      elif b.state == ButtonState.PENDING_TRIPLE: # Waiting for a triple press timed out, must be single press
        self.set(button, ButtonState.DOUBLE_PRESS)
      else: # Unknown state transition
        self.set(button, ButtonState.ERROR)

# States that can be returned by the simple_status
SIMPLE_STATES = [ButtonState.SINGLE_PRESS, ButtonState.DOUBLE_PRESS, ButtonState.TRIPLE_PRESS, ButtonState.LONG_PRESS]

# states that should activate immediately but the state machine still needs to track the release of
IMMEDIATE_STATES = [ButtonState.PENDING_LONG_RELEASED, ButtonState.PENDING_TRIPLE_RELEASED]

class ButtonObserver:
  def __init__(self, manager: ButtonManager, name: str, button: str):
    self.manager: ButtonManager = manager
    self.button: str = button
    self.name = name
    self.last_observed_status = ButtonStatus(0, ButtonState.NONE)

  @property
  def mem_name(self) -> str:
    return f'button_observer_{self.name}_{self.button}'

  def simple_status(self) -> Optional[ButtonStatus]:
    """
    Returns a simple status of the button. This is a single press, double press, triple press, or long press.
    If the status has already been consumed from this observer it returns None
    """
    b = self.status

    # Return None if the status hasn't changed since last observed
    if b.timestamp <= self.last_observed_status.timestamp:
      return None

    # Return None if it is not a simple state and cannot be simplified
    if b.state not in SIMPLE_STATES and b.state not in IMMEDIATE_STATES:
      return None

    last_status = self.last_observed_status
    # Store the current status as last observed as we are now guaranteed to use the status
    self.last_observed_status = b

    # simplify long button press
    if b.state == ButtonState.PENDING_LONG_RELEASED:
      # Convert the PENDING_LONG_RELEASED to a LONG_PRESS but keep the timestamp
      status = ButtonStatus(b.id, ButtonState.LONG_PRESS)
      status.timestamp = b.timestamp
      return status

    # This is the "same" status
    if b.state == ButtonState.LONG_PRESS and b.id == last_status.id:
      return None

    # simplify triple button press
    if b.state == ButtonState.PENDING_TRIPLE_RELEASED:
      # Convert the PENDING_TRIPLE_RELEASED to a TRIPLE_PRESS but keep the timestamp
      status = ButtonStatus(b.id, ButtonState.TRIPLE_PRESS)
      status.timestamp = b.timestamp
      return status

    # This is the "same" status
    if b.state == ButtonState.TRIPLE_PRESS and b.id == last_status.id:
      return None

    # Return the current button status as it's (mostly) guaranteed to be the latest unobserved status
    return b


  @property
  def status(self) -> ButtonStatus:
    """
    Returns the current status of the button.
    """
    return self.manager.get(self.button)

bm = ButtonManager()
