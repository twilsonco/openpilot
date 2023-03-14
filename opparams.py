#!/usr/bin/env python3
import time, sys
from common.op_params import opParams, read_history
from common.params import Params
import ast
import difflib
from common.colors import COLORS
from math import log10
from textwrap import wrap

class opEdit:  # use by running `python /data/openpilot/op_edit.py`
  def __init__(self, restart=False, restart_immediate=False):
    self.sleep_time = 0.5
    if restart or restart_immediate:
      self.C3_rebootless_restart(restart_immediate)
      return
    self.op_params = opParams(calling_function="opparams editor")
    self.params = None
    self.live_tuning_enabled = self.op_params.get('op_params_live_tune_enabled', force_update=True)
    self.live_tuning = self.live_tuning_enabled and self.op_params.get('op_edit_live_mode', force_update=True)
    if not self.live_tuning_enabled:
      self.op_params.put('op_edit_live_mode', self.live_tuning, reason=False)
    self.compact_view = self.op_params.get('op_edit_compact_view', force_update=True)
    self.section = self.op_params.get('op_edit_section', force_update=True)
    self.username = self.op_params.get('MISC_username', force_update=True)
    self.type_colors = {int: COLORS.BASE(179), float: COLORS.BASE(179),
                        bool: {False: COLORS.RED, True: COLORS.OKGREEN},
                        type(None): COLORS.BASE(177),
                        str: COLORS.BASE(77)}

    self.last_choice = None

    self.run_init()

  def run_init(self):
    if self.username is None:
      self.success('\nWelcome to the {}opParams{} command line editor!'.format(COLORS.CYAN, COLORS.SUCCESS), sleep_time=0)
      self.prompt('Would you like to add your Discord username for easier crash debugging for the fork owner?')
      self.prompt('Your username is only used for reaching out if a crash occurs.')

      username_choice = self.input_with_options(['Y', 'N', 'don\'t ask again'], default=2)[0]
      if username_choice == 0:
        self.prompt('Enter a unique identifer/Discord username:')
        username = ''
        while username == '':
          username = input('>> ').strip()
        self.op_params.put('MISC_username', username, show_alert=True)
        self.username = username
        self.success('Thanks! Saved your username\n'
                     'Edit the \'username\' parameter at any time to update', sleep_time=1.5)
      elif username_choice == 2:
        self.op_params.put('MISC_username', False)
        self.info('Got it, bringing you into opEdit\n'
                  'Edit the \'username\' parameter at any time to update', sleep_time=1.0)
    else:
      self.success('\nWelcome to the {}opParams{} command line editor, {}!'.format(COLORS.CYAN, COLORS.SUCCESS, self.username if self.username else "you lucky user, you"), sleep_time=0)
    self.first_run = True
    self.run_loop()

  def run_loop(self):
    while True:
      if self.section != '':
        self.info('Here are all the {}parameters for\n{} ({}):'.format('LIVE ' if self.live_tuning else '', self.op_params._param_sections[self.section], self.section), sleep_time=0)
      elif not self.live_tuning:
        self.info('Here are all the parameters and parameter groups:', sleep_time=0)
      else:
        if not self.live_tuning_enabled:
          self.warning('Cannot show live parameters because the')
          self.warning('"Customization over SSH" toggle is disabled')
          self.warning('in Openpilot settings. Activate that toggle')
          self.warning('and restart opparams.py in order to live-tune.\n')
          time.sleep(5)
          self.live_tuning = False
          self.op_params.put('op_edit_live_mode', self.live_tuning, reason=False)
          continue
        self.info('Here are your live parameters:', sleep_time=0)
        self.info('(changes take effect within a second)', sleep_time=0)
      self.info('(startup (s) require car or OpenPilot restart)', sleep_time=0)
      self.info('(parameters have default (.) or changed (C) values)', sleep_time=0)
      if (self.section != '' or not self.live_tuning) \
          and not self.live_tuning_enabled:
        if self.first_run:
          self.first_run = False
          self.warning('Not showing live parameters because the')
          self.warning('"Customization over SSH" toggle is disabled')
          self.warning('in Openpilot settings. Activate that toggle')
          self.warning('and restart opparams.py in order to live-tune.\n')
        else:
          self.warning('Enable live-tuning in OpenPilot settings\n')
      else:
        print("\n")
      self.params = self.op_params.get(force_update=True)
      if self.section != '':
        self.params = {k: v for k, v in self.params.items() if k.startswith(self.section)}
      elif self.compact_view:
        self.params = {k: v for k, v in self.params.items() if self.op_params.fork_params[k].is_common or not any([k.startswith(kk) for kk in self.op_params._param_sections.keys()])}
      if self.live_tuning:  # only display live tunable params
        self.params = {k: v for k, v in self.params.items() if self.op_params.fork_params[k].live}
      
      tmp_removed = set()
      for k,v in self.params.items():
        if (self.section or self.compact_view) \
          and self.op_params.fork_params[k].show_op_param != '' \
          and self.op_params.fork_params[k].show_op_param_check_val is not None \
          and self.op_params.fork_params[k].show_op_param in self.op_params.fork_params \
          and self.op_params.get(self.op_params.fork_params[k].show_op_param, force_update=True) != self.op_params.fork_params[k].show_op_param_check_val:
            tmp_removed.add(k)
      for p in tmp_removed:
        del self.params[p]
      
      if len(self.params) == 0 and self.section != '':
        self.warning(f"No params to list!\n")
        self.section = ''
        self.op_params.put('op_edit_section', self.section, reason=False)
        self.last_choice = None
        self.live_tuning = False
        self.op_params.put('op_edit_live_mode', self.live_tuning, reason=False)
        continue
        
      
      sections = []
      if len(self.op_params._param_sections) > 0:
        for i,v in self.op_params._param_sections.items():
          for p in self.op_params.fork_params:
            if (self.section == '' and p.startswith(i)) or (self.section != '' and p in self.params and i in p and p.startswith(self.section)):
              sections.append((i,v))
              break
            
        # n_pad = max([len(i[0]) for i in sections])
        # self.info('Abbreviations:\n' + '\n'.join([f'  {abb[0].ljust(n_pad)} : {abb[1]}' for abb in sorted(sections, key=lambda x:x[0])]), end='\n', sleep_time=0)

      values_list = []
      for k, v in self.params.items():
        if len(str(v)) < 40:
          v = self.color_from_type(v)
        else:
          v = '{} ... {}'.format(str(v)[:30], str(v)[-15:])
        values_list.append(v)
      
      static = [COLORS.INFO + 's' + COLORS.ENDC if self.op_params.fork_params[k].static or self.op_params.fork_params[k].fake_live else 'l' for k in self.params]

      to_print = []
      if self.section == '':
        to_print.append(f'{COLORS.INFO}{COLORS.UNDERLINE}{"Common" if self.compact_view else "All"} parameters:{COLORS.ENDC}{COLORS.ENDC}')
      blue_gradient = [33, 39, 45, 51, 87]
      n_pad = max([len(i) for i in self.params])
      n_pad_idx = int(log10(len(self.params) + (0 if self.section != '' else len(sections))))+2
      n_pad_vals = max([len(str(i)) for i in values_list])+1
      for idx, param in enumerate(self.params):
        _color = blue_gradient[min(round(idx / len(self.params) * len(blue_gradient)), len(blue_gradient) - 1)]
        is_default = (self.op_params.get(param, force_update=True) == self.op_params.fork_params[param].default_value)
        is_default = f"{COLORS.BASE(_color)}.{COLORS.ENDC}" if is_default else f"{COLORS.INFO}C{COLORS.ENDC}"
        line = '{} {} {} {} {}'.format(f"{idx + 1}.".rjust(n_pad_idx), f"{param}".ljust(n_pad,'.'), static[idx], is_default, values_list[idx])
        if idx == self.last_choice and self.last_choice is not None and self.last_choice < len(self.params):
          line = COLORS.OKGREEN + line
        else:
          line = COLORS.BASE(_color) + line
        to_print.append(line)
      
      if len(sections) > 0:
        n_pad = max([len(i[0]) for i in sections])
        to_print.append(f'{COLORS.INFO}{COLORS.UNDERLINE}{"Groups" if not self.section else "Abbreviations"}:{COLORS.ENDC}{COLORS.ENDC}')
        blue_gradient = blue_gradient[::-1]
        for abb in sections:
          idx += 1
          _color = blue_gradient[min(round((idx - len(self.params)) / len(sections) * len(blue_gradient)), len(blue_gradient) - 1)]
          line = COLORS.BASE(_color) + '{} {}'.format(f"{idx + 1}.".rjust(n_pad_idx) if not self.section else "", f"{COLORS.BOLD}{abb[0].rjust(n_pad)} {abb[1]}{COLORS.ENDC}")
          to_print.append(line)

      extras = {'o': ('Restart openpilot (C3 only)', COLORS.FAIL),
                'l': ('Toggle live params', COLORS.WARNING)}
      if self.section == '':
        extras.update({'c': ('Switch to {} view'.format('full' if self.compact_view else 'compact'), COLORS.WARNING)})
      extras.update({'r': ('Reset {} params to default'.format("shown" if self.compact_view or self.section else "ALL"), COLORS.WARNING),
              'e': ('Exit opEdit', COLORS.PINK)})
      if self.section != '':
        extras.update({'h': ('Exit {} group and return to home'.format(self.section), COLORS.PINK)})

      to_print += ['---'] + ['{}. {}'.format(ext_col + e, ext_txt + COLORS.ENDC) for e, (ext_txt, ext_col) in extras.items()]
      print('\n'.join(to_print))
      self.prompt('\nChoose a parameter{} to edit\n(by index or name):'.format(' or group' if self.section == '' and len(sections) > 0 else ''))

      choice = input('>> ').strip().lower()
      parsed, choice = self.parse_choice(choice, len(to_print) - len(extras) + (len(sections) if self.section == '' else 0))
      if parsed == 'continue':
        continue
      elif parsed == 'change':
        if choice >= len(self.params):
          choice -= len(self.params)
          self.section = sections[choice][0]
          self.op_params.put('op_edit_section', self.section, reason=False)
          self.last_choice = None
        else:
          self.last_choice = choice
          self.change_parameter(choice)
      elif parsed == 'live':
        self.last_choice = None
        self.live_tuning = not self.live_tuning
        self.op_params.put('op_edit_live_mode', self.live_tuning, reason=False)  # for next opEdit startup
      elif parsed == 'compact':
        self.last_choice = None
        self.compact_view = not self.compact_view
        self.op_params.put('op_edit_compact_view', self.compact_view, reason=False)
      elif parsed == 'restart':
        self.C3_rebootless_restart()
      elif parsed == 'reset':
        self.last_choice = None
        if self.compact_view or self.section:
          for k in self.params:
            v = self.op_params.fork_params[k]
            if not v.hidden:
              self.op_params.put(k,v.default_value, reason="user resetting to default")
        else:
          for k, v in self.op_params.fork_params.items():
            if not v.hidden:
              self.op_params.put(k,v.default_value, reason="user resetting to default")
        continue
      elif parsed == 'home':
        self.section = ''
        self.op_params.put('op_edit_section', self.section, reason=False)
        self.last_choice = None
        continue
      elif parsed == 'exit':
        return

  def parse_choice(self, choice, opt_len):
    if choice.isdigit():
      choice = int(choice)
      choice -= 1
      if choice not in range(opt_len):  # number of options to choose from
        self.error('Not in range!')
        return 'continue', choice
      return 'change', choice

    if choice in ['l', 'live']:  # live tuning mode
      return 'live', choice
    elif choice in ['c', 'compact']:  # live tuning mode
      return 'compact', choice
    elif choice in ['exit', 'e', '']:
      self.error('Exiting opEdit', sleep_time=0)
      return 'exit', choice
    elif choice in ['home', 'h']:
      self.warning('Exiting {} group'.format(self.section))
      return 'home', choice
    elif choice in ['restart', 'o']:
      self.warning("This will restart OpenPilot. You need to assume control of the vehicle. Continue?")
      if self.input_with_options(['Y','N'], default=0)[0] == 0:
        return 'restart', choice
      else:
        return 'continue', choice
    elif choice in ['reset', 'r']:
      self.warning("Resetting params to default values! Continue?")
      if self.input_with_options(['Y','N'], default=0)[0] == 0:
        self.warning("Resetting params to default values!")
        return 'reset', choice
      else:
        return 'continue', choice
    else:  # find most similar param to user's input
      param_sims = [(idx, self.str_sim(choice, param.lower())) for idx, param in enumerate(self.params)]
      param_sims = [param for param in param_sims if param[1] > 0.33]
      if len(param_sims) > 0:
        chosen_param = sorted(param_sims, key=lambda param: param[1], reverse=True)[0]
        return 'change', chosen_param[0]  # return idx

    self.error('Invalid choice!')
    return 'continue', choice

  def str_sim(self, a, b):
    return difflib.SequenceMatcher(a=a, b=b).ratio()

  def change_parameter(self, choice):
    while True:
      chosen_key = list(self.params)[choice]
      param_info = self.op_params.fork_params[chosen_key]

      old_value = self.params[chosen_key]
      if param_info.static or param_info.fake_live:
        self.info2('Chosen parameter: {}{}\n(refresh on startup)'.format(chosen_key, COLORS.BASE(207)), sleep_time=0)
      elif param_info.live:
        self.info2('Chosen parameter: {}{} (live every 1s)'.format(chosen_key, COLORS.BASE(207)), sleep_time=0)
      else:
        self.info2('Chosen parameter: {}{} (live every 10s)'.format(chosen_key, COLORS.BASE(207)), sleep_time=0)

      to_print = []
      if param_info.has_description:
        to_print.append(COLORS.OKGREEN + '>>  {}'.format('\n    '.join(wrap(param_info.description, width=50, initial_indent=f'Description: '))) + COLORS.ENDC)
      if param_info.static or param_info.fake_live:
        to_print.append(COLORS.BLUE_GREEN + '>>  A car or openpilot restart is required\n    for changes to this parameter!' + COLORS.ENDC)
      elif not param_info.live:
        to_print.append(COLORS.WARNING + '>>  Changes take effect within 10 seconds for this parameter!' + COLORS.ENDC)
      elif param_info.live:
        to_print.append(COLORS.WARNING + '>>  Changes take effect within 1 second for this parameter!' + COLORS.ENDC)
      if param_info.has_allowed_types:
        to_print.append(COLORS.WARNING + '>>  Allowed types: {}{}'.format(COLORS.PRETTY_YELLOW,', '.join([at.__name__ if at is not None else 'none' for at in param_info.allowed_types])) + COLORS.ENDC)
      if param_info.has_allowed_vals:
        allowed_val_str = '\n'
        n_pad_idx = int(log10(len(param_info.allowed_vals))) + 1
        n_pad = max([len(i) for i in param_info.allowed_vals]) + 1
        for i in range(0,len(param_info.allowed_vals),2):
          allowed_val_str += ">>  {}. {}".format(str(i).rjust(n_pad_idx), param_info.allowed_vals[i].ljust(n_pad)) + (" {}. {}\n".format(str(i+1).rjust(n_pad_idx), param_info.allowed_vals[i+1]) if i+1 < len(param_info.allowed_vals) else '\n')
        to_print.append(COLORS.WARNING + '>>  Allowed values:{} {}{}{}'.format(COLORS.ENDC, COLORS.PRETTY_YELLOW, allowed_val_str, COLORS.ENDC))
      if param_info.min_val is not None or param_info.max_val is not None:
        to_print.append(COLORS.WARNING + '>>  Bounds:{} ({}, {})'.format(COLORS.ENDC, self.color_from_type(param_info.min_val), self.color_from_type(param_info.max_val)) + COLORS.ENDC)
      if param_info.linked_op_param_check_param != '' and param_info.linked_op_param_check_param in self.op_params.fork_params and self.op_params.get(param_info.linked_op_param_check_param, force_update=True):
        to_print.append(COLORS.WARNING + '>>  ALL VALUES IN LIST ARE SYNCED' + COLORS.ENDC)
      to_print.append(COLORS.WARNING + '>>  Default value: {} {}'.format(self.color_from_type(param_info.default_value), param_info.unit) + COLORS.ENDC)
      
      history = read_history(chosen_key)
      if len(history) > 1:
        history = history[-min(6,len(history)):-1]
        num_hist = len(history)
        to_print.append(COLORS.WARNING + '>>  Last {}value{}:{}\n      {}'.format(f'{num_hist} ' if num_hist > 1 else '', 's' if num_hist > 1 else '', COLORS.ENDC, '\n      '.join([f'{val["time of change"]} -- {self.color_from_type(val["value"])}' for val in history])))

      if to_print:
        print('\n{}\n'.format('\n'.join(to_print)))

      if param_info.is_list:
        self.change_param_list(old_value, param_info, chosen_key)  # TODO: need to merge the code in this function with the below to reduce redundant code
        return

      self.info('Current value: {}{}{} (type: {})'.format(self.color_from_type(old_value), ' ' + param_info.unit, COLORS.INFO, type(old_value).__name__), sleep_time=0)

      while True:
        self.prompt('\nEnter your new value (enter to exit):')
        new_value = input('>> ').strip()
        if new_value == '':
          self.info('Exiting this parameter...\n')
          return

        new_value = self.str_eval(new_value)
        if not param_info.type_is_valid(new_value):
          try:
            new_value = param_info.allowed_types[0](new_value)
          except ValueError:
            self.error('The type of data you entered ({}) is not allowed with this parameter!'.format(type(new_value).__name__))
            continue
        if not param_info.value_is_valid(new_value):
          if type(new_value) == int and 0 <= new_value < len(param_info.allowed_vals):
            new_value = param_info.allowed_vals[new_value]
          else:
            self.error('The type of data you entered ({}) is not one of the allowed values ({}) for this parameter!'.format(new_value, ", ".join(param_info.allowed_vals)))
            continue
        new_value, was_clipped = param_info.value_clipped(new_value)
        if was_clipped:
          self.warning('Value out of bounds: clipped to {}'.format(new_value))

        # if not param_info.static:  # stay in live tuning interface
        self.op_params.put(chosen_key, new_value, show_alert=True)
        self.success('Saved {} with value: {}{}{} (type: {})'.format(chosen_key, self.color_from_type(new_value), ' ' + param_info.unit, COLORS.SUCCESS, type(new_value).__name__))
        # else:  # else ask to save and break
        #   self.warning('\nOld value: {}{} (type: {})'.format(self.color_from_type(old_value), COLORS.WARNING, type(old_value).__name__))
        #   self.success('New value: {}{} (type: {})'.format(self.color_from_type(new_value), COLORS.OKGREEN, type(new_value).__name__), sleep_time=0)
        #   self.prompt('\nDo you want to save this?')
        #   if input('Y/n (blank for yes)').lower().strip in ['','y']:
        #     self.op_params.put(chosen_key, new_value)
        #     self.success('Saved!')
        #   else:
        #     self.info('Not saved!')
        #   return

  def change_param_list(self, old_value, param_info, chosen_key):
    while True:
      self.info('Current value: {} (type: {})'.format(old_value, type(old_value).__name__), sleep_time=0)
      if param_info.linked_op_param_check_param != '' and param_info.linked_op_param_check_param in self.op_params.fork_params and self.op_params.get(param_info.linked_op_param_check_param, force_update=True):
        choice_idx = 0
      else:
        self.prompt('\nEnter index to edit (0 to {}):'.format(len(old_value) - 1))
        choice_idx = self.str_eval(input('>> '))
        if choice_idx == '':
          self.info('Exiting this parameter...')
          return

      if not isinstance(choice_idx, int) or choice_idx not in range(len(old_value)):
        self.error('Must be an integar within list range!')
        continue

      while True:
        self.info('Chosen index: {}'.format(choice_idx), sleep_time=0)
        self.info('Value: {} (type: {})'.format(old_value[choice_idx], type(old_value[choice_idx]).__name__), sleep_time=0)
        self.prompt('\nEnter your new value:')
        new_value = input('>> ').strip()
        if new_value == '':
          self.info('Exiting this list item...')
          break
        
        if not param_info.type_is_valid(new_value):
          try:
            new_value = param_info.allowed_types[0](new_value)
          except ValueError:
            self.error('The type of data you entered ({}) is not allowed with this parameter!'.format(type(new_value).__name__))
            continue
        elif not param_info.value_is_valid(new_value):
          self.error('The type of data you entered ({}) is not one of the allowed values ({}) for this parameter!'.format(new_value, ", ".join(param_info.allowed_vals)))
          continue
        if param_info.value_clipped(new_value):
          self.warning('Value out of bounds: clipped to {}'.format(new_value))
          
        if param_info.linked_op_param_check_param != '' and param_info.linked_op_param_check_param in self.op_params.fork_params and self.op_params.get(param_info.linked_op_param_check_param, force_update=True):
          old_value = [new_value for i in range(len(old_value))]
        else:
          old_value[choice_idx] = new_value

        self.op_params.put(chosen_key, old_value, show_alert=True)
        self.success('Saved {} with value: {}{} (type: {})'.format(chosen_key, self.color_from_type(new_value), COLORS.SUCCESS, type(new_value).__name__), end='\n')
        break
      
      if new_value == '' and param_info.linked_op_param_check_param != '' and param_info.linked_op_param_check_param in self.op_params.fork_params and self.op_params.get(param_info.linked_op_param_check_param, force_update=True):
        break
      
  def C3_rebootless_restart(self, immediate=False):
    import os
    
    if Params().get_bool('IsOnroad') and not immediate:
      for i in range(5):
        Params().put('OPParamsRebootInNSeconds', str(5-i))
        self.info("PERFORMING ONROAD OPENPILOT RESTART IN {} second{}!! ASSUME CONTROL OF VEHICLE!!".format(5-i,'' if i == 5-1 else 's'), sleep_time=1)
      Params().put('OPParamsRebootInNSeconds', str(-1))
    else:
      self.info("PERFORMING {}OPENPILOT RESTART".format(" FORCED " if immediate else ""), sleep_time=0)
    i=0
    while (ret := os.system('tmux kill-session -t comma; rm -f /tmp/safe_staging_overlay.lock; tmux new -s comma -d "/data/openpilot/launch_openpilot.sh"')) != 0:
      i += 1
      if i > 5:
        self.error("giving up...")
        break
      self.error("RESTART COMMAND FAILED. TRYING AGAIN IN 3 SECONDS...")
      time.sleep(3)
    
    if ret == 0:
      self.info("Restarting now", end='\n\n')
    else:
      self.error("Restart failed!")
    return ret

  def color_from_type(self, v):
    v_color = ''
    if type(v) in self.type_colors:
      v_color = self.type_colors[type(v)]
      if isinstance(v, bool):
        v_color = v_color[v]
    v = '{}{}{}'.format(v_color, v, COLORS.ENDC)
    return v

  def cyan(self, msg, end=''):
    msg = self.str_color(msg, style='cyan')
    # print(msg, flush=True, end='\n' + end)
    return msg

  def prompt(self, msg, end=''):
    msg = self.str_color(msg, style='prompt')
    print(msg, flush=True, end='\n' + end)

  def warning(self, msg, end=''):
    msg = self.str_color(msg, style='warning')
    print(msg, flush=True, end='\n' + end)

  def info(self, msg, sleep_time=None, end=''):
    if sleep_time is None:
      sleep_time = self.sleep_time
    msg = self.str_color(msg, style='info')

    print(msg, flush=True, end='\n' + end)
    time.sleep(sleep_time)

  def info2(self, msg, sleep_time=None, end=''):
    if sleep_time is None:
      sleep_time = self.sleep_time
    msg = self.str_color(msg, style=86)

    print(msg, flush=True, end='\n' + end)
    time.sleep(sleep_time)

  def error(self, msg, sleep_time=None, end='', surround=True):
    if sleep_time is None:
      sleep_time = self.sleep_time
    msg = self.str_color(msg, style='fail', surround=surround)

    print(msg, flush=True, end='\n' + end)
    time.sleep(sleep_time)

  def success(self, msg, sleep_time=None, end=''):
    if sleep_time is None:
      sleep_time = self.sleep_time
    msg = self.str_color(msg, style='success')

    print(msg, flush=True, end='\n' + end)
    time.sleep(sleep_time)

  @staticmethod
  def str_color(msg, style, surround=False):
    if style == 'success':
      style = COLORS.SUCCESS
    elif style == 'fail':
      style = COLORS.FAIL
    elif style == 'prompt':
      style = COLORS.PROMPT
    elif style == 'info':
      style = COLORS.INFO
    elif style == 'cyan':
      style = COLORS.CYAN
    elif style == 'warning':
      style = COLORS.WARNING
    elif isinstance(style, int):
      style = COLORS.BASE(style)

    if surround:
      msg = '{}--------\n{}\n{}--------{}'.format(style, msg, COLORS.ENDC + style, COLORS.ENDC)
    else:
      msg = '{}{}{}'.format(style, msg, COLORS.ENDC)

    return msg

  def input_with_options(self, options, default=None):
    """
    Takes in a list of options and asks user to make a choice.
    The most similar option list index is returned along with the similarity percentage from 0 to 1
    """
    user_input = input('[{}]{}: '.format('/'.join(options), f" (blank for '{options[default]}')" if default is not None else '')).lower().strip()
    if not user_input:
      return default, 0.0
    sims = [self.str_sim(i.lower().strip(), user_input) for i in options]
    argmax = sims.index(max(sims))
    return argmax, sims[argmax]

  def str_eval(self, dat):
    dat = dat.strip()
    try:
      dat = ast.literal_eval(dat)
    except:
      if dat.lower() == 'none':
        dat = None
      elif dat.lower() == 'false':
        dat = False
      elif dat.lower() == 'true':  # else, assume string
        dat = True
    return dat


opEdit(restart='-r' in sys.argv, restart_immediate='-n' in sys.argv)
