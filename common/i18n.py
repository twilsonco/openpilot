import gettext
from common.params import Params

locale_dir = "/data/openpilot/selfdrive/assets/locales"
# supported_language = ["en-US", "zh-TW", "zh-CN", "ja-JP", "ko-KR"]
supported_languages = {
  "main_en": "en-US",
  "main_zh-CHT": "zh-TW",
  "main_zh-CHS": "zh-CN",
  "main_ko": "ko-KR",
  "main_ja": "ja-JP",
}

def events():
  locale = Params().get("LanguageSetting", encoding='utf8')
  try:
    if locale is not None:
      locale = supported_languages[locale.strip()]
    else:
      locale = "en-US"
  except KeyError:
    locale = "en-US"
  i18n = gettext.translation("events", localedir=locale_dir, fallback=True, languages=[locale])
  i18n.install()
  return i18n.gettext
