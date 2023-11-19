#include "selfdrive/ui/soundd/sound.h"

#include <cmath>

#include <QAudio>
#include <QAudioDeviceInfo>
#include <QDebug>

#include "cereal/messaging/messaging.h"
#include "common/util.h"

// TODO: detect when we can't play sounds
// TODO: detect when we can't display the UI

Sound::Sound(QObject *parent) : sm({"controlsState", "microphone"}) {
  qInfo() << "default audio device: " << QAudioDeviceInfo::defaultOutputDevice().deviceName();

  // FrogPilot variables
  isSilentMode = params.getBool("SilentMode");

  isCustomTheme = params.getBool("CustomTheme");
  customSounds = isCustomTheme ? params.getInt("CustomSounds") : 0;

  const std::unordered_map<int, QString> themeConfiguration = {
    {0, "stock"},
    {1, "frog_theme"},
    {2, "tesla_theme"},
    {3, "stalin_theme"}
  };

  for (const auto &[key, themeName] : themeConfiguration) {
    QString base = themeName == "stock" ? "../../assets/sounds" : QString("../../assets/custom_themes/%1/sounds").arg(themeName);
    soundPaths[key] = base;
  }

  for (auto &[alert, fn, loops, volume] : sound_list) {
    QSoundEffect *s = new QSoundEffect(this);
    QObject::connect(s, &QSoundEffect::statusChanged, [=]() {
      assert(s->status() != QSoundEffect::Error);
    });
    s->setSource(QUrl::fromLocalFile(soundPaths[customSounds] + "/" + fn));
    s->setVolume(isSilentMode ? 0 : volume);
    sounds[alert] = {s, loops};
  }

  QTimer *timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &Sound::update);
  QObject::connect(uiState(), &UIState::uiUpdateFrogPilotParams, this, &Sound::updateFrogPilotParams);
  timer->start(1000 / UI_FREQ);

  updateFrogPilotParams();
}

void Sound::update() {
  sm.update(0);

  // scale volume using ambient noise level
  if (sm.updated("microphone")) {
    float volume = util::map_val(sm["microphone"].getMicrophone().getFilteredSoundPressureWeightedDb(), 30.f, 60.f, 0.f, 1.f);
    volume = QAudio::convertVolume(volume, QAudio::LogarithmicVolumeScale, QAudio::LinearVolumeScale);
    // set volume on changes
    if (std::exchange(current_volume, std::nearbyint(volume * 10)) != current_volume) {
      Hardware::set_volume(volume);
    }
  }

  setAlert(Alert::get(sm, 0));
}

void Sound::updateFrogPilotParams() {
  // Update FrogPilot parameters upon toggle change
  customSounds = isCustomTheme ? params.getInt("CustomSounds") : 0;
  isSilentMode = params.getBool("SilentMode");

  for (auto &[alert, fn, loops, volume] : sound_list) {
    auto &[s, _] = sounds[alert];
    s->setSource(QUrl::fromLocalFile(soundPaths[customSounds] + "/" + fn));
    s->setVolume(isSilentMode ? 0 : volume);
  }
}

void Sound::setAlert(const Alert &alert) {
  if (!current_alert.equal(alert)) {
    current_alert = alert;
    // stop sounds
    for (auto &[s, loops] : sounds) {
      // Only stop repeating sounds
      if (s->loopsRemaining() > 1 || s->loopsRemaining() == QSoundEffect::Infinite) {
        s->stop();
      }
    }

    // play sound
    if (alert.sound != AudibleAlert::NONE) {
      auto &[s, loops] = sounds[alert.sound];
      s->setLoopCount(loops);
      s->play();
    }
  }
}
