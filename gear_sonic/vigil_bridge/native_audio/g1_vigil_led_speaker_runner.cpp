#include <algorithm>
#include <condition_variable>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ifaddrs.h>
#include <netdb.h>
#include <sys/socket.h>

#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

namespace {

struct Args {
  std::string iface;
  std::string wav = "g1_music_16k_mono_pcm16_peak27800.wav";
  std::string tts_text;
  int tts_speaker_id = 0;
  int volume = 80;
  bool skip_tts = false;
  bool skip_music = false;
  bool stream_stdin = false;
  bool stream_reactive_led = false;
  bool allow_volume_100 = false;
  bool allow_volume_over_100 = false;
  std::string led_plan;
  int led_refresh_hz = 50;
  int led_end_to_dark_ms = 250;
  int led_dark_to_bright_ms = 500;
  int led_bright_hold_ms = 500;
};

struct Color {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

constexpr Color kDarkBlue{0, 0, 40};
constexpr Color kBrightBlue{0, 0, 255};
constexpr const char* kAppName = "g1_vigil_led_speaker";
constexpr double kPi = 3.14159265358979323846;

struct WavData {
  int sample_rate = 0;
  int channels = 0;
  int sample_width = 0;
  std::vector<uint8_t> pcm;
};

uint16_t ReadLe16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t ReadLe32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

std::string FindUnitreeInterface() {
  struct ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0 || ifaddr == nullptr) {
    return "";
  }

  std::string result;
  char host[NI_MAXHOST];
  for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host,
                    NI_MAXHOST, nullptr, 0, NI_NUMERICHOST) != 0) {
      continue;
    }
    std::string ip(host);
    if (ip.rfind("192.168.123.", 0) == 0) {
      result = ifa->ifa_name;
      break;
    }
  }
  freeifaddrs(ifaddr);
  return result;
}

Args ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    std::string key(argv[i]);
    if (key == "--iface" && i + 1 < argc) {
      args.iface = argv[++i];
    } else if (key == "--volume" && i + 1 < argc) {
      args.volume = std::stoi(argv[++i]);
    } else if (key == "--wav" && i + 1 < argc) {
      args.wav = argv[++i];
    } else if (key == "--tts-text" && i + 1 < argc) {
      args.tts_text = argv[++i];
    } else if (key == "--tts-speaker-id" && i + 1 < argc) {
      args.tts_speaker_id = std::stoi(argv[++i]);
    } else if (key == "--skip-tts") {
      args.skip_tts = true;
    } else if (key == "--skip-music") {
      args.skip_music = true;
    } else if (key == "--stream-stdin") {
      args.stream_stdin = true;
    } else if (key == "--stream-reactive-led") {
      args.stream_reactive_led = true;
    } else if (key == "--allow-volume-100") {
      args.allow_volume_100 = true;
    } else if (key == "--allow-volume-over-100") {
      args.allow_volume_over_100 = true;
    } else if (key == "--led-plan" && i + 1 < argc) {
      args.led_plan = argv[++i];
    } else if (key == "--led-refresh-hz" && i + 1 < argc) {
      args.led_refresh_hz = std::stoi(argv[++i]);
    } else if (key == "--led-end-to-dark-ms" && i + 1 < argc) {
      args.led_end_to_dark_ms = std::stoi(argv[++i]);
    } else if (key == "--led-dark-to-bright-ms" && i + 1 < argc) {
      args.led_dark_to_bright_ms = std::stoi(argv[++i]);
    } else if (key == "--led-bright-hold-ms" && i + 1 < argc) {
      args.led_bright_hold_ms = std::stoi(argv[++i]);
    } else {
      throw std::runtime_error("unknown or incomplete argument: " + key);
    }
  }
  if (args.iface.empty()) {
    args.iface = FindUnitreeInterface();
  }
  if (args.iface.empty()) {
    throw std::runtime_error("no interface provided and no 192.168.123.x interface found");
  }
  if (args.volume < 0) {
    throw std::runtime_error("volume must be non-negative");
  }
  if (args.volume >= 100 && !args.allow_volume_100 &&
      !args.allow_volume_over_100) {
    throw std::runtime_error(
        "volume 100 requires --allow-volume-100 or --allow-volume-over-100");
  }
  if (args.volume > 100 && !args.allow_volume_over_100) {
    throw std::runtime_error(
        "volume above 100 requires --allow-volume-over-100");
  }
  if (args.volume > 255) {
    throw std::runtime_error("volume must not exceed uint8_t max 255");
  }
  if (args.tts_speaker_id < 0) {
    throw std::runtime_error("tts speaker id must be non-negative");
  }
  if (args.led_refresh_hz <= 0 || args.led_refresh_hz > 100) {
    throw std::runtime_error("LED refresh_hz must be in 1..100");
  }
  if (args.led_end_to_dark_ms < 0 || args.led_dark_to_bright_ms < 0 ||
      args.led_bright_hold_ms < 0) {
    throw std::runtime_error("LED end animation durations must be non-negative");
  }
  if (args.stream_reactive_led && !args.stream_stdin) {
    throw std::runtime_error("--stream-reactive-led requires --stream-stdin");
  }
  if (args.volume > 90 && !args.allow_volume_100 &&
      !args.allow_volume_over_100) {
    throw std::runtime_error(
        "volume above 90 requires --allow-volume-100 or "
        "--allow-volume-over-100");
  }
  return args;
}

WavData ReadPcmWav(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("failed to open wav: " + path);
  }
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                             std::istreambuf_iterator<char>());
  if (bytes.size() < 44 || std::memcmp(bytes.data(), "RIFF", 4) != 0 ||
      std::memcmp(bytes.data() + 8, "WAVE", 4) != 0) {
    throw std::runtime_error("not a RIFF/WAVE file: " + path);
  }

  WavData wav;
  bool saw_fmt = false;
  bool saw_data = false;
  size_t pos = 12;
  while (pos + 8 <= bytes.size()) {
    const char* id = reinterpret_cast<const char*>(bytes.data() + pos);
    uint32_t size = ReadLe32(bytes.data() + pos + 4);
    size_t chunk_data = pos + 8;
    if (chunk_data + size > bytes.size()) {
      throw std::runtime_error("invalid wav chunk size");
    }

    if (std::memcmp(id, "fmt ", 4) == 0) {
      if (size < 16) {
        throw std::runtime_error("invalid fmt chunk");
      }
      uint16_t audio_format = ReadLe16(bytes.data() + chunk_data);
      wav.channels = ReadLe16(bytes.data() + chunk_data + 2);
      wav.sample_rate = ReadLe32(bytes.data() + chunk_data + 4);
      uint16_t bits_per_sample = ReadLe16(bytes.data() + chunk_data + 14);
      wav.sample_width = bits_per_sample / 8;
      if (audio_format != 1) {
        throw std::runtime_error("wav is not PCM format");
      }
      saw_fmt = true;
    } else if (std::memcmp(id, "data", 4) == 0) {
      wav.pcm.assign(bytes.begin() + chunk_data,
                     bytes.begin() + chunk_data + size);
      saw_data = true;
    }

    pos = chunk_data + size + (size % 2);
  }

  if (!saw_fmt || !saw_data) {
    throw std::runtime_error("wav missing fmt or data chunk");
  }
  if (wav.sample_rate != 16000 || wav.channels != 1 || wav.sample_width != 2) {
    throw std::runtime_error("wav must be 16 kHz mono PCM16");
  }
  return wav;
}

std::vector<Color> ReadLedPlan(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("failed to open LED plan: " + path);
  }
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                             std::istreambuf_iterator<char>());
  if (bytes.empty() || bytes.size() % 3 != 0) {
    throw std::runtime_error("LED plan must contain packed RGB triples");
  }
  std::vector<Color> colors;
  colors.reserve(bytes.size() / 3);
  for (size_t offset = 0; offset < bytes.size(); offset += 3) {
    Color color{bytes[offset], bytes[offset + 1], bytes[offset + 2]};
    if (color.b != 0) {
      throw std::runtime_error("speech LED plan must keep blue channel at zero");
    }
    colors.push_back(color);
  }
  return colors;
}

double Smoothstep(double value) {
  value = std::clamp(value, 0.0, 1.0);
  return value * value * (3.0 - 2.0 * value);
}

Color MixColor(const Color& from, const Color& to, double amount) {
  amount = Smoothstep(amount);
  return {
      static_cast<uint8_t>(std::lround(from.r + (to.r - from.r) * amount)),
      static_cast<uint8_t>(std::lround(from.g + (to.g - from.g) * amount)),
      static_cast<uint8_t>(std::lround(from.b + (to.b - from.b) * amount)),
  };
}

void SetLedOrThrow(unitree::robot::g1::AudioClient& client,
                   const Color& color) {
  const int32_t ret = client.LedControl(color.r, color.g, color.b);
  if (ret != 0) {
    throw std::runtime_error("LedControl failed: " + std::to_string(ret));
  }
}

void FadeLed(unitree::robot::g1::AudioClient& client, const Color& from,
             const Color& to, int duration_ms, int refresh_hz) {
  const int steps = std::max(1, static_cast<int>(
      std::lround(duration_ms * refresh_hz / 1000.0)));
  for (int step = 1; step <= steps; ++step) {
    SetLedOrThrow(client, MixColor(from, to,
                                  static_cast<double>(step) / steps));
    std::this_thread::sleep_for(
        std::chrono::milliseconds(std::max(1, duration_ms / steps)));
  }
}

void ApplyEndAnimation(unitree::robot::g1::AudioClient& client,
                       const Color& speech_color, const Args& args,
                       std::ostream& output = std::cout) {
  // Blue is introduced only after PlayStop has completed.
  FadeLed(client, speech_color, kDarkBlue, args.led_end_to_dark_ms,
          args.led_refresh_hz);
  FadeLed(client, kDarkBlue, kBrightBlue, args.led_dark_to_bright_ms,
          args.led_refresh_hz);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(args.led_bright_hold_ms));
  output << "[RESULT] LedEndAnimation to_dark_ms="
         << args.led_end_to_dark_ms
         << " dark_to_bright_ms=" << args.led_dark_to_bright_ms
         << " bright_hold_ms=" << args.led_bright_hold_ms
         << " final_rgb=0,0,255" << std::endl;
}

int32_t PlayWavWithLedPlan(unitree::robot::g1::AudioClient& client,
                           const WavData& wav, const Args& args,
                           const std::string& stream_id) {
  std::vector<Color> colors = ReadLedPlan(args.led_plan);
  const size_t bytes_per_second =
      wav.sample_rate * wav.channels * wav.sample_width;
  const size_t frame_bytes = bytes_per_second / args.led_refresh_hz;
  if (frame_bytes == 0 || bytes_per_second % args.led_refresh_hz != 0) {
    throw std::runtime_error("audio format must divide evenly by LED refresh_hz");
  }
  const size_t expected_frames =
      (wav.pcm.size() + frame_bytes - 1) / frame_bytes;
  if (colors.size() != expected_frames) {
    throw std::runtime_error(
        "LED plan frame count does not match WAV duration: expected=" +
        std::to_string(expected_frames) + " actual=" +
        std::to_string(colors.size()));
  }

  constexpr size_t kChunkBytes = 96000;  // Three seconds at 16k mono PCM16.
  size_t plan_index = 0;
  Color current = colors.front();
  bool playback_started = false;
  try {
    // Speaking starts directly in orange/yellow; there is no blue-to-speech fade.
    SetLedOrThrow(client, current);
    for (size_t offset = 0; offset < wav.pcm.size(); offset += kChunkBytes) {
      const size_t end = std::min(offset + kChunkBytes, wav.pcm.size());
      std::vector<uint8_t> chunk(wav.pcm.begin() + offset,
                                 wav.pcm.begin() + end);
      SetLedOrThrow(client, current);
      const int32_t play_ret =
          client.PlayStream(kAppName, stream_id, chunk);
      playback_started = true;
      std::cout << "[RESULT] PlayStream offset=" << offset
                << " bytes=" << chunk.size() << " ret=" << play_ret
                << std::endl;
      if (play_ret != 0) {
        throw std::runtime_error("PlayStream failed: " +
                                 std::to_string(play_ret));
      }
      // Reassert immediately because firmware may briefly restore its default LED.
      SetLedOrThrow(client, current);

      const auto chunk_clock = std::chrono::steady_clock::now();
      const size_t chunk_frames =
          (chunk.size() + frame_bytes - 1) / frame_bytes;
      for (size_t frame = 0; frame < chunk_frames; ++frame) {
        current = colors.at(plan_index++);
        SetLedOrThrow(client, current);
        const size_t consumed =
            std::min((frame + 1) * frame_bytes, chunk.size());
        const auto target_time =
            chunk_clock + std::chrono::microseconds(
                static_cast<long long>(consumed) * 1000000LL /
                bytes_per_second);
        std::this_thread::sleep_until(target_time);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    const int32_t stop_ret = client.PlayStop(kAppName);
    playback_started = false;
    std::cout << "[RESULT] PlayStop app_name=" << kAppName
              << " ret=" << stop_ret << std::endl;
    ApplyEndAnimation(client, current, args);
    std::cout << "[RESULT] LedPlan frames=" << colors.size()
              << " refresh_hz=" << args.led_refresh_hz
              << " speech_blue_channel=0" << std::endl;
    return stop_ret;
  } catch (...) {
    if (playback_started) {
      client.PlayStop(kAppName);
    }
    client.LedControl(kBrightBlue.r, kBrightBlue.g, kBrightBlue.b);
    throw;
  }
}

Color StreamingSpeechColor(double level) {
  level = std::clamp(level, 0.0, 1.0);
  const Color low{46, 14, 0};
  const Color mid{158, 79, 0};
  const Color high{255, 234, 0};
  if (level <= 0.5) {
    return MixColor(low, mid, level * 2.0);
  }
  return MixColor(mid, high, (level - 0.5) * 2.0);
}

class StreamingLedPlanner {
 public:
  explicit StreamingLedPlanner(int refresh_hz)
      : refresh_hz_(refresh_hz) {}

  void Reset() {
    rms_reference_ = 8000.0;
    envelope_ = 0.0;
    display_ = 0.0;
    frame_index_ = 0;
  }

  std::vector<Color> Plan(const std::vector<uint8_t>& pcm) {
    constexpr int kSampleRate = 16000;
    constexpr int kSampleWidth = 2;
    const size_t frame_samples = kSampleRate / refresh_hz_;
    const size_t sample_count = pcm.size() / kSampleWidth;
    std::vector<Color> colors;
    colors.reserve((sample_count + frame_samples - 1) / frame_samples);
    for (size_t start = 0; start < sample_count; start += frame_samples) {
      const size_t end = std::min(start + frame_samples, sample_count);
      double sum_sq = 0.0;
      for (size_t index = start; index < end; ++index) {
        const size_t offset = index * 2;
        const int16_t sample = static_cast<int16_t>(
            static_cast<uint16_t>(pcm[offset]) |
            (static_cast<uint16_t>(pcm[offset + 1]) << 8));
        sum_sq += static_cast<double>(sample) * sample;
      }
      const double rms = end > start
                             ? std::sqrt(sum_sq / static_cast<double>(end - start))
                             : 0.0;
      const double reference_alpha = rms > rms_reference_ ? 0.08 : 0.002;
      rms_reference_ += reference_alpha * (std::max(rms, 2000.0) - rms_reference_);
      const double noise_floor = std::max(180.0, rms_reference_ * 0.035);
      const double linear = std::clamp(
          (rms - noise_floor) / std::max(rms_reference_ - noise_floor, 1.0),
          0.0, 1.0);
      const double target = std::pow(linear, 1.8);
      const double envelope_alpha = target > envelope_ ? 0.18 : 0.045;
      envelope_ += envelope_alpha * (target - envelope_);
      display_ += 0.18 * (envelope_ - display_);
      const double breath =
          0.5 - 0.5 * std::cos(2.0 * kPi * 0.3 * frame_index_ / refresh_hz_);
      const double level = std::max(0.12, display_) * (0.9 + 0.1 * breath);
      colors.push_back(StreamingSpeechColor(level));
      ++frame_index_;
    }
    return colors;
  }

 private:
  int refresh_hz_;
  double rms_reference_ = 8000.0;
  double envelope_ = 0.0;
  double display_ = 0.0;
  uint64_t frame_index_ = 0;
};

class StreamingLedPlayer {
 public:
  StreamingLedPlayer(unitree::robot::g1::AudioClient& client,
                     std::mutex& client_mutex, const Args& args)
      : client_(client), client_mutex_(client_mutex), args_(args),
        planner_(args.led_refresh_hz), thread_(&StreamingLedPlayer::Run, this) {}

  ~StreamingLedPlayer() { Shutdown(); }

  void StartUtterance() {
    WaitUntilIdle();
    planner_.Reset();
    {
      std::lock_guard<std::mutex> lock(client_mutex_);
      SetLedOrThrow(client_, StreamingSpeechColor(0.12));
    }
  }

  void EnqueuePcm(const std::vector<uint8_t>& pcm) {
    std::vector<Color> colors = planner_.Plan(pcm);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const Color& color : colors) {
        colors_.push_back(color);
      }
    }
    cv_.notify_all();
  }

  void WaitUntilIdle() {
    std::unique_lock<std::mutex> lock(mutex_);
    idle_cv_.wait(lock, [this] { return colors_.empty() && !applying_color_; });
  }

  void Abort() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      colors_.clear();
    }
    cv_.notify_all();
    WaitUntilIdle();
    std::lock_guard<std::mutex> lock(client_mutex_);
    client_.LedControl(kBrightBlue.r, kBrightBlue.g, kBrightBlue.b);
  }

  void FinishUtterance() {
    WaitUntilIdle();
    std::lock_guard<std::mutex> lock(client_mutex_);
    ApplyEndAnimation(client_, last_color_, args_, std::cerr);
  }

  void Shutdown() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) {
        return;
      }
      stopping_ = true;
      colors_.clear();
    }
    cv_.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

 private:
  void Run() {
    const auto frame_duration =
        std::chrono::microseconds(1000000 / args_.led_refresh_hz);
    auto next_frame = std::chrono::steady_clock::now();
    while (true) {
      Color color;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return stopping_ || !colors_.empty(); });
        if (stopping_) {
          applying_color_ = false;
          idle_cv_.notify_all();
          return;
        }
        color = colors_.front();
        colors_.pop_front();
        applying_color_ = true;
      }
      next_frame = std::max(next_frame + frame_duration,
                            std::chrono::steady_clock::now());
      {
        std::lock_guard<std::mutex> lock(client_mutex_);
        const int32_t ret = client_.LedControl(color.r, color.g, color.b);
        if (ret != 0) {
          std::cerr << "[WARN] streaming LedControl failed: " << ret
                    << std::endl;
        }
      }
      last_color_ = color;
      std::this_thread::sleep_until(next_frame);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        applying_color_ = false;
        if (colors_.empty()) {
          idle_cv_.notify_all();
          next_frame = std::chrono::steady_clock::now();
        }
      }
    }
  }

  unitree::robot::g1::AudioClient& client_;
  std::mutex& client_mutex_;
  Args args_;
  StreamingLedPlanner planner_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::condition_variable idle_cv_;
  std::deque<Color> colors_;
  bool applying_color_ = false;
  bool stopping_ = false;
  Color last_color_{46, 14, 0};
  std::thread thread_;
};

bool ReadExact(std::istream& input, std::vector<uint8_t>& output,
               size_t size) {
  output.resize(size);
  input.read(reinterpret_cast<char*>(output.data()),
             static_cast<std::streamsize>(size));
  return input.good() || static_cast<size_t>(input.gcount()) == size;
}

void ProtocolOk(const std::string& message) {
  std::cout << "OK " << message << std::endl;
}

void ProtocolError(const std::string& message) {
  std::cout << "ERR " << message << std::endl;
}

int RunPersistentPcmStream(unitree::robot::g1::AudioClient& client,
                           const Args& args) {
  std::mutex client_mutex;
  std::unique_ptr<StreamingLedPlayer> led;
  if (args.stream_reactive_led) {
    led = std::make_unique<StreamingLedPlayer>(client, client_mutex, args);
  }
  std::string active_stream_id;
  ProtocolOk("READY");

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream header(line);
    std::string command;
    header >> command;
    try {
      if (command == "START") {
        std::string stream_id;
        header >> stream_id;
        if (stream_id.empty()) {
          ProtocolError("START requires a stream id");
          continue;
        }
        if (!active_stream_id.empty()) {
          ProtocolError("a stream is already active");
          continue;
        }
        if (led) {
          led->StartUtterance();
        }
        active_stream_id = stream_id;
        ProtocolOk("START " + stream_id);
      } else if (command == "PCM") {
        size_t pcm_bytes = 0;
        header >> pcm_bytes;
        if (pcm_bytes == 0) {
          ProtocolError("PCM length must be positive");
          continue;
        }
        std::vector<uint8_t> pcm;
        if (!ReadExact(std::cin, pcm, pcm_bytes)) {
          ProtocolError("unexpected EOF while reading PCM payload");
          break;
        }
        if (pcm_bytes % 2 != 0) {
          ProtocolError("PCM length must be a multiple of 2");
          continue;
        }
        if (active_stream_id.empty()) {
          ProtocolError("PCM received without an active stream");
          continue;
        }
        int32_t ret = 0;
        {
          std::lock_guard<std::mutex> lock(client_mutex);
          ret = client.PlayStream(kAppName, active_stream_id, pcm);
        }
        if (ret != 0) {
          ProtocolError("PlayStream failed: " + std::to_string(ret));
          continue;
        }
        if (led) {
          led->EnqueuePcm(pcm);
        }
        ProtocolOk("PCM " + std::to_string(pcm_bytes));
      } else if (command == "END") {
        if (active_stream_id.empty()) {
          ProtocolError("END received without an active stream");
          continue;
        }
        if (led) {
          led->WaitUntilIdle();
        }
        int32_t ret = 0;
        {
          std::lock_guard<std::mutex> lock(client_mutex);
          ret = client.PlayStop(kAppName);
        }
        if (ret != 0) {
          ProtocolError("PlayStop failed: " + std::to_string(ret));
          continue;
        }
        if (led) {
          led->FinishUtterance();
        }
        active_stream_id.clear();
        ProtocolOk("END");
      } else if (command == "STOP") {
        if (led) {
          led->Abort();
        }
        if (!active_stream_id.empty()) {
          std::lock_guard<std::mutex> lock(client_mutex);
          client.PlayStop(kAppName);
        }
        active_stream_id.clear();
        ProtocolOk("STOP");
      } else if (command == "QUIT") {
        if (led) {
          led->Abort();
        }
        if (!active_stream_id.empty()) {
          std::lock_guard<std::mutex> lock(client_mutex);
          client.PlayStop(kAppName);
        }
        active_stream_id.clear();
        ProtocolOk("QUIT");
        break;
      } else {
        ProtocolError("unsupported command: " + command);
      }
    } catch (const std::exception& exc) {
      ProtocolError(exc.what());
    }
  }

  if (!active_stream_id.empty()) {
    std::lock_guard<std::mutex> lock(client_mutex);
    client.PlayStop(kAppName);
  }
  if (led) {
    led->Shutdown();
  }
  return 0;
}

void SleepSeconds(double seconds) {
  std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(seconds * 1000.0)));
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Args args = ParseArgs(argc, argv);
    std::ostream& log = args.stream_stdin ? std::cerr : std::cout;
    log << "WARNING: about to play audio at volume " << args.volume
        << ". Please keep people away from the robot speaker."
        << std::endl;
    log << "[INFO] interface=" << args.iface << std::endl;
    log << "[INFO] wav=" << args.wav << std::endl;
    if (!args.tts_text.empty()) {
      log << "[INFO] tts_text_chars=" << args.tts_text.size()
          << " tts_speaker_id=" << args.tts_speaker_id << std::endl;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, args.iface);
    unitree::robot::g1::AudioClient client;
    client.Init();
    client.SetTimeout(10.0f);
    log << "[RESULT] AudioClient init=OK" << std::endl;

    uint8_t original_volume = 0;
    int32_t ret = client.GetVolume(original_volume);
    log << "[RESULT] GetVolume ret=" << ret
        << " volume=" << static_cast<int>(original_volume) << std::endl;

    ret = client.SetVolume(static_cast<uint8_t>(args.volume));
    log << "[RESULT] SetVolume requested=" << args.volume
        << " ret=" << ret << std::endl;

    uint8_t current_volume = 0;
    int32_t current_ret = client.GetVolume(current_volume);
    log << "[RESULT] GetVolumeAfterSet ret=" << current_ret
        << " volume=" << static_cast<int>(current_volume) << std::endl;

    if (args.stream_stdin) {
      log << "[INFO] persistent_pcm_stream=true reactive_led="
          << args.stream_reactive_led << std::endl;
      return RunPersistentPcmStream(client, args);
    }

    if (!args.tts_text.empty()) {
      ret = client.TtsMaker(args.tts_text, args.tts_speaker_id);
      std::cout << "[RESULT] TtsMaker text ret=" << ret
                << " speaker_id=" << args.tts_speaker_id
                << " text_chars=" << args.tts_text.size() << std::endl;
      SleepSeconds(3.0);
    } else if (!args.skip_tts) {
      ret = client.TtsMaker("你好，这是 G1 扬声器大音量测试。", 0);
      std::cout << "[RESULT] TtsMaker zh ret=" << ret << std::endl;
      SleepSeconds(4.0);
      ret = client.TtsMaker("Hello, this is a loud G1 speaker test.", 1);
      std::cout << "[RESULT] TtsMaker en ret=" << ret << std::endl;
      SleepSeconds(4.0);
    } else {
      std::cout << "[RESULT] TtsMaker skipped=true" << std::endl;
    }

    if (!args.skip_music) {
      WavData wav = ReadPcmWav(args.wav);
      const double duration =
          static_cast<double>(wav.pcm.size()) /
          (wav.sample_rate * wav.channels * wav.sample_width);
      std::cout << "[RESULT] WavInfo sample_rate=" << wav.sample_rate
                << " channels=" << wav.channels
                << " sample_width=" << wav.sample_width
                << " pcm_bytes=" << wav.pcm.size()
                << " duration=" << duration << std::endl;

      const std::string stream_id =
          std::to_string(unitree::common::GetCurrentTimeMillisecond());
      if (!args.led_plan.empty()) {
        ret = PlayWavWithLedPlan(client, wav, args, stream_id);
      } else {
        constexpr size_t kChunkBytes = 32000;
        for (size_t offset = 0; offset < wav.pcm.size();
             offset += kChunkBytes) {
          const size_t end = std::min(offset + kChunkBytes, wav.pcm.size());
          std::vector<uint8_t> chunk(wav.pcm.begin() + offset,
                                     wav.pcm.begin() + end);
          ret = client.PlayStream(kAppName, stream_id, chunk);
          std::cout << "[RESULT] PlayStream offset=" << offset
                    << " bytes=" << chunk.size() << " ret=" << ret
                    << std::endl;
          if (ret != 0) {
            break;
          }
          SleepSeconds(1.0);
        }
        SleepSeconds(0.5);
        ret = client.PlayStop(kAppName);
        std::cout << "[RESULT] PlayStop app_name=" << kAppName
                  << " ret=" << ret << std::endl;
      }
    } else {
      std::cout << "[RESULT] PlayStream skipped=true" << std::endl;
      std::cout << "[RESULT] PlayStop skipped=true" << std::endl;
    }

    std::cout << "[DONE] G1 loud speaker + music runner complete" << std::endl;
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "[EXCEPTION] " << exc.what() << std::endl;
    return 1;
  }
}
