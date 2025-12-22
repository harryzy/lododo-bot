import json
import threading
import queue
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
from ament_index_python.packages import get_package_share_directory

try:
    from vosk import Model, KaldiRecognizer
    import sounddevice as sd
    import numpy as np
    from scipy import signal
except Exception:
    Model = None
    KaldiRecognizer = None
    sd = None
    np = None
    signal = None


class STTNode(Node):
    """Local Vosk-based STT node that publishes normalized robot commands to /bot_command."""

    def __init__(self, model_path=None, device=None):
        super().__init__("bot_voice_node")

        # Load configuration from YAML file
        model_path, device_index, voice_commands = self._load_yaml_vosk()
        self.voice_commands = voice_commands

        self.pub = self.create_publisher(String, "/bot_command", 10)
        self.get_logger().info(f"STT node init, model_path={model_path}")

        if Model is None:
            self.get_logger().error(
                "vosk or sounddevice not available. Please pip install vosk sounddevice"
            )
            raise RuntimeError("Vosk dependencies missing")

        if not os.path.exists(model_path):
            self.get_logger().error(
                f"Model not found: {model_path}. Please download and extract a Vosk model."
            )
            raise RuntimeError("Vosk model missing")

        # load model
        self.model = Model(model_path)
        self.target_sample_rate = 16000  # Vosk requires 16kHz
        self.rec = KaldiRecognizer(self.model, self.target_sample_rate)
        self.get_logger().info(f"KaldiRecognizer using samplerate={self.target_sample_rate}")

        try:
            device = int(device_index)
        except Exception:
            device = None

        self.device = device
        self.device_sample_rate = None  # Will be set from device info
        self.resample_buffer = np.array([], dtype=np.int16)  # Buffer for resampling

        # Log chosen device name for debugging
        try:
            dev_name = None
            if self.device is not None:
                info = sd.query_devices(self.device)
                dev_name = info.get("name")
            else:
                d = sd.default.device
                if isinstance(d, (list, tuple)):
                    idx = d[0]
                else:
                    idx = d
                info = sd.query_devices(idx)
                dev_name = info.get("name")
        except Exception:
            dev_name = "unknown"
        self.get_logger().info(
            f'Using audio device index={self.device} name="{dev_name}"'
        )

        try:
            # Determine samplerate from device info to avoid unsupported-rate errors
            try:
                if self.device is not None:
                    dev_info = sd.query_devices(self.device)
                else:
                    d = sd.default.device
                    idx = d[0] if isinstance(d, (list, tuple)) else d
                    dev_info = sd.query_devices(idx)
                self.device_sample_rate = int(dev_info.get("default_samplerate", 16000))
            except Exception:
                self.device_sample_rate = 16000

            self.get_logger().info(f"Opening audio stream with samplerate={self.device_sample_rate}")
            self.get_logger().info(f"Will resample from {self.device_sample_rate}Hz to {self.target_sample_rate}Hz for Vosk")
            
            # choose a reasonable blocksize (frames per buffer)
            blocksize = 1024
            self._audio_stream = sd.RawInputStream(
                samplerate=self.device_sample_rate,
                blocksize=blocksize,
                dtype="int16",
                channels=1,
                callback=self._audio_callback,
                device=self.device,
            )
            self._audio_stream.start()
        except Exception as e:
            self.get_logger().error(f"Failed to open audio input stream: {e}")
            raise

        self._queue = queue.Queue()
        self._thread = threading.Thread(target=self._process_loop, daemon=True)
        self._thread.start()

        self.get_logger().info("STT node ready and listening")

    def _load_yaml_vosk(self):
        """
        Load Vosk configuration from YAML file.
        Returns model_path, device_index, and voice_commands.
        Only loads from config file, no hardcoded defaults.
        """
        model_path = None
        device_index = None
        voice_commands = {}
        
        try:
            share_dir = get_package_share_directory("bot_voice")
            cfg_path = os.path.join(share_dir, "config", "vosk_config.yaml")
            
            if not os.path.exists(cfg_path):
                raise FileNotFoundError(f"Configuration file not found: {cfg_path}")
                
            with open(cfg_path, "r") as f:
                cfg = yaml.safe_load(f)
                
            # Load core parameters
            model_type = cfg.get("model_type")
            if not model_type:
                raise ValueError("model_type not specified in config file")
                
            device_index = cfg.get("device_index")
            self.get_logger().info(f"Loaded model_type: {model_type}")
            
            # Load model path based on model_type
            if model_type == "cn":
                model_path = cfg.get("model_path_cn")
            elif model_type == "en":
                model_path = cfg.get("model_path_en")
            else:
                raise ValueError(f"Unsupported model_type: {model_type}")

            if not model_path:
                raise ValueError(f"model_path_{model_type} not specified in config file")

            # Allow override via environment variable and expand user vars (~) and env vars
            env_override = os.environ.get("VOSK_MODEL_PATH")
            if model_path is None and env_override:
                model_path = env_override

            # Expand ~ and any environment variables, then make absolute
            model_path = os.path.expanduser(os.path.expandvars(model_path))
            if not os.path.isabs(model_path):
                # If a relative path was provided, resolve it relative to the package share dir
                model_path = os.path.abspath(os.path.join(share_dir, model_path))
                
            # Load voice commands from config
            voice_commands_config = cfg.get("voice_commands", {})
            if not voice_commands_config:
                raise ValueError("voice_commands section not found in config file")
                
            # Get voice commands for the selected language
            language_commands = voice_commands_config.get(model_type, {})
            if not language_commands:
                raise ValueError(f"voice_commands for {model_type} not found in config file")
                
            voice_commands = language_commands.copy()
            self.get_logger().info(f"Loaded {len(voice_commands)} voice commands: {list(voice_commands.keys())}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load vosk_config.yaml: {e}")
            raise RuntimeError(f"Configuration loading failed: {e}")
            
        return model_path, device_index, voice_commands

    def _audio_callback(self, indata, frames, time_info, status):
        # put raw audio bytes into queue for processing
        if status:
            self.get_logger().warning(f"Audio callback status: {status}")
        try:
            if indata is not None:
                self._queue.put(bytes(indata))
                # log queue size occasionally for debugging
                qsize = self._queue.qsize()
                if qsize and qsize % 50 == 0:
                    self.get_logger().info(f"audio queue size: {qsize}")
        except Exception:
            pass

    def _normalize_text_to_command(self, text: str) -> str:
        """
        Normalize recognized text to command using configured voice commands.
        Matches against voice_commands loaded from YAML config.
        """
        t = text.lower().strip()
        if not t:
            return ""
        
        self.get_logger().info(f"_normalize_text_to_command  read  text = {t}")
        # Remove all spaces from both the input text and command phrases for better matching
        # This helps match "扫描 前面" (scan front with space) to "扫描前面" (without space) and "scan front" to "scanfront"
        t_no_space = t.replace(" ", "").replace("　", "")  # Remove both ASCII and full-width spaces
        
        # Check commands in order of specificity (longer phrases first)
        # Sort commands by phrase length (descending) to match more specific commands first
        sorted_commands = sorted(
            self.voice_commands.items(), 
            key=lambda x: len(x[1]), 
            reverse=True
        )
        
        for command_key, command_phrase in sorted_commands:
            # Remove spaces from command phrase as well for matching
            phrase_no_space = command_phrase.lower().replace(" ", "").replace("　", "")
            
            # Check if the normalized phrase matches the normalized text
            if phrase_no_space in t_no_space:
                return command_key
        
        # Fallback: return empty string if no match found
        self.get_logger().debug(f"No matching command found for text: {t}")
        return ""

    def _resample_audio(self, audio_data):
        """Resample audio from device_sample_rate to target_sample_rate (16kHz for Vosk)"""
        if self.device_sample_rate == self.target_sample_rate:
            return audio_data
        
        try:
            # Convert bytes to numpy array
            audio_np = np.frombuffer(audio_data, dtype=np.int16)
            
            # Append to buffer to have enough samples for resampling
            self.resample_buffer = np.append(self.resample_buffer, audio_np)
            
            # Calculate how many samples we need for resampling
            # We want to process chunks that can be cleanly resampled
            min_input_samples = int(self.device_sample_rate * 0.1)  # 100ms worth
            
            if len(self.resample_buffer) < min_input_samples:
                return None  # Not enough data yet
            
            # Resample using scipy
            num_output_samples = int(len(self.resample_buffer) * self.target_sample_rate / self.device_sample_rate)
            resampled = signal.resample(self.resample_buffer, num_output_samples)
            
            # Convert back to int16
            resampled = np.clip(resampled, -32768, 32767).astype(np.int16)
            
            # Clear buffer
            self.resample_buffer = np.array([], dtype=np.int16)
            
            return resampled.tobytes()
        except Exception as e:
            self.get_logger().warning(f"Resampling failed: {e}")
            return None

    def _process_loop(self):
        while rclpy.ok():
            try:
                data = self._queue.get(timeout=0.1)
            except Exception:
                continue
            try:
                # Resample audio if needed
                resampled_data = self._resample_audio(data)
                if resampled_data is None:
                    continue
                
                if self.rec.AcceptWaveform(resampled_data):
                    res = json.loads(self.rec.Result())
                    text = res.get("text", "").strip()
                    if text:
                        cmd = self._normalize_text_to_command(text)
                        if cmd:
                            msg = String()
                            msg.data = cmd
                            self.pub.publish(msg)
                            self.get_logger().info(
                                f"publish command: {cmd} (raw: {text})"
                            )
                else:
                    # Check partial results for debugging
                    partial = json.loads(self.rec.PartialResult())
                    partial_text = partial.get("partial", "").strip()
                    if partial_text and len(partial_text) > 5:  # Only log substantial partial results
                        self.get_logger().debug(f"Partial: {partial_text}")
            except Exception as e:
                self.get_logger().warning(f"processing audio failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = STTNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            try:
                node.get_logger().info("shutting down stt node")
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
