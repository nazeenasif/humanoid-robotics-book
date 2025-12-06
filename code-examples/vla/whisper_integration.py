import asyncio
import openai
import pyaudio
import wave
import audioop
import threading
import queue
import time
import logging
from typing import Optional, Dict, Any, Callable
from dataclasses import dataclass
import tempfile
import os

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class SpeechRecognitionResult:
    """Represents the result of speech recognition"""
    text: str
    confidence: float
    language: str
    duration: float
    timestamp: float

class WhisperSpeechRecognizer:
    """
    Integrates OpenAI Whisper for speech recognition in VLA systems
    """

    def __init__(self, api_key: str, model: str = "whisper-1"):
        """
        Initialize the Whisper speech recognizer

        Args:
            api_key: OpenAI API key
            model: Whisper model to use
        """
        openai.api_key = api_key
        self.model = model
        self.is_listening = False
        self.audio_queue = queue.Queue()
        self.result_callback: Optional[Callable[[SpeechRecognitionResult], None]] = None

        # Audio configuration
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000  # Whisper works best at 16kHz
        self.chunk = 1024
        self.silence_threshold = 500  # Adjust based on your environment
        self.silence_duration = 1.0   # Seconds of silence to stop recording

    def set_result_callback(self, callback: Callable[[SpeechRecognitionResult], None]):
        """
        Set a callback function to handle speech recognition results
        """
        self.result_callback = callback

    def is_silent(self, data_chunk: bytes) -> bool:
        """
        Check if the audio chunk is silent
        """
        rms = audioop.rms(data_chunk, 2)  # 2 is the sample width
        return rms < self.silence_threshold

    async def record_audio_chunk(self, duration: float = 30.0) -> Optional[str]:
        """
        Record a single audio chunk and save to a temporary file

        Args:
            duration: Maximum recording duration in seconds

        Returns:
            Path to the temporary audio file, or None if no speech detected
        """
        try:
            audio = pyaudio.PyAudio()

            # Open audio stream
            stream = audio.open(
                format=self.audio_format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk
            )

            logger.info("Recording... Speak now.")

            frames = []
            start_time = time.time()
            silence_start_time = None

            while time.time() - start_time < duration:
                data = stream.read(self.chunk)
                frames.append(data)

                # Check for silence to stop recording early
                if self.is_silent(data):
                    if silence_start_time is None:
                        silence_start_time = time.time()
                    elif time.time() - silence_start_time > self.silence_duration:
                        logger.info("Silence detected, stopping recording...")
                        break
                else:
                    silence_start_time = None  # Reset silence timer

            # Stop and close the stream
            stream.stop_stream()
            stream.close()
            audio.terminate()

            if not frames:
                logger.warning("No audio frames recorded")
                return None

            # Create a temporary WAV file
            temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
            temp_filename = temp_file.name
            temp_file.close()

            # Write the recorded frames to the temporary file
            with wave.open(temp_filename, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(audio.get_sample_size(self.audio_format))
                wf.setframerate(self.rate)
                wf.writeframes(b''.join(frames))

            logger.info(f"Audio recorded to temporary file: {temp_filename}")
            return temp_filename

        except Exception as e:
            logger.error(f"Error recording audio: {e}")
            return None

    async def transcribe_audio(self, audio_file_path: str, language: Optional[str] = None) -> SpeechRecognitionResult:
        """
        Transcribe audio using OpenAI Whisper API

        Args:
            audio_file_path: Path to the audio file to transcribe
            language: Optional language code (e.g., 'en', 'es', 'fr')

        Returns:
            Speech recognition result
        """
        try:
            start_time = time.time()

            with open(audio_file_path, 'rb') as audio_file:
                transcription = await openai.Audio.atranscribe(
                    model=self.model,
                    file=audio_file,
                    language=language,
                    response_format="verbose_json",  # This provides more detailed response
                    timestamp_granularities=["segment"]
                )

            duration = time.time() - start_time

            # For Whisper, confidence is not directly provided, so we'll estimate based on other factors
            # In a real implementation, you might use a different method to estimate confidence
            estimated_confidence = min(0.95, len(transcription.text) / (len(transcription.text) + 10))

            result = SpeechRecognitionResult(
                text=transcription.text.strip(),
                confidence=estimated_confidence,
                language=transcription.language,
                duration=duration,
                timestamp=time.time()
            )

            logger.info(f"Transcription completed: '{result.text}' (confidence: {result.confidence:.2f})")
            return result

        except Exception as e:
            logger.error(f"Error transcribing audio: {e}")
            return SpeechRecognitionResult(
                text="",
                confidence=0.0,
                language="unknown",
                duration=0.0,
                timestamp=time.time()
            )

    async def continuous_listening(self, max_duration: float = 30.0):
        """
        Continuously listen for speech and process it

        Args:
            max_duration: Maximum duration for each recording session
        """
        self.is_listening = True
        logger.info("Starting continuous listening...")

        while self.is_listening:
            try:
                # Record audio
                audio_file = await self.record_audio_chunk(max_duration)
                if not audio_file:
                    continue

                # Transcribe audio
                result = await self.transcribe_audio(audio_file)

                # Remove temporary file
                try:
                    os.unlink(audio_file)
                except:
                    pass  # Ignore errors when removing temp file

                # Call the result callback if set
                if self.result_callback and result.text:
                    self.result_callback(result)

                # Small delay to prevent rapid successive recordings
                await asyncio.sleep(0.5)

            except Exception as e:
                logger.error(f"Error in continuous listening: {e}")
                await asyncio.sleep(1)  # Wait before retrying

    def stop_listening(self):
        """
        Stop the continuous listening process
        """
        self.is_listening = False
        logger.info("Stopped continuous listening")

    async def transcribe_from_file(self, audio_file_path: str, language: Optional[str] = None) -> SpeechRecognitionResult:
        """
        Transcribe audio from an existing file

        Args:
            audio_file_path: Path to the audio file to transcribe
            language: Optional language code

        Returns:
            Speech recognition result
        """
        return await self.transcribe_audio(audio_file_path, language)

    async def transcribe_from_microphone(self, duration: float = 10.0) -> SpeechRecognitionResult:
        """
        Record from microphone and transcribe in one step

        Args:
            duration: Recording duration in seconds

        Returns:
            Speech recognition result
        """
        audio_file = await self.record_audio_chunk(duration)
        if not audio_file:
            return SpeechRecognitionResult(
                text="",
                confidence=0.0,
                language="unknown",
                duration=0.0,
                timestamp=time.time()
            )

        result = await self.transcribe_audio(audio_file)

        # Remove temporary file
        try:
            os.unlink(audio_file)
        except:
            pass

        return result

# Example usage and integration with VLA system
async def main():
    """
    Example usage of the Whisper speech recognizer
    """
    # Note: In a real implementation, you would use a real API key
    # This is just a demonstration
    recognizer = WhisperSpeechRecognizer(api_key="YOUR_OPENAI_API_KEY_HERE")

    def handle_recognition_result(result: SpeechRecognitionResult):
        """
        Handle speech recognition results
        """
        print(f"\nSpeech recognized: '{result.text}'")
        print(f"Confidence: {result.confidence:.2f}")
        print(f"Language: {result.language}")
        print(f"Duration: {result.duration:.2f}s")

        # Here you would typically pass the recognized text to your
        # language understanding system in the VLA pipeline
        if result.confidence > 0.5:  # Only process if confidence is high enough
            print(f"Processing command: {result.text}")
            # In a real system, you would call your VLA processing here
        else:
            print("Low confidence recognition, ignoring...")

    # Set up the result callback
    recognizer.set_result_callback(handle_recognition_result)

    print("Starting speech recognition demo...")
    print("Press Ctrl+C to stop")

    try:
        # Option 1: Single transcription from microphone
        print("\nOption 1: Recording for 5 seconds...")
        result = await recognizer.transcribe_from_microphone(duration=5.0)
        print(f"Single transcription result: '{result.text}'")

        # Option 2: Continuous listening (uncomment to use)
        # await recognizer.continuous_listening()

    except KeyboardInterrupt:
        print("\nStopping...")
        recognizer.stop_listening()

    print("Demo completed.")

# Additional utility functions for VLA integration
class VLAAudioProcessor:
    """
    Additional audio processing utilities for VLA systems
    """

    @staticmethod
    def preprocess_audio_for_robot(audio_data: bytes) -> bytes:
        """
        Preprocess audio data for better recognition in robot environments
        """
        # Apply noise reduction, normalize volume, etc.
        # This is a simplified example
        return audio_data

    @staticmethod
    def detect_wake_word(audio_data: bytes, wake_words: list = ["robot", "hey robot", "hello"]) -> bool:
        """
        Simple wake word detection (in practice, you'd use a dedicated model)
        """
        # This is a simplified example - in practice, use a proper wake word detection model
        # For demonstration only
        return any(wake_word.lower() in audio_data.decode('utf-8', errors='ignore').lower()
                  for wake_word in wake_words if isinstance(audio_data, bytes))

    @staticmethod
    def filter_speech_for_vla(text: str) -> Dict[str, Any]:
        """
        Filter and structure recognized speech for VLA processing
        """
        # Identify if the speech contains commands, questions, or other types
        is_command = any(word in text.lower() for word in ["go", "move", "pick", "place", "grasp", "walk"])
        is_question = "?" in text or any(word in text.lower() for word in ["what", "where", "how", "who", "why"])

        return {
            "text": text,
            "is_command": is_command,
            "is_question": is_question,
            "intent": "command" if is_command else "question" if is_question else "statement"
        }

if __name__ == "__main__":
    # Run the example
    # Note: This requires an active OpenAI API key to work properly
    asyncio.run(main())