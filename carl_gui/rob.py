import hsrb_interface
import speech_control

hsrb_interface.Robot()
talker = speech_control.SpeechControl()

talker.speak("hello!")