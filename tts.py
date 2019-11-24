# Import the required module for text  
# to speech conversion 
from gtts import gTTS 
from io import BytesIO  
# This module is imported so that we can  
# play the converted audio 
import os 
 

def speak(audioString):
    print(audioString)
   # mp3_fp= ByteIO()
    tts = gTTS(text=audioString, lang='hi')
   # tts.write_to_fp(mp3_fb)
    tts.save("audio.mp3")
    os.system("mpg321 audio.mp3")

